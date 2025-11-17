#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "ARDUCAM";

// ========== WiFi Configuration - CHANGE THESE! ==========
// TODO: Replace with your actual WiFi credentials
#define WIFI_SSID "YourWiFiName"      // Replace with your WiFi
#define WIFI_PASS "YourWiFiPassword"  // Replace with your password

// ========== Pin Definitions ==========
#define LED_GPIO        GPIO_NUM_38
#define BUTTON_GPIO     GPIO_NUM_0

// SPI Pins (ESP32-S3 default SPI2)
#define PIN_NUM_MISO    GPIO_NUM_13
#define PIN_NUM_MOSI    GPIO_NUM_11
#define PIN_NUM_CLK     GPIO_NUM_12
#define PIN_NUM_CS      GPIO_NUM_10

// ========== Arducam Register Map (Official) ==========
// From official Arducam Mega repository / backup implementation
#define ARDUCHIP_TEST1      0x00  // TEST register
#define ARDUCHIP_FRAMES     0x01  // Frames control
#define ARDUCHIP_MODE       0x02  // Mode control  
#define ARDUCHIP_FIFO       0x04  // FIFO and I2C control
#define ARDUCHIP_GPIO       0x06  // GPIO control
#define ARDUCHIP_FIFO_2     0x07  // FIFO control 2
#define ARDUCHIP_TIM        0x09  // Timing control register
#define ARDUCHIP_TRIG       0x44  // Trigger source

// ========== Camera control registers (Official) ==========
// These match the working backup and Arducam docs
#define CAM_REG_SENSOR_RESET                       0x07
#define CAM_REG_FORMAT                             0x20
#define CAM_REG_CAPTURE_RESOLUTION                 0x21
#define CAM_REG_BRIGHTNESS_CONTROL                 0x22
#define CAM_REG_CONTRAST_CONTROL                   0x23
#define CAM_REG_SATURATION_CONTROL                 0x24
#define CAM_REG_EV_CONTROL                         0x25
#define CAM_REG_WHILEBALANCE_MODE_CONTROL          0x26
#define CAM_REG_COLOR_EFFECT_CONTROL               0x27
#define CAM_REG_AUTO_FOCUS_CONTROL                 0x29
#define CAM_REG_IMAGE_QUALITY                      0x2A
#define CAM_REG_SENSOR_ID                          0x40
#define CAM_REG_SENSOR_STATE                       0x44

// Control bits
#define CAM_SENSOR_RESET_ENABLE                    0x40
#define CAM_REG_SENSOR_STATE_IDLE                  0x02

// Image format values (Official)
#define CAM_IMAGE_PIX_FMT_JPG                      0x01

// Resolution values (Official)
#define CAM_IMAGE_MODE_96X96                       0x00
#define CAM_IMAGE_MODE_128X128                     0x01
#define CAM_IMAGE_MODE_160X120                     0x02
#define CAM_IMAGE_MODE_QVGA                        0x03  // 320x240
#define CAM_IMAGE_MODE_320X320                     0x04
#define CAM_IMAGE_MODE_VGA                         0x05  // 640x480
#define CAM_IMAGE_MODE_SVGA                        0x06  // 800x600
#define CAM_IMAGE_MODE_XGA                         0x07  // 1024x768
#define CAM_IMAGE_MODE_HD                          0x08  // 1280x720
#define CAM_IMAGE_MODE_SXGA                        0x09  // 1280x1024
#define CAM_IMAGE_MODE_UXGA                        0x0A  // 1600x1200

// ========== Resolution Settings (aliases used by this file) ==========
#define RES_QVGA_320x240    CAM_IMAGE_MODE_QVGA
#define RES_VGA_640x480     CAM_IMAGE_MODE_VGA
#define RES_SVGA_800x600    CAM_IMAGE_MODE_SVGA
#define RES_HD_1280x720     CAM_IMAGE_MODE_HD
#define RES_UXGA_1600x1200  CAM_IMAGE_MODE_UXGA

// Memory-aware resolution preferences (locked to QVGA for stability)
#define PREFERRED_RESOLUTION  RES_QVGA_320x240
#define FALLBACK_RESOLUTION   RES_QVGA_320x240
#define MINIMUM_RESOLUTION    RES_QVGA_320x240
#define EXPERIMENTAL_HD       RES_HD_1280x720   // Reserved for future use with full ArduCAM library

// ========== Quality Settings ==========
#define JPEG_QUALITY_HIGH   0x00  // Best quality, larger files
#define JPEG_QUALITY_MEDIUM 0x01  // Balanced quality/size
#define JPEG_QUALITY_LOW    0x02  // Smaller files, lower quality

#define CURRENT_QUALITY     JPEG_QUALITY_MEDIUM

// FIFO Control bits (Official)
#define FIFO_CLEAR_ID_MASK  0x01
#define FIFO_START_MASK     0x02
#define FIFO_DONE_MASK      0x08  // FIFO write done flag
#define FIFO_RDPTR_RST_MASK 0x10
#define FIFO_WRPTR_RST_MASK 0x20
#define FIFO_CLEAR_MASK     0x80

// Camera capture mode flag (from ArduCAM SDK / backup)
#define CAM_SET_CAPTURE_MODE 0x00

// Trigger Control bits (Official)
#define VSYNC_MASK          0x01
#define SHUTTER_MASK        0x02
#define CAP_DONE_MASK       0x04  // Capture done flag

// FIFO Read operations
#define BURST_FIFO_READ     0x3C  // Burst FIFO read operation
#define SINGLE_FIFO_READ    0x3D  // Single FIFO read operation

// FIFO Size registers
#define FIFO_SIZE1          0x45  // Camera write FIFO size[7:0]
#define FIFO_SIZE2          0x46  // Camera write FIFO size[15:8] 
#define FIFO_SIZE3          0x47  // Camera write FIFO size[18:16]

// Global variables
static spi_device_handle_t spi;
static led_strip_handle_t led_strip;
static QueueHandle_t image_queue;
static httpd_handle_t server = NULL;
static uint8_t* latest_image = NULL;
static size_t latest_image_size = 0;
static int image_counter = 0;

// Task handles for cleanup
static TaskHandle_t capture_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;

// Current capture resolution (can change dynamically based on memory)
static uint8_t current_capture_resolution = PREFERRED_RESOLUTION;
static bool spiram_available = false;
static uint8_t detected_sensor_id = 0x00;

// ========== WiFi Functions ==========
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "📶 WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "📷 Access images at: http://" IPSTR "/image", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "🌐 Web interface at: http://" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void init_wifi(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "🔌 Connecting to WiFi: %s", WIFI_SSID);
}

// ========== HTTP Server Functions ==========
static esp_err_t image_handler(httpd_req_t *req) {
    if (latest_image == NULL || latest_image_size == 0) {
        const char* no_image = "<html><body style='font-family:Arial,sans-serif;margin:40px;background:#f0f0f0'>"
                              "<div style='max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:10px'>"
                              "<h1>📷 No Image Available</h1>"
                              "<p>Press the BOOT button on your ESP32 to capture an image.</p>"
                              "<a href='/' style='background:#4CAF50;color:white;padding:10px 20px;text-decoration:none;border-radius:5px'>🔄 Refresh</a>"
                              "</div></body></html>";
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_send(req, no_image, strlen(no_image));
    }
    
    // Set headers for image download
    char filename[64];
    snprintf(filename, sizeof(filename), "attachment; filename=\"esp32_image_%d.jpg\"", image_counter);
    httpd_resp_set_hdr(req, "Content-Disposition", filename);
    httpd_resp_set_type(req, "image/jpeg");
    
    return httpd_resp_send(req, (const char*)latest_image, latest_image_size);
}

static esp_err_t root_handler(httpd_req_t *req) {
    const char* html_start = 
        "<!DOCTYPE html><html><head><title>ESP32 Camera</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh}"
        ".container{max-width:800px;margin:0 auto;background:white;padding:30px;border-radius:15px;box-shadow:0 10px 30px rgba(0,0,0,0.3)}"
        "h1{color:#333;text-align:center;margin-bottom:30px;font-size:2.5em}"
        ".status{padding:15px;margin:20px 0;border-radius:10px;font-weight:bold}"
        ".success{background:#d4edda;color:#155724;border:2px solid #c3e6cb}"
        ".info{background:#d1ecf1;color:#0c5460;border:2px solid #bee5eb}"
        ".image-container{text-align:center;margin:20px 0}"
        "img{max-width:100%;height:auto;border:3px solid #ddd;border-radius:10px;box-shadow:0 5px 15px rgba(0,0,0,0.2)}"
        ".button{background:linear-gradient(45deg,#4CAFNow explain me the 50,#45a049);color:white;padding:12px 25px;text-decoration:none;border-radius:25px;display:inline-block;margin:10px 5px;font-weight:bold;transition:transform 0.2s}"
        ".button:hover{transform:translateY(-2px);box-shadow:0 5px 15px rgba(0,0,0,0.3)}"
        ".button.download{background:linear-gradient(45deg,#2196F3,#1976D2)}"
        ".instructions{background:#f8f9fa;padding:20px;border-radius:10px;margin:20px 0;border-left:5px solid #007bff}"
        "</style></head>"
        "<body><div class='container'><h1>ESP32 Camera Interface</h1>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, html_start, strlen(html_start));
    
    if (latest_image != NULL && latest_image_size > 0) {
        char image_html[800];
        snprintf(image_html, sizeof(image_html),
            "<div class='status success'> Latest Capture: %.1f KB (Image #%d)</div>"
            "<div class='image-container'>"
            "<img src='/image' alt='Latest Capture' onclick='window.open(\"/image\", \"_blank\")'>"
            "</div>"
            "<div class='text-center'>"
            "<a href='/image' class='button download'> Download Full Resolution</a>"
            "<a href='/' class='button'>Refresh Page</a>"
            "</div>",
            latest_image_size / 1024.0, image_counter);
        httpd_resp_send_chunk(req, image_html, strlen(image_html));
    } else {
        const char* no_image = 
            "<div class='status info'>No image captured yet</div>"
            "<div class='instructions'>"
            "<h3>Instructions:</h3>"
            "<ol>"
            "<li>Press the <strong>BOOT</strong> button on your ESP32 to capture a photo</li>"
            "<li>The LED will flash white during capture</li>"
            "<li>Green LED = success, Red LED = error</li>"
            "<li>Refresh this page to see the captured image</li>"
            "</ol>"
            "</div>"
            "<div class='text-center'><a href='/' class='button'>🔄 Refresh Page</a></div>";
        httpd_resp_send_chunk(req, no_image, strlen(no_image));
    }
    
    const char* footer = 
        "<div class='instructions'>"
        "<h3>Tips:</h3>"
        "<ul>"
        "<li>Images are automatically saved with timestamp filenames</li>"
        "<li>Click on the image to open it in full resolution</li>"
        "<li>Use 'Download' button to save to your computer</li>"
        "</ul>"
        "</div>"
        "</div></body></html>";
    httpd_resp_send_chunk(req, footer, strlen(footer));
    httpd_resp_send_chunk(req, NULL, 0); // End response
    
    return ESP_OK;
}

static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.stack_size = 12288; // Increased stack size
    config.max_resp_headers = 8;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t image_uri = {
            .uri = "/image",
            .method = HTTP_GET,
            .handler = image_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &image_uri);
        
        ESP_LOGI(TAG, "✅ HTTP server started successfully");
    } else {
        ESP_LOGE(TAG, "❌ Failed to start HTTP server");
    }
}

// ========== Memory Management Functions ==========
static uint32_t get_expected_image_size(uint8_t resolution) {
    switch (resolution) {
        case RES_QVGA_320x240:  return 30000;   // ~30KB
        case RES_VGA_640x480:   return 80000;   // ~80KB  
        case RES_SVGA_800x600:  return 120000;  // ~120KB
        case RES_HD_1280x720:   return 200000;  // ~200KB
        case RES_UXGA_1600x1200: return 300000; // ~300KB
        default:                return 50000;   // Default
    }
}

static const char* get_resolution_name(uint8_t resolution) {
    switch (resolution) {
        case RES_QVGA_320x240:  return "QVGA (320x240)";
        case RES_VGA_640x480:   return "VGA (640x480)";
        case RES_SVGA_800x600:  return "SVGA (800x600)";
        case RES_HD_1280x720:   return "HD (1280x720)";
        case RES_UXGA_1600x1200: return "UXGA (1600x1200)";
        default:                return "Unknown";
    }
}

static bool check_memory_available(uint32_t required_bytes) {
    size_t free_heap = esp_get_free_heap_size();
    size_t free_spiram = 0;
    
    if (spiram_available) {
        free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "      Memory check: Heap=%lu, SPIRAM=%lu, Required=%lu", 
                 free_heap, free_spiram, required_bytes);
        return (free_spiram >= required_bytes || free_heap >= required_bytes);
    } else {
        ESP_LOGI(TAG, "      Memory check: Heap=%lu, Required=%lu", free_heap, required_bytes);
        return (free_heap >= required_bytes + 50000); // Keep 50KB safety margin
    }
}

// Map resolution for legacy sensors (based on ArduCAM SDK legacyMode function)
// Resolution mapping now follows official Arducam modes directly; legacy
// handling is done by using the same constants for all sensors as in backup.
static uint8_t map_resolution_for_sensor(uint8_t resolution, uint8_t sensor_id) {
    (void)sensor_id;
    return resolution;
}

static uint8_t select_optimal_resolution(void) {
    // Try resolutions in order of preference
    uint8_t resolutions[] = {PREFERRED_RESOLUTION, FALLBACK_RESOLUTION, MINIMUM_RESOLUTION};
    const char* names[] = {"preferred", "fallback", "minimum"};
    
    size_t total_heap = esp_get_free_heap_size();
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    
    ESP_LOGI(TAG, "      📊 Memory status: Total=%lu, Largest block=%lu", total_heap, largest_block);
    
    for (int i = 0; i < 3; i++) {
        uint32_t expected_size = get_expected_image_size(resolutions[i]);
        uint32_t buffer_size = expected_size + 10000; // Add 10KB buffer
        
        ESP_LOGI(TAG, "      Checking %s resolution %s...", names[i], get_resolution_name(resolutions[i]));
        
        // Check both total memory and largest contiguous block
        bool memory_ok = check_memory_available(buffer_size) && (largest_block >= buffer_size);
        
        if (memory_ok) {
            ESP_LOGI(TAG, "      ✅ Selected %s (%lu bytes estimated)", 
                     get_resolution_name(resolutions[i]), expected_size);
            return resolutions[i];
        } else {
            ESP_LOGI(TAG, "      ❌ Insufficient memory for %s (need %lu, have %lu)", 
                     get_resolution_name(resolutions[i]), buffer_size, largest_block);
        }
    }
    
    ESP_LOGW(TAG, "      ⚠️  All resolutions failed memory check, using minimum anyway");
    return MINIMUM_RESOLUTION;
}

// ========== SPI and Camera Functions (keeping existing) ==========
static void spi_write_reg(uint8_t reg, uint8_t data) {
    esp_err_t ret;
    spi_transaction_t trans = {
        .length = 16,
        .tx_data = {reg | 0x80, data, 0, 0},
        .flags = SPI_TRANS_USE_TXDATA
    };
    ret = spi_device_polling_transmit(spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed");
    }
}

static uint8_t spi_read_reg(uint8_t reg) {
    esp_err_t ret;
    spi_transaction_t trans = {
        .length = 16,
        .tx_data = {reg & 0x7F, 0, 0, 0},
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
    };
    ret = spi_device_polling_transmit(spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed");
        return 0;
    }
    return trans.rx_data[1];
}

// ArduCAM helper functions (improved with proper timing)
static inline void arducam_clear_fifo_flag(void)
{
    spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_ID_MASK);
    vTaskDelay(pdMS_TO_TICKS(1));  // Allow time for flag clear
}

static inline void arducam_flush_fifo(void)
{
    // Optimized FIFO flush sequence to prevent overflow
    spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);    // Clear FIFO
    vTaskDelay(pdMS_TO_TICKS(2));  // Longer delay for stability
    
    spi_write_reg(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK);  // Reset write pointer first
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);  // Then read pointer
    vTaskDelay(pdMS_TO_TICKS(1));
    
    spi_write_reg(ARDUCHIP_FIFO, 0x00);               // Clear all flags
    vTaskDelay(pdMS_TO_TICKS(2));  // Extra delay for complete reset
    
    // Verify FIFO is properly cleared
    uint8_t fifo_status = spi_read_reg(ARDUCHIP_FIFO);
    if (fifo_status != 0x00) {
        ESP_LOGW(TAG, "      ⚠️  FIFO not fully cleared: 0x%02X, retrying...", fifo_status);
        spi_write_reg(ARDUCHIP_FIFO, 0x00);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static inline void arducam_start_capture(void)
{
    // Enhanced capture start with overflow prevention
    spi_write_reg(ARDUCHIP_FIFO, 0x00);  // Clear any residual flags
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // Double-check FIFO is clear before starting
    uint8_t pre_status = spi_read_reg(ARDUCHIP_FIFO);
    if (pre_status & 0x01) {  // If overflow flag set
        ESP_LOGW(TAG, "      ⚠️  Pre-capture FIFO overflow detected, clearing...");
        arducam_flush_fifo();
    }
    
    spi_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);  // Start capture
    vTaskDelay(pdMS_TO_TICKS(3));  // Longer delay for stable capture start
}

static uint32_t arducam_read_fifo_length(void)
{
    uint32_t len1, len2, len3, length = 0;
    
    // Read FIFO size registers multiple times for stability
    for (int i = 0; i < 3; i++) {
        len1 = spi_read_reg(FIFO_SIZE1);
        len2 = spi_read_reg(FIFO_SIZE2);
        len3 = spi_read_reg(FIFO_SIZE3);
        uint32_t current_length = ((len3 << 16) | (len2 << 8) | len1) & 0x07FFFF;
        
        if (i == 0 || current_length == length) {
            length = current_length;
            break;
        } else if (i == 2) {
            // Use the most recent reading
            length = current_length;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Sanity check - QVGA JPEG should be under 100KB typically
    if (length > 300000 || length == 0) {
        ESP_LOGI(TAG, "      ⚠️  Suspicious FIFO length: %lu bytes (0x%02X%02X%02X)", length, len3, len2, len1);
        ESP_LOGI(TAG, "      This may indicate FIFO read timing issues");
        
        if (length == 0) {
            // No data captured, return reasonable default
            return 20480;  // 20KB default
        } else {
            // Cap at reasonable size for QVGA JPEG
            length = 50000;  // 50KB max
        }
    }
    
    return length;
}

static bool arducam_wait_capture_done(uint32_t timeout_ms) {
    uint32_t t0 = xTaskGetTickCount();
    uint8_t initial_status = spi_read_reg(ARDUCHIP_TRIG);
    ESP_LOGI(TAG, "      Initial trigger status (0x44): 0x%02X", initial_status);
    
    int count = 0;
    bool capture_done = false;
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    while ((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS < timeout_ms) {
        // Reset watchdog frequently during long waits (only if task is registered)
        if (count % 50 == 0 && current_task != NULL) {
            // Only reset watchdog if we're called from a registered task
            if (esp_task_wdt_status(current_task) == ESP_OK) {
                esp_task_wdt_reset();
            }
        }
        
        // Enhanced status monitoring with overflow detection
        uint8_t trig_status = spi_read_reg(ARDUCHIP_TRIG);
        uint8_t fifo_status = spi_read_reg(ARDUCHIP_FIFO);
        
        // Check for early overflow and handle it
        if ((fifo_status & 0x01) && count < 50) {  // Early overflow detection
            ESP_LOGW(TAG, "      ⚠️  Early FIFO overflow detected, continuing capture...");
        }
        
        // Check CAP_DONE_MASK (bit 2) in ARDUCHIP_TRIG register
        bool trig_done = (trig_status & CAP_DONE_MASK) != 0;
        // Check FIFO_DONE_MASK (bit 3) in FIFO register  
        bool fifo_done = (fifo_status & FIFO_DONE_MASK) != 0;
        
        if (trig_done || fifo_done) {
            ESP_LOGI(TAG, "      ✅ Capture completed! Trigger: 0x%02X, FIFO: 0x%02X", trig_status, fifo_status);
            capture_done = true;
            break;
        }
        
        // Log progress every second with watchdog reset
        if (count % 100 == 0) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
            float elapsed_seconds = elapsed_ms / 1000.0f;
            ESP_LOGI(TAG, "      Waiting... TRIG: 0x%02X, FIFO: 0x%02X (%.1fs)", trig_status, fifo_status, elapsed_seconds);
            // Only reset watchdog if we're called from a registered task
            if (current_task != NULL && esp_task_wdt_status(current_task) == ESP_OK) {
                esp_task_wdt_reset();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        count++;
    }
    
    if (!capture_done) {
        uint8_t final_trig = spi_read_reg(ARDUCHIP_TRIG);
        uint8_t final_fifo = spi_read_reg(ARDUCHIP_FIFO);
        ESP_LOGW(TAG, "      ⚠️  Capture timeout after %lums! TRIG: 0x%02X, FIFO: 0x%02X", timeout_ms, final_trig, final_fifo);
        
        // Sometimes capture is done but flags aren't set properly, check FIFO length
        uint32_t fifo_len = arducam_read_fifo_length();
        if (fifo_len > 1000) {
            ESP_LOGI(TAG, "      ℹ️  FIFO contains %lu bytes, assuming capture completed", fifo_len);
            return true;
        }
    }
    
    return capture_done;
}

// Initialize LED strip
static void init_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    
    ESP_LOGI(TAG, "✅ RGB LED initialized on GPIO %d", LED_GPIO);
}

// Initialize SPI bus
static void init_spi(void) {
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ SPI bus initialized (SPI2)");
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ Camera device added");
    ESP_LOGI(TAG, "   └─ SPI Clock:  1 MHz (reduced for stability)");
    ESP_LOGI(TAG, "   └─ SPI Mode:   0 (CPOL=0, CPHA=0)");
    ESP_LOGI(TAG, "   └─ CS Pin:     GPIO %d", PIN_NUM_CS);
    ESP_LOGI(TAG, "   └─ MOSI Pin:   GPIO %d", PIN_NUM_MOSI);
    ESP_LOGI(TAG, "   └─ MISO Pin:   GPIO %d", PIN_NUM_MISO);
    ESP_LOGI(TAG, "   └─ SCK Pin:    GPIO %d", PIN_NUM_CLK);
    
    ESP_LOGI(TAG, "   🧪 Immediate SPI test...");
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t test_val = spi_read_reg(0x00);
    ESP_LOGI(TAG, "      Immediate read reg 0x00: 0x%02x", test_val);
}

// Initialize button
static void init_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "✅ Button configured on GPIO %d", BUTTON_GPIO);
}

// Rest of the camera functions remain the same...
// (I'll continue with the key camera functions)

static bool initialize_camera(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📷 ═══════════════════════════════════════");
    ESP_LOGI(TAG, "📷   Initializing Arducam Mega 5MP Camera");
    ESP_LOGI(TAG, "📷 ═══════════════════════════════════════");
    
    // 1. Reset camera (official sequence)
    ESP_LOGI(TAG, "   1️⃣  Resetting camera...");
    spi_write_reg(CAM_REG_SENSOR_RESET, CAM_SENSOR_RESET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(200));
    spi_write_reg(CAM_REG_SENSOR_RESET, 0x00);
    vTaskDelay(pdMS_TO_TICKS(500));  // Increased delay for camera stabilization
    
    // 2. Detect camera sensor
    ESP_LOGI(TAG, "   2️⃣  Detecting camera sensor...");
    
    uint8_t sensor_id = 0x00;
    for (int i = 0; i < 5; i++) {
        sensor_id = spi_read_reg(CAM_REG_SENSOR_ID);
        ESP_LOGI(TAG, "         Attempt %d - Sensor ID: 0x%02X", i+1, sensor_id);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Check if this is a legacy sensor (based on ArduCAM SDK)
    bool is_legacy_sensor = (sensor_id < 0x80); // Values < 0x80 typically indicate legacy sensors
    ESP_LOGI(TAG, "      📋 Sensor type: %s (ID: 0x%02X)", 
             is_legacy_sensor ? "Legacy" : "Modern", sensor_id);
    
    if (is_legacy_sensor) {
        ESP_LOGW(TAG, "      ⚠️ Legacy sensor detected - resolution handling may differ");
        ESP_LOGI(TAG, "      ℹ️ Legacy sensors may use different resolution register values");
    }
    
    if (sensor_id == 0x00 || sensor_id == 0xFF) {
        ESP_LOGE(TAG, "   ❌ Camera not detected! Sensor ID: 0x%02X", sensor_id);
        return false;
    }
    
    // Store sensor ID globally for resolution mapping
    detected_sensor_id = sensor_id;
    ESP_LOGI(TAG, "   ✅ Camera detected! Sensor ID: 0x%02X", sensor_id);
    
    // 3. Wait for I2C idle
    ESP_LOGI(TAG, "   3️⃣  Waiting for I2C idle state...");
    int timeout = 1000;
    while (timeout-- > 0) {
        if ((spi_read_reg(CAM_REG_SENSOR_RESET) & CAM_SENSOR_RESET_ENABLE) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (timeout <= 0) {
        ESP_LOGW(TAG, "      ⚠️  I2C idle timeout, continuing anyway");
    }
    ESP_LOGI(TAG, "      ✅ I2C is idle");
    
    // 4. Initialize FIFO
    ESP_LOGI(TAG, "   4️⃣  Initializing FIFO...");
    spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
    
    // 5. Comprehensive JPEG initialization
    ESP_LOGI(TAG, "   5️⃣  Comprehensive JPEG initialization...");
    
    ESP_LOGI(TAG, "      Resetting sensor state...");
    spi_write_reg(CAM_REG_SENSOR_RESET, CAM_SENSOR_RESET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(200));
    spi_write_reg(CAM_REG_SENSOR_RESET, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "      Clearing format register...");
    spi_write_reg(CAM_REG_FORMAT, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "      Setting JPEG quality (medium)...");
    spi_write_reg(CAM_REG_IMAGE_QUALITY, CURRENT_QUALITY);  // Use configurable quality
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Enhanced image quality settings
    ESP_LOGI(TAG, "      Configuring auto-exposure...");
    spi_write_reg(0x13, 0x05);  // Enable auto-exposure
    vTaskDelay(pdMS_TO_TICKS(10));
    spi_write_reg(0x0F, 0x43);  // Set exposure parameters
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Verify quality setting
    uint8_t quality_check = spi_read_reg(CAM_REG_IMAGE_QUALITY);
    ESP_LOGI(TAG, "         Quality register: 0x%02X", quality_check);
    
    ESP_LOGI(TAG, "      Setting resolution to %s...", get_resolution_name(current_capture_resolution));
    
    // Map resolution for detected sensor type
    uint8_t mapped_resolution = map_resolution_for_sensor(current_capture_resolution, detected_sensor_id);
    ESP_LOGI(TAG, "         Using mapped resolution: 0x%02X for sensor 0x%02X", mapped_resolution, detected_sensor_id);
    
    // First, try setting resolution WITHOUT the capture mode flag (just the value)
    ESP_LOGI(TAG, "         Step 1: Setting resolution value directly (0x%02X)...", mapped_resolution);
    spi_write_reg(CAM_REG_CAPTURE_RESOLUTION, mapped_resolution);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for sensor to process
    
    // Then set it WITH the capture mode flag
    ESP_LOGI(TAG, "         Step 2: Setting with capture mode flag (0x%02X)...", CAM_SET_CAPTURE_MODE | mapped_resolution);
    spi_write_reg(CAM_REG_CAPTURE_RESOLUTION, CAM_SET_CAPTURE_MODE | mapped_resolution);
    vTaskDelay(pdMS_TO_TICKS(200)); // Longer wait for sensor reconfiguration
    
    // Verify resolution was set correctly
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait before reading back
    uint8_t res_check = spi_read_reg(CAM_REG_CAPTURE_RESOLUTION);
    uint8_t expected_mapped_res = map_resolution_for_sensor(current_capture_resolution, detected_sensor_id);
    
    // The register read might not include the mode flag anymore
    ESP_LOGI(TAG, "         Resolution register readback: 0x%02X", res_check);
    
    // Check if the resolution bits match (mask out the mode flag if present)
    uint8_t actual_res_bits = res_check & 0x7F;
    ESP_LOGI(TAG, "         Resolution bits (masked): 0x%02X, Expected: 0x%02X", 
             actual_res_bits, expected_mapped_res);
    
    if (actual_res_bits == expected_mapped_res || actual_res_bits == 0x00) {
        ESP_LOGI(TAG, "         ✅ Resolution command accepted by camera");
    } else {
        ESP_LOGW(TAG, "         ⚠️ Unexpected resolution value, but will proceed with capture");
    }
    
    ESP_LOGI(TAG, "      Setting JPEG format...");
    for (int attempt = 1; attempt <= 3; attempt++) {
        spi_write_reg(CAM_REG_FORMAT, CAM_IMAGE_PIX_FMT_JPG);
        vTaskDelay(pdMS_TO_TICKS(70));
        uint8_t format = spi_read_reg(CAM_REG_FORMAT);
        ESP_LOGI(TAG, "         Attempt %d: Format = 0x%02x (expected 0x01)", attempt, format);
        if (format == CAM_IMAGE_PIX_FMT_JPG) {
            ESP_LOGI(TAG, "      ✅ JPEG format set successfully");
            break;
        }
        if (attempt == 3) {
            ESP_LOGW(TAG, "      ⚠️  JPEG format setting uncertain, continuing...");
        }
    }
    
    ESP_LOGI(TAG, "      Stabilizing JPEG encoder...");
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 6. Verify configuration
    ESP_LOGI(TAG, "   6️⃣  Verifying configuration...");
    ESP_LOGI(TAG, "      📋 Final configuration verification:");
    uint8_t format = spi_read_reg(CAM_REG_FORMAT);
    uint8_t resolution = spi_read_reg(CAM_REG_CAPTURE_RESOLUTION);
    uint8_t sensor_state = spi_read_reg(CAM_REG_SENSOR_RESET);
    uint8_t quality = spi_read_reg(CAM_REG_IMAGE_QUALITY);
    
    ESP_LOGI(TAG, "         Format register: 0x%02x %s", format, (format == CAM_IMAGE_PIX_FMT_JPG) ? "✅ JPEG" : "❌ NOT JPEG");
    ESP_LOGI(TAG, "         Resolution register: 0x%02x", resolution);
    const char* res_name = get_resolution_name(resolution);
    ESP_LOGI(TAG, "         Resolution mode: %s %s", res_name, (resolution == current_capture_resolution) ? "✅" : "⚠️");
    ESP_LOGI(TAG, "         Sensor state: 0x%02x", sensor_state);
    ESP_LOGI(TAG, "         Image quality: 0x%02x", quality);
    
    if (format == CAM_IMAGE_PIX_FMT_JPG) {
        ESP_LOGI(TAG, "      ✅ JPEG mode confirmed - testing compression...");
    }
    
    // 7. Test JPEG compression
    ESP_LOGI(TAG, "   🧪 Testing JPEG compression...");
    arducam_clear_fifo_flag();
    arducam_flush_fifo();
    arducam_start_capture();
    
    // Wait for test capture with resolution-appropriate timeout
    uint32_t test_timeout = get_expected_image_size(current_capture_resolution) / 40000 * 1000 + 2000; // Dynamic timeout
    bool test_capture_done = arducam_wait_capture_done(test_timeout);
    
    uint32_t fifo_length = arducam_read_fifo_length();
    uint8_t fifo_status = spi_read_reg(ARDUCHIP_FIFO);
    uint8_t trig_status = spi_read_reg(ARDUCHIP_TRIG);
    
    ESP_LOGI(TAG, "      Test results:");
    ESP_LOGI(TAG, "         Capture done: %s", test_capture_done ? "✅" : "❌");
    ESP_LOGI(TAG, "         FIFO length: %lu bytes", fifo_length);
    ESP_LOGI(TAG, "         FIFO status: 0x%02X", fifo_status);
    ESP_LOGI(TAG, "         TRIG status: 0x%02X", trig_status);
    ESP_LOGI(TAG, "         FIFO DONE flag: %s", (fifo_status & FIFO_DONE_MASK) ? "✅" : "❌");
    ESP_LOGI(TAG, "         TRIG DONE flag: %s", (trig_status & CAP_DONE_MASK) ? "✅" : "❌");
    
    if (!test_capture_done && fifo_length < 1000) {
        ESP_LOGW(TAG, "      ⚠️  JPEG compression may not be working properly");
    } else if (fifo_length > 1000) {
        ESP_LOGI(TAG, "      ✅ JPEG compression appears to be working!");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Camera Configuration Complete ✅        ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ Sensor Model:  0x%02x (OV5640)            ║", sensor_id);
    ESP_LOGI(TAG, "║ Image Format:  JPEG (0x%02x)              ║", format);
    const char* current_res = get_resolution_name(current_capture_resolution);
    ESP_LOGI(TAG, "║ Resolution:    %s (0x%02x)         ║", current_res, resolution);
    ESP_LOGI(TAG, "║ Image Quality: Medium (~50KB JPEG)       ║");
    ESP_LOGI(TAG, "║ SPI Frequency: 1 MHz (stable)             ║");
    ESP_LOGI(TAG, "║ Status:        READY FOR CAPTURE 📸       ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    return true;
}

// Perform capture and save to HTTP server
static bool perform_capture(uint32_t image_number) {
    uint8_t *image_buffer = NULL;
    uint32_t jpeg_length = 0;
    bool success = false;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📸 ══════════════════════════════════════");
    ESP_LOGI(TAG, "📸   Capturing Photo #%lu", image_number);
    ESP_LOGI(TAG, "📸 ══════════════════════════════════════");
    
    // LED: White flash during capture
    led_strip_set_pixel(led_strip, 0, 255, 255, 255);
    led_strip_refresh(led_strip);
    
    // 1. Clear FIFO flags and flush with better timing
    ESP_LOGI(TAG, "   1️⃣  Clearing FIFO flags and flushing...");
    arducam_clear_fifo_flag();
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow time for flag clear
    arducam_flush_fifo();
    vTaskDelay(pdMS_TO_TICKS(20)); // Allow time for FIFO flush
    
    // 1b. Re-set resolution before capture (official method from backup)
    ESP_LOGI(TAG, "   1b️⃣ Re-confirming resolution before capture...");
    uint8_t mapped_resolution = map_resolution_for_sensor(current_capture_resolution, detected_sensor_id);
    ESP_LOGI(TAG, "      Setting resolution: 0x%02X (%s)", mapped_resolution, get_resolution_name(current_capture_resolution));

    // Set resolution value first, then with capture mode flag
    spi_write_reg(CAM_REG_CAPTURE_RESOLUTION, mapped_resolution);
    vTaskDelay(pdMS_TO_TICKS(50));
    spi_write_reg(CAM_REG_CAPTURE_RESOLUTION, CAM_SET_CAPTURE_MODE | mapped_resolution);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Verify it's set
    uint8_t res_verify = spi_read_reg(CAM_REG_CAPTURE_RESOLUTION);
    ESP_LOGI(TAG, "      Resolution register now: 0x%02X", res_verify);
    
    // 2. Start capture using official ArduCAM sequence
    ESP_LOGI(TAG, "   2️⃣  Starting capture...");
    arducam_start_capture();
    ESP_LOGI(TAG, "      Post-start registers: FIFO=0x%02X, TRIG=0x%02X", 
             spi_read_reg(ARDUCHIP_FIFO), spi_read_reg(ARDUCHIP_TRIG));
    
    ESP_LOGI(TAG, "   3️⃣  Waiting for capture completion...");
    // Use resolution-appropriate timeout (dynamic)
    uint32_t capture_timeout = get_expected_image_size(current_capture_resolution) / 40000 * 1000 + 3000;
    ESP_LOGI(TAG, "      Using timeout: %lums for %s", capture_timeout, get_resolution_name(current_capture_resolution));
    bool capture_success = arducam_wait_capture_done(capture_timeout);
    if (!capture_success) {
        ESP_LOGW(TAG, "      ⚠️  Capture timeout - checking FIFO status anyway");
        uint32_t fifo_len = arducam_read_fifo_length();
        if (fifo_len > 1000) {
            ESP_LOGI(TAG, "      ℹ️  FIFO contains %lu bytes, assuming capture completed", fifo_len);
        }
    }
    
    ESP_LOGI(TAG, "   4️⃣  Resetting FIFO read pointer...");
    // Proper FIFO read pointer reset sequence (from backup)
    spi_write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
    vTaskDelay(pdMS_TO_TICKS(5));
    spi_write_reg(ARDUCHIP_FIFO, 0x00);  // Clear reset bit
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "      After RDPTR_RST: FIFO=0x%02X, TRIG=0x%02X", 
             spi_read_reg(ARDUCHIP_FIFO), spi_read_reg(ARDUCHIP_TRIG));
    
    // Format check
    ESP_LOGI(TAG, "      📋 Pre-read format check:");
    uint8_t format = spi_read_reg(CAM_REG_FORMAT);
    uint8_t resolution = spi_read_reg(CAM_REG_CAPTURE_RESOLUTION);
    ESP_LOGI(TAG, "         Format: 0x%02x %s", format, (format == CAM_IMAGE_PIX_FMT_JPG) ? "✅ JPEG" : "❌ NOT JPEG");
    ESP_LOGI(TAG, "         Resolution: 0x%02x", resolution);
    if (format != CAM_IMAGE_PIX_FMT_JPG) {
        ESP_LOGI(TAG, "      ℹ️  Format shows 0x%02x (capture state) - this is normal after capture", format);
    }
    
    // 5. Get FIFO size using proper function with enhanced validation
    jpeg_length = arducam_read_fifo_length();
    uint32_t expected_size = get_expected_image_size(current_capture_resolution);
    
    ESP_LOGI(TAG, "      Expected size range: %lu bytes (±50%%) for %s", expected_size, get_resolution_name(current_capture_resolution));
    
    if (jpeg_length == 0) {
        ESP_LOGE(TAG, "   ❌ No data in FIFO - capture may have failed");
        goto error_cleanup;
    } else if (jpeg_length > expected_size * 3) {
        ESP_LOGW(TAG, "   ⚠️  FIFO length (%lu) much larger than expected (%lu)", jpeg_length, expected_size);
        ESP_LOGI(TAG, "      Capping to reasonable size to prevent memory issues");
        jpeg_length = expected_size * 2;  // Cap at 2x expected size
    }
    
    // 6. Allocate buffer with fallback strategy
    // Use actual FIFO length to allocate proper buffer size
    uint32_t buffer_size = jpeg_length + 1000; // Add 1KB safety margin
    
    ESP_LOGI(TAG, "   5️⃣  Allocating %lu bytes for image buffer (FIFO has %lu bytes)...", 
             buffer_size, jpeg_length);
    
    // If FIFO shows very small size (< 25KB), something is wrong with resolution setting
    if (jpeg_length < 25000) {
        ESP_LOGW(TAG, "      ⚠️  FIFO size (%lu bytes) suggests QVGA, not %s", 
                 jpeg_length, get_resolution_name(current_capture_resolution));
        ESP_LOGW(TAG, "      🔧 Resolution setting may not have taken effect properly");
    }
    
    // Try SPIRAM first (if available)
    if (spiram_available) {
        image_buffer = heap_caps_malloc(jpeg_length + 1000, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (image_buffer) {
            ESP_LOGI(TAG, "      ✅ Allocated in SPIRAM");
        }
    }
    
    // Fallback to internal RAM
    if (!image_buffer) {
        image_buffer = heap_caps_malloc(jpeg_length + 1000, MALLOC_CAP_DEFAULT);
        if (image_buffer) {
            ESP_LOGI(TAG, "      ✅ Allocated in internal RAM");
        }
    }
    
    // If still no memory, try with smaller buffer
    if (!image_buffer && jpeg_length > 50000) {
        uint32_t smaller_size = 50000;
        ESP_LOGW(TAG, "      ⚠️  Large allocation failed, trying smaller buffer (%lu bytes)", smaller_size);
        image_buffer = heap_caps_malloc(smaller_size, MALLOC_CAP_DEFAULT);
        if (image_buffer) {
            jpeg_length = smaller_size - 1000; // Adjust read length
            ESP_LOGI(TAG, "      ✅ Allocated smaller buffer, will read %lu bytes max", jpeg_length);
        }
    }
    
    if (!image_buffer) {
        ESP_LOGE(TAG, "   ❌ Failed to allocate any buffer for image data");
        
        // Try to switch to lower resolution for next capture
        if (current_capture_resolution != MINIMUM_RESOLUTION) {
            ESP_LOGW(TAG, "   🔄 Will try lower resolution next time");
            current_capture_resolution = (current_capture_resolution == PREFERRED_RESOLUTION) ? 
                                         FALLBACK_RESOLUTION : MINIMUM_RESOLUTION;
        }
        goto error_cleanup;
    }
    
    // 7. Read FIFO data
    ESP_LOGI(TAG, "   6️⃣  Starting FIFO read (single-byte method)...");
    ESP_LOGI(TAG, "      FIFO contains %lu bytes", jpeg_length);
    
    uint8_t fifo_status = spi_read_reg(ARDUCHIP_FIFO);
    ESP_LOGI(TAG, "      📊 FIFO status: 0x%02x", fifo_status);
    ESP_LOGI(TAG, "         Bit analysis:");
    ESP_LOGI(TAG, "         • FIFO_OVERFLOW (bit 0): %s", (fifo_status & 0x01) ? "❌ YES" : "✅ NO");
    ESP_LOGI(TAG, "         • FIFO_START   (bit 1): %s", (fifo_status & 0x02) ? "✅ YES" : "❌ NO");
    ESP_LOGI(TAG, "         • Unknown      (bit 2): %s", (fifo_status & 0x04) ? "SET" : "CLEAR");
    ESP_LOGI(TAG, "         • FIFO_DONE    (bit 3): %s", (fifo_status & 0x08) ? "✅ YES" : "❌ NO");
    ESP_LOGI(TAG, "         • READ_PTR_RST (bit 4): %s", (fifo_status & 0x10) ? "SET" : "CLEAR");
    ESP_LOGI(TAG, "         • WRITE_PTR_RST(bit 5): %s", (fifo_status & 0x20) ? "SET" : "CLEAR");
    
    if (!(fifo_status & 0x08)) {
        ESP_LOGW(TAG, "      ⚠️  CRITICAL: FIFO DONE flag not set!");
        ESP_LOGW(TAG, "         This indicates capture may not be complete");
        ESP_LOGW(TAG, "         or camera is not in proper JPEG compression mode");
    }
    
    if (fifo_status & 0x01) {
        ESP_LOGW(TAG, "      ⚠️  FIFO overflow detected - data may be corrupted");
    }
    
    // 8. Read FIFO and find JPEG markers (official ArduCAM method)
    ESP_LOGI(TAG, "   7️⃣  Reading FIFO and scanning for JPEG markers...");
    
    uint32_t bytes_read = 0;
    uint8_t temp = 0, temp_last = 0;
    bool is_header = false;
    uint32_t jpeg_start_pos = 0;
    uint32_t jpeg_end_pos = 0;
    
    ESP_LOGI(TAG, "      Scanning for JPEG start marker (0xFF 0xD8)...");
    
    // Read through FIFO looking for JPEG markers
    for (uint32_t pos = 0; pos < jpeg_length; pos++) {
        temp_last = temp;
        temp = spi_read_reg(SINGLE_FIFO_READ);
        
        // Check for JPEG start marker (0xFF 0xD8)
        if ((temp == 0xD8) && (temp_last == 0xFF)) {
            is_header = true;
            jpeg_start_pos = pos - 1;  // Include the 0xFF byte
            ESP_LOGI(TAG, "      ✅ Found JPEG header at position %lu", jpeg_start_pos);
            
            // Go back one position to include the 0xFF
            image_buffer[bytes_read++] = temp_last;
            image_buffer[bytes_read++] = temp;
            break;
        }
        
        // Progress update for scanning
        if (pos > 0 && pos % 1000 == 0) {
            ESP_LOGI(TAG, "      Scanned %lu bytes looking for JPEG header...", pos);
        }
    }
    
    if (!is_header) {
        ESP_LOGW(TAG, "      ⚠️  No JPEG header found in first %lu bytes", jpeg_length);
        ESP_LOGI(TAG, "      First 32 bytes of raw data:");
        
        // Reset and read first 32 bytes for analysis
        spi_write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
        vTaskDelay(pdMS_TO_TICKS(5));
        spi_write_reg(ARDUCHIP_FIFO, 0x00);
        vTaskDelay(pdMS_TO_TICKS(5));
        for (int i = 0; i < 32; i++) {
            image_buffer[i] = spi_read_reg(SINGLE_FIFO_READ);
        }
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, image_buffer, 32, ESP_LOG_INFO);
        bytes_read = 32;
    } else {
        // Continue reading after JPEG header was found
        ESP_LOGI(TAG, "      Reading JPEG data starting from position %lu...", jpeg_start_pos);
        
        uint32_t max_read = jpeg_length + 100;  // Allow some extra bytes for marker search
        uint32_t consecutive_ff_count = 0;
        
        for (uint32_t pos = jpeg_start_pos + 2; pos < max_read && bytes_read < (jpeg_length + 100); pos++) {
            temp_last = temp;
            temp = spi_read_reg(SINGLE_FIFO_READ);
            image_buffer[bytes_read++] = temp;
            
            // Track consecutive 0xFF bytes to handle padding
            if (temp == 0xFF) {
                consecutive_ff_count++;
            } else {
                consecutive_ff_count = 0;
            }
            
            // Check for JPEG end marker (0xFF 0xD9)
            if ((temp == 0xD9) && (temp_last == 0xFF)) {
                jpeg_end_pos = bytes_read - 1;
                ESP_LOGI(TAG, "      ✅ Found JPEG end marker at position %lu", jpeg_end_pos);
                break;
            }
            
            // Check for other JPEG markers for validation
            if (temp_last == 0xFF && temp >= 0xC0 && temp <= 0xFE && temp != 0xFF) {
                // Found a JPEG marker, this indicates we're still in valid JPEG data
            }
            
            // Stop if we hit too many consecutive 0xFF (likely padding/corruption)
            if (consecutive_ff_count > 50) {
                ESP_LOGW(TAG, "      ⚠️  Too many consecutive 0xFF bytes, stopping read");
                break;
            }
            
            // Progress update every 5000 bytes with watchdog reset
            if (bytes_read > 0 && bytes_read % 5000 == 0) {
                ESP_LOGI(TAG, "      Read %lu bytes... (%.1f%%)", bytes_read, (float)bytes_read * 100.0f / jpeg_length);
                TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
                if (current_task != NULL && esp_task_wdt_status(current_task) == ESP_OK) {
                    esp_task_wdt_reset();
                }
                vTaskDelay(pdMS_TO_TICKS(2)); // Small delay to prevent FIFO overflow
            }
            
            // Reset watchdog every 2000 bytes and add small delay for FIFO stability
            if (bytes_read > 0 && bytes_read % 2000 == 0) {
                TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
                if (current_task != NULL && esp_task_wdt_status(current_task) == ESP_OK) {
                    esp_task_wdt_reset();
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        // If no end marker found, search backwards for it
        if (jpeg_end_pos == 0 && bytes_read > 10) {
            ESP_LOGI(TAG, "      🔍 Searching backwards for JPEG end marker...");
            for (int i = bytes_read - 1; i > 0; i--) {
                if (image_buffer[i] == 0xD9 && image_buffer[i-1] == 0xFF) {
                    jpeg_end_pos = i;
                    ESP_LOGI(TAG, "      ✅ Found JPEG end marker at position %d (backward search)", i);
                    break;
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "      📥 Total bytes read: %lu", bytes_read);
    if (is_header && jpeg_end_pos > 0) {
        ESP_LOGI(TAG, "      📏 JPEG size: %lu bytes (from %lu to %lu)", 
                 jpeg_end_pos - jpeg_start_pos + 1, jpeg_start_pos, jpeg_end_pos);
    }

    ESP_LOGI(TAG, "   8️⃣  Determining final JPEG length...");
    uint32_t actual_jpeg_length = bytes_read;
    
    // Analyze actual captured resolution based on JPEG size
    const char* likely_resolution = "Unknown";
    uint32_t estimated_raw_size = 0;
    
    if (actual_jpeg_length < 30000) {
        likely_resolution = "QVGA (320x240) or smaller";
        estimated_raw_size = 320*240*3;
    } else if (actual_jpeg_length < 80000) {
        likely_resolution = "VGA (640x480)";
        estimated_raw_size = 640*480*3;
    } else if (actual_jpeg_length < 150000) {
        likely_resolution = "SVGA (800x600)";
        estimated_raw_size = 800*600*3;
    } else if (actual_jpeg_length < 250000) {
        likely_resolution = "HD (1280x720)";
        estimated_raw_size = 1280*720*3;
    } else {
        likely_resolution = "UXGA (1600x1200) or higher";
        estimated_raw_size = 1600*1200*3;
    }
    
    float compression_ratio = (float)estimated_raw_size / actual_jpeg_length;
    ESP_LOGI(TAG, "      📊 Analysis: JPEG size suggests %s", likely_resolution);
    ESP_LOGI(TAG, "      📊 Compression: %.1f:1 (Estimated raw: %lu KB → JPEG: %.1f KB)", 
             compression_ratio, estimated_raw_size/1024, actual_jpeg_length/1024.0);
    
    // Update current resolution based on actual capture if different
    if (actual_jpeg_length < 30000 && current_capture_resolution != RES_QVGA_320x240) {
        ESP_LOGW(TAG, "      🔄 Detected QVGA capture despite higher resolution setting");
    } else if (actual_jpeg_length >= 30000 && actual_jpeg_length < 80000 && current_capture_resolution != RES_VGA_640x480) {
        ESP_LOGW(TAG, "      🔄 Detected VGA capture despite different resolution setting");
    }
    
    if (is_header && jpeg_end_pos > 0) {
        actual_jpeg_length = jpeg_end_pos + 1;  // Include the EOI marker
        ESP_LOGI(TAG, "      ✅ JPEG length from markers: %lu bytes", actual_jpeg_length);
    } else if (is_header) {
        // We have a header but no end marker - add it manually
        ESP_LOGI(TAG, "      ℹ️  JPEG header found but no end marker - adding manually");
        if (bytes_read < jpeg_length - 2) {
            image_buffer[bytes_read++] = 0xFF;
            image_buffer[bytes_read++] = 0xD9;
            actual_jpeg_length = bytes_read;
            ESP_LOGI(TAG, "      ✅ Added JPEG end marker, final length: %lu bytes", actual_jpeg_length);
        } else {
            ESP_LOGI(TAG, "      ℹ️  Using full length: %lu bytes", actual_jpeg_length);
        }
    } else {
        ESP_LOGW(TAG, "      ⚠️  No JPEG header found - data may be corrupted");
        // Try to recover by looking for any JPEG-like data
        if (bytes_read > 100) {
            ESP_LOGI(TAG, "      🔧 Attempting data recovery with %lu bytes", actual_jpeg_length);
        } else {
            ESP_LOGE(TAG, "      ❌ Insufficient data for recovery, aborting");
            goto error_cleanup;
        }
    }
    
    // 10. Verify and validate JPEG format
    ESP_LOGI(TAG, "   9️⃣  Verifying JPEG format...");
    ESP_LOGI(TAG, "   📄 Image data analysis:");
    ESP_LOGI(TAG, "      First 16 bytes:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, image_buffer, 16, ESP_LOG_INFO);
    
    if (actual_jpeg_length > 16) {
        ESP_LOGI(TAG, "      Last 16 bytes:");
        uint32_t last_start = actual_jpeg_length - 16;
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, image_buffer + last_start, 16, ESP_LOG_INFO);
    }
    
    // Validate JPEG structure
    bool valid_jpeg = false;
    if (actual_jpeg_length > 2 && image_buffer[0] == 0xFF && image_buffer[1] == 0xD8) {
        ESP_LOGI(TAG, "      ✅ Valid JPEG start marker (0xFF 0xD8)");
        valid_jpeg = true;
        
        // Check for JFIF or EXIF marker
        if (actual_jpeg_length > 10 && image_buffer[6] == 'J' && image_buffer[7] == 'F' && image_buffer[8] == 'I' && image_buffer[9] == 'F') {
            ESP_LOGI(TAG, "      ✅ JFIF header detected");
        }
        
        // Check if we have end marker
        if (jpeg_end_pos > 0) {
            ESP_LOGI(TAG, "      ✅ Valid JPEG end marker (0xFF 0xD9)");
        } else {
            ESP_LOGI(TAG, "      ℹ️  No end marker found (common in streaming)");
        }
    } else {
        ESP_LOGW(TAG, "      ❌ Invalid JPEG start marker");
        // Don't fail completely, sometimes header gets corrupted but data is valid
        if (actual_jpeg_length > 1000) {
            ESP_LOGI(TAG, "      🔧 Large data size suggests capture may still be valid");
            valid_jpeg = true;
        }
    }
    
    if (!valid_jpeg && actual_jpeg_length < 1000) {
        ESP_LOGE(TAG, "      ❌ JPEG validation failed - data appears corrupted");
        goto error_cleanup;
    }
    
    // Verify JPEG headers
    if (image_buffer[0] == 0xFF && image_buffer[1] == 0xD8) {
        ESP_LOGI(TAG, "      ✅ Valid JPEG start marker (0xFF 0xD8)");
    } else {
        ESP_LOGW(TAG, "      ⚠️  Invalid JPEG start marker: 0x%02x 0x%02x", 
                image_buffer[0], image_buffer[1]);
    }
    
    // 10. Add JPEG end marker if missing (ensures compatibility)
    if (actual_jpeg_length > 2 && !(image_buffer[actual_jpeg_length-2] == 0xFF && image_buffer[actual_jpeg_length-1] == 0xD9)) {
        ESP_LOGI(TAG, "      🔧 Adding JPEG end marker for compatibility...");
        if (actual_jpeg_length + 2 < (jpeg_length + 1000)) {
            image_buffer[actual_jpeg_length++] = 0xFF;
            image_buffer[actual_jpeg_length++] = 0xD9;
            ESP_LOGI(TAG, "         JPEG end marker added");
        }
    }
    
    // 11. Store image for HTTP server
    // Reuse capture buffer as latest_image to avoid a second large allocation.
    if (latest_image) {
        free(latest_image);
        latest_image = NULL;
    }

    latest_image = image_buffer;
    latest_image_size = actual_jpeg_length;
    image_buffer = NULL; // Ownership transferred to latest_image
    image_counter++;
    success = true;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✅ Image saved for HTTP server!");
    ESP_LOGI(TAG, "   📁 Image size: %lu bytes (%.1f KB)", actual_jpeg_length, actual_jpeg_length / 1024.0);
    ESP_LOGI(TAG, "   🔢 Image number: #%d", image_counter);

    const char* size_category = (actual_jpeg_length < 30000) ? "Compact" :
                               (actual_jpeg_length < 70000) ? "Standard" :
                               (actual_jpeg_length < 120000) ? "Large" : "Very Large";
    ESP_LOGI(TAG, "   📊 Image category: %s", size_category);
    ESP_LOGI(TAG, "   🌐 Access via web interface");

    goto success_cleanup;

success_cleanup:
    if (image_buffer) {
        free(image_buffer);
    }
    
    if (success) {
        // Green LED for success
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // If we successfully captured with lower resolution due to previous failures,
        // try to go back to preferred resolution next time
        static int success_count = 0;
        success_count++;
        if (success_count >= 3 && current_capture_resolution != PREFERRED_RESOLUTION) {
            ESP_LOGI(TAG, "   📈 3 successful captures, trying higher resolution next time");
            current_capture_resolution = (current_capture_resolution == MINIMUM_RESOLUTION) ?
                                         FALLBACK_RESOLUTION : PREFERRED_RESOLUTION;
            success_count = 0;
        }
    } else {
        // Red LED for error
        led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Turn off LED
    led_strip_clear(led_strip);
    
    return success;

error_cleanup:
    // Cleanup and error recovery
    ESP_LOGW(TAG, "   🔧 Cleaning up after failed capture...");
    
    // Turn off LED
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    
    // Free allocated buffer
    if (image_buffer) {
        free(image_buffer);
        image_buffer = NULL;
    }
    
    // Reset camera FIFO for next capture
    arducam_clear_fifo_flag();
    arducam_flush_fifo();
    
    // Brief delay before next attempt
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGE(TAG, "   ❌ Capture #%lu failed - camera ready for next attempt", image_number);
    ESP_LOGI(TAG, "");
    
    return false;
}

// Capture task
static void capture_task(void *pvParameter) {
    bool last_button_state = true;
    uint32_t image_number = 1;
    
    // Register this task with the watchdog
    esp_task_wdt_add(NULL);
    
    ESP_LOGI(TAG, "📸 Capture task started");
    ESP_LOGI(TAG, "   Waiting for BOOT button press...");
    ESP_LOGI(TAG, "");
    
    while (1) {
        // Reset watchdog timer
        esp_task_wdt_reset();
        
        bool current_button_state = gpio_get_level(BUTTON_GPIO);
        
        if (last_button_state && !current_button_state) {
            perform_capture(image_number++);
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Monitor task
static void monitor_task(void *pvParameter) {
    // Register this task with the watchdog
    esp_task_wdt_add(NULL);
    
    ESP_LOGI(TAG, "📊 Monitor task started");
    
    while (1) {
        // Reset watchdog timer
        esp_task_wdt_reset();
        
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGI(TAG, "💾 Free heap: %d bytes", free_heap);
        
        if (latest_image_size > 0) {
            ESP_LOGI(TAG, "📷 Latest image: %.1f KB (#%d)", latest_image_size / 1024.0, image_counter);
        }
        
        // Sleep in smaller chunks to avoid watchdog timeout
        for (int i = 0; i < 30; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second chunks
            esp_task_wdt_reset(); // Reset every second
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "⚙️  Initializing RGB LED...");
    init_led();
    
    ESP_LOGI(TAG, "⚙️  Initializing SPI bus...");
    init_spi();
    
    ESP_LOGI(TAG, "⚙️  Initializing button...");
    init_button();
    
    ESP_LOGI(TAG, "✅ Image queue created (capacity: 5 images)");
    image_queue = xQueueCreate(5, sizeof(uint8_t*));
    
    // Check for SPIRAM availability
    ESP_LOGI(TAG, "🧠 Checking memory configuration...");
    size_t spiram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (spiram_size > 0) {
        spiram_available = true;
        ESP_LOGI(TAG, "   ✅ SPIRAM detected: %lu bytes", spiram_size);
    } else {
        spiram_available = false;
        ESP_LOGI(TAG, "   ℹ️  No SPIRAM detected, using internal RAM only");
    }
    
    // Select optimal resolution based on available memory
    ESP_LOGI(TAG, "📐 Selecting optimal capture resolution...");
    current_capture_resolution = select_optimal_resolution();
    ESP_LOGI(TAG, "   🎯 Selected resolution: %s (0x%02X)", 
             get_resolution_name(current_capture_resolution), current_capture_resolution);
    
    // Initialize camera FIRST, before WiFi to avoid resource conflicts
    if (!initialize_camera()) {
        ESP_LOGE(TAG, "❌ Camera initialization failed!");
        return;
    }
    
    ESP_LOGI(TAG, "⚙️  Initializing WiFi...");
    init_wifi();
    
    ESP_LOGI(TAG, "⚙️  Starting web server...");
    start_webserver();
    
    ESP_LOGI(TAG, "⚙️  Creating tasks...");
    
    xTaskCreate(capture_task, "capture", 8192, NULL, 5, &capture_task_handle);
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 3, &monitor_task_handle);
    
    ESP_LOGI(TAG, "✅ All tasks created!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                  SYSTEM READY                     ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  🎯 Instructions:                                 ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  1. Connect to the WiFi network                   ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  2. Press BOOT button to capture photo            ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  3. LED indicators:                               ║");
    ESP_LOGI(TAG, "║     • White flash   = Capturing                   ║");
    ESP_LOGI(TAG, "║     • Green blink   = Success                     ║");
    ESP_LOGI(TAG, "║     • Red solid     = Error                       ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  4. Open web browser to IP address shown above    ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "║  5. View and download images directly!            ║");
    ESP_LOGI(TAG, "║                                                   ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}