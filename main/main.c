#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "fonts/ink_free_12.h"
#include "ssd1680_regs.h"  // E-Paper register definitions

// Pins
#define EPD_DC     8
#define EPD_RST    9
#define EPD_CS    10
#define EPD_BUSY  13
#define EPD_MOSI  11
#define EPD_SCLK  12

// Waveshare 2.13" V4 correct dimensions with offset
#define EPD_WIDTH        122  // Actual visible width
#define EPD_HEIGHT       250  // Actual visible height
#define EPD_WIDTH_BYTES  16   // Width in bytes (must be 16 for alignment)
#define X_OFFSET         0    // X offset in bytes (0 = no offset)
#define EPD_BUFFER_SIZE  (EPD_WIDTH_BYTES * EPD_HEIGHT)

static const char *TAG = "LVGL_WS213";
static spi_device_handle_t spi;
static bool g_lvgl_ready = false;

// SPI transfer callback
static void spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_DC, dc);
}

// Send command
static void epd_send_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0,  // DC = 0 for command
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

// Send data byte
static void epd_send_data(uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .user = (void*)1,  // DC = 1 for data
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

// Send data buffer
static void epd_send_data_buffer(const uint8_t *buffer, size_t len)
{
    size_t chunk_size = 4000;
    size_t offset = 0;
    
    while (offset < len) {
        size_t to_send = (len - offset > chunk_size) ? chunk_size : (len - offset);
        
        spi_transaction_t t = {
            .length = to_send * 8,
            .tx_buffer = buffer + offset,
            .user = (void*)1,  // DC = 1 for data
        };
        ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
        
        offset += to_send;
    }
}

// Wait for busy signal
static void epd_wait_busy(void)
{
    ESP_LOGD(TAG, "Waiting for display...");
    int timeout = 0;
    while(gpio_get_level(EPD_BUSY) == 0 && timeout < 500) {  // LOW = busy
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout++;
    }
    if (timeout >= 500) {
        ESP_LOGE(TAG, "Busy timeout!");
    } else {
        ESP_LOGD(TAG, "Ready (%dms)", timeout * 10);
    }
}

// Hardware reset
static void epd_reset(void)
{
    gpio_set_level(EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Initialize display with correct settings for Waveshare 2.13" V4
static void epd_init(void)
{
    ESP_LOGI(TAG, "Initializing Waveshare 2.13\" V4...");
    ESP_LOGI(TAG, "Width: %d, Height: %d, Bytes/row: %d", EPD_WIDTH, EPD_HEIGHT, EPD_WIDTH_BYTES);
    
    epd_reset();
    epd_wait_busy();
    
    // Software reset
    epd_send_cmd(SSD1680_CMD_SW_RESET);
    epd_wait_busy();
    
    // Driver output control - rotated coordinates
    epd_send_cmd(SSD1680_CMD_DRIVER_OUTPUT_CONTROL);
    epd_send_data((EPD_WIDTH - 1) & 0xFF);     // Gate drivers (rotated)
    epd_send_data(((EPD_WIDTH - 1) >> 8) & 0xFF);
    epd_send_data(SSD1680_GATE_SCAN_UP);  // GD=0, SM=0, TB=0
    
    // Booster soft start
    epd_send_cmd(SSD1680_CMD_BOOSTER_SOFT_START);
    epd_send_data(SSD1680_BOOSTER_PHASE1_DEFAULT);
    epd_send_data(SSD1680_BOOSTER_PHASE2_DEFAULT);
    epd_send_data(SSD1680_BOOSTER_PHASE3_DEFAULT);
    
    // VCOM Voltage
    epd_send_cmd(SSD1680_CMD_WRITE_VCOM_REGISTER);
    epd_send_data(SSD1680_VCOM_VOLTAGE_DEFAULT);
    
    // Set dummy line period
    epd_send_cmd(SSD1680_CMD_DUMMY_LINE_PERIOD);
    epd_send_data(SSD1680_DUMMY_LINE_PERIOD_DEFAULT);
    
    // Set Gate line width
    epd_send_cmd(SSD1680_CMD_GATE_LINE_WIDTH);
    epd_send_data(SSD1680_GATE_LINE_WIDTH_DEFAULT);
    
    // Data entry mode - ROTATED 90 degrees + Y mirrored
    epd_send_cmd(SSD1680_CMD_DATA_ENTRY_MODE);
    epd_send_data(SSD1680_DATA_ENTRY_XINC_YINC_YDIR);  // 90 degree rotation
    
    // Set RAM X address start/end position
    epd_send_cmd(SSD1680_CMD_SET_RAM_X_START_END);
    epd_send_data(X_OFFSET);  // RAM X start = offset
    epd_send_data(X_OFFSET + (EPD_WIDTH_BYTES - 1));  // RAM X end
    
    // Set RAM Y address start/end position - rotated
    epd_send_cmd(SSD1680_CMD_SET_RAM_Y_START_END);
    epd_send_data(0x00);  // Y start low
    epd_send_data(0x00);  // Y start high
    epd_send_data((EPD_WIDTH - 1) & 0xFF);  // Y end low (rotated)
    epd_send_data(((EPD_WIDTH - 1) >> 8) & 0xFF);  // Y end high
    
    // Border waveform
    epd_send_cmd(SSD1680_CMD_BORDER_WAVEFORM_CONTROL);
    epd_send_data(SSD1680_BORDER_WAVEFORM_DEFAULT);
    
    // Display update control
    epd_send_cmd(SSD1680_CMD_DISPLAY_UPDATE_CONTROL_1);
    epd_send_data(SSD1680_UPDATE_CTRL1_DEFAULT_1);
    epd_send_data(SSD1680_UPDATE_CTRL1_DEFAULT_2);
    
    // Temperature sensor
    epd_send_cmd(SSD1680_CMD_TEMP_SENSOR_CONTROL);
    epd_send_data(SSD1680_TEMP_SENSOR_INTERNAL);
    
    ESP_LOGI(TAG, "Init complete with X offset: %d", X_OFFSET);
}

// Set cursor position
static void epd_set_cursor(uint8_t x, uint16_t y)
{
    epd_send_cmd(SSD1680_CMD_SET_RAM_X_COUNTER);  // Set RAM X address counter
    epd_send_data(x + X_OFFSET);  // Add offset
    
    epd_send_cmd(SSD1680_CMD_SET_RAM_Y_COUNTER);  // Set RAM Y address counter
    epd_send_data(y & 0xFF);
    epd_send_data((y >> 8) & 0xFF);
}

// Display update
static void epd_display_frame(void)
{
    epd_send_cmd(SSD1680_CMD_DISPLAY_UPDATE_CONTROL_2);  // Display Update Control 2
    epd_send_data(SSD1680_DISPLAY_UPDATE_FULL);  // Full update
    epd_send_cmd(SSD1680_CMD_MASTER_ACTIVATION);  // Master Activation
    epd_wait_busy();
}

// Converts RGB565 buffer to monochrome and Rotate 90 degree clockwise % X Axis Mirror
static void rgb565_to_mono(const lv_color16_t *src, uint8_t *dst, int w, int h)
{
    memset(dst, 0xFF, EPD_BUFFER_SIZE);  // Start with white
    
    // Rotate 90 degrees clockwise during conversion
    // LVGL coordinate (x, y) -> EPD coordinate (y, HEIGHT-1-x)
    for (int lvgl_y = 0; lvgl_y < h && lvgl_y < EPD_WIDTH; lvgl_y++) {
        for (int lvgl_x = 0; lvgl_x < w && lvgl_x < EPD_HEIGHT; lvgl_x++) {
            lv_color16_t px = src[lvgl_y * w + lvgl_x];
            
            // RGB565 â†’ Grayscale
            uint8_t gray = ((px.red << 3) * 30 + (px.green << 2) * 59 + (px.blue << 3) * 11) / 100;
            
            // Rotate + Mirror X coordinates: (lvgl_x, lvgl_y) -> (WIDTH-1-lvgl_y, HEIGHT-1-lvgl_x)
            /*int epd_x = EPD_WIDTH - 1 - lvgl_y;  // X mirrored
            int epd_y = EPD_HEIGHT - 1 - lvgl_x;*/
			
			int epd_x = lvgl_y;              // X normal
			int epd_y = lvgl_x;              // Y mirror
            
            // Convert to buffer position
            int byte_x = epd_x / 8;
            int bit_x = 7 - (epd_x % 8);  // MSB first
            int byte_idx = epd_y * EPD_WIDTH_BYTES + byte_x;
            
            if (byte_idx < EPD_BUFFER_SIZE) {
                // Threshold: <128 = black
                if (gray < 128) {
                    dst[byte_idx] &= ~(1 << bit_x);  // Clear bit for black
                } else {
                    dst[byte_idx] |= (1 << bit_x);   // Set bit for white
                }
            }
        }
    }
}

// Flush callback
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    ESP_LOGI(TAG, ">>> FLUSH CALLED <<<");
    
    static uint8_t mono_buf[EPD_BUFFER_SIZE];
    
    rgb565_to_mono((lv_color16_t *)px_map, mono_buf, 250, 122);
    
    // Send to e-Paperr
    epd_set_cursor(0, 0);
    epd_send_cmd(SSD1680_CMD_WRITE_RAM_BW);  // Write RAM (Black/White)
    epd_send_data_buffer(mono_buf, EPD_BUFFER_SIZE);
    epd_display_frame();
    
    lv_display_flush_ready(disp);
    g_lvgl_ready = true;
    ESP_LOGI(TAG, ">>> FLUSH DONE <<<");
}

// Tick task
static void lv_tick_task(void *arg)
{
    while (1) {
        lv_tick_inc(10);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Clear screen and Turn off e-Paper
static void screen_off_task(void *arg)
{
    // Wait 10 seconds
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    ESP_LOGI(TAG, "10 seconds passed - clearing and turning off screen");
    
    // Convert screen to white
    uint8_t *white_buf = heap_caps_malloc(EPD_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (white_buf) {
        memset(white_buf, 0xFF, EPD_BUFFER_SIZE);  // 0xFF = White
        
        ESP_LOGI(TAG, "Clearing screen...");
        epd_set_cursor(0, 0);
        epd_send_cmd(SSD1680_CMD_WRITE_RAM_BW);  // Write RAM (Black/White)
        epd_send_data_buffer(white_buf, EPD_BUFFER_SIZE);
        epd_display_frame();
        ESP_LOGI(TAG, "Screen cleared");
        
        heap_caps_free(white_buf);
    }
    
    // Wait 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Deep sleep screen
    ESP_LOGI(TAG, "Turning off display");
    epd_send_cmd(SSD1680_CMD_DEEP_SLEEP_MODE);  // Deep sleep
    epd_send_data(SSD1680_DEEP_SLEEP_MODE_1);  // Mode 1: Retain RAM
    
    ESP_LOGI(TAG, "Display is now OFF - entering idle mode");
    
    // End task
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Waveshare 2.13\" V4 LVGL ===");
    ESP_LOGI(TAG, "Display: %dx%d, Buffer: %d bytes", EPD_WIDTH, EPD_HEIGHT, EPD_BUFFER_SIZE);
    
    esp_task_wdt_deinit();
    
    // GPIO init
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EPD_DC) | (1ULL << EPD_RST) | (1ULL << EPD_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << EPD_BUSY);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    gpio_set_level(EPD_CS, 1);
    gpio_set_level(EPD_RST, 1);
    
    // SPI init
    spi_bus_config_t buscfg = {
        .mosi_io_num = EPD_MOSI,
        .sclk_io_num = EPD_SCLK,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,  // 4MHz
        .mode = 0,
        .spics_io_num = EPD_CS,
        .queue_size = 7,
        .pre_cb = spi_pre_transfer_callback,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
    
    // Initialize display
    epd_init();
    
    // LVGL
    lv_init();
    xTaskCreate(lv_tick_task, "lv_tick", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "LVGL init");

    // RGB565 buffer - for user-facing rotated dimensions
    size_t buf_size = 250 * 122 * 2;  // Width * Height * 2 bytes per pixel
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) {
        buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    assert(buf1);
    ESP_LOGI(TAG, "RGB565 buffer: %d bytes (250x122)", buf_size);

    // LVGL display dimensions - these are the user-facing dimensions after rotation
    // Physical display is 122x250, but after 90 degree rotation user sees 250x122
    lv_display_t *disp = lv_display_create(250, 122);  // Width x Height as user sees it
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(disp, buf1, NULL, buf_size, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(disp, my_disp_flush);
    ESP_LOGI(TAG, "Display OK - User sees: 250x122 (rotated from physical 122x250)");

    // UI - Beyaz arka plan
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    // Label
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Süleyman GÜNEL 1976");
    lv_obj_set_style_text_color(label, lv_color_black(), 0);
	lv_obj_set_style_text_font(label, &ink_free_12 , 0);
	lv_obj_set_pos(label, 5, 5);
    //lv_obj_center(label);
    ESP_LOGI(TAG, "UI created");
	
	// Label2
    lv_obj_t *label2 = lv_label_create(scr);
    lv_label_set_text(label2, "Günel");
    lv_obj_set_style_text_color(label2, lv_color_black(), 0);
	lv_obj_set_style_text_font(label2, &ink_free_12 , 0);
	lv_obj_set_pos(label2, 150, 100);
    //lv_obj_center(label);
    ESP_LOGI(TAG, "UI created");

    // First render
    ESP_LOGI(TAG, "Starting render...");
    for (int i = 0; i < 30; i++) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(50));
        
        if (g_lvgl_ready) {
            ESP_LOGI(TAG, "Rendered at iteration %d", i);
            break;
        }
    }
    
    if (g_lvgl_ready) {
        ESP_LOGI(TAG, "Done!");
    } else {
        ESP_LOGE(TAG, "Timeout!");
    }

    // Start e-Paper stand by task after 10 seconds
    xTaskCreate(screen_off_task, "screen_off", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Screen off task started (will trigger in 10 seconds)");

    // Main loop
    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
