# ESP32S3-ePaper-WaveShare213v4

> Minimal ESP-IDF + LVGL driver demo for Waveshare 2.13" V4 E-Paper (SSD1680) display.

This project demonstrates how to integrate the **LVGL graphics library** with a **Waveshare 2.13" e-paper display (SSD1680 controller)** using **ESP-IDF** on an **ESP32-S3 DevKitC-1**.  
It handles full SPI communication, monochrome buffer rotation, LVGL framebuffer flushing, and custom font rendering.

---

## 🖥️ Features

- 🧠 **SSD1680 Register-Level Driver**  
  Full initialization and control sequence implemented from the official datasheet.

- 🖼️ **LVGL Integration**  
  Converts RGB565 buffers from LVGL into 1-bit monochrome frames with rotation and mirror control.

- 🖋️ **Custom Font Support**  
  Includes `Ink Free` TrueType font converted with `lv_font_conv`.

- 🔁 **Full E-Paper Refresh Handling**  
  Proper busy-pin synchronization and master activation cycle for clean screen updates.

- ⚡ **Optimized SPI Transfer**  
  Chunked DMA-safe transfers for large frame buffers (up to 4 KB per transaction).

---

## 🧩 Hardware Setup

| Signal | ESP32-S3 GPIO | Description        |
|--------|----------------|--------------------|
| `EPD_DC` | 8  | Data/Command select |
| `EPD_RST` | 9  | Hardware reset pin  |
| `EPD_CS` | 10 | SPI chip select     |
| `EPD_BUSY` | 13 | Busy signal (active LOW) |
| `EPD_MOSI` | 11 | SPI MOSI           |
| `EPD_SCLK` | 12 | SPI Clock          |

> ⚠️ Tested with **Waveshare 2.13" V4** (SSD1680).  
> Adjust `EPD_WIDTH`, `EPD_HEIGHT`, and offsets in `main.c` if using another model.

---

## 🧱 Directory Structure


