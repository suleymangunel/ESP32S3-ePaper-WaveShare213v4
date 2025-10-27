/**
 * @file ssd1680_regs.h
 * @brief SSD1680 E-Paper Display Controller Register Definitions
 * 
 * Waveshare 2.13" V4 e-Paper Display Controller
 * Datasheet: https://www.good-display.com/companyfile/32.html
 */

#ifndef SSD1680_REGS_H
#define SSD1680_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * SSD1680 Command Register Definitions
 * ======================================================================== */

/** @defgroup Basic_Commands Basic Commands */
/** @{ */
#define SSD1680_CMD_DRIVER_OUTPUT_CONTROL       0x01  /**< Driver Output Control */
#define SSD1680_CMD_GATE_DRIVING_VOLTAGE        0x03  /**< Gate Driving Voltage Control */
#define SSD1680_CMD_SOURCE_DRIVING_VOLTAGE      0x04  /**< Source Driving Voltage Control */
#define SSD1680_CMD_DEEP_SLEEP_MODE             0x10  /**< Deep Sleep Mode */
#define SSD1680_CMD_DATA_ENTRY_MODE             0x11  /**< Data Entry Mode Setting */
#define SSD1680_CMD_SW_RESET                    0x12  /**< Software Reset */
#define SSD1680_CMD_TEMP_SENSOR_CONTROL         0x18  /**< Temperature Sensor Control */
#define SSD1680_CMD_MASTER_ACTIVATION           0x20  /**< Master Activation */
#define SSD1680_CMD_DISPLAY_UPDATE_CONTROL_1    0x21  /**< Display Update Control 1 */
#define SSD1680_CMD_DISPLAY_UPDATE_CONTROL_2    0x22  /**< Display Update Control 2 */
/** @} */

/** @defgroup RAM_Commands RAM Commands */
/** @{ */
#define SSD1680_CMD_WRITE_RAM_BW                0x24  /**< Write RAM (Black/White) */
#define SSD1680_CMD_WRITE_RAM_RED               0x26  /**< Write RAM (Red) */
#define SSD1680_CMD_READ_RAM                    0x27  /**< Read RAM */
/** @} */

/** @defgroup VCOM_Commands VCOM Commands */
/** @{ */
#define SSD1680_CMD_VCOM_SENSE                  0x28  /**< VCOM Sense */
#define SSD1680_CMD_VCOM_SENSE_DURATION         0x29  /**< VCOM Sense Duration */
#define SSD1680_CMD_WRITE_VCOM_REGISTER         0x2C  /**< Write VCOM Register */
/** @} */

/** @defgroup LUT_Commands LUT Commands */
/** @{ */
#define SSD1680_CMD_WRITE_LUT_REGISTER          0x32  /**< Write LUT Register */
/** @} */

/** @defgroup Border_Commands Border Commands */
/** @{ */
#define SSD1680_CMD_BORDER_WAVEFORM_CONTROL     0x3C  /**< Border Waveform Control */
/** @} */

/** @defgroup Window_Commands Window Setting Commands */
/** @{ */
#define SSD1680_CMD_SET_RAM_X_START_END         0x44  /**< Set RAM X Address Start/End Position */
#define SSD1680_CMD_SET_RAM_Y_START_END         0x45  /**< Set RAM Y Address Start/End Position */
/** @} */

/** @defgroup Counter_Commands Counter Commands */
/** @{ */
#define SSD1680_CMD_SET_RAM_X_COUNTER           0x4E  /**< Set RAM X Address Counter */
#define SSD1680_CMD_SET_RAM_Y_COUNTER           0x4F  /**< Set RAM Y Address Counter */
/** @} */

/** @defgroup Analog_Commands Analog Block Control Commands */
/** @{ */
#define SSD1680_CMD_BOOSTER_SOFT_START          0x0C  /**< Booster Soft Start Control */
#define SSD1680_CMD_GATE_SCAN_START             0x0F  /**< Gate Scan Start Position */
#define SSD1680_CMD_DUMMY_LINE_PERIOD           0x3A  /**< Set Dummy Line Period */
#define SSD1680_CMD_GATE_LINE_WIDTH             0x3B  /**< Set Gate Line Width */
/** @} */

/* ========================================================================
 * Data Entry Mode Settings (0x11)
 * ======================================================================== */

/** @defgroup Data_Entry_Mode Data Entry Mode Values */
/** @{ */
#define SSD1680_DATA_ENTRY_XDEC_YDEC            0x00  /**< X decrement, Y decrement */
#define SSD1680_DATA_ENTRY_XDEC_YINC            0x01  /**< X decrement, Y increment */
#define SSD1680_DATA_ENTRY_XINC_YDEC            0x02  /**< X increment, Y decrement */
#define SSD1680_DATA_ENTRY_XINC_YINC            0x03  /**< X increment, Y increment (Normal) */
#define SSD1680_DATA_ENTRY_XINC_YINC_YDIR       0x07  /**< X increment, Y increment, Y direction first (90° rotation) */
/** @} */

/* ========================================================================
 * Display Update Control Settings (0x22)
 * ======================================================================== */

/** @defgroup Display_Update Display Update Control Values */
/** @{ */
#define SSD1680_DISPLAY_UPDATE_PARTIAL          0xC7  /**< Partial update */
#define SSD1680_DISPLAY_UPDATE_FULL             0xF7  /**< Full update */
/** @} */

/* ========================================================================
 * Deep Sleep Mode Settings (0x10)
 * ======================================================================== */

/** @defgroup Deep_Sleep Deep Sleep Mode Values */
/** @{ */
#define SSD1680_DEEP_SLEEP_MODE_1               0x01  /**< Deep Sleep Mode 1 (retain RAM) */
#define SSD1680_DEEP_SLEEP_MODE_2               0x03  /**< Deep Sleep Mode 2 (RAM cleared) */
/** @} */

/* ========================================================================
 * Booster Soft Start Default Values
 * ======================================================================== */

/** @defgroup Booster_Defaults Booster Soft Start Defaults */
/** @{ */
#define SSD1680_BOOSTER_PHASE1_DEFAULT          0xD7  /**< Phase 1 period (default) */
#define SSD1680_BOOSTER_PHASE2_DEFAULT          0xD6  /**< Phase 2 period (default) */
#define SSD1680_BOOSTER_PHASE3_DEFAULT          0x9D  /**< Phase 3 period (default) */
/** @} */

/* ========================================================================
 * VCOM Default Values
 * ======================================================================== */

/** @defgroup VCOM_Defaults VCOM Voltage Defaults */
/** @{ */
#define SSD1680_VCOM_VOLTAGE_DEFAULT            0xA8  /**< VCOM Voltage (default: -1.5V) */
/** @} */

/* ========================================================================
 * Dummy Line & Gate Line Width Defaults
 * ======================================================================== */

/** @defgroup Timing_Defaults Timing Defaults */
/** @{ */
#define SSD1680_DUMMY_LINE_PERIOD_DEFAULT       0x1A  /**< Dummy line period (default) */
#define SSD1680_GATE_LINE_WIDTH_DEFAULT         0x08  /**< Gate line width (default) */
/** @} */

/* ========================================================================
 * Border Waveform Defaults
 * ======================================================================== */

/** @defgroup Border_Defaults Border Waveform Defaults */
/** @{ */
#define SSD1680_BORDER_WAVEFORM_DEFAULT         0x05  /**< Border waveform (default) */
/** @} */

/* ========================================================================
 * Display Update Control 1 Defaults
 * ======================================================================== */

/** @defgroup Update_Control_1 Display Update Control 1 Values */
/** @{ */
#define SSD1680_UPDATE_CTRL1_DEFAULT_1          0x00  /**< Update control 1 - byte 1 */
#define SSD1680_UPDATE_CTRL1_DEFAULT_2          0x80  /**< Update control 1 - byte 2 */
/** @} */

/* ========================================================================
 * Temperature Sensor Defaults
 * ======================================================================== */

/** @defgroup Temp_Sensor_Defaults Temperature Sensor Defaults */
/** @{ */
#define SSD1680_TEMP_SENSOR_INTERNAL            0x80  /**< Use internal temperature sensor */
#define SSD1680_TEMP_SENSOR_EXTERNAL            0x48  /**< Use external temperature sensor */
/** @} */

/* ========================================================================
 * Driver Output Control Bits
 * ======================================================================== */

/** @defgroup Driver_Output_Bits Driver Output Control Bits */
/** @{ */
#define SSD1680_GATE_SCAN_UP                    0x00  /**< Gate scan up */
#define SSD1680_GATE_SCAN_DOWN                  0x01  /**< Gate scan down (bit 2) */
#define SSD1680_INTERLACE_ENABLE                0x02  /**< Enable interlace (bit 1) */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SSD1680_REGS_H */
