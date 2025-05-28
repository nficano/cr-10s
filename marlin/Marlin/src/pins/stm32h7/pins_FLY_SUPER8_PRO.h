/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Bootloader Download
 * https://mellow.klipper.cn/docs/ProductDoc/MainBoard/fly-super/fly-super8-pro/flash/bl
 */

#define ALLOW_STM32DUINO
#include "env_validate.h"

#define BOARD_INFO_NAME      "FLY_SUPER8_PRO"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

// Avoid conflict with fans and TIMER_TONE
#define TEMP_TIMER  3
#define STEP_TIMER  5

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  //#define SRAM_EEPROM_EMULATION
  //#define I2C_EEPROM
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across
  // the 128kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#elif ENABLED(I2C_EEPROM)
  #define MARLIN_EEPROM_SIZE              0x2000  // 8K
#endif

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                          PC6

//
// Limit Switches
//
#define X_MIN_PIN                           PG12
#define X_MAX_PIN                           PG11
#define Y_MIN_PIN                           PG10
#define Y_MAX_PIN                           PG9
#define Z_MIN_PIN                           PD7
#define Z_MAX_PIN                           PD6

#define IO6                                 PA8
#define IN7                                 PF8
#define HVIN                                PF3

//
// Z Probe (when not Z_MIN_PIN)
//
#define Z_MIN_PROBE_PIN                     PC3   // Z3_PIN

//
// Steppers
//

#define X_STEP_PIN                          PE2
#define X_DIR_PIN                           PC5
#define X_ENABLE_PIN                        PF11
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC4
#endif

#define Y_STEP_PIN                          PE3
#define Y_DIR_PIN                           PF13
#define Y_ENABLE_PIN                        PF14
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PF12
#endif

#define Z_STEP_PIN                          PE4
#define Z_DIR_PIN                           PG0
#define Z_ENABLE_PIN                        PG1
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PF15
#endif

#define E0_STEP_PIN                         PE14
#define E0_DIR_PIN                          PE8
#define E0_ENABLE_PIN                       PE9
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PE7
#endif

#define E1_STEP_PIN                         PE15
#define E1_DIR_PIN                          PE11
#define E1_ENABLE_PIN                       PF2
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PE10
#endif

#define E2_STEP_PIN                         PE1
#define E2_DIR_PIN                          PF0
#define E2_ENABLE_PIN                       PC15
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PF1
#endif

#define E3_STEP_PIN                         PE0
#define E3_DIR_PIN                          PG3
#define E3_ENABLE_PIN                       PG4
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PG2
#endif

#define E4_STEP_PIN                         PE6
#define E4_DIR_PIN                          PG6
#define E4_ENABLE_PIN                       PG7
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PG5
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PF4   // Analog Input
#define TEMP_1_PIN                          PF5   // Analog Input
#define TEMP_2_PIN                          PF9   // Analog Input
#define TEMP_3_PIN                          PF10  // Analog Input
#define TEMP_4_PIN                          PC0   // Analog Input
#define TEMP_BED_PIN                        PC1   // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB0
#define HEATER_1_PIN                        PB1
#define HEATER_2_PIN                        PC7
#define HEATER_3_PIN                        PF7
#define HEATER_4_PIN                        PF6
#define HEATER_BED_PIN                      PE5

#ifndef FAN0_PIN
  #define FAN0_PIN                          PA0
#endif
#define FAN1_PIN                            PA1
#define FAN2_PIN                            PA2
#define FAN3_PIN                            PA3
#define FAN4_PIN                            PA15
#define FAN5_PIN                            PB11
#define FAN8_PIN                            PB10
#define FAN9_PIN                            PD12
#define FAN6_PIN                            PD14
#define FAN7_PIN                            PD15

/**
 *        ------                ------
 *  PE12 | 1  2 | PE13    PA6  | 1  2 | PA5
 *  PB2  | 3  4 | PG8     PB7  | 3  4 | PA4
 *  PC14   5  6 | PC13    PB6    5  6 | PA7
 *  PG14 | 7  8 | PG13    PG15 | 7  8 | RESET
 *   GND | 9 10 | 5V       GND | 9 10 | --
 *        ------                ------
 *         EXP1                  EXP2
 */
#define EXP1_01_PIN                         PE12
#define EXP1_02_PIN                         PE13
#define EXP1_03_PIN                         PB2
#define EXP1_04_PIN                         PG8
#define EXP1_05_PIN                         PC14
#define EXP1_06_PIN                         PC13
#define EXP1_07_PIN                         PG14
#define EXP1_08_PIN                         PG13

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PB7
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PB6
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PG15
#define EXP2_08_PIN                         -1    // RESET

//
// Onboard SD support
//
//#define SD_CARD_DETECT_PIN                PC13

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              LCD
#endif

#if SD_CONNECTION_IS(ONBOARD)

  #define ONBOARD_SDIO                            // Use SDIO for onboard SD

#elif SD_CONNECTION_IS(LCD)

  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN

#endif

//
// Trinamic SPI
//
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB3
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB4
#endif
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB5
#endif

//
// Trinamic Software Serial
//

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PC4
  #define Y_SERIAL_TX_PIN                   PF12
  #define Z_SERIAL_TX_PIN                   PF15
  #define E0_SERIAL_TX_PIN                  PE7
  #define E1_SERIAL_TX_PIN                  PE10
  #define E2_SERIAL_TX_PIN                  PF1
  #define E3_SERIAL_TX_PIN                  PG2
  #define E4_SERIAL_TX_PIN                  PG5
#endif

//
// LCD / Controller
//

#if ENABLED(FYSETC_242_OLED_12864)

  #define BTN_EN1                    EXP1_01_PIN
  #define BTN_EN2                    EXP1_08_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define BEEPER_PIN                 EXP2_03_PIN

  #define LCD_PINS_DC                EXP1_06_PIN
  #define LCD_PINS_RS                EXP2_05_PIN  // LCD_RST
  #define DOGLCD_CS                  EXP1_04_PIN
  #define DOGLCD_MOSI                EXP1_05_PIN
  #define DOGLCD_SCK                 EXP1_03_PIN
  #define DOGLCD_A0                  LCD_PINS_DC
  #define FORCE_SOFT_SPI

  #define KILL_PIN                          -1    // NC
  #define BOARD_NEOPIXEL_PIN         EXP1_07_PIN

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_SDSS                 EXP2_04_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      // See https://wiki.fysetc.com/Mini12864_Panel
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #if ENABLED(FYSETC_GENERIC_12864_1_1)
        #define LCD_BACKLIGHT_PIN    EXP1_07_PIN
      #endif
      #define LCD_RESET_PIN          EXP1_05_PIN  // Must be high or open for LCD to operate normally.
      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_06_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_07_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_08_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_06_PIN
      #endif
    #endif

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif
    #endif

  #endif

#endif // HAS_WIRED_LCD

//
// Filament runout
//

#define FIL_RUNOUT_PIN                      PA8

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1                96
  #define BOARD_ST7920_DELAY_2                48
  #define BOARD_ST7920_DELAY_3               715
#endif
