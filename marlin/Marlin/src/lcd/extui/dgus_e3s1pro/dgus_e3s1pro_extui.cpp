/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * lcd/extui/dgus_e3s1pro/dgus_e3s1pro_extui.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DGUS_LCD_UI_E3S1PRO)

#include "../ui_api.h"
#include "DGUSScreenHandler.h"

#define DEBUG_OUT 1
#include "../../../core/debug_out.h"

namespace ExtUI {

  void onStartup() {
    screen.init();
  }

  void onIdle() {
    static bool processing = false;

    // Prevent recursion
    if (!processing) {
      processing = true;
      screen.loop();
      processing = false;
    }
  }

  void onPrinterKilled(FSTR_P const error, FSTR_P const component) {
    screen.printerKilled(error, component);
  }

  void onMediaMounted() { TERN_(HAS_MEDIA, screen.sdCardInserted()); }
  void onMediaError()   { TERN_(HAS_MEDIA, screen.sdCardError()); }
  void onMediaRemoved() { TERN_(HAS_MEDIA, screen.sdCardRemoved()); }

  void onHeatingError(const heater_id_t header_id) {}
  void onMinTempError(const heater_id_t header_id) {}
  void onMaxTempError(const heater_id_t header_id) {}

  void onPlayTone(const uint16_t frequency, const uint16_t duration/*=0*/) {
    screen.playTone(frequency, duration);
  }

  void onPrintTimerStarted() {
    screen.printTimerStarted();
  }

  void onPrintTimerPaused() {
    screen.printTimerPaused();
  }

  void onPrintTimerStopped() {
    screen.printTimerStopped();
  }

  void onFilamentRunout(const extruder_t extruder) {
    screen.filamentRunout(extruder);
  }

  void onUserConfirmRequired(const char * const msg) {
    screen.userConfirmRequired(msg);
  }

  // For fancy LCDs include an icon ID, message, and translated button title
  void onUserConfirmRequired(const int, const char * const cstr, FSTR_P const) {
    onUserConfirmRequired(cstr);
  }
  void onUserConfirmRequired(const int, FSTR_P const fstr, FSTR_P const) {
    onUserConfirmRequired(fstr);
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void onPauseMode(
      const PauseMessage message,
      const PauseMode mode/*=PAUSE_MODE_SAME*/,
      const uint8_t extruder/*=active_extruder*/
    ) {
      stdOnPauseMode(message, mode, extruder);
    }
  #endif

  void onStatusChanged(const char * const msg) {
    screen.setStatusMessage(msg);
  }

  void onHomingStart() {}

  void onHomingDone() {
    screen.homingDone();
  }

  void onPrintDone() {}

  void onFactoryReset() {
    screen.settingsReset();
  }

  void onStoreSettings(char *buff) {
    screen.storeSettings(buff);
  }

  void onLoadSettings(const char *buff) {
    screen.loadSettings(buff);
  }

  void onPostprocessSettings() {}

  void onSettingsStored(const bool success) {
    screen.configurationStoreWritten(success);
  }

  void onSettingsLoaded(const bool success) {
    screen.configurationStoreRead(success);
  }

  #if HAS_LEVELING
    void onLevelingStart() { screen.levelingStart(); }
    void onLevelingDone() { screen.levelingEnd(); }
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp() { return LEVELING_BED_TEMP; }
    #endif
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
      screen.meshUpdate(xpos, ypos);
    }
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const probe_state_t state) { }
  #endif

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    void onSetMinExtrusionTemp(const celsius_t) {}
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void onSetPowerLoss(const bool onoff) {
      // Called when power-loss is enabled/disabled
    }
    void onPowerLoss() {
      // Called when power-loss state is detected
    }
    void onPowerLossResume() {
      // Called on resume from power-loss
      screen.powerLossResume();
    }
  #endif

  #if HAS_PID_HEATING
    void onPIDTuning(const pidresult_t rst) {
      // Called for temperature PID tuning result
      screen.pidTuning(rst);
    }
    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
      // Called by M303 to update the UI
    }
  #endif

  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst) {
      // Called for temperature PID tuning result
    }
  #endif

  #if ENABLED(PLATFORM_M997_SUPPORT)
    void onFirmwareFlash() {}
  #endif

  void onSteppersDisabled() {
    screen.steppersStatusChanged(false);
  }

  void onSteppersEnabled() {
    screen.steppersStatusChanged(true);
  }

  void onAxisDisabled(const axis_t) {}
  void onAxisEnabled(const axis_t) {}
}

#endif // DGUS_LCD_UI_RELOADED
