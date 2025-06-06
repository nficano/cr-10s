/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../../platforms.h"

#ifdef HAL_STM32

#include "../../../inc/MarlinConfig.h"

#if ENABLED(SRAM_EEPROM_EMULATION)

#include "../../shared/eeprom_if.h"
#include "../../shared/eeprom_api.h"

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE 0x1000 // 4KB
#endif
size_t PersistentStore::capacity()    { return MARLIN_EEPROM_SIZE - eeprom_exclude_size; }

bool PersistentStore::access_start()  { return true; }
bool PersistentStore::access_finish() { return true; }

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  while (size--) {
    uint8_t v = *value;

    // Save to Backup SRAM
    *(__IO uint8_t *)(BKPSRAM_BASE + (uint8_t * const)pos) = v;

    crc16(crc, &v, 1);
    pos++;
    value++;
  };

  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    // Read from either external EEPROM, program flash or Backup SRAM
    const uint8_t c = ( *(__IO uint8_t *)(BKPSRAM_BASE + ((uint8_t*)pos)) );
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

#endif // SRAM_EEPROM_EMULATION
#endif // HAL_STM32
