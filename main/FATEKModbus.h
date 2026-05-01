/*
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 3.
 */

#ifndef FATEK_MODBUS_H
#define FATEK_MODBUS_H

#include <Arduino.h>
#include "ModbusClientRTU.h"
#include "RTUutils.h"

using namespace Modbus;

// Structure that stores the read values
struct RegisterValues {
  // Discrete M Relays
  bool M71, M72, M73, M74, M75, M76, M77, M78, M79;  // M71-M79 (9 coils)
  bool M94, M95, M96, M97, M98, M99;                 // M94-M99 (6 coils)
  bool M120;                                           // M120 (1 coil)
  bool M320, M321, M322;                              // M320-M322 (3 coils)
  bool M_valid[19];  // Validation for all coils (9+6+1+3=19)
  
  // Data Registers (D)
  uint16_t D0;                                        // D0
  uint16_t D10;                                       // D10
  uint16_t D47, D48, D49;                            // D47-D49 (3 registers)
  uint16_t D57, D58, D59;                            // D57-D59 (3 registers)
  bool D_valid[8];  // Validation for all D registers (1+1+3+3=8)
  
  // Holding Registers (R)
  uint16_t R[180];  // R10-R99 (90 registers) + R110-R199 (90 registers) = 180 registers (without R100-R109)
  bool R_valid[180];
};

// Index to track what is read now
enum ReadState {
  READ_M71_M79,       // Read M71-M79 (9 coils, address 0x817)
  READ_M94_M99,       // Read M94-M99 (6 coils, address 0x82E)
  READ_M120,          // Read M120 (1 coil, address 0x848)
  READ_M320_M322,     // Read M320-M322 (3 coils, address 0x910)
  READ_D0,            // Read D0 (1 register, address 0x1770)
  READ_D10,           // Read D10 (1 register, address 0x177A)
  READ_D47_D49,       // Read D47-D49 (3 registers, address 0x179F)
  READ_D57_D59,       // Read D57-D59 (3 registers, address 0x17A9)
  READ_R10_R99,       // Read R10-R99 (90 registers, address 0xA)
  READ_R110_R199      // Read R110-R199 (90 registers, address 0x6E)
};

// Function declarations
void initFATEKModbus();
void loopFATEKModbus();
RegisterValues* getRegisterValues();  // Get register values
void displayRegisterValues();  // Print register values to Serial Monitor
void setSerialMonitorEnabled(bool enabled);  // Enable/disable Serial Monitor output
bool getSerialMonitorEnabled();              // Check Serial Monitor output state
bool isCommunicationActive();                // Check if PLC communication is active
uint8_t getCurrentSlaveID();                 // Get current slave ID

#endif // FATEK_MODBUS_H

