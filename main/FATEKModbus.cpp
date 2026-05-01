/*
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 3.
 */

#include "FATEKModbus.h"

// ESP32 pin definitions (can be overridden in main.ino)
#ifndef MODBUS_RX_PIN
#define MODBUS_RX_PIN 16
#endif
#ifndef MODBUS_TX_PIN
#define MODBUS_TX_PIN 17
#endif
#ifndef MODBUS_RE_DE_PIN
#define MODBUS_RE_DE_PIN 4
#endif
#ifndef MODBUS_BAUDRATE
#define MODBUS_BAUDRATE 115200
#endif
#ifndef MODBUS_SLAVE_ID
#define MODBUS_SLAVE_ID 1
#endif

// Modbus RTU client initialization
ModbusClientRTU MB(MODBUS_RE_DE_PIN);

// Modbus slave address (fixed to configured value)
uint8_t slaveID = MODBUS_SLAVE_ID;

// Global variables
RegisterValues regValues;
bool waitingForResponse = false;
uint32_t currentToken = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 10; // Read interval in milliseconds
ReadState currentState = READ_M71_M79;

// Modbus error statistics (reset after each print)
uint16_t errorCounts[256] = {0};  // Array for all error types (0x00-0xFF)
bool hasErrorsInPeriod = false;   // Flag that marks errors in current period

// Timestamp of last valid communication
unsigned long lastSuccessfulCommTime = 0;
const unsigned long COMM_TIMEOUT = 3000; // 3-second communication timeout

// Serial Monitor output control
bool serialMonitorEnabled = true;  // Default: enabled

// Callback for Modbus responses
void handleData(ModbusMessage response, uint32_t token) {
  waitingForResponse = false;
  
  if (response.getError() != SUCCESS) {
    // Error in response - store the error
    Error err = response.getError();
    if (err < 256) {
      errorCounts[err]++;
      hasErrorsInPeriod = true;
    }
    return;
  }
  
  // Valid response - update last successful communication timestamp
  lastSuccessfulCommTime = millis();
  
  uint8_t fc = response.getFunctionCode();
  
  // Process response based on Modbus function
  // FC 0x03: Read Holding Registers - for D and R registers
  if (fc == 0x03) {  // Read Holding Registers
    uint8_t byteCount = response[2];
    uint8_t dataIndex = 3;
    
    if (currentState == READ_D0) {
      if (byteCount >= 2 && dataIndex + 1 < response.size()) {
        regValues.D0 = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[0] = true;
      }
    } else if (currentState == READ_D10) {
      if (byteCount >= 2 && dataIndex + 1 < response.size()) {
        regValues.D10 = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[1] = true;
      }
    } else if (currentState == READ_D47_D49) {
      // Read 3 registers (D47-D49)
      if (byteCount >= 6 && dataIndex + 5 < response.size()) {
        regValues.D47 = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D48 = (response[dataIndex + 2] << 8) | response[dataIndex + 3];
        regValues.D49 = (response[dataIndex + 4] << 8) | response[dataIndex + 5];
        regValues.D_valid[2] = true;
        regValues.D_valid[3] = true;
        regValues.D_valid[4] = true;
      }
    } else if (currentState == READ_D57_D59) {
      // Read 3 registers (D57-D59)
      if (byteCount >= 6 && dataIndex + 5 < response.size()) {
        regValues.D57 = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D58 = (response[dataIndex + 2] << 8) | response[dataIndex + 3];
        regValues.D59 = (response[dataIndex + 4] << 8) | response[dataIndex + 5];
        regValues.D_valid[5] = true;  // D57
        regValues.D_valid[6] = true;  // D58
        regValues.D_valid[7] = true;  // D59
      }
    } else if (currentState == READ_R10_R99) {
      // Read 90 registers (R10-R99)
      if (byteCount >= 180 && dataIndex + 179 < response.size()) {
        for (int i = 0; i < 90; i++) {
          regValues.R[i] = (response[dataIndex] << 8) | response[dataIndex + 1];
          regValues.R_valid[i] = true;
          dataIndex += 2;
        }
      }
    } else if (currentState == READ_R110_R199) {
      // Read 90 registers (R110-R199)
      if (byteCount >= 180 && dataIndex + 179 < response.size()) {
        for (int i = 0; i < 90; i++) {
          regValues.R[90 + i] = (response[dataIndex] << 8) | response[dataIndex + 1];
          regValues.R_valid[90 + i] = true;
          dataIndex += 2;
        }
      }
    }
  } 
  // FC 0x01: Read Coils - for M registers (discrete coils)
  else if (fc == 0x01) {  // Read Coils
    uint8_t byteCount = response[2];
    uint8_t dataIndex = 3;
    
    if (currentState == READ_M71_M79) {
      // Read 9 coils (M71-M79)
      if (byteCount >= 2) {
        uint8_t byte1 = response[dataIndex];
        uint8_t byte2 = response[dataIndex + 1];
        regValues.M71 = (byte1 & 0x01) != 0;
        regValues.M72 = (byte1 & 0x02) != 0;
        regValues.M73 = (byte1 & 0x04) != 0;
        regValues.M74 = (byte1 & 0x08) != 0;
        regValues.M75 = (byte1 & 0x10) != 0;
        regValues.M76 = (byte1 & 0x20) != 0;
        regValues.M77 = (byte1 & 0x40) != 0;
        regValues.M78 = (byte1 & 0x80) != 0;
        regValues.M79 = (byte2 & 0x01) != 0;
        for (int i = 0; i < 9; i++) regValues.M_valid[i] = true;
      }
    } else if (currentState == READ_M94_M99) {
      // Read 6 coils (M94-M99)
      if (byteCount >= 1) {
        uint8_t byte1 = response[dataIndex];
        regValues.M94 = (byte1 & 0x01) != 0;
        regValues.M95 = (byte1 & 0x02) != 0;
        regValues.M96 = (byte1 & 0x04) != 0;
        regValues.M97 = (byte1 & 0x08) != 0;
        regValues.M98 = (byte1 & 0x10) != 0;
        regValues.M99 = (byte1 & 0x20) != 0;
        for (int i = 9; i < 15; i++) regValues.M_valid[i] = true;  // M94-M99 (6 coils)
      }
    } else if (currentState == READ_M120) {
      if (byteCount >= 1) {
        regValues.M120 = (response[dataIndex] & 0x01) != 0;
        regValues.M_valid[15] = true;  // M120
      }
    } else if (currentState == READ_M320_M322) {
      // Read 3 coils (M320-M322)
      if (byteCount >= 1) {
        uint8_t byte1 = response[dataIndex];
        regValues.M320 = (byte1 & 0x01) != 0;
        regValues.M321 = (byte1 & 0x02) != 0;
        regValues.M322 = (byte1 & 0x04) != 0;
        regValues.M_valid[16] = true;  // M320
        regValues.M_valid[17] = true;  // M321
        regValues.M_valid[18] = true;  // M322
      }
    }
  }
}

// Error callback
void handleError(Error error, uint32_t token) {
  waitingForResponse = false;
  // Modbus error - store the error
  if (error < 256) {
    errorCounts[error]++;
    hasErrorsInPeriod = true;
  }
}

// Get pointer to register values
RegisterValues* getRegisterValues() {
  return &regValues;
}

// Get Modbus error text
String getErrorText(Error error) {
  switch (error) {
    case SUCCESS: return "Success";
    case ILLEGAL_FUNCTION: return "Illegal function";
    case ILLEGAL_DATA_ADDRESS: return "Illegal data address";
    case ILLEGAL_DATA_VALUE: return "Illegal data value";
    case SERVER_DEVICE_FAILURE: return "Server device failure";
    case ACKNOWLEDGE: return "Acknowledge";
    case SERVER_DEVICE_BUSY: return "Server device busy";
    case NEGATIVE_ACKNOWLEDGE: return "Negative acknowledge";
    case MEMORY_PARITY_ERROR: return "Memory parity error";
    case GATEWAY_PATH_UNAVAIL: return "Gateway path unavailable";
    case GATEWAY_TARGET_NO_RESP: return "Gateway target no response";
    case TIMEOUT: return "Timeout";
    case INVALID_SERVER: return "Invalid server";
    case CRC_ERROR: return "CRC error";
    case FC_MISMATCH: return "Function code mismatch";
    case SERVER_ID_MISMATCH: return "Server ID mismatch";
    case PACKET_LENGTH_ERROR: return "Packet length error";
    case PARAMETER_COUNT_ERROR: return "Parameter count error";
    case PARAMETER_LIMIT_ERROR: return "Parameter limit error";
    case REQUEST_QUEUE_FULL: return "Request queue full";
    case ILLEGAL_IP_OR_PORT: return "Illegal IP or port";
    case IP_CONNECTION_FAILED: return "IP connection failed";
    case TCP_HEAD_MISMATCH: return "TCP head mismatch";
    case EMPTY_MESSAGE: return "Empty message";
    case ASCII_FRAME_ERR: return "ASCII frame error";
    case ASCII_CRC_ERR: return "ASCII CRC error";
    case ASCII_INVALID_CHAR: return "ASCII invalid char";
    case BROADCAST_ERROR: return "Broadcast error";
    case UNDEFINED_ERROR: return "Undefined error";
    default: return "Unknown error (0x" + String((uint8_t)error, HEX) + ")";
  }
}

// Convert register value to ASCII characters
String valueToASCII(uint16_t value) {
  if (value == 0) {
    return "0";
  }
  
  // Step 1: modulo 256 (low byte)
  uint8_t lowByte = value % 256;
  char lowChar = (lowByte >= 32 && lowByte <= 126) ? (char)lowByte : '?';
  
  // Step 2: divide by 256 (high byte)
  uint8_t highByte = value / 256;
  
  // If high byte is 0, show only low byte
  if (highByte == 0) {
    return String(lowChar);
  }
  
  // Otherwise show both characters (low byte first, high byte second)
  char highChar = (highByte >= 32 && highByte <= 126) ? (char)highByte : '?';
  return String(lowChar) + String(highChar);
}

// Functions for Serial Monitor output control
void setSerialMonitorEnabled(bool enabled) {
  serialMonitorEnabled = enabled;
}

bool getSerialMonitorEnabled() {
  return serialMonitorEnabled;
}

// Print register values to Serial Monitor
void displayRegisterValues() {
  // Check if Serial Monitor output is enabled
  if (!serialMonitorEnabled) {
    return;  // Print nothing if Serial Monitor output is disabled
  }
  
  Serial.println("\n========================================");
  Serial.println("  VALORI CITITE - " + String(millis() / 1000.0, 2) + "s");
  Serial.println("========================================");
  
  // Print Discrete M Relays
  Serial.println("\n--- DISCRETE M RELAYS ---");
  
  Serial.print("M71-M79 (0x817-0x81F): ");
  bool allValid = true;
  for (int i = 0; i < 9; i++) {
    if (!regValues.M_valid[i]) {
      allValid = false;
      break;
    }
  }
  if (allValid) {
    Serial.println();
    Serial.println(" M71=" + String(regValues.M71 ? "ON" : "OFF"));
    Serial.println(" M72=" + String(regValues.M72 ? "ON" : "OFF"));
    Serial.println(" M73=" + String(regValues.M73 ? "ON" : "OFF"));
    Serial.println(" M74=" + String(regValues.M74 ? "ON" : "OFF"));
    Serial.println(" M75=" + String(regValues.M75 ? "ON" : "OFF"));
    Serial.println(" M76=" + String(regValues.M76 ? "ON" : "OFF"));
    Serial.println(" M77=" + String(regValues.M77 ? "ON" : "OFF"));
    Serial.println(" M78=" + String(regValues.M78 ? "ON" : "OFF"));
    Serial.println(" M79=" + String(regValues.M79 ? "ON" : "OFF"));
    Serial.println();
  } else {
    Serial.println("EROARE - valori nevalide");
  }
  
  if (regValues.M_valid[9] && regValues.M_valid[10] && regValues.M_valid[11] && 
      regValues.M_valid[12] && regValues.M_valid[13] && regValues.M_valid[14]) {
    Serial.println(" M94 (0x82E): " + String(regValues.M94 ? "ON" : "OFF"));
    Serial.println(" M95 (0x82F): " + String(regValues.M95 ? "ON" : "OFF"));
    Serial.println(" M96 (0x830): " + String(regValues.M96 ? "ON" : "OFF"));
    Serial.println(" M97 (0x831): " + String(regValues.M97 ? "ON" : "OFF"));
    Serial.println(" M98 (0x832): " + String(regValues.M98 ? "ON" : "OFF"));
    Serial.println(" M99 (0x833): " + String(regValues.M99 ? "ON" : "OFF"));
  } else {
    Serial.println("M94-M99 (0x82E-0x833): EROARE - valori nevalide");
  }
  
  if (regValues.M_valid[15]) {
    Serial.println(" M120 (0x848): " + String(regValues.M120 ? "ON" : "OFF"));
  } else {
    Serial.println("M120 (0x848): EROARE - valoare nevalidă");
  }
  
  if (regValues.M_valid[16] && regValues.M_valid[17] && regValues.M_valid[18]) {
    Serial.println(" M320 (0x910): " + String(regValues.M320 ? "ON" : "OFF"));
    Serial.println(" M321 (0x911): " + String(regValues.M321 ? "ON" : "OFF"));
    Serial.println(" M322 (0x912): " + String(regValues.M322 ? "ON" : "OFF"));
  } else {
    Serial.println("M320-M322 (0x910-0x912): EROARE - valori nevalide");
  }
  
  // Print Data Registers
  Serial.println("\n--- DATA REGISTERS (D) ---");
  
  if (regValues.D_valid[0]) {
    Serial.print(" D0 (0x1770): ");
    Serial.print(regValues.D0);
    Serial.print(" (0x");
    Serial.print(regValues.D0, HEX);
    Serial.println(")");
  } else {
    Serial.println("D0 (0x1770): EROARE - valoare nevalidă");
  }
  
  if (regValues.D_valid[1]) {
    Serial.print(" D10 (0x177A): ");
    Serial.print(regValues.D10);
    Serial.print(" (0x");
    Serial.print(regValues.D10, HEX);
    Serial.println(")");
  } else {
    Serial.println("D10 (0x177A): EROARE - valoare nevalidă");
  }
  
  if (regValues.D_valid[2] && regValues.D_valid[3] && regValues.D_valid[4]) {
    Serial.print(" D47 (0x179F): ");
    Serial.print(regValues.D47);
    Serial.print(" (0x");
    Serial.print(regValues.D47, HEX);
    Serial.println(")");
    Serial.print(" D48 (0x17A0): ");
    Serial.print(regValues.D48);
    Serial.print(" (0x");
    Serial.print(regValues.D48, HEX);
    Serial.println(")");
    Serial.print(" D49 (0x17A1): ");
    Serial.print(regValues.D49);
    Serial.print(" (0x");
    Serial.print(regValues.D49, HEX);
    Serial.println(")");
  } else {
    Serial.println("D47-D49 (0x179F-0x17A1): EROARE - valori nevalide");
  }
  
  if (regValues.D_valid[5] && regValues.D_valid[6] && regValues.D_valid[7]) {
    Serial.print(" D57 (0x17A9): ");
    Serial.print(regValues.D57);
    Serial.print(" (0x");
    Serial.print(regValues.D57, HEX);
    Serial.println(")");
    Serial.print(" D58 (0x17AA): ");
    Serial.print(regValues.D58);
    Serial.print(" (0x");
    Serial.print(regValues.D58, HEX);
    Serial.println(")");
    Serial.print(" D59 (0x17AB): ");
    Serial.print(regValues.D59);
    Serial.print(" (0x");
    Serial.print(regValues.D59, HEX);
    Serial.println(")");
  } else {
    Serial.println("D57-D59 (0x17A9-0x17AB): EROARE - valori nevalide");
  }
  
  // Print Holding Registers
  Serial.println("\n--- HOLDING REGISTERS (R) ---");
  Serial.println("R10-R99, R110-R199 (0xA-0x63, 0x6E-0xC7):");
  
  bool hasErrors = false;
  
  // Print R10-R99 (90 registers, index 0-89)
  for (int i = 0; i < 90; i++) {
    int regNum = i + 10;
    if (regValues.R_valid[i]) {
      Serial.print(" R" + String(regNum) + " (0x" + String(0xA + i, HEX) + "): ");
      Serial.print(regValues.R[i]);
      Serial.print(" (0x");
      Serial.print(regValues.R[i], HEX);
      Serial.print(") - ");
      Serial.println(valueToASCII(regValues.R[i]));
    } else {
      if (!hasErrors) {
        Serial.println("EROARE - unele valori nevalide:");
        hasErrors = true;
      }
      Serial.println(" R" + String(regNum) + " (0x" + String(0xA + i, HEX) + "): EROARE");
    }
  }
  
  // Skip R100-R109 (not read)
  
  // Print R110-R199 (90 registers, index 90-179)
  for (int i = 0; i < 90; i++) {
    int regNum = i + 110;
    if (regValues.R_valid[90 + i]) {
      Serial.print(" R" + String(regNum) + " (0x" + String(0x6E + i, HEX) + "): ");
      Serial.print(regValues.R[90 + i]);
      Serial.print(" (0x");
      Serial.print(regValues.R[90 + i], HEX);
      Serial.print(") - ");
      Serial.println(valueToASCII(regValues.R[90 + i]));
    } else {
      if (!hasErrors) {
        Serial.println("EROARE - unele valori nevalide:");
        hasErrors = true;
      }
      Serial.println(" R" + String(regNum) + " (0x" + String(0x6E + i, HEX) + "): EROARE");
    }
  }
  
  Serial.println("========================================");
  
  // Print error statistics
  Serial.println("\n--- STATISTICI ERORI MODBUS ---");
  if (hasErrorsInPeriod) {
    Serial.println("EROARE DETECTATĂ în perioada de 1 secundă:");
    bool foundAnyError = false;
    for (int i = 0; i < 256; i++) {
      if (errorCounts[i] > 0 && i != SUCCESS) {
        foundAnyError = true;
        Serial.print("  ");
        Serial.print(getErrorText((Error)i));
        Serial.print(" (0x");
        Serial.print(i, HEX);
        Serial.print("): ");
        Serial.print(errorCounts[i]);
        Serial.println(" dată/dăți");
      }
    }
    if (!foundAnyError) {
      Serial.println("  Nu au fost detectate erori.");
    }
  } else {
    Serial.println("Fără erori în perioada de 1 secundă.");
  }
  
  // Reset error counters for next period
  for (int i = 0; i < 256; i++) {
    errorCounts[i] = 0;
  }
  hasErrorsInPeriod = false;
  
  Serial.println("========================================\n");
}

// Check if PLC communication is active
bool isCommunicationActive() {
  // Check if more than 3 seconds passed since last valid communication
  unsigned long currentTime = millis();
  
  // If no valid communication happened yet and startup just finished
  if (lastSuccessfulCommTime == 0) {
    // On first run, communication is inactive until first response
    // But if startup is recent (< 3 seconds), allow connection time
    if (currentTime < 3000) {
      return true; // Allow startup time
    }
    return false;
  }
  
  // Check timeout against last valid communication
  if (currentTime - lastSuccessfulCommTime > COMM_TIMEOUT) {
    return false; // Communication timed out
  }
  
  return true; // Communication active
}


// Initialization
void initFATEKModbus() {
  // Modbus initialization - debug prints only if Serial Monitor is enabled
  // Note: to fully disable prints, use enableSerialMonitor in main.ino
  
  // Initialize values
  for (int i = 0; i < 18; i++) {
    regValues.M_valid[i] = false;
  }
  
  // Initialize communication timestamp
  lastSuccessfulCommTime = 0; // Updated on first valid response
  regValues.M71 = regValues.M72 = regValues.M73 = regValues.M74 = regValues.M75 = false;
  regValues.M76 = regValues.M77 = regValues.M78 = regValues.M79 = false;
  regValues.M94 = regValues.M95 = regValues.M96 = regValues.M97 = regValues.M98 = regValues.M99 = false;
  regValues.M120 = false;
  regValues.M320 = regValues.M321 = regValues.M322 = false;
  
  for (int i = 0; i < 8; i++) {
    regValues.D_valid[i] = false;
  }
  regValues.D0 = 0;
  regValues.D10 = 0;
  regValues.D47 = regValues.D48 = regValues.D49 = 0;
  regValues.D57 = regValues.D58 = regValues.D59 = 0;
  
  for (int i = 0; i < 180; i++) {
    regValues.R[i] = 0;
    regValues.R_valid[i] = false;
  }
  
  // Configure RE/DE pin
  pinMode(MODBUS_RE_DE_PIN, OUTPUT);
  digitalWrite(MODBUS_RE_DE_PIN, LOW);
  
  // Prepare hardware serial interface
  RTUutils::prepareHardwareSerial(Serial2);
  
  // Initialize serial interface
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);  
  delay(50);
  
  // Set callbacks
  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);
  
  // Set timeout
  MB.setTimeout(2000);
  
  // Start Modbus client
  MB.begin(Serial2);
  
  // Slave ID is fixed to MODBUS_SLAVE_ID (default: 1)
  // No automatic search is used
  slaveID = MODBUS_SLAVE_ID;
  
  if (serialMonitorEnabled) {
    Serial.printf("Comunicare Modbus inițializată cu slave ID: %d\n", slaveID);
  }
  
  lastReadTime = millis();
}

// Get current slave ID
uint8_t getCurrentSlaveID() {
  return slaveID;
}

// Main loop
void loopFATEKModbus() {
  unsigned long currentTime = millis();
  
  // Check interval and ensure no response is pending
  if (currentTime - lastReadTime >= readInterval && !waitingForResponse) {
    Error err = SUCCESS;
    ModbusMessage request;
    
    // Build request based on current state
    // IMPORTANT: use 0x01 (READ_COIL) for M registers
    //            and 0x03 (READ_HOLD_REGISTER) for D and R registers
    switch (currentState) {
      case READ_M71_M79:
        // Function 0x01: Read Coils - M71-M79 (9 coils, address 0x817)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_COIL, 0x817, 9);
        break;
        
      case READ_M94_M99:
        // Function 0x01: Read Coils - M94-M99 (6 coils, address 0x82E)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_COIL, 0x82E, 6);
        break;
        
      case READ_M120:
        // Function 0x01: Read Coils - M120 (1 coil, address 0x848)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_COIL, 0x848, 1);
        break;
        
      case READ_M320_M322:
        // Function 0x01: Read Coils - M320-M322 (3 coils, address 0x910)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_COIL, 0x910, 3);
        break;
        
      case READ_D0:
        // Function 0x03: Read Holding Registers - D0 (1 register, address 0x1770)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0x1770, 1);
        break;
        
      case READ_D10:
        // Function 0x03: Read Holding Registers - D10 (1 register, address 0x177A)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0x177A, 1);
        break;
        
      case READ_D47_D49:
        // Function 0x03: Read Holding Registers - D47-D49 (3 registers, address 0x179F)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0x179F, 3);
        break;
        
      case READ_D57_D59:
        // Function 0x03: Read Holding Registers - D57-D59 (3 registers, address 0x17A9)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0x17A9, 3);
        break;
        
      case READ_R10_R99:
        // Function 0x03: Read Holding Registers - R10-R99 (90 registers, address 0xA)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0xA, 90);
        break;
        
      case READ_R110_R199:
        // Function 0x03: Read Holding Registers - R110-R199 (90 registers, address 0x6E)
        err = MB.addRequest((uint32_t)millis(), slaveID, READ_HOLD_REGISTER, 0x6E, 90);
        break;
    }
    
    if (err != SUCCESS) {
      waitingForResponse = false;
      // Request send error - do not print to Serial
    } else {
      waitingForResponse = true;
      currentToken = millis();
    }
    
    // Wait for response (with yield to avoid blocking animation)
    unsigned long responseStart = millis();
    while (waitingForResponse && (millis() - responseStart < 3000)) {
      delay(10);
      yield();  // Let scroll animation continue
    }
    
    if (waitingForResponse) {
      // Read timeout - do not print to Serial
      waitingForResponse = false;
    }
    
    // Move to next read
    switch (currentState) {
      case READ_M71_M79: currentState = READ_M94_M99; break;
      case READ_M94_M99: currentState = READ_M120; break;
      case READ_M120: currentState = READ_M320_M322; break;
      case READ_M320_M322: currentState = READ_D0; break;
      case READ_D0: currentState = READ_D10; break;
      case READ_D10: currentState = READ_D47_D49; break;
      case READ_D47_D49: currentState = READ_D57_D59; break;
      case READ_D57_D59: currentState = READ_R10_R99; break;
      case READ_R10_R99: currentState = READ_R110_R199; break;
      case READ_R110_R199: 
        currentState = READ_M71_M79;  // Return to start
        break;
    }
    
    // Delay between requests so PLC has processing time
    delay(20);
    
    lastReadTime = currentTime;
  }
  
  delay(10);
  yield();  // Allow other tasks (such as scroll animation) to run
}

