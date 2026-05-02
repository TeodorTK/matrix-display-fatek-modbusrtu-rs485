#include <Arduino.h>
#include "ModbusClientRTU.h"
#include "RTUutils.h"
#include <SPIFFS.h>

// Definirea pinilor ESP32
#define RX_PIN 16
#define TX_PIN 17
#define RE_DE_PIN 22
#define BAUDRATE 9600

// Numele fișierului TXT (lăsați gol "" pentru a citi de la Serial Monitor)
// Exemplu: "/registrii.txt" sau "/config.txt"
const char* TXT_FILENAME = "/registrii.txt";  // <-- AICI PUI NUMELE FIȘIERULUI TXT (trebuie să înceapă cu "/")
// Dacă TXT_FILENAME este gol (""), se va citi de la Serial Monitor
// Dacă TXT_FILENAME are un nume (ex: "/registrii.txt"), se va citi din SPIFFS și se va verifica automat pentru modificări

// Variabile pentru detectarea modificărilor fișierului TXT
unsigned long lastFileCheckTime = 0;
const unsigned long fileCheckInterval = 1000; // Verificăm fișierul la fiecare secundă
time_t lastFileModificationTime = 0;

// Inițializarea clientului Modbus RTU
ModbusClientRTU MB(RE_DE_PIN);

// Adresa slave-ului Modbus
const uint8_t slaveID = 5;

// Structură pentru stocarea valorilor citite
struct RegisterValues {
  // Data Registers (D0-D10, D46-D48, D57-D58)
  uint16_t D[16];  // D0-D10 (11), D46-D48 (3), D57-D58 (2)
  bool D_valid[16];
  
  // Discrete M Relays
  bool M0;
  bool M70, M71, M72, M73, M74, M75, M76, M77, M78, M79;
  bool M93, M94, M95;
  bool M97, M98;
  bool M120;
  
  bool M_valid[18];  // 1 + 10 + 3 + 2 + 1 = 17, dar folosim 18 pentru siguranță
};

RegisterValues regValues;

// Variabile pentru gestionarea citirilor și scrierilor
bool waitingForResponse = false;
bool waitingForWriteResponse = false;
uint32_t currentToken = 0;
unsigned long lastReadTime = 0;
unsigned long lastWriteTime = 0;
const unsigned long readInterval = 200; // 200ms = 5 citiri/secundă
const unsigned long writeInterval = 200; // 200ms = 5 scrieri/secundă

// Structură pentru datele de scriere din JSON
struct WriteValues {
  // Data Registers
  bool D_write_enabled[16];
  uint16_t D_write[16];
  
  // Discrete M Relays
  bool M_write_enabled[18];
  bool M_write[18];
  
  bool hasData; // Flag pentru a ști dacă avem date de scris
  int currentWriteIndex; // Index pentru următorul registru de scris (0-15 pentru D, 16-33 pentru M)
  bool writingD; // Flag pentru a ști dacă scriem D sau M
};

WriteValues writeValues;

// Index pentru a ști ce citim acum
enum ReadState {
  READ_D0_D10,      // Citim D0-D10 (11 registri)
  READ_D46_D48,     // Citim D46-D48 (3 registri)
  READ_D57_D58,     // Citim D57-D58 (2 registri)
  READ_M0,          // Citim M0
  READ_M70_M79,     // Citim M70-M79 (10 coils)
  READ_M93_M95,     // Citim M93-M95 (3 coils)
  READ_M97_M98,     // Citim M97-M98 (2 coils)
  READ_M120         // Citim M120
};

ReadState currentState = READ_D0_D10;

// Callback pentru răspunsuri Modbus
void handleData(ModbusMessage response, uint32_t token) {
  uint8_t fc = response.getFunctionCode();
  
  // Verificăm dacă este răspuns la scriere (FC05 sau FC06)
  if (fc == 0x05 || fc == 0x06) {
    waitingForWriteResponse = false;
    
    if (response.getError() != SUCCESS) {
      Serial.print("EROARE în răspuns la scriere: 0x");
      Serial.print(response.getError(), HEX);
      Serial.print(" - ");
      ModbusError me(response.getError());
      Serial.println((const char *)me);
      return;
    }
    
    // Răspuns de scriere reușit
    Serial.print(">>> RĂSPUNS SCRIERE PRIMIT - FC");
    Serial.print(fc, HEX);
    Serial.print(", Token: 0x");
    Serial.print(token, HEX);
    if (fc == 0x06) {
      Serial.println(" - Registru D scris cu succes!");
    } else if (fc == 0x05) {
      Serial.println(" - Coil M scris cu succes!");
    }
    return;
  }
  
  // Răspuns la citire
  waitingForResponse = false;
  
  if (response.getError() != SUCCESS) {
    Serial.print("EROARE în răspuns: 0x");
    Serial.print(response.getError(), HEX);
    Serial.print(" - ");
    ModbusError me(response.getError());
    Serial.println((const char *)me);
    return;
  }
  
  // Procesăm răspunsul în funcție de funcția Modbus
  if (fc == 0x03) {  // Read Holding Registers
    uint8_t byteCount = response[2];
    uint8_t dataIndex = 3;
    
    if (currentState == READ_D0_D10) {
      // Citim 11 registri (D0-D10)
      for (int i = 0; i < 11 && dataIndex + 1 < response.size(); i++) {
        regValues.D[i] = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[i] = true;
        dataIndex += 2;
      }
    } else if (currentState == READ_D46_D48) {
      // Citim 3 registri (D46-D48), stocăm la index 11-13
      for (int i = 0; i < 3 && dataIndex + 1 < response.size(); i++) {
        regValues.D[11 + i] = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[11 + i] = true;
        dataIndex += 2;
      }
    } else if (currentState == READ_D57_D58) {
      // Citim 2 registri (D57-D58), stocăm la index 14-15
      for (int i = 0; i < 2 && dataIndex + 1 < response.size(); i++) {
        regValues.D[14 + i] = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[14 + i] = true;
        dataIndex += 2;
      }
    }
  } else if (fc == 0x01) {  // Read Coils
    uint8_t byteCount = response[2];
    uint8_t dataIndex = 3;
    
    if (currentState == READ_M0) {
      if (byteCount >= 1) {
        regValues.M0 = (response[dataIndex] & 0x01) != 0;
        regValues.M_valid[0] = true;
      }
    } else if (currentState == READ_M70_M79) {
      // Citim 10 coils (M70-M79)
      if (byteCount >= 2) {
        uint8_t byte1 = response[dataIndex];
        uint8_t byte2 = response[dataIndex + 1];
        regValues.M70 = (byte1 & 0x01) != 0;
        regValues.M71 = (byte1 & 0x02) != 0;
        regValues.M72 = (byte1 & 0x04) != 0;
        regValues.M73 = (byte1 & 0x08) != 0;
        regValues.M74 = (byte1 & 0x10) != 0;
        regValues.M75 = (byte1 & 0x20) != 0;
        regValues.M76 = (byte1 & 0x40) != 0;
        regValues.M77 = (byte1 & 0x80) != 0;
        regValues.M78 = (byte2 & 0x01) != 0;
        regValues.M79 = (byte2 & 0x02) != 0;
        for (int i = 1; i <= 10; i++) regValues.M_valid[i] = true;
      }
    } else if (currentState == READ_M93_M95) {
      // Citim 3 coils (M93-M95)
      if (byteCount >= 1) {
        uint8_t byte1 = response[dataIndex];
        regValues.M93 = (byte1 & 0x01) != 0;
        regValues.M94 = (byte1 & 0x02) != 0;
        regValues.M95 = (byte1 & 0x04) != 0;
        for (int i = 11; i <= 13; i++) regValues.M_valid[i] = true;
      }
    } else if (currentState == READ_M97_M98) {
      // Citim 2 coils (M97-M98)
      if (byteCount >= 1) {
        uint8_t byte1 = response[dataIndex];
        regValues.M97 = (byte1 & 0x01) != 0;
        regValues.M98 = (byte1 & 0x02) != 0;
        for (int i = 14; i <= 15; i++) regValues.M_valid[i] = true;
      }
    } else if (currentState == READ_M120) {
      if (byteCount >= 1) {
        regValues.M120 = (response[dataIndex] & 0x01) != 0;
        regValues.M_valid[17] = true;
      }
    }
  }
}

// Callback pentru erori
void handleError(Error error, uint32_t token) {
  waitingForResponse = false;
  waitingForWriteResponse = false;
  
  ModbusError me(error);
  Serial.print("EROARE Modbus: 0x");
  Serial.print((int)me, HEX);
  Serial.print(" - ");
  Serial.println((const char *)me);
}

// Funcție pentru verificarea dacă fișierul TXT a fost modificat
bool checkFileModified() {
  if (strlen(TXT_FILENAME) == 0) {
    return false; // Nu verificăm dacă citim de la Serial
  }
  
  if (!SPIFFS.begin(true)) {
    return false;
  }
  
  if (!SPIFFS.exists(TXT_FILENAME)) {
    SPIFFS.end();
    return false;
  }
  
  File file = SPIFFS.open(TXT_FILENAME, "r");
  if (!file) {
    SPIFFS.end();
    return false;
  }
  
  // Obținem timpul de modificare al fișierului
  time_t currentModTime = file.getLastWrite();
  file.close();
  SPIFFS.end();
  
  // Comparăm cu ultima modificare cunoscută
  if (currentModTime != lastFileModificationTime) {
    lastFileModificationTime = currentModTime;
    return true; // Fișierul a fost modificat
  }
  
  return false; // Fișierul nu a fost modificat
}

// Funcție pentru procesarea unei linii din fișierul TXT
bool processTXTLine(String line) {
  // Eliminăm spațiile
  line.trim();
  
  // Ignorăm liniile goale și comentariile
  if (line.length() == 0 || line.startsWith("#")) {
    return false;
  }
  
  // Căutăm semnul "="
  int equalPos = line.indexOf('=');
  if (equalPos == -1) {
    return false; // Linie invalidă
  }
  
  String regName = line.substring(0, equalPos);
  String regValue = line.substring(equalPos + 1);
  regName.trim();
  regValue.trim();
  
  // Procesăm registrii D
  if (regName.startsWith("D")) {
    int regNum = regName.substring(1).toInt();
    uint16_t value = regValue.toInt();
    
    // Verificăm dacă este un registru valid
    if ((regNum >= 0 && regNum <= 10) || 
        (regNum >= 46 && regNum <= 48) || 
        (regNum >= 57 && regNum <= 58)) {
      
      int idx;
      if (regNum <= 10) {
        idx = regNum;
      } else if (regNum >= 46 && regNum <= 48) {
        idx = 11 + (regNum - 46);
      } else { // 57-58
        idx = 14 + (regNum - 57);
      }
      
      writeValues.D_write[idx] = value;
      writeValues.D_write_enabled[idx] = true;
      writeValues.hasData = true;
      
      Serial.print("  [D] D" + String(regNum) + " -> valoare: ");
      Serial.print(value);
      Serial.print(" (adresa Modbus: 0x");
      uint16_t address;
      if (regNum <= 10) {
        address = 0x1770 + regNum;
      } else if (regNum >= 46 && regNum <= 48) {
        address = 0x179E + (regNum - 46);
      } else {
        address = 0x17A9 + (regNum - 57);
      }
      Serial.print(address, HEX);
      Serial.println(")");
      return true;
    }
  }
  
  // Procesăm registrii M
  if (regName.startsWith("M")) {
    int regNum = regName.substring(1).toInt();
    bool value = (regValue == "true" || regValue == "1" || regValue == "ON");
    
    // Verificăm dacă este un registru valid
    if (regNum == 0 || 
        (regNum >= 70 && regNum <= 79) || 
        (regNum >= 93 && regNum <= 95) || 
        (regNum >= 97 && regNum <= 98) || 
        regNum == 120) {
      
      int idx;
      if (regNum == 0) {
        idx = 0;
      } else if (regNum >= 70 && regNum <= 79) {
        idx = 1 + (regNum - 70);
      } else if (regNum >= 93 && regNum <= 95) {
        idx = 11 + (regNum - 93);
      } else if (regNum >= 97 && regNum <= 98) {
        idx = 14 + (regNum - 97);
      } else { // 120
        idx = 17;
      }
      
      writeValues.M_write[idx] = value;
      writeValues.M_write_enabled[idx] = true;
      writeValues.hasData = true;
      
      Serial.print("  [M] M" + String(regNum) + " -> valoare: ");
      Serial.print(value ? "ON" : "OFF");
      Serial.print(" (adresa Modbus: 0x");
      uint16_t address;
      if (regNum == 0) {
        address = 0x7D0;
      } else if (regNum >= 70 && regNum <= 79) {
        address = 0x816 + (regNum - 70);
      } else if (regNum >= 93 && regNum <= 95) {
        address = 0x82D + (regNum - 93);
      } else if (regNum >= 97 && regNum <= 98) {
        address = 0x831 + (regNum - 97);
      } else {
        address = 0x848;
      }
      Serial.print(address, HEX);
      Serial.println(")");
      return true;
    }
  }
  
  return false;
}

// Funcție pentru citirea TXT din fișier SPIFFS
bool readTXTFromFile() {
  if (strlen(TXT_FILENAME) == 0) {
    // Dacă nu este specificat un fișier, citim de la Serial
    return readTXTFromSerial();
  }
  
  Serial.print(">>> Citire TXT din fișier: ");
  Serial.println(TXT_FILENAME);
  
  if (!SPIFFS.begin(true)) {
    Serial.println(">>> EROARE: Nu s-a putut inițializa SPIFFS!");
    return false;
  }
  
  if (!SPIFFS.exists(TXT_FILENAME)) {
    Serial.print(">>> EROARE: Fișierul ");
    Serial.print(TXT_FILENAME);
    Serial.println(" nu există în SPIFFS!");
    SPIFFS.end();
    return false;
  }
  
  File file = SPIFFS.open(TXT_FILENAME, "r");
  if (!file) {
    Serial.print(">>> EROARE: Nu s-a putut deschide fișierul ");
    Serial.println(TXT_FILENAME);
    SPIFFS.end();
    return false;
  }
  
  // Actualizăm timpul de modificare
  lastFileModificationTime = file.getLastWrite();
  
  Serial.println("\n========================================");
  Serial.println(">>> TXT PRIMIT ȘI PARSAT CU SUCCES! <<<");
  Serial.println("========================================");
  Serial.println("Registrii de scris din TXT:");
  Serial.println("---");
  
  // Resetăm flag-urile
  for (int i = 0; i < 16; i++) {
    writeValues.D_write_enabled[i] = false;
  }
  for (int i = 0; i < 18; i++) {
    writeValues.M_write_enabled[i] = false;
  }
  writeValues.hasData = false;
  writeValues.currentWriteIndex = 0;
  writeValues.writingD = true;
  
  // Citim fișierul linie cu linie
  while (file.available()) {
    String line = file.readStringUntil('\n');
    processTXTLine(line);
  }
  
  file.close();
  SPIFFS.end();
  
  Serial.println("---");
  if (writeValues.hasData) {
    Serial.println(">>> Scrierea registrilor va începe la 5 Hz (200ms interval) <<<");
  } else {
    Serial.println(">>> ATENȚIE: Nu s-au găsit registrii valizi în TXT! <<<");
  }
  Serial.println("========================================\n");
  
  return writeValues.hasData;
}

// Funcție pentru citirea TXT de la Serial
bool readTXTFromSerial() {
  if (Serial.available() == 0) return false;
  
  String txtContent = "";
  unsigned long startTime = millis();
  
  // Așteptăm să primim întregul conținut (timeout 2 secunde)
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char c = Serial.read();
      txtContent += c;
      // Dacă primim un caracter de terminare, continuăm să citim până la timeout
    }
    delay(1);
  }
  
  if (txtContent.length() == 0) return false;
  
  Serial.println("\n========================================");
  Serial.println(">>> TXT PRIMIT DE LA SERIAL <<<");
  Serial.println("========================================");
  Serial.println("Registrii de scris din TXT:");
  Serial.println("---");
  
  // Resetăm flag-urile
  for (int i = 0; i < 16; i++) {
    writeValues.D_write_enabled[i] = false;
  }
  for (int i = 0; i < 18; i++) {
    writeValues.M_write_enabled[i] = false;
  }
  writeValues.hasData = false;
  writeValues.currentWriteIndex = 0;
  writeValues.writingD = true;
  
  // Procesăm linie cu linie
  int startPos = 0;
  while (startPos < txtContent.length()) {
    int endPos = txtContent.indexOf('\n', startPos);
    if (endPos == -1) endPos = txtContent.length();
    
    String line = txtContent.substring(startPos, endPos);
    processTXTLine(line);
    
    startPos = endPos + 1;
  }
  
  Serial.println("---");
  if (writeValues.hasData) {
    Serial.println(">>> Scrierea registrilor va începe la 5 Hz (200ms interval) <<<");
  } else {
    Serial.println(">>> ATENȚIE: Nu s-au găsit registrii valizi în TXT! <<<");
  }
  Serial.println("========================================\n");
  
  return writeValues.hasData;
}


// Funcție pentru scrierea unui registru D (FC06)
bool writeDRegister(uint16_t address, uint16_t value) {
  if (waitingForWriteResponse) return false;
  
  Serial.println("\n>>> TRIMITERE SCRIERE REGISTRU D <<<");
  Serial.print("  Adresa Modbus: 0x");
  Serial.print(address, HEX);
  Serial.print(" | Valoare: ");
  Serial.println(value);
  
  ModbusMessage writeRequest;
  writeRequest.add(slaveID);
  writeRequest.add((uint8_t)0x06);  // FC06: Write Single Holding Register
  writeRequest.add(address);  // Adresa registrului (biblioteca face conversia automată)
  writeRequest.add(value);    // Valoarea de scris (biblioteca face conversia automată)
  
  digitalWrite(RE_DE_PIN, HIGH);
  delay(2);
  
  currentToken = millis();
  waitingForWriteResponse = true;
  Error err = MB.addRequest(writeRequest, currentToken);
  
  delay(50);
  digitalWrite(RE_DE_PIN, LOW);
  
  if (err != SUCCESS) {
    waitingForWriteResponse = false;
    ModbusError e(err);
    Serial.print("EROARE la trimiterea cererii de scriere D: 0x");
    Serial.print((int)e, HEX);
    Serial.print(" - ");
    Serial.println((const char *)e);
    return false;
  }
  
  // Așteptăm răspunsul (biblioteca procesează mesajele automat)
  unsigned long responseStart = millis();
  while (waitingForWriteResponse && (millis() - responseStart < 6000)) {
    delay(10);  // Permitem procesarea mesajelor Modbus
  }
  
  if (waitingForWriteResponse) {
    Serial.println("TIMEOUT la scrierea registrului D");
    waitingForWriteResponse = false;
    return false;
  }
  
  return true;
}

// Funcție pentru scrierea unui coil M (FC05)
bool writeMCoil(uint16_t address, bool value) {
  if (waitingForWriteResponse) return false;
  
  Serial.println("\n>>> TRIMITERE SCRIERE COIL M <<<");
  Serial.print("  Adresa Modbus: 0x");
  Serial.print(address, HEX);
  Serial.print(" | Valoare: ");
  Serial.println(value ? "ON" : "OFF");
  
  ModbusMessage writeRequest;
  writeRequest.add(slaveID);
  writeRequest.add((uint8_t)0x05);  // FC05: Write Single Coil
  writeRequest.add(address);  // Adresa coil-ului (biblioteca face conversia automată)
  writeRequest.add(value ? (uint16_t)0xFF00 : (uint16_t)0x0000);  // Valoarea coil-ului
  
  digitalWrite(RE_DE_PIN, HIGH);
  delay(2);
  
  currentToken = millis();
  waitingForWriteResponse = true;
  Error err = MB.addRequest(writeRequest, currentToken);
  
  delay(50);
  digitalWrite(RE_DE_PIN, LOW);
  
  if (err != SUCCESS) {
    waitingForWriteResponse = false;
    ModbusError e(err);
    Serial.print("EROARE la trimiterea cererii de scriere M: 0x");
    Serial.print((int)e, HEX);
    Serial.print(" - ");
    Serial.println((const char *)e);
    return false;
  }
  
  // Așteptăm răspunsul (biblioteca procesează mesajele automat)
  unsigned long responseStart = millis();
  while (waitingForWriteResponse && (millis() - responseStart < 6000)) {
    delay(10);  // Permitem procesarea mesajelor Modbus
  }
  
  if (waitingForWriteResponse) {
    Serial.println("TIMEOUT la scrierea coil-ului M");
    waitingForWriteResponse = false;
    return false;
  }
  
  return true;
}

// Funcție pentru procesarea scrierilor din JSON (scrie un singur registru la un moment dat)
void processWriteOperations() {
  if (!writeValues.hasData) {
    Serial.println("processWriteOperations: Nu avem date de scris");
    return;
  }
  if (waitingForWriteResponse) {
    Serial.println("processWriteOperations: Așteptăm răspuns la scriere");
    return;
  }
  
  Serial.println("processWriteOperations: Începem procesarea scrierilor...");
  
  // Căutăm următorul registru D de scris
  if (writeValues.writingD) {
    for (int i = writeValues.currentWriteIndex; i < 16; i++) {
      if (writeValues.D_write_enabled[i]) {
        uint16_t address;
        int regNum;
        
        // Determinăm adresa și numărul registrului
        if (i <= 10) {
          // D0-D10
          address = 0x1770 + i;
          regNum = i;
        } else if (i >= 11 && i <= 13) {
          // D46-D48
          address = 0x179E + (i - 11);
          regNum = 46 + (i - 11);
        } else if (i >= 14 && i <= 15) {
          // D57-D58
          address = 0x17A9 + (i - 14);
          regNum = 57 + (i - 14);
        } else {
          continue;
        }
        
        // Încercăm să scriem
        if (writeDRegister(address, writeValues.D_write[i])) {
          Serial.print("D" + String(regNum) + " scris cu succes: ");
          Serial.println(writeValues.D_write[i]);
          writeValues.D_write_enabled[i] = false;
        }
        
        writeValues.currentWriteIndex = i + 1;
        return; // Scriem doar un registru la un moment dat
      }
    }
    
    // Am terminat cu D, trecem la M
    writeValues.writingD = false;
    writeValues.currentWriteIndex = 0;
  }
  
  // Căutăm următorul coil M de scris
  if (!writeValues.writingD) {
    for (int i = writeValues.currentWriteIndex; i < 18; i++) {
      if (writeValues.M_write_enabled[i]) {
        uint16_t address;
        String regName;
        
        // Determinăm adresa și numele registrului
        if (i == 0) {
          // M0
          address = 0x7D0;
          regName = "M0";
        } else if (i >= 1 && i <= 10) {
          // M70-M79
          address = 0x816 + (i - 1);
          regName = "M" + String(70 + (i - 1));
        } else if (i >= 11 && i <= 13) {
          // M93-M95
          address = 0x82D + (i - 11);
          regName = "M" + String(93 + (i - 11));
        } else if (i >= 14 && i <= 15) {
          // M97-M98
          address = 0x831 + (i - 14);
          regName = "M" + String(97 + (i - 14));
        } else if (i == 17) {
          // M120
          address = 0x848;
          regName = "M120";
        } else {
          continue;
        }
        
        // Încercăm să scriem
        Serial.print(">>> Încep scrierea " + regName + " (adresa 0x");
        Serial.print(address, HEX);
        Serial.print(", valoare: ");
        Serial.print(writeValues.M_write[i] ? "ON" : "OFF");
        Serial.println(") <<<");
        
        if (writeMCoil(address, writeValues.M_write[i])) {
          Serial.println(">>> ✓ " + regName + " scris cu succes: " + String(writeValues.M_write[i] ? "ON" : "OFF"));
          writeValues.M_write_enabled[i] = false;
        } else {
          Serial.println(">>> ✗ EROARE la scrierea " + regName + " <<<");
        }
        
        writeValues.currentWriteIndex = i + 1;
        return; // Scriem doar un coil la un moment dat
      }
    }
    
    // Am terminat cu M, revenim la D
    writeValues.writingD = true;
    writeValues.currentWriteIndex = 0;
  }
  
  // Verificăm dacă mai avem date de scris
  writeValues.hasData = false;
  for (int i = 0; i < 16; i++) {
    if (writeValues.D_write_enabled[i]) {
      writeValues.hasData = true;
      return;
    }
  }
  for (int i = 0; i < 18; i++) {
    if (writeValues.M_write_enabled[i]) {
      writeValues.hasData = true;
      return;
    }
  }
  
  // Nu mai avem date de scris, resetăm
  writeValues.currentWriteIndex = 0;
  writeValues.writingD = true;
}

void setup() {
  // Inițializarea Serial pentru debug
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("__ OK __");
  Serial.println("\n=== ESP32 Modbus RTU Client - Citire Registri FATEK ===");
  Serial.println("Configurare Modbus RTU:");
  Serial.println("  Baudrate: 9600");
  Serial.println("  Parity: Even (8E1)");
  Serial.println("  RX Pin: 16");
  Serial.println("  TX Pin: 17");
  Serial.println("  RE/DE Pin: 22");
  Serial.print("  Slave ID: ");
  Serial.println(slaveID);
  Serial.println("  Frecvență citire: 5 citiri/secundă (200ms)");
  Serial.println("  Frecvență scriere: 5 scrieri/secundă (200ms)");
  Serial.println();
  Serial.println("Trimite TXT prin Serial pentru a scrie registrii.");
  Serial.println("Format TXT: D0=123, D1=456, M0=true, M70=false, etc. (un registru pe linie)");
  Serial.println();
  
  // Inițializare valori
  for (int i = 0; i < 16; i++) {
    regValues.D[i] = 0;
    regValues.D_valid[i] = false;
  }
  for (int i = 0; i < 18; i++) {
    regValues.M_valid[i] = false;
  }
  regValues.M0 = false;
  regValues.M70 = regValues.M71 = regValues.M72 = regValues.M73 = regValues.M74 = false;
  regValues.M75 = regValues.M76 = regValues.M77 = regValues.M78 = regValues.M79 = false;
  regValues.M93 = regValues.M94 = regValues.M95 = false;
  regValues.M97 = regValues.M98 = false;
  regValues.M120 = false;
  
  // Inițializare valori de scriere
  writeValues.hasData = false;
  writeValues.currentWriteIndex = 0;
  writeValues.writingD = true;
  for (int i = 0; i < 16; i++) {
    writeValues.D_write_enabled[i] = false;
    writeValues.D_write[i] = 0;
  }
  for (int i = 0; i < 18; i++) {
    writeValues.M_write_enabled[i] = false;
    writeValues.M_write[i] = false;
  }
  
  // Configurăm pinul RE/DE
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  
  // Pregătirea interfeței seriale hardware
  RTUutils::prepareHardwareSerial(Serial2);
  
  // Inițializarea interfeței seriale
  Serial2.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  delay(50);
  
  Serial.println("Serial2 inițializat.");
  
  // Setarea callback-urilor
  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);
  
  // Setarea timeout-ului (mărit pentru a permite procesarea mai lentă a PLC-ului)
  MB.setTimeout(5000);
  
  // Pornirea clientului Modbus
  MB.begin(Serial2);
  
  Serial.println("Modbus client inițializat cu succes!");
  Serial.println("Începem citirea registrilor...\n");
  
  // Inițializăm SPIFFS și citim fișierul TXT la pornire (dacă este specificat)
  if (strlen(TXT_FILENAME) > 0) {
    Serial.print(">>> Inițializare SPIFFS pentru citirea fișierului: ");
    Serial.println(TXT_FILENAME);
    if (SPIFFS.begin(true)) {
      Serial.println(">>> SPIFFS inițializat cu succes!");
      if (SPIFFS.exists(TXT_FILENAME)) {
        Serial.print(">>> Fișierul ");
        Serial.print(TXT_FILENAME);
        Serial.println(" există în SPIFFS.");
        
        // Citim fișierul la pornire pentru a inițializa lastFileModificationTime
        File file = SPIFFS.open(TXT_FILENAME, "r");
        if (file) {
          lastFileModificationTime = file.getLastWrite();
          file.close();
          Serial.println(">>> Fișierul va fi verificat automat pentru modificări la fiecare secundă.");
          Serial.println(">>> Modificările vor fi detectate și procesate automat!");
          
          // Citim și procesăm TXT-ul la pornire
          Serial.println(">>> Citire TXT la pornire...");
          readTXTFromFile();
        }
      } else {
        Serial.print(">>> ATENȚIE: Fișierul ");
        Serial.print(TXT_FILENAME);
        Serial.println(" NU există în SPIFFS!");
        Serial.println(">>> Folosește 'ESP32 Sketch Data Upload' pentru a încărca fișierul.");
      }
    } else {
      Serial.println(">>> EROARE: Nu s-a putut inițializa SPIFFS!");
    }
  } else {
    Serial.println(">>> Mod de citire: Serial Monitor (TXT_FILENAME este gol)");
  }
  Serial.println();
  
  lastReadTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Verificăm dacă fișierul TXT a fost modificat (doar dacă citim din fișier)
  if (strlen(TXT_FILENAME) > 0 && (currentTime - lastFileCheckTime >= fileCheckInterval)) {
    lastFileCheckTime = currentTime;
    
    if (checkFileModified()) {
      Serial.println("\n>>> FIȘIER TXT MODIFICAT DETECTAT! <<<");
      Serial.println(">>> Reîncărcare și procesare TXT... <<<");
      readTXTFromFile();
    }
  } else if (strlen(TXT_FILENAME) == 0) {
    // Dacă citim de la Serial, verificăm doar o dată
    readTXTFromFile();
  }
  
  // Procesăm scrierile din JSON (dacă există date și nu așteptăm răspuns)
  // Scriem un registru la fiecare 200ms (5 ori pe secundă)
  if (writeValues.hasData && !waitingForWriteResponse && (currentTime - lastWriteTime >= writeInterval)) {
    Serial.println("Procesăm scrierile din JSON...");
    processWriteOperations();
    lastWriteTime = currentTime;
  }
  
  // Verificăm dacă au trecut 200ms ȘI nu așteptăm un răspuns
  if (currentTime - lastReadTime >= readInterval && !waitingForResponse) {
    Error err = SUCCESS;
    ModbusMessage request;
    
    // Construim cererea în funcție de starea curentă
    switch (currentState) {
      case READ_D0_D10:
        // FC03: Read Holding Registers - D0-D10 (11 registri, adresa 0x1770)
        request.add(slaveID);
        request.add((uint8_t)0x03);  // FC03
        request.add((uint16_t)0x1770);  // Adresa de start
        request.add((uint16_t)11);  // Număr de registri
        break;
        
      case READ_D46_D48:
        // FC03: Read Holding Registers - D46-D48 (3 registri, adresa 0x179E)
        request.add(slaveID);
        request.add((uint8_t)0x03);  // FC03
        request.add((uint16_t)0x179E);  // Adresa de start
        request.add((uint16_t)3);  // Număr de registri
        break;
        
      case READ_D57_D58:
        // FC03: Read Holding Registers - D57-D58 (2 registri, adresa 0x17A9)
        request.add(slaveID);
        request.add((uint8_t)0x03);  // FC03
        request.add((uint16_t)0x17A9);  // Adresa de start
        request.add((uint16_t)2);  // Număr de registri
        break;
        
      case READ_M0:
        // FC01: Read Coils - M0 (adresa 0x7D0)
        request.add(slaveID);
        request.add((uint8_t)0x01);  // FC01
        request.add((uint16_t)0x7D0);  // Adresa de start
        request.add((uint16_t)1);  // Număr de coils
        break;
        
      case READ_M70_M79:
        // FC01: Read Coils - M70-M79 (10 coils, adresa 0x816)
        request.add(slaveID);
        request.add((uint8_t)0x01);  // FC01
        request.add((uint16_t)0x816);  // Adresa de start
        request.add((uint16_t)10);  // Număr de coils
        break;
        
      case READ_M93_M95:
        // FC01: Read Coils - M93-M95 (3 coils, adresa 0x82D)
        request.add(slaveID);
        request.add((uint8_t)0x01);  // FC01
        request.add((uint16_t)0x82D);  // Adresa de start
        request.add((uint16_t)3);  // Număr de coils
        break;
        
      case READ_M97_M98:
        // FC01: Read Coils - M97-M98 (2 coils, adresa 0x831)
        request.add(slaveID);
        request.add((uint8_t)0x01);  // FC01
        request.add((uint16_t)0x831);  // Adresa de start
        request.add((uint16_t)2);  // Număr de coils
        break;
        
      case READ_M120:
        // FC01: Read Coils - M120 (adresa 0x848)
        request.add(slaveID);
        request.add((uint8_t)0x01);  // FC01
        request.add((uint16_t)0x848);  // Adresa de start
        request.add((uint16_t)1);  // Număr de coils
        break;
    }
    
    // Control manual al pinului RE/DE pentru transmitere
    digitalWrite(RE_DE_PIN, HIGH);
    delay(2);
    
    // Trimitem cererea
    currentToken = millis();
    waitingForResponse = true;
    err = MB.addRequest(request, currentToken);
    
    // Așteptăm ca mesajul să fie trimis complet (mărit pentru stabilitate)
    delay(50);
    
    // Comutăm înapoi pe recepție
    digitalWrite(RE_DE_PIN, LOW);
    
    if (err != SUCCESS) {
      waitingForResponse = false;
      ModbusError e(err);
      Serial.print("EROARE la trimiterea cererii: 0x");
      Serial.print((int)e, HEX);
      Serial.print(" - ");
      Serial.println((const char *)e);
    }
    
    // Așteptăm răspunsul (timeout mărit pentru a permite procesarea mai lentă)
    unsigned long responseStart = millis();
    while (waitingForResponse && (millis() - responseStart < 6000)) {
      delay(20);  // Delay mai mare pentru a nu suprasolicita
    }
    
    if (waitingForResponse) {
      Serial.print("TIMEOUT pentru citirea: ");
      switch (currentState) {
        case READ_D0_D10: Serial.println("D0-D10"); break;
        case READ_D46_D48: Serial.println("D46-D48"); break;
        case READ_D57_D58: Serial.println("D57-D58"); break;
        case READ_M0: Serial.println("M0"); break;
        case READ_M70_M79: Serial.println("M70-M79"); break;
        case READ_M93_M95: Serial.println("M93-M95"); break;
        case READ_M97_M98: Serial.println("M97-M98"); break;
        case READ_M120: Serial.println("M120"); break;
      }
      waitingForResponse = false;
    }
    
    // Trecem la următoarea citire
    switch (currentState) {
      case READ_D0_D10: currentState = READ_D46_D48; break;
      case READ_D46_D48: currentState = READ_D57_D58; break;
      case READ_D57_D58: currentState = READ_M0; break;
      case READ_M0: currentState = READ_M70_M79; break;
      case READ_M70_M79: currentState = READ_M93_M95; break;
      case READ_M93_M95: currentState = READ_M97_M98; break;
      case READ_M97_M98: currentState = READ_M120; break;
      case READ_M120: 
        currentState = READ_D0_D10;  // Revenim la început
        // Afișăm toate valorile citite
        displayValues();
        // Delay suplimentar după afișare pentru a da timp PLC-ului
        delay(50);
        break;
    }
    
    // Delay între cereri pentru a da timp PLC-ului să proceseze
    delay(30);
    
    lastReadTime = currentTime;
  }
  
  delay(10);
}

// Funcție pentru afișarea valorilor citite
void displayValues() {
  Serial.println("\n========================================");
  Serial.println("  VALORI CITITE - " + String(millis() / 1000.0, 2) + "s");
  Serial.println("========================================");
  
  // Afișăm Data Registers
  Serial.println("\n--- DATA REGISTERS ---");
  Serial.println("D0 -D10:");
  for (int i = 0; i <= 10; i++) {
    if (regValues.D_valid[i]) {
      Serial.print("  D" + String(i) + " (0x" + String(0x1770 + i, HEX) + "): ");
      Serial.print(regValues.D[i]);
      Serial.print(" (0x");
      Serial.print(regValues.D[i], HEX);
      Serial.println(")");
    } else {
      Serial.println("  D" + String(i) + ": EROARE - valoare nevalidă");
    }
  }
  
  Serial.println("\nD46-D48:");
  for (int i = 46; i <= 48; i++) {
    int idx = 11 + (i - 46);
    if (regValues.D_valid[idx]) {
      Serial.print("  D" + String(i) + " (0x" + String(0x179E + (i - 46), HEX) + "): ");
      Serial.print(regValues.D[idx]);
      Serial.print(" (0x");
      Serial.print(regValues.D[idx], HEX);
      Serial.println(")");
    } else {
      Serial.println("  D" + String(i) + ": EROARE - valoare nevalidă");
    }
  }
  
  Serial.println("\nD57-D58:");
  for (int i = 57; i <= 58; i++) {
    int idx = 14 + (i - 57);
    if (regValues.D_valid[idx]) {
      Serial.print("  D" + String(i) + " (0x" + String(0x17A9 + (i - 57), HEX) + "): ");
      Serial.print(regValues.D[idx]);
      Serial.print(" (0x");
      Serial.print(regValues.D[idx], HEX);
      Serial.println(")");
    } else {
      Serial.println("  D" + String(i) + ": EROARE - valoare nevalidă");
    }
  }
  
  // Afișăm Discrete M Relays
  Serial.println("\n--- DISCRETE M RELAYS ---");
  
  if (regValues.M_valid[0]) {
    Serial.println();
    Serial.println(" M0   (0x7D0): " + String(regValues.M0 ? "ON" : "OFF"));
    Serial.println();
  } else {
    Serial.println("M0   (0x7D0): EROARE - valoare nevalidă");
  }
  
  Serial.print("M70-M79 (0x816-0x81F): ");
  bool allValid = true;
  for (int i = 1; i <= 10; i++) {
    if (!regValues.M_valid[i]) {
      allValid = false;
      break;
    }
  }
  if (allValid) {
    Serial.println();
    Serial.println(" M70=" + String(regValues.M70 ? "ON" : "OFF"));
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
  
  Serial.print("M93-M95 (0x82D-0x82F): ");
  allValid = true;
  for (int i = 11; i <= 13; i++) {
    if (!regValues.M_valid[i]) {
      allValid = false;
      break;
    }
  }
  if (allValid) {
    Serial.println();
    Serial.println(" M93=" + String(regValues.M93 ? "ON" : "OFF"));
    Serial.println(" M94=" + String(regValues.M94 ? "ON" : "OFF"));
    Serial.println(" M95=" + String(regValues.M95 ? "ON" : "OFF"));
    Serial.println();
  } else {
    Serial.println("EROARE - valori nevalide");
  }
  
  Serial.print("M97-M98 (0x831-0x832): ");
  allValid = true;
  for (int i = 14; i <= 15; i++) {
    if (!regValues.M_valid[i]) {
      allValid = false;
      break;
    }
  }
  if (allValid) {
    Serial.println();
    Serial.println(" M97=" + String(regValues.M97 ? "ON" : "OFF"));
    Serial.println(" M98=" + String(regValues.M98 ? "ON" : "OFF"));
    Serial.println();
  } else {
    Serial.println("EROARE - valori nevalide");
  }
  
  if (regValues.M_valid[17]) {
    Serial.println(" M120 (0x848): " + String(regValues.M120 ? "ON" : "OFF"));
  } else {
    Serial.println("M120 (0x848): EROARE - valoare nevalidă");
  }
  
  Serial.println("========================================\n");
}

