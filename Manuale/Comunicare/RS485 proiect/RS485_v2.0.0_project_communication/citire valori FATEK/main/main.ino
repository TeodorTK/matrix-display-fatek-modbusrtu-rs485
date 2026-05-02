#include <Arduino.h>
#include "ModbusClientRTU.h"
#include "RTUutils.h"

// Definirea pinilor ESP32
#define RX_PIN 16
#define TX_PIN 17
#define RE_DE_PIN 4
#define BAUDRATE 115200

// Inițializarea clientului Modbus RTU
ModbusClientRTU MB(RE_DE_PIN);

// Adresa slave-ului Modbus
const uint8_t slaveID = 1;

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

// Variabile pentru gestionarea citirilor
bool waitingForResponse = false;
uint32_t currentToken = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 250; // 100ms = 10 citiri/secundă

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
  waitingForResponse = false;
  
  if (response.getError() != SUCCESS) {
    Serial.print("EROARE în răspuns: 0x");
    Serial.print(response.getError(), HEX);
    Serial.print(" - ");
    ModbusError me(response.getError());
    Serial.println((const char *)me);
    return;
  }
  
  uint8_t fc = response.getFunctionCode();
  
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
  
  ModbusError me(error);
  Serial.print("EROARE Modbus: 0x");
  Serial.print((int)me, HEX);
  Serial.print(" - ");
  Serial.println((const char *)me);
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
  Serial.println("  Frecvență citire: 10 citiri/secundă (100ms)");
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
  
  lastReadTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
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
