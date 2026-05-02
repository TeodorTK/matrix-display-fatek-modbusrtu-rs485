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
  uint16_t R[6];  // R0-R5
  bool R_valid[6];
  
  uint16_t D[6];  // D0-D5
  bool D_valid[6];
};

RegisterValues regValues;

// Variabile pentru gestionarea citirilor
bool waitingForResponse = false;
uint32_t currentToken = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 250; // 250ms între citiri

// Index pentru a ști ce citim acum
enum ReadState {
  READ_R0_R5,      // Citim R0-R5 (6 registri, adresa 0x0)
  READ_D0_D5       // Citim D0-D5 (6 registri, adresa 0x1770)
};

ReadState currentState = READ_R0_R5;

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
  
  // Procesăm răspunsul pentru funcția Modbus 0x03 (Read Holding Registers)
  if (fc == 0x03) {
    uint8_t byteCount = response[2];
    uint8_t dataIndex = 3;
    
    if (currentState == READ_R0_R5) {
      // Citim 6 registri (R0-R5)
      for (int i = 0; i < 6 && dataIndex + 1 < response.size(); i++) {
        regValues.R[i] = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.R_valid[i] = true;
        dataIndex += 2;
      }
    } else if (currentState == READ_D0_D5) {
      // Citim 6 registri (D0-D5)
      for (int i = 0; i < 6 && dataIndex + 1 < response.size(); i++) {
        regValues.D[i] = (response[dataIndex] << 8) | response[dataIndex + 1];
        regValues.D_valid[i] = true;
        dataIndex += 2;
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
  Serial.println("\n=== ESP32 Modbus RTU Client - Citire Registri R și D ===");
  Serial.println("Configurare Modbus RTU:");
  Serial.println("  Baudrate: 9600");
  Serial.println("  Parity: Even (8E1)");
  Serial.println("  RX Pin: 16");
  Serial.println("  TX Pin: 17");
  Serial.println("  RE/DE Pin: 4");
  Serial.print("  Slave ID: ");
  Serial.println(slaveID);
  Serial.println();
  
  // Inițializare valori
  for (int i = 0; i < 6; i++) {
    regValues.R[i] = 0;
    regValues.R_valid[i] = false;
    regValues.D[i] = 0;
    regValues.D_valid[i] = false;
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
  
  // Setarea timeout-ului
  MB.setTimeout(5000);
  
  // Pornirea clientului Modbus
  MB.begin(Serial2);
  
  Serial.println("Modbus client inițializat cu succes!");
  Serial.println("Începem citirea registrilor...\n");
  
  lastReadTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Verificăm dacă au trecut intervalul ȘI nu așteptăm un răspuns
  if (currentTime - lastReadTime >= readInterval && !waitingForResponse) {
    Error err = SUCCESS;
    ModbusMessage request;
    
    // Construim cererea în funcție de starea curentă
    switch (currentState) {
      case READ_R0_R5:
        // FC03: Read Holding Registers - R0-R5 (6 registri, adresa 0x0 = 40000)
        request.add(slaveID);
        request.add((uint8_t)0x03);  // FC03
        request.add((uint16_t)0x0000);  // Adresa de start (0x0)
        request.add((uint16_t)6);  // Număr de registri
        break;
        
      case READ_D0_D5:
        // FC03: Read Holding Registers - D0-D5 (6 registri, adresa 0x1770 = 46000)
        request.add(slaveID);
        request.add((uint8_t)0x03);  // FC03
        request.add((uint16_t)0x1770);  // Adresa de start (0x1770)
        request.add((uint16_t)6);  // Număr de registri
        break;
    }
    
    // Control manual al pinului RE/DE pentru transmitere
    digitalWrite(RE_DE_PIN, HIGH);
    delay(2);
    
    // Trimitem cererea
    currentToken = millis();
    waitingForResponse = true;
    err = MB.addRequest(request, currentToken);
    
    // Așteptăm ca mesajul să fie trimis complet
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
      delay(20);
    }
    
    if (waitingForResponse) {
      Serial.print("TIMEOUT pentru citirea: ");
      switch (currentState) {
        case READ_R0_R5: Serial.println("R0-R5"); break;
        case READ_D0_D5: Serial.println("D0-D5"); break;
      }
      waitingForResponse = false;
    }
    
    // Trecem la următoarea citire
    switch (currentState) {
      case READ_R0_R5:
        currentState = READ_D0_D5;
        break;
      case READ_D0_D5:
        currentState = READ_R0_R5;  // Revenim la început
        // Afișăm toate valorile citite
        displayValues();
        // Delay suplimentar după afișare
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
  
  // Afișăm Registri R
  Serial.println("\n--- REGISTRI R (adresa 0x0 = 40000) ---");
  for (int i = 0; i < 6; i++) {
    if (regValues.R_valid[i]) {
      Serial.print("  R" + String(i) + " (0x" + String(0x0 + i, HEX) + "): ");
      Serial.print(regValues.R[i]);
      Serial.print(" (0x");
      Serial.print(regValues.R[i], HEX);
      Serial.println(")");
    } else {
      Serial.println("  R" + String(i) + ": EROARE - valoare nevalidă");
    }
  }
  
  // Afișăm Registri D
  Serial.println("\n--- REGISTRI D (adresa 0x1770 = 46000) ---");
  for (int i = 0; i < 6; i++) {
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
  
  Serial.println("========================================\n");
}
