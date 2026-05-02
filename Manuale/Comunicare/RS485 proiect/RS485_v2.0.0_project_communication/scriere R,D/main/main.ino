#include <Arduino.h>
#include "ModbusClientRTU.h"
#include "RTUutils.h"

// Definirea pinilor ESP32
#define RX_PIN 16
#define TX_PIN 17
#define RE_DE_PIN 22
#define BAUDRATE 9600

// Inițializarea clientului Modbus RTU
ModbusClientRTU MB(RE_DE_PIN);

// Adresa slave-ului Modbus
const uint8_t slaveID = 5;

// Adresa registrului R0 (0x0 = 40000)
// Adresa registrului D0 (0x1770 = 46000)
const uint16_t registerAddress = 0x1775;

// Valoarea de scris în registru (1234)
const uint16_t registerValue = 100;

// Variabilă pentru a urmări dacă avem cereri în așteptare
bool waitingForResponse = false;
uint32_t lastRequestToken = 0;

// Variabilă pentru timp
unsigned long lastWriteTime = 0;
const unsigned long writeInterval = 2000; // 2 secunde

// Callback pentru răspunsuri Modbus
void handleData(ModbusMessage response, uint32_t token) {
  waitingForResponse = false;
  
  Serial.println("\n>>> RĂSPUNS PRIMIT <<<");
  Serial.print("Token: 0x");
  Serial.print(token, HEX);
  Serial.print(", Server ID: ");
  Serial.print(response.getServerID());
  Serial.print(", Function Code: 0x");
  Serial.print(response.getFunctionCode(), HEX);
  Serial.print(" (FC06-Write Single Register)");
  Serial.print(", RE/DE pin: ");
  Serial.println(digitalRead(RE_DE_PIN) ? "HIGH" : "LOW");
  
  if (response.getError() == SUCCESS) {
    Serial.println(" - SUCCESS");
    
    // Dacă este o scriere de registru (FC06)
    if (response.getFunctionCode() == 0x06) {
      Serial.println(">>> Registrul R0 (0x9C40) a fost scris cu succes!");
      Serial.print(">>> Valoare scrisă: ");
      Serial.println(registerValue);
    }
  } else {
    Serial.print(" - ERROR: 0x");
    Serial.println(response.getError(), HEX);
  }
  Serial.println();
}

// Callback pentru erori
void handleError(Error error, uint32_t token) {
  waitingForResponse = false;
  
  ModbusError me(error);
  Serial.print("Error response: 0x");
  Serial.print((int)me, HEX);
  Serial.print(" - ");
  Serial.print((const char *)me);
  Serial.print(", Token: 0x");
  Serial.print(token, HEX);
  Serial.print(", RE/DE pin: ");
  Serial.println(digitalRead(RE_DE_PIN) ? "HIGH" : "LOW");
}

void setup() {
  // Inițializarea Serial pentru debug
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("__ OK __");
  Serial.println("\n=== ESP32 Modbus RTU Client - Scriere Registru R0 ===");
  Serial.println("Configurare Modbus RTU:");
  Serial.println("  Baudrate: 9600");
  Serial.println("  Parity: None (0)");
  Serial.println("  Data bits: 8");
  Serial.println("  Stop bits: 1");
  Serial.println("  Format: 8E1");
  Serial.print("  RX Pin: ");
  Serial.println(RX_PIN);
  Serial.print("  TX Pin: ");
  Serial.println(TX_PIN);
  Serial.print("  RE/DE Pin: ");
  Serial.println(RE_DE_PIN);
  Serial.print("  Slave ID: ");
  Serial.println(slaveID);
  Serial.print("  Register Address R0: 0x");
  Serial.print(registerAddress, HEX);
  Serial.print(" (");
  Serial.print(registerAddress);
  Serial.println(" dec)");
  Serial.print("  Valoare de scris: ");
  Serial.println(registerValue);
  Serial.println();
  Serial.println("NOTĂ: Dacă primești timeout, verifică:");
  Serial.println("  1. Slave ID-ul PLC-ului (acum: " + String(slaveID) + ")");
  Serial.println("  2. Adresa registrului R0 (acum: 0x" + String(registerAddress, HEX) + ")");
  Serial.println("  3. Conexiunile RS485 (A, B, GND)");
  Serial.println("  4. Parametrii seriali (9600, 8N1)");
  Serial.println();

  // Configurăm pinul RE/DE manual
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);  // Start în modul receive
  
  // Pregătirea interfeței seriale hardware
  RTUutils::prepareHardwareSerial(Serial2);

  // Inițializarea interfeței seriale cu parametrii specificați
  Serial2.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  
  // Așteptăm puțin pentru inițializare
  delay(50);
  
  Serial.println("Serial2 inițializat.");

  // Setarea callback-urilor
  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);

  // Setarea timeout-ului pentru clientul Modbus (în ms)
  MB.setTimeout(2000);

  // Pornirea clientului Modbus
  MB.begin(Serial2);

  Serial.println("Modbus client inițializat cu succes!");
  Serial.println("Începem să scriem valoarea 1234 în registrul R0 (0x9C40)...\n");
  
  lastWriteTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Verificăm dacă au trecut 2 secunde ȘI nu așteptăm un răspuns
  if (currentTime - lastWriteTime >= writeInterval && !waitingForResponse) {
    Serial.print("=== Scriere în registrul R0 (0x9C40) ===");
    Serial.println();
    
    // Construim cererea FC06 (Write Single Holding Register)
    Serial.print("FC06 Request: Slave=");
    Serial.print(slaveID);
    Serial.print(", Addr=0x");
    Serial.print(registerAddress, HEX);
    Serial.print(" (");
    Serial.print(registerAddress);
    Serial.print(" dec), Value=");
    Serial.println(registerValue);
    
    // Construim mesajul Modbus pentru scriere
    ModbusMessage writeRequest;
    writeRequest.add(slaveID);        // Slave ID
    writeRequest.add((uint8_t)0x06);  // Function Code FC06 (Write Single Holding Register)
    writeRequest.add(registerAddress); // Adresa registrului (2 bytes, big-endian)
    writeRequest.add(registerValue);   // Valoarea de scris (2 bytes, big-endian)
    
    // Control manual al pinului RE/DE pentru transmitere
    digitalWrite(RE_DE_PIN, HIGH);  // Setăm pe transmitere
    delay(2);  // Mic delay pentru stabilizare
    
    // Trimitem cererea de scriere
    lastRequestToken = millis();
    waitingForResponse = true;
    Error err = MB.addRequest(writeRequest, lastRequestToken);
    
    // Așteptăm ca mesajul să fie trimis complet
    delay(15);  // Delay pentru a permite transmiterea completă
    
    // Comutăm înapoi pe recepție
    digitalWrite(RE_DE_PIN, LOW);  // Setăm pe recepție
    
    if (err != SUCCESS) {
      waitingForResponse = false;
      ModbusError e(err);
      Serial.print("Eroare la adăugarea cererii de scriere: 0x");
      Serial.print((int)e, HEX);
      Serial.print(" - ");
      Serial.println((const char *)e);
    } else {
      Serial.println("Cerere FC06 trimisă, așteptăm răspuns...");
    }
    
    // Așteptăm răspunsul (timeout-ul este 2000ms)
    unsigned long responseStart = millis();
    while (waitingForResponse && (millis() - responseStart < 2500)) {
      delay(10);
    }
    
    if (waitingForResponse) {
      Serial.println("Timeout - nu s-a primit răspuns pentru FC06");
      waitingForResponse = false;
    }
    
    Serial.println();
    
    // Actualizăm timpul ultimei scrieri
    lastWriteTime = currentTime;
  }
  
  // Mic delay pentru a nu suprasolicita procesorul
  delay(10);
}
