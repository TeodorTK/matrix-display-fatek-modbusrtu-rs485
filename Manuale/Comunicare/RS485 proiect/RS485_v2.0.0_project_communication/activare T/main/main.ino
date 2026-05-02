#include <Arduino.h>
#include "ModbusClientRTU.h"
#include "RTUutils.h"

// Definirea pinilor ESP32
#define RX_PIN 16
#define TX_PIN 17
#define RE_DE_PIN 22
#define BAUDRATE 9600

// Inițializarea clientului Modbus RTU
// Constructorul primește doar pinul RE/DE
ModbusClientRTU MB(RE_DE_PIN);

// Adresa slave-ului Modbus (modifică dacă este necesar)
// NOTĂ: Multe PLC-uri nu acceptă slave ID 0, încearcă cu 1
const uint8_t slaveID = 5;  // Schimbat de la 0 la 1

// Adresa coil-ului T0 (0x7D1)
const uint16_t coilAddress = 0x2328;

// Variabilă pentru starea coil-ului
bool coilState = false;

// Variabilă pentru timp
unsigned long lastToggleTime = 0;
const unsigned long toggleInterval = 2000; // 2 secunde

// Variabilă pentru a urmări dacă avem cereri în așteptare
bool waitingForResponse = false;
uint32_t lastRequestToken = 0;

// Callback pentru răspunsuri Modbus
void handleData(ModbusMessage response, uint32_t token) {
  waitingForResponse = false; // Răspuns primit, nu mai așteptăm
  Serial.println("\n>>> RĂSPUNS PRIMIT <<<");
  Serial.print("Token: 0x");
  Serial.print(token, HEX);
  Serial.print(", Server ID: ");
  Serial.print(response.getServerID());
  Serial.print(", Function Code: 0x");
  Serial.print(response.getFunctionCode(), HEX);
  Serial.print(" (FC05-Write)");
  Serial.print(", RE/DE pin: ");
  Serial.println(digitalRead(RE_DE_PIN) ? "HIGH" : "LOW");
  
  if (response.getError() == SUCCESS) {
    Serial.println(" - SUCCESS");
    
    // Dacă este o scriere de coil (FC05)
    if (response.getFunctionCode() == 0x05) {
      Serial.println(">>> Coil T0 scris cu succes!");
    }
  } else {
    Serial.print(" - ERROR: 0x");
    Serial.println(response.getError(), HEX);
  }
  Serial.println();
}

// Callback pentru erori
void handleError(Error error, uint32_t token) {
  waitingForResponse = false; // Eroare primită, nu mai așteptăm
  // ModbusError wraps the error code and provides a readable error message for it
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
  Serial.println("\n=== ESP32 Modbus RTU Client - T0 ===");
  Serial.println("Configurare Modbus RTU:");
  Serial.println("  Baudrate: 9600");
  Serial.println("  Parity: None (0)");
  Serial.println("  Data bits: 8");
  Serial.println("  Stop bits: 1");
  Serial.println("  Format: 8N1 (conform programului MicroPython)");
  Serial.print("  RX Pin: ");
  Serial.println(RX_PIN);
  Serial.print("  TX Pin: ");
  Serial.println(TX_PIN);
  Serial.print("  RE/DE Pin: ");
  Serial.println(RE_DE_PIN);
  Serial.print("  Slave ID: ");
  Serial.println(slaveID);
  Serial.print("  Coil Address T0: 0x");
  Serial.println(coilAddress, HEX);
  Serial.println();
  Serial.println("NOTĂ: Dacă primești timeout, verifică:");
  Serial.println("  1. Slave ID-ul PLC-ului (acum: " + String(slaveID) + ")");
  Serial.println("  2. Adresa coil-ului T0 (acum: 0x" + String(coilAddress, HEX) + ")");
  Serial.println("  3. Conexiunile RS485 (A, B, GND)");
  Serial.println("  4. Parametrii seriali (9600, 8N1 - conform MicroPython)");
  Serial.println();

  // Configurăm pinul RE/DE manual (biblioteca nu îl comută automat)
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);  // Start în modul receive
  
  // Pregătirea interfeței seriale hardware (IMPORTANT: înainte de begin!)
  RTUutils::prepareHardwareSerial(Serial2);

  // Inițializarea interfeței seriale cu parametrii specificați
  // SERIAL_8N1 = 8 data bits, No parity, 1 stop bit (conform programului MicroPython)
  // Programul MicroPython folosește: parity=0 (No parity), data_bits=8
  Serial2.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  
  // Așteptăm puțin pentru inițializare
  delay(50);
  
  Serial.println("Serial2 inițializat.");

  // Setarea callback-urilor
  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);

  // Setarea timeout-ului pentru clientul Modbus (în ms)
  MB.setTimeout(2000);

  // Pornirea clientului Modbus - trebuie să treacă Serial2
  MB.begin(Serial2);

  Serial.println("Modbus client inițializat cu succes!");
  Serial.println("Începem să activăm/dezactivăm coil-ul T0 (0x7D1)...\n");
  
  lastToggleTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Verificăm dacă au trecut 2 secunde ȘI nu așteptăm un răspuns
  if (currentTime - lastToggleTime >= toggleInterval && !waitingForResponse) {
    // Inversăm starea coil-ului
    coilState = !coilState;
    
    Serial.print("=== ");
    Serial.print(coilState ? "Activare" : "Dezactivare");
    Serial.print(" coil T0 (0x7D1) ===");
    Serial.println();
    
    // Construim cererea FC05 (Write Single Coil)
    Serial.print("FC05 Request: Slave=");
    Serial.print(slaveID);
    Serial.print(", Addr=0x");
    Serial.print(coilAddress, HEX);
    Serial.print(", Value=");
    Serial.println(coilState ? "ON(0xFF00)" : "OFF(0x0000)");
    
    // Construim mesajul Modbus manual
    ModbusMessage writeRequest;
    writeRequest.add(slaveID);        // Slave ID - primul byte
    writeRequest.add((uint8_t)0x05);  // Function Code FC05
    writeRequest.add(coilAddress);     // Adresa coil-ului (2 bytes)
    // Pentru Write Single Coil: 0xFF00 = ON, 0x0000 = OFF
    writeRequest.add(coilState ? (uint16_t)0xFF00 : (uint16_t)0x0000);
    
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
      Serial.println("Cerere FC05 trimisă, așteptăm răspuns...");
    }
    
    // Așteptăm răspunsul (timeout-ul este 2000ms)
    unsigned long responseStart = millis();
    while (waitingForResponse && (millis() - responseStart < 2500)) {
      delay(10);
    }
    
    if (waitingForResponse) {
      Serial.println("Timeout - nu s-a primit răspuns pentru FC05");
      waitingForResponse = false;
    }
    
    Serial.println();
    
    // Actualizăm timpul ultimului toggle
    lastToggleTime = currentTime;
  }
  
  // Mic delay pentru a nu suprasolicita procesorul
  delay(10);
}
