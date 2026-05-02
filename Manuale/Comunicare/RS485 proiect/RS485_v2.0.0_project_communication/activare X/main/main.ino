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
const uint8_t slaveID = 5;

// Adresa primului input X (X0 = 0x3E9)
// NOTĂ: În Modbus, adresele pot fi 0-based sau 1-based
// Dacă 0x3E9 nu funcționează, încearcă: 0x3E8, 0x0000, sau 0x0001
const uint16_t inputAddress = 0x3E8;

// Numărul de input-uri de citit (poți modifica dacă vrei să citești mai multe)
const uint16_t numInputs = 1;

// Variabilă pentru starea input-ului
bool inputState = false;

// Variabilă pentru timp
unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000; // 1 secundă

// Variabilă pentru a urmări dacă avem cereri în așteptare
bool waitingForResponse = false;
uint32_t lastRequestToken = 0;

// Callback pentru răspunsuri Modbus
void handleData(ModbusMessage response, uint32_t token) {
  waitingForResponse = false;
  
  Serial.print("[DEBUG] Răspuns primit - Token: 0x");
  Serial.print(token, HEX);
  Serial.print(", Size: ");
  Serial.print(response.size());
  Serial.print(", Server ID: ");
  Serial.print(response.getServerID());
  Serial.print(", Function Code: 0x");
  Serial.print(response.getFunctionCode(), HEX);
  Serial.print(", Error: ");
  Serial.println(response.getError() == SUCCESS ? "SUCCESS" : "ERROR");
  
  if (response.getError() == SUCCESS) {
    Serial.print("[DEBUG] Răspuns SUCCESS - ");
    Serial.print("Function Code: 0x");
    Serial.print(response.getFunctionCode(), HEX);
    Serial.print(", Size: ");
    Serial.println(response.size());
    
    // Verificăm că răspunsul este de tipul așteptat (FC01 - Read Coils)
    if (response.getFunctionCode() == 0x01) {
      Serial.println("[DEBUG] Function Code corect (FC01)");
      
      if (response.size() > 2) {
        uint8_t byteCount = response[2];
        Serial.print("[DEBUG] Byte Count: ");
        Serial.println(byteCount);
        
        // Citim primul byte de date (conține starea primelor 8 input-uri)
        if (byteCount > 0 && response.size() > 3) {
          uint8_t dataByte = response[3];
          Serial.print("[DEBUG] Data Byte: 0x");
          if (dataByte < 0x10) Serial.print("0");
          Serial.print(dataByte, HEX);
          Serial.print(" (0b");
          for (int i = 7; i >= 0; i--) {
            Serial.print((dataByte >> i) & 0x01);
          }
          Serial.print(")");
          
          // Bitul 0 (LSB) reprezintă starea primului input
          inputState = (dataByte & 0x01) != 0;
          
          Serial.print(" -> Input State: ");
          Serial.println(inputState ? "ACTIV" : "INACTIV");
          
          // Afișăm starea portului
          Serial.println(inputState ? "ACTIV" : "INACTIV");
        } else {
          Serial.print("[DEBUG] EROARE: Dimensiune insuficientă - Size: ");
          Serial.print(response.size());
          Serial.print(", ByteCount: ");
          Serial.println(byteCount);
        }
      } else {
        Serial.print("[DEBUG] EROARE: Răspuns prea scurt - Size: ");
        Serial.println(response.size());
      }
    } else {
      Serial.print("[DEBUG] EROARE: Function Code greșit - Așteptat: 0x01, Primit: 0x");
      Serial.println(response.getFunctionCode(), HEX);
    }
  } else {
    Serial.print("[DEBUG] EROARE în răspuns: 0x");
    Serial.println(response.getError(), HEX);
  }
  
  Serial.println();
  
  // Actualizăm timpul după primirea răspunsului pentru interval precis
  lastReadTime = millis();
}

// Callback pentru erori
void handleError(Error error, uint32_t token) {
  waitingForResponse = false;
  
  ModbusError me(error);
  Serial.print("[DEBUG] EROARE Modbus - Token: 0x");
  Serial.print(token, HEX);
  Serial.print(", Error Code: 0x");
  Serial.print((int)me, HEX);
  Serial.print(" - ");
  Serial.println((const char *)me);
  
  // Interpretare erori comune
  if ((int)me == 0xE2) {
    Serial.println("[DEBUG] EROARE CRC - Verifică:");
    Serial.println("  1. Adresa input-ului (acum: 0x3E9)");
    Serial.println("  2. Slave ID (acum: 5)");
    Serial.println("  3. Conexiunile RS485 (A, B, GND)");
    Serial.println("  4. Parametrii seriali (9600, 8E1)");
    Serial.println("[DEBUG] Sugestie: Încearcă adresa 0x0000 sau 0x0001");
  } else if ((int)me == 0x02) {
    Serial.println("[DEBUG] ILLEGAL DATA ADDRESS - Adresa nu este validă!");
    Serial.println("[DEBUG] Verifică adresa input-ului X0 în documentația PLC-ului");
  }
  Serial.println();
  
  // Actualizăm timpul și la eroare pentru a continua ciclul
  lastReadTime = millis();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println("\n=== DEBUG MODE ACTIVAT ===");
  Serial.println("[DEBUG] Inițializare ESP32 Modbus RTU Client");
  Serial.print("[DEBUG] Slave ID: ");
  Serial.println(slaveID);
  Serial.print("[DEBUG] Input Address: 0x");
  Serial.println(inputAddress, HEX);
  Serial.print("[DEBUG] RX Pin: ");
  Serial.println(RX_PIN);
  Serial.print("[DEBUG] TX Pin: ");
  Serial.println(TX_PIN);
  Serial.print("[DEBUG] RE/DE Pin: ");
  Serial.println(RE_DE_PIN);
  Serial.print("[DEBUG] Baudrate: ");
  Serial.println(BAUDRATE);
  Serial.println();

  // Configurăm pinul RE/DE manual
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  Serial.println("[DEBUG] Pin RE/DE configurat (LOW - Receive mode)");
  
  // Pregătirea interfeței seriale hardware
  RTUutils::prepareHardwareSerial(Serial2);
  Serial.println("[DEBUG] Serial2 hardware pregătit");

  // Inițializarea interfeței seriale
  Serial2.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  delay(50);
  Serial.println("[DEBUG] Serial2 inițializat (8E1)");

  // Setarea callback-urilor
  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);
  Serial.println("[DEBUG] Callback-uri setate");

  // Setarea timeout-ului pentru clientul Modbus
  MB.setTimeout(1000);
  Serial.println("[DEBUG] Timeout setat: 1000ms");

  // Pornirea clientului Modbus
  MB.begin(Serial2);
  Serial.println("[DEBUG] Modbus client pornit");
  Serial.println("[DEBUG] Încep citirea input-ului X0...\n");
  
  lastReadTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= readInterval && !waitingForResponse) {
    Serial.print("[DEBUG] === Trimite cerere FC01 ===");
    Serial.print(" (Timp: ");
    Serial.print(millis());
    Serial.println("ms)");
    
    // Construim mesajul Modbus pentru citire
    ModbusMessage readRequest;
    readRequest.add(slaveID);
    readRequest.add((uint8_t)0x01);  // Function Code FC01 (Read Coils)
    readRequest.add((uint16_t)inputAddress);
    readRequest.add((uint16_t)numInputs);
    
    Serial.print("[DEBUG] Cerere construită - Slave: ");
    Serial.print(slaveID);
    Serial.print(", FC: 0x01, Addr: 0x");
    Serial.print(inputAddress, HEX);
    Serial.print(" (");
    Serial.print(inputAddress);
    Serial.print(" dec), Qty: ");
    Serial.println(numInputs);
    
    // Debug: afișăm bytes-ii mesajului trimis
    Serial.print("[DEBUG] Mesaj trimis (hex): ");
    for (size_t i = 0; i < readRequest.size(); i++) {
      if (readRequest[i] < 0x10) Serial.print("0");
      Serial.print(readRequest[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // Control manual al pinului RE/DE pentru transmitere
    digitalWrite(RE_DE_PIN, HIGH);
    delay(5);  // Delay pentru stabilizare înainte de transmitere
    Serial.println("[DEBUG] RE/DE setat pe HIGH (Transmit mode)");
    
    // Trimitem cererea de citire
    lastRequestToken = millis();
    waitingForResponse = true;
    Error err = MB.addRequest(readRequest, lastRequestToken);
    
    delay(20);  // Delay pentru transmiterea completă a datelor către PLC
    
    // Comutăm înapoi pe recepție
    digitalWrite(RE_DE_PIN, LOW);
    delay(5);  // Delay pentru stabilizare înainte de recepție
    Serial.println("[DEBUG] RE/DE setat pe LOW (Receive mode)");
    
    if (err != SUCCESS) {
      waitingForResponse = false;
      ModbusError me(err);
      Serial.print("[DEBUG] EROARE la trimiterea cererii: 0x");
      Serial.print((int)me, HEX);
      Serial.print(" - ");
      Serial.println((const char *)me);
      lastReadTime = millis();
    } else {
      Serial.print("[DEBUG] Cerere trimisă, Token: 0x");
      Serial.print(lastRequestToken, HEX);
      Serial.println(", Aștept răspuns...");
      
      // Așteptăm răspunsul
      unsigned long responseStart = millis();
      while (waitingForResponse && (millis() - responseStart < 1500)) {
        delay(10);
      }
      
      // Dacă timeout fără răspuns
      if (waitingForResponse) {
        Serial.print("[DEBUG] TIMEOUT - Nu s-a primit răspuns după ");
        Serial.print(millis() - responseStart);
        Serial.println("ms");
        waitingForResponse = false;
        lastReadTime = millis();
      }
    }
    
    Serial.println();
  }
  
  delay(10);
}
