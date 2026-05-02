#include <Arduino.h>
#include <ModbusRTU.h>   // eModbus (https://github.com/emelianov/modbus-esp8266) adaptat pentru ESP32

// -------------------- Constante adresare Fatek --------------------
static const uint16_t Y0 = 0x0000;  // Coil Y0 - output boolean

// -------------------- UART & RS485 pinout --------------------
#define MOD_TX_PIN      17
#define MOD_RX_PIN      16
#define MOD_DE_RE_PIN   22

// -------------------- Parametri Modbus --------------------
#define FATEK_SLAVE_ID  1      // ID-ul dispozitivului Fatek pe magistrala Modbus
#define MODBUS_BAUDRATE 9600   // Baudrate pentru comunicare cu Fatek

// -------------------- Obiect Modbus Master --------------------
ModbusRTU fatekMaster;

// -------------------- Variabile pentru control Y0 --------------------
bool y0_state = false;                    // Starea curentă a Y0
unsigned long lastToggleTime = 0;         // Timpul ultimei comutări
const unsigned long TOGGLE_INTERVAL_MS = 3000;  // Interval de 3 secunde

// Variabile pentru verificare conexiune
bool fatek_connected = false;             // Status conexiune Fatek
unsigned long lastConnectionCheck = 0;     // Timpul ultimei verificări
const unsigned long CONNECTION_CHECK_INTERVAL_MS = 5000;  // Verifică conexiunea la fiecare 5 secunde

// -------------------- Funcție verificare conexiune Fatek --------------------
bool checkFatekConnection() {
  // Încearcă să citească un registru intern Fatek pentru a verifica conexiunea
  // Folosim R0 (Holding Register) - un registru care există întotdeauna în Fatek
  // Alternativ, putem citi un coil intern sau un registru D0
  
  uint16_t testValue = 0;
  
  // Metoda 1: Încearcă să citească un Holding Register (R0)
  // R0 este la adresa Modbus 0x0000 pentru holding registers
  bool success = fatekMaster.readHreg(FATEK_SLAVE_ID, 0x0000, &testValue, 1);
  
  // Metoda 2 (alternativă): Citește un coil intern (M0) dacă metoda 1 nu funcționează
  // bool coilValue = false;
  // bool success = fatekMaster.readCoil(FATEK_SLAVE_ID, 0x07D8, &coilValue, 1);  // M0 = 0x07D8
  
  if (success) {
    if (!fatek_connected) {
      // Conexiunea s-a stabilit acum
      Serial.println(">>> Fatek CONNECTED! Communication OK.");
      fatek_connected = true;
    }
    return true;
  } else {
    if (fatek_connected) {
      // Conexiunea s-a pierdut
      Serial.println(">>> Fatek DISCONNECTED! Communication FAILED.");
      fatek_connected = false;
    }
    return false;
  }
}

// -------------------- setup() --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 Modbus RTU - Fatek Y0 Control...");

  // Configurare UART pentru comunicare cu Fatek
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, MOD_RX_PIN, MOD_TX_PIN);
  
  // Inițializare ModbusRTU cu control automat DE/RE
  // begin(&Serial, DE_RE_PIN) gestionează automat pinul DE/RE
  fatekMaster.begin(&Serial2, MOD_DE_RE_PIN);
  fatekMaster.master();  // Configurează ca master Modbus

  Serial.println("Initialization complete.");
  Serial.printf("Fatek Slave ID: %d, Y0 Address: 0x%04X, Interval: %lu ms\n", 
                FATEK_SLAVE_ID, Y0, TOGGLE_INTERVAL_MS);
  
  // Verifică conexiunea cu Fatek la pornire
  Serial.println("\n>>> Checking Fatek connection...");
  delay(500);  // Mic delay pentru stabilizare
  
  if (checkFatekConnection()) {
    Serial.println(">>> Fatek connection verified! Starting Y0 toggle...\n");
  } else {
    Serial.println(">>> WARNING: Could not connect to Fatek!");
    Serial.println(">>> Please check:");
    Serial.println("    - RS485 wiring (TX, RX, GND, DE/RE)");
    Serial.println("    - Fatek Slave ID (currently set to " + String(FATEK_SLAVE_ID) + ")");
    Serial.println("    - Baudrate settings (currently " + String(MODBUS_BAUDRATE) + ")");
    Serial.println("    - Power supply to Fatek");
    Serial.println(">>> Will continue trying to connect...\n");
  }
}

// -------------------- loop() --------------------
void loop() {
  unsigned long now = millis();

  // Verifică periodic conexiunea cu Fatek (la fiecare 5 secunde)
  if (now - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL_MS) {
    lastConnectionCheck = now;
    checkFatekConnection();
  }

  // Verifică dacă a trecut intervalul de 3 secunde pentru comutare Y0
  if (now - lastToggleTime >= TOGGLE_INTERVAL_MS) {
    lastToggleTime = now;

    // Verifică conexiunea înainte de a comuta Y0
    if (fatek_connected) {
      // Comută starea Y0
      y0_state = !y0_state;

      // Scrie starea în coil-ul Y0
      // Biblioteca gestionează automat DE/RE când folosești begin(&Serial, DE_RE_PIN)
      bool success = fatekMaster.writeCoil(FATEK_SLAVE_ID, Y0, y0_state);

      // Afișează rezultatul
      if (success) {
        Serial.printf("[%lu ms] Y0 set to: %s (SUCCESS)\n", now, y0_state ? "ON (1)" : "OFF (0)");
      } else {
        Serial.printf("[%lu ms] Y0 set to: %s (FAILED - write error)\n", now, y0_state ? "ON (1)" : "OFF (0)");
        // Dacă scrierea eșuează, verifică din nou conexiunea
        fatek_connected = false;
      }
    } else {
      // Nu este conectat - afișează mesaj de așteptare
      Serial.printf("[%lu ms] Y0 toggle skipped - Fatek not connected\n", now);
    }
  }

  // Procesează task-urile Modbus (dacă este necesar)
  // fatekMaster.task();
  
  yield();  // Permite altor task-uri să ruleze
}
