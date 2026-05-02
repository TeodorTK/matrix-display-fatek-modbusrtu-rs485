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
#define MODBUS_BAUDRATE 115200   // Baudrate pentru comunicare cu Fatek

// -------------------- Obiect Modbus Master --------------------
ModbusRTU fatekMaster;

// -------------------- Variabile pentru control Y0 --------------------
bool y0_state = false;                    // Starea curentă a Y0
unsigned long lastToggleTime = 0;         // Timpul ultimei comutări
const unsigned long TOGGLE_INTERVAL_MS = 3000;  // Interval de 3 secunde

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

  Serial.println("Initialization complete. Starting Y0 toggle...");
  Serial.printf("Fatek Slave ID: %d, Y0 Address: 0x%04X, Interval: %lu ms\n", 
                FATEK_SLAVE_ID, Y0, TOGGLE_INTERVAL_MS);
}

// -------------------- loop() --------------------
void loop() {
  unsigned long now = millis();

  // Verifică dacă a trecut intervalul de 3 secunde
  if (now - lastToggleTime >= TOGGLE_INTERVAL_MS) {
    lastToggleTime = now;

    // Comută starea Y0
    y0_state = !y0_state;

    // Scrie starea în coil-ul Y0
    // Biblioteca gestionează automat DE/RE când folosești begin(&Serial, DE_RE_PIN)
    bool success = fatekMaster.writeCoil(FATEK_SLAVE_ID, Y0, y0_state);

    // Afișează rezultatul
    if (success) {
      Serial.printf("[%lu ms] Y0 set to: %s (SUCCESS)\n", now, y0_state ? "ON (1)" : "OFF (0)");
    } else {
      Serial.printf("[%lu ms] Y0 set to: %s (FAILED - check connection)\n", now, y0_state ? "ON (1)" : "OFF (0)");
    }
  }

  // Procesează task-urile Modbus (dacă este necesar)
  // fatekMaster.task();
  
  yield();  // Permite altor task-uri să ruleze
}
