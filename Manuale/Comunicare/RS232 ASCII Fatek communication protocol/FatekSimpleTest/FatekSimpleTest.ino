/*
 * FATEK PLC Communication - Vertical Display
 * Citeste: M71-M79, M94-M96, M97-M99, M120, M320-M322; D0, D10, D47-D49, D57-D59; R10-R99, R110-R199
 * Afisare: Linie cu linie
 */

#include <Arduino.h>

// --- Configurație Serială ---
#define RX_PIN 16
#define TX_PIN 17
#define BAUDRATE 115200 
#define STATION_NO "01" 
#define TIMEOUT_MS 2000

// --- Control Serial Monitor ---
#define SERIAL_MONITOR_ENABLED true  // Setează false pentru a dezactiva afișarea în Serial Monitor

// --- Timing Control ---
#define READ_INTERVAL_MS 250  // Interval între citiri: 250ms = 4 citiri/secundă

// --- Structură răspuns ---
struct FatekResponse {
    bool success;
    String rawData; 
    String errorMsg;
};

// Prototypes
FatekResponse fatekReadRegisters(char type, int startAddr, int count);
FatekResponse fatekReadBits(char type, int startAddr, int count);
String calculateLRC(String input);
FatekResponse sendFatekCommand(String commandCode, String body);
void readAndPrintRegistersVertical(char type, int startAddr, int count);
void readAndPrintBitsVertical(char type, int startAddr, int count);
void readAllRegisters();

// Variabilă pentru controlul timing-ului
unsigned long lastReadTime = 0;

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    Serial2.begin(BAUDRATE, SERIAL_7E1, RX_PIN, TX_PIN);
    
    delay(1000);
    if (SERIAL_MONITOR_ENABLED) {
        Serial.println("--- MONITORIZARE FATEK PLC ---");
    }
}

// ================= LOOP PRINCIPAL =================
void loop() {
    // Verifică dacă a trecut intervalul de timp pentru următoarea citire
    unsigned long currentTime = millis();
    
    if (currentTime - lastReadTime >= READ_INTERVAL_MS) {
        lastReadTime = currentTime;
        
        if (SERIAL_MONITOR_ENABLED) {
            Serial.println("\n========================================");
            Serial.printf("      CICLU CITIRE - %lu ms      \n", currentTime);
            Serial.println("========================================");
        }
        
        // Citește toate registrele
        readAllRegisters();
        
        if (SERIAL_MONITOR_ENABLED) {
            Serial.println("----------------------------------------");
        }
    }
}

// ================= FUNCȚIE CITIRE TOATE REGISTRELE =================
void readAllRegisters() {
    // 1. M71 - M79 (9 biți)
    readAndPrintBitsVertical('M', 71, 9);

    // 2. M94 - M96 (3 biți)
    readAndPrintBitsVertical('M', 94, 3);

    // 3. M97 - M99 (3 biți)
    readAndPrintBitsVertical('M', 97, 3);

    // 4. M120 (1 bit)
    readAndPrintBitsVertical('M', 120, 1);

    // 5. M320 - M322 (3 biți)
    readAndPrintBitsVertical('M', 320, 3);

    // 6. D0 (1 registru)
    readAndPrintRegistersVertical('D', 0, 1);

    // 7. D10 (1 registru)
    readAndPrintRegistersVertical('D', 10, 1);

    // 8. D47 - D49 (3 registre)
    readAndPrintRegistersVertical('D', 47, 3);

    // 9. D57 - D59 (3 registre)
    readAndPrintRegistersVertical('D', 57, 3);

    // 10. R10 - R99 (90 registre)
    readAndPrintRegistersVertical('R', 10, 90);

    // 11. R110 - R199 (90 registre)
    readAndPrintRegistersVertical('R', 110, 90);
}

// ================= FUNCȚII AJUTĂTOARE AFISARE VERTICALA =================

// Afișează registre numerice (R, D) unul sub altul
void readAndPrintRegistersVertical(char type, int startAddr, int count) {
    FatekResponse resp = fatekReadRegisters(type, startAddr, count);
    
    if (!SERIAL_MONITOR_ENABLED) {
        return; // Ieșim dacă serial monitor este dezactivat
    }
    
    if (resp.success) {
        Serial.printf("--- Grup %c%d - %c%d ---\n", type, startAddr, type, startAddr + count - 1);
        
        for(int i=0; i<count; i++) {
            // Extragem 4 caractere HEX pentru fiecare registru
            String hexVal = resp.rawData.substring(i*4, (i+1)*4);
            // Convertim în număr zecimal
            long value = strtol(hexVal.c_str(), NULL, 16);
            
            // Afisare formatată: TipAdresa : Valoare
            // %-5d aliniaza numarul la stanga
            Serial.printf("  %c%-5d : %ld\n", type, startAddr+i, value);
        }
    } else {
        Serial.printf("[EROARE] %c%d-%c%d : %s\n", type, startAddr, type, startAddr + count - 1, resp.errorMsg.c_str());
    }
}

// Afișează biți (M, X, Y) unul sub altul cu status ON/OFF
void readAndPrintBitsVertical(char type, int startAddr, int count) {
    FatekResponse resp = fatekReadBits(type, startAddr, count);
    
    if (!SERIAL_MONITOR_ENABLED) {
        return; // Ieșim dacă serial monitor este dezactivat
    }
    
    if (resp.success) {
        Serial.printf("--- Grup %c%d - %c%d ---\n", type, startAddr, type, startAddr + count - 1);
        
        for(int i=0; i<resp.rawData.length(); i++) {
            char statusChar = resp.rawData[i];
            String statusText = (statusChar == '1') ? "ON" : "OFF";
            
            Serial.printf("  %c%-5d : %s\n", type, startAddr+i, statusText.c_str());
        }
    } else {
        Serial.printf("[EROARE] %c%d-%c%d : %s\n", type, startAddr, type, startAddr + count - 1, resp.errorMsg.c_str());
    }
}

// ================= IMPLEMENTARE PROTOCOL =================

FatekResponse fatekReadBits(char type, int startAddr, int count) {
    char countBuf[3];
    sprintf(countBuf, "%02d", count);
    char addrBuf[5];
    sprintf(addrBuf, "%04d", startAddr); 
    
    String body = String(countBuf) + String(type) + String(addrBuf);
    return sendFatekCommand("44", body);
}

FatekResponse fatekReadRegisters(char type, int startAddr, int count) {
    char countBuf[3];
    sprintf(countBuf, "%02d", count);
    char addrBuf[6];
    sprintf(addrBuf, "%05d", startAddr); 
    
    String body = String(countBuf) + String(type) + String(addrBuf);
    return sendFatekCommand("46", body);
}

FatekResponse sendFatekCommand(String commandCode, String body) {
    FatekResponse result;
    result.success = false;

    String payload = String(STATION_NO) + commandCode + body;
    String lrc = calculateLRC(payload);
    String fullCommand = "\x02" + payload + lrc + "\x03";

    while(Serial2.available()) Serial2.read(); 
    Serial2.print(fullCommand);

    String response = "";
    unsigned long startTime = millis();
    bool etxFound = false;

    while (millis() - startTime < TIMEOUT_MS) {
        if (Serial2.available()) {
            char c = Serial2.read();
            response += c;
            if (c == 0x03) { etxFound = true; break; }
        }
    }

    if (!etxFound) { result.errorMsg = "Timeout"; return result; }
    if (response.length() < 9) { result.errorMsg = "Incomplete"; return result; }

    char errorCode = response.charAt(5);
    if (errorCode != '0') {
        result.errorMsg = String("PLC Err: ") + errorCode;
        return result;
    }

    if (response.length() > 9) {
        result.rawData = response.substring(6, response.length() - 3);
    } else {
        result.rawData = "";
    }

    result.success = true;
    return result;
}

String calculateLRC(String input) {
    int sum = 0x02; 
    for (int i = 0; i < input.length(); i++) {
        sum += (int)input[i];
    }
    String lrc = String(sum & 0xFF, HEX);
    lrc.toUpperCase();
    if (lrc.length() < 2) lrc = "0" + lrc;
    return lrc;
}