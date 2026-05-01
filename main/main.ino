#include <Arduino.h>
#include "FATEKModbus.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <cstring>

/*
    LED Matrix display based on FATEK Modbus registers
    Display condition: M320 = 1 and D0 = 0
    3 text lines:
    1. "Bine ati venit!" - scrolling from right to left
    2. "Introduceti" - static and centered
    3. "minim X lei/leu" - static and centered (X = D10, leu if D10=1, lei if D10>1)
*/

// ========================================
// MODBUS RTU COMMUNICATION PARAMETERS
// ========================================
#define MODBUS_RX_PIN 16
#define MODBUS_TX_PIN 17
#define MODBUS_RE_DE_PIN 4
#define MODBUS_BAUDRATE 115200 //8E1 - 8 data bits, 1 stop bit, Even parity
#define MODBUS_SLAVE_ID 1

// ========================================
// HUB75 LED MATRIX PARAMETERS
// ========================================
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

// HUB75 LED matrix pins (ESP32 default pins)
#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 12
#define B2_PIN 13

#define A_PIN 23
#define B_PIN 19
#define C_PIN 18
#define D_PIN 5
#define E_PIN -1  // For 64x64 panels, set a valid pin

#define LAT_PIN 32
#define OE_PIN 15
#define CLK_PIN 33

MatrixPanel_I2S_DMA *dma_display = nullptr;

// Serial Monitor output control
bool enableSerialMonitor = true;  // true to enable Serial Monitor output, false to disable

// Scroll variables
int scrollX = 0;
int scrollX_line1_scenario3 = 0;  // Scroll for line 1 in scenario 3
int scrollX_line2_scenario3 = 0;  // Scroll for line 2 in scenario 3
int scrollX_scenario5 = 0;  // Scroll for line 1 in scenario 5
int scrollX_line3_scenario12 = 0;  // Scroll for line 3 in scenario 1.2
int scrollX_line3_scenario6 = 0;  // Scroll for line 3 in scenario 6 (PLC communication error)
int scrollX_line3_scenario7 = 0;  // Scroll for line 3 in scenario 7 (M120=0 unavailable)
int line1Width = 0;
int line2Width = 0;
int line3Width = 0;
int longestLineWidth = 0;
uint16_t textColor = 0;
bool displayActive = false;

// Refresh sync variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 10; // Refresh every 10ms (100 FPS)

// Scroll variables (separate from refresh for independent speed control)
unsigned long lastScrollUpdate = 0;
const unsigned long SCROLL_UPDATE_INTERVAL = 40; // Scroll updated every 40ms
const int SCROLL_EXTRA_SPACES = 5; // Extra spaces after text before reset
const int SCROLL_EXTRA_PIXELS = SCROLL_EXTRA_SPACES * 6; // 5 spaces * 6px per character = 30px

// Helper function to calculate text width
int calcTextWidth(const char *str) {
  return strlen(str) * 6;  // font 5px + 1px space
}

// Helper function to build text from an R register range
String buildTextFromRRange(RegisterValues* regValues, int startIndex, int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    int regIndex = startIndex + i;
    if (regIndex < 180 && regValues->R_valid[regIndex]) {
      uint16_t regValue = regValues->R[regIndex];
      if (regValue != 0) {  // Skip register value 0
        // Extract ASCII characters (each register can contain 2 chars)
        uint8_t lowByte = regValue % 256;
        uint8_t highByte = regValue / 256;
        
        // Low byte (first character)
        if (lowByte >= 32 && lowByte <= 126) {
          result += (char)lowByte;
        }
        
        // High byte (second character)
        if (highByte >= 32 && highByte <= 126) {
          result += (char)highByte;
        }
      }
    }
  }
  return result;
}

// Helper function to get matching color for M71-M79 in scenario 3
uint16_t getColorForMIndex(int mIndex) {
  switch(mIndex) {
    case 0: return dma_display->color565(0, 255, 128);    // M71: (0, 255, 128)
    case 1: return dma_display->color565(0, 128, 255);    // M72: (0, 128, 255)
    case 2: return dma_display->color565(255, 0, 255);    // M73: (255, 0, 255)
    case 3: return dma_display->color565(0, 255, 255);    // M74: (0, 255, 255)
    case 4: return dma_display->color565(128, 255, 0);    // M75: (128, 255, 0)
    case 5: return dma_display->color565(255, 0, 128);    // M76: (255, 0, 128)
    case 6: return dma_display->color565(128, 0, 255);    // M77: (128, 0, 255)
    case 7: return dma_display->color565(0, 255, 0);      // M78: (0, 255, 0)
    case 8: return dma_display->color565(255, 255, 0);    // M79: (255, 255, 0)
    default: return dma_display->color565(255, 255, 255); // Default: white
  }
}

// Helper function to format time values (00 for 0, 01-09 for 1-9, normal for >=10)
void formatTimeValue(uint16_t value, char* buffer) {
  if (value == 0) {
    strcpy(buffer, "00");
  } else if (value < 10) {
    buffer[0] = '0';
    itoa(value, buffer + 1, 10);
    buffer[2] = '\0';
  } else {
    itoa(value, buffer, 10);
  }
}

// Task for Modbus communication on Core 0
void taskModbusCommunication(void *parameter) {
  // Sync Serial Monitor state with Modbus module
  setSerialMonitorEnabled(enableSerialMonitor);
  
  // Initialize FATEK Modbus
  initFATEKModbus();
  
  // Variables for Serial Monitor output
  unsigned long lastSerialDisplay = 0;
  const unsigned long SERIAL_DISPLAY_INTERVAL = 1000; // Print to Serial once per second
  
  // Variables for communication state checks
  bool wasCommError = false;
  
  // Infinite Modbus communication loop
  while (true) {
    unsigned long currentTime = millis();
    
    // Sync Serial Monitor state (if changed)
    if (getSerialMonitorEnabled() != enableSerialMonitor) {
      setSerialMonitorEnabled(enableSerialMonitor);
    }
    
    
    // Modbus communication
    loopFATEKModbus();
    
    // Print registers to Serial Monitor once per second (if enabled)
    if (enableSerialMonitor && currentTime - lastSerialDisplay >= SERIAL_DISPLAY_INTERVAL) {
      lastSerialDisplay = currentTime;
      displayRegisterValues();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Yield control to FreeRTOS
  }
}

// Task for LED matrix display on Core 1
void taskMatrixDisplay(void *parameter) {
  // Configure HUB75 LED matrix pins
  HUB75_I2S_CFG::i2s_pins _pins = {
    R1_PIN, G1_PIN, B1_PIN,    // R1, G1, B1
    R2_PIN, G2_PIN, B2_PIN,    // R2, G2, B2
    A_PIN, B_PIN, C_PIN, D_PIN, E_PIN,  // Address pins (A, B, C, D, E)
    LAT_PIN, OE_PIN, CLK_PIN   // Latch, Output Enable, Clock
  };
  
  // Initialize Matrix Display with higher refresh rate and configured pins
  HUB75_I2S_CFG matrix_config(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN, _pins);
  matrix_config.min_refresh_rate = 120; // Increase refresh rate to 120 Hz for smoother output
  dma_display = new MatrixPanel_I2S_DMA(matrix_config);
  dma_display->begin();
  dma_display->setBrightness8(255);  // 0-255 Brightness
  dma_display->clearScreen();
  dma_display->setTextWrap(false);
  dma_display->setTextSize(1);
  textColor = dma_display->color565  (128, 128, 255);  // white
  
  // Infinite display loop
  while (true) {
    unsigned long currentTime = millis();
    
    // Update display at a fixed interval to avoid flicker
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      lastDisplayUpdate = currentTime;
      
      // Get register values
      RegisterValues* regValues = getRegisterValues();
      
      // Check display conditions
      bool shouldDisplayWelcome = false;  // M320 = 1 and D0 = 0
      bool shouldDisplayWelcome11 = false;  // Scenario 1.1: M322=1 and D0=0
      bool shouldDisplayWelcome12 = false;  // Scenario 1.2: M321=1 and D0=0
      bool shouldDisplayIntroduced = false;  // D0 > 0
      bool shouldDisplayFunction = false;  // Scenario 3: ONE of M71-M79=1, D0>0, M94=1
      bool shouldDisplayPause = false;  // Scenario 4: M97=1, D0>0
      bool shouldDisplayPauseFinished = false;  // Scenario 5: M99=1, D0>0
      bool shouldDisplayUnavailable = false;  // Scenario 7: M120=0 (unavailable)
      bool shouldDisplayCommError = false;  // Scenario 6: PLC communication error
      int activeMIndex = -1;  // Active M index (0=M71, 1=M72, ..., 8=M79)
      
      // Check Scenario 6 (highest priority): PLC communication error
      shouldDisplayCommError = !isCommunicationActive();
      
      // Check Scenario 7 (high priority, after 6): M120=0 -> unavailable (only if communication works)
      if (!shouldDisplayCommError && regValues->M_valid[15]) {  // M120 is at index 15
        shouldDisplayUnavailable = (regValues->M120 == false);  // M120=0
      }
      
      // Check Scenario 5 (priority): M99=1 and D0>0 (only if scenarios 6 and 7 are not active)
      if (!shouldDisplayCommError && !shouldDisplayUnavailable && regValues->D_valid[0] && regValues->D0 > 0 && regValues->M_valid[14]) {  // M99 is at index 14
        shouldDisplayPauseFinished = (regValues->M99 == true);
      }
      
      // Check Scenario 4 (higher than 3, lower than 5): M97=1 and D0>0 (only if 6 and 7 are not active)
      if (!shouldDisplayCommError && !shouldDisplayUnavailable && !shouldDisplayPauseFinished && regValues->D_valid[0] && regValues->D0 > 0 && regValues->M_valid[12]) {  // M97 is at index 12
        shouldDisplayPause = (regValues->M97 == true);
      }
      
      // Check Scenario 3 only if scenarios 4, 5, 6, and 7 are not active
      if (!shouldDisplayCommError && !shouldDisplayUnavailable && !shouldDisplayPause && !shouldDisplayPauseFinished && regValues->D_valid[0] && regValues->D0 > 0) {
        // Check M94 = 1
        bool m94Active = false;
        if (regValues->M_valid[9]) {  // M94 is at index 9 in the array
          m94Active = (regValues->M94 == true);
        }
        
        // Check if exactly ONE of M71-M79 is active
        if (m94Active && regValues->M_valid[0] && regValues->M_valid[1] && 
            regValues->M_valid[2] && regValues->M_valid[3] && regValues->M_valid[4] &&
            regValues->M_valid[5] && regValues->M_valid[6] && regValues->M_valid[7] && 
            regValues->M_valid[8]) {
          int activeCount = 0;
          if (regValues->M71) { activeCount++; activeMIndex = 0; }
          if (regValues->M72) { activeCount++; activeMIndex = 1; }
          if (regValues->M73) { activeCount++; activeMIndex = 2; }
          if (regValues->M74) { activeCount++; activeMIndex = 3; }
          if (regValues->M75) { activeCount++; activeMIndex = 4; }
          if (regValues->M76) { activeCount++; activeMIndex = 5; }
          if (regValues->M77) { activeCount++; activeMIndex = 6; }
          if (regValues->M78) { activeCount++; activeMIndex = 7; }
          if (regValues->M79) { activeCount++; activeMIndex = 8; }
          
          if (activeCount == 1) {  // Exactly one active
            shouldDisplayFunction = true;
          }
        }
      }
      
      if (regValues->D_valid[0]) {
        // Check Scenario 1: M320=1 and D0=0 (same priority as 1.1)
        if (!shouldDisplayCommError && !shouldDisplayUnavailable && regValues->M_valid[16] && regValues->D0 == 0 && !shouldDisplayFunction && !shouldDisplayPause && !shouldDisplayPauseFinished) {
          shouldDisplayWelcome = (regValues->M320 == 1);
        }
        
        // Check Scenario 1.1: M322=1 and D0=0 (same priority as 1)
        if (!shouldDisplayCommError && !shouldDisplayUnavailable && regValues->M_valid[18] && regValues->D0 == 0 && !shouldDisplayFunction && !shouldDisplayPause && !shouldDisplayPauseFinished) {
          shouldDisplayWelcome11 = (regValues->M322 == true);  // M322 is at index 18
        }
        
        // Check Scenario 1.2: M321=1 and D0=0 (same priority as 1 and 1.1)
        if (!shouldDisplayCommError && !shouldDisplayUnavailable && regValues->M_valid[17] && regValues->D0 == 0 && !shouldDisplayFunction && !shouldDisplayPause && !shouldDisplayPauseFinished) {
          shouldDisplayWelcome12 = (regValues->M321 == true);  // M321 is at index 17
        }
        
        // Check Scenario 2 (D0 > 0)
        if (!shouldDisplayCommError && !shouldDisplayUnavailable && regValues->D0 > 0 && !shouldDisplayFunction && !shouldDisplayPause && !shouldDisplayPauseFinished) {
          shouldDisplayIntroduced = true;
        }
      }
      
      // If scenario 6 is active (communication error), disable all other scenarios
      if (shouldDisplayCommError) {
        shouldDisplayWelcome = false;
        shouldDisplayWelcome11 = false;
        shouldDisplayWelcome12 = false;
        shouldDisplayIntroduced = false;
        shouldDisplayFunction = false;
        shouldDisplayPause = false;
        shouldDisplayPauseFinished = false;
        shouldDisplayUnavailable = false;
      } else if (shouldDisplayUnavailable) {
        // If scenario 7 is active (unavailable), disable all other scenarios (except 6)
        shouldDisplayWelcome = false;
        shouldDisplayWelcome11 = false;
        shouldDisplayWelcome12 = false;
        shouldDisplayIntroduced = false;
        shouldDisplayFunction = false;
        shouldDisplayPause = false;
        shouldDisplayPauseFinished = false;
      } else {
        // Scenarios 3 and 4 must be off if scenario 5 is active
        if (shouldDisplayPauseFinished) {
          shouldDisplayFunction = false;
          shouldDisplayPause = false;
        }
        
        // Scenario 3 must be off if scenario 4 is active
        if (shouldDisplayPause) {
          shouldDisplayFunction = false;
        }
      }
      
      if (shouldDisplayWelcome || shouldDisplayWelcome11 || shouldDisplayWelcome12) {
        // SCENARIO 1, 1.1, or 1.2: welcome message
        // Build the 3 text lines
        const char *line1 = "Bine ati venit!";
        char line2[50] = "";
        char line3[50] = "";
        
        // Set lines 2 and 3 based on scenario
        if (shouldDisplayWelcome12) {
          // Scenario 1.2: M321=1 and D0=0
          strcpy(line2, "Boxa ");
          // Add slave ID
          char slaveIDStr[10];
          itoa(getCurrentSlaveID(), slaveIDStr, 10);
          strcat(line2, slaveIDStr);
          
          strcpy(line3, "Introduceti Bani la Bancomat!");
        } else if (shouldDisplayWelcome11) {
          // Scenario 1.1: M322=1 and D0=0
          strcpy(line2, "Introduceti");
          strcpy(line3, "Jetoane");
        } else {
          // Scenario 1: M320=1 and D0=0
          strcpy(line2, "Introduceti");
          strcpy(line3, "minim ");
          
          // Build line 3 based on D10
          // Check if D10 is valid
          if (regValues->D_valid[1]) {
            char d10Str[20];
            itoa(regValues->D10, d10Str, 10);
            strcat(line3, d10Str);
            
            // D10 == 1 -> "leu", otherwise -> "lei"
            if (regValues->D10 == 1) {
              strcat(line3, "Leu");
            } else {
              strcat(line3, "Lei");
            }
          } else {
            strcat(line3, "?Lei");
          }
        }
        
        // Check if line 3 needs smaller text (when D10 > 9, scenario 1 only)
        bool useSmallText = false;
        if (shouldDisplayWelcome && regValues->D_valid[1] && regValues->D10 > 9) {
          useSmallText = true;
        }
        
        // Calculate text widths
        line1Width = calcTextWidth(line1);
        line2Width = calcTextWidth(line2);
        
        // For line 3, calculate width based on text size
        // Default char width is 5px + 1px space = 6px
        // For smaller text, use scale 0.8 (5*0.8 + 1 = 5px per char)
        if (useSmallText) {
          line3Width = (int)(strlen(line3) * 5.0f);  // 5px per character for smaller text
        } else {
          line3Width = calcTextWidth(line3);  // 6px per character for normal text
        }
        
        // Calculate X positions - scenario 1.2 uses scroll for lines 1 and 3
        int centerX_line2 = (dma_display->width() - line2Width) / 2;
        int posX_line1, posX_line3;
        
        if (shouldDisplayWelcome12) {
          // In scenario 1.2, lines 1 and 3 use synchronized scroll (like scenario 3)
          // If display was inactive or lines are not synced
          if (!displayActive) {
            displayActive = true;
            scrollX = dma_display->width();
            scrollX_line3_scenario12 = dma_display->width();
          } else {
            // If both need scroll, sync them to start together
            // Check if one was recently reset or if they are out of sync
            if (scrollX == dma_display->width() || scrollX_line3_scenario12 == dma_display->width()) {
              // If one was reset, reset the other too
              scrollX = dma_display->width();
              scrollX_line3_scenario12 = dma_display->width();
            }
          }
          
          // Scroll positions
          posX_line1 = scrollX;
          posX_line3 = scrollX_line3_scenario12;
        } else {
          // For scenarios 1 and 1.1, only line 1 scrolls
          if (!displayActive) {
            scrollX = dma_display->width();
            displayActive = true;
          }
          posX_line1 = scrollX;
          posX_line3 = (dma_display->width() - line3Width) / 2;  // Line 3 centered
        }
        
        // Draw text (DMA buffer updates while output remains smooth)
        dma_display->fillScreen(0);
        // Scenario 1, 1.1, 1.2: (255, 255, 255) - white
        dma_display->setTextColor(dma_display->color565(255, 255, 255));
        
        // Line 1 - right-to-left scroll (synced with line 3 in scenario 1.2)
        dma_display->setCursor(posX_line1, 2);
        dma_display->setTextSize(1);  // Normal text size
        dma_display->print(line1);
        
        // Line 2 - static and centered
        dma_display->setCursor(centerX_line2, 12);
        dma_display->setTextSize(1);  // Normal text size
        dma_display->print(line2);
        
        // Line 3 - scroll in scenario 1.2 (synced with line 1), static and centered otherwise (smaller text if D10 > 9)
        dma_display->setCursor(posX_line3, 22);
        if (useSmallText) {
          // Use text size 1 (minimum) and draw chars one by one
          // to simulate smaller text by reducing spacing
          dma_display->setTextSize(1);
          // Draw each char with reduced spacing for smaller-text effect
          for (int i = 0; i < strlen(line3); i++) {
            dma_display->print(line3[i]);
            // Move cursor slightly left for compact text effect
            if (i < strlen(line3) - 1) {
              int16_t x, y;
              uint16_t w, h;
              dma_display->getTextBounds(String(line3[i]), 0, 0, &x, &y, &w, &h);
              dma_display->setCursor(dma_display->getCursorX() - 1, dma_display->getCursorY());
            }
          }
        } else {
          dma_display->setTextSize(1);  // Normal text size
          dma_display->print(line3);
        }
        
        // Update scroll positions if needed (similar to scenario 3)
        if (currentTime - lastScrollUpdate >= SCROLL_UPDATE_INTERVAL) {
          lastScrollUpdate = currentTime;
          
          if (shouldDisplayWelcome12) {
            // For scenario 1.2, use the same offset for both lines (synced like scenario 3)
            scrollX--;
            scrollX_line3_scenario12 = scrollX;  // Sync to same position
            
            // Reset both only when line 3 (the longest) fully leaves the screen
            // Line 3 fully passed when: scrollX_line3_scenario12 + line3Width < 0
            if (scrollX_line3_scenario12 + line3Width < 0) {
              // Line 3 finished scroll, reset both to start again
              scrollX = dma_display->width();
              scrollX_line3_scenario12 = dma_display->width();
            }
          } else {
            // For scenarios 1 and 1.1, only line 1 scrolls
            scrollX--;
            // Reset when all characters pass the screen (last char leaves left side)
            if (scrollX + line1Width < 0) {
              // All characters passed the screen, reset to start again
              scrollX = dma_display->width();
            }
          }
        }
      } else if (shouldDisplayIntroduced) {
        // SCENARIO 2: D0 > 0 -> confirmed insert message
        const char *line1 = "Ai";
        const char *line2 = "introdus";
        
        // Build line 3 in "x.x lei/leu" format
        // x.x = (D0/10).(D0%10)
        char line3[50] = "";
        
        if (regValues->D_valid[0]) {
          uint16_t d0 = regValues->D0;
          uint16_t parteIntreaga = d0 / 10;  // D0/10 (integer part)
          uint16_t parteZecimala = d0 % 10;  // D0%10 (remainder)
          
          // Convert integer part to string
          char intStr[20];
          itoa(parteIntreaga, intStr, 10);
          strcat(line3, intStr);
          
          // Add decimal point
          strcat(line3, ".");
          
          // Convert decimal part to string
          char decStr[20];
          itoa(parteZecimala, decStr, 10);
          strcat(line3, decStr);
          
          // Add space and "lei" or "leu"
          strcat(line3, " ");
          if (parteIntreaga == 1) {
            strcat(line3, "Leu");
          } else {
            strcat(line3, "Lei");
          }
        } else {
          strcat(line3, "?.? Lei");
        }
        
        // Calculate text widths
        line1Width = calcTextWidth(line1);
        line2Width = calcTextWidth(line2);
        line3Width = calcTextWidth(line3);
        
        // If display was inactive, reset state (no scroll used here)
        if (!displayActive) {
          displayActive = true;
        }
        
        // Calculate X positions for centered static lines
        int centerX_line1 = (dma_display->width() - line1Width) / 2;
        int centerX_line2 = (dma_display->width() - line2Width) / 2;
        int centerX_line3 = (dma_display->width() - line3Width) / 2;
        
        // Draw text
        dma_display->fillScreen(0);
        // Scenario 2: (128, 255, 128) - light green
        dma_display->setTextColor(dma_display->color565(128, 255, 128));
        
        // Line 1 - static and centered
        dma_display->setCursor(centerX_line1, 2);
        dma_display->print(line1);
        
        // Line 2 - static and centered
        dma_display->setCursor(centerX_line2, 10);
        dma_display->print(line2);
        
        // Line 3 - static and centered
        dma_display->setCursor(centerX_line3, 21);
        dma_display->print(line3);
      } else if (shouldDisplayFunction) {
        // SCENARIO 3: M94=1, ONLY ONE of M71-M79=1, D0>0 -> function display
        // Mapping M71-M79 -> R ranges
        // M71->R10-R19(line1), R110-R119(line2)
        // M72->R20-R29(line1), R120-R129(line2)
        // M73->R30-R39(line1), R130-R139(line2)
        // M74->R40-R49(line1), R140-R149(line2)
        // M75->R50-R59(line1), R150-R159(line2)
        // M76->R60-R69(line1), R160-R169(line2)
        // M77->R70-R79(line1), R170-R179(line2)
        // M78->R80-R89(line1), R180-R189(line2)
        // M79->R90-R99(line1), R190-R199(line2)
        
        // Calculate correct indexes for R[] array
        // R[0-89] = R10-R99, R[90-179] = R110-R199
        // M71 (index 0): R10-R19 (R[0-9]), R110-R119 (R[90-99])
        // M72 (index 1): R20-R29 (R[10-19]), R120-R129 (R[100-109])
        // etc.
        int firstRangeStart = activeMIndex * 10;      // R10-R99 (for line 1): R[0-89]
        int secondRangeStart = 90 + activeMIndex * 10;  // R110-R199 (for line 2): R[90-179]
        
        // Build text for each range
        String text1 = buildTextFromRRange(regValues, firstRangeStart, 10);   // Line 1
        String text2 = buildTextFromRRange(regValues, secondRangeStart, 10);  // Line 2
        
        char line1[200] = "";
        char line2[200] = "";
        char line3[50] = "";
        
        bool line1Empty = (text1.length() == 0);
        bool line2Empty = (text2.length() == 0);
        
        // Display logic: 2 lines / 1 line (line 1 empty) / 1 line (line 2 empty) / FUNCTION ?
        if (line1Empty && line2Empty) {
          // Both lines are empty -> FUNCTION ? on one line
          strcpy(line1, "Denumire functie");
          line2[0] = '\0';
        } else if (line1Empty) {
          // Line 1 is empty -> show only line 2
          line1[0] = '\0';
          text2.toCharArray(line2, 200);
        } else if (line2Empty) {
          // Line 2 is empty -> show only line 1
          text1.toCharArray(line1, 200);
          line2[0] = '\0';
        } else {
          // Both lines have characters -> show both
          text1.toCharArray(line1, 200);
          text2.toCharArray(line2, 200);
        }
        
        // Check if both lines are shown or only one/"Denumire functie"
        bool bothLinesDisplayed = !line1Empty && !line2Empty;
        bool singleLineOrFunctionDisplayed = (line1Empty && line2Empty) || (line1Empty && !line2Empty) || (!line1Empty && line2Empty);
        
        // Line 3 variables: font size and Y position
        int line3FontSize = 1;
        int line3YPosition = 22;
        
        // Build line 3 as "x:x:x" (D49:D48:D47) as requested
        if (regValues->D_valid[4] && regValues->D_valid[3] && regValues->D_valid[2]) {
          uint16_t d49 = regValues->D49;
          uint16_t d48 = regValues->D48;
          uint16_t d47 = regValues->D47;
          
          if (d49 > 0) {
            // D49 > 0 -> always show D49:D48:D47 (hour:minute:second, x:x:x)
            char d49Str[10], d48Str[10], d47Str[10];
            formatTimeValue(d49, d49Str);
            strcat(line3, d49Str);
            
            // D48 is always shown when D49 > 0 ("00" if D48 = 0)
            strcat(line3, ":");
            if (d48 == 0) {
              strcat(line3, "00");
            } else {
              formatTimeValue(d48, d48Str);
              strcat(line3, d48Str);
            }
            
            // D47 is always shown when D49 > 0 ("00" if D47 = 0)
            strcat(line3, ":");
            if (d47 == 0) {
              strcat(line3, "00");
            } else {
              formatTimeValue(d47, d47Str);
              strcat(line3, d47Str);
            }
            
            // Set font size and Y based on case
            if (bothLinesDisplayed) {
              // Case 1: both lines shown -> font size 1, Y = 22
              line3FontSize = 1;
              line3YPosition = 22;
            } else if (singleLineOrFunctionDisplayed) {
              // Case 2: only one line or "Denumire functie" -> font size 1, Y = 19
              line3FontSize = 1;
              line3YPosition = 19;
            }
          } else if (d48 > 0) {
            // D49 = 0, D48 > 0 -> show D48:D47
            char d48Str[10], d47Str[10];
            formatTimeValue(d48, d48Str);
            strcat(line3, d48Str);
            
            // D47 is always shown when D48 > 0 ("00" if D47 = 0)
            strcat(line3, ":");
            if (d47 == 0) {
              strcat(line3, "00");
            } else {
              formatTimeValue(d47, d47Str);
              strcat(line3, d47Str);
            }
            
            // Set font size and Y based on case
            if (bothLinesDisplayed) {
              // Case 1: both lines shown -> font size 1, Y = 22
              line3FontSize = 1;
              line3YPosition = 22;
            } else if (singleLineOrFunctionDisplayed) {
              // Case 2: only one line or "Denumire functie" -> font size 2, Y = 16
              line3FontSize = 2;
              line3YPosition = 16;
            }
          } else if (d47 > 0) {
            // D49 = 0, D48 = 0, D47 > 0 -> show only D47
            char d47Str[10];
            formatTimeValue(d47, d47Str);
            strcat(line3, d47Str);
            
            // Set font size and Y based on case
            if (bothLinesDisplayed) {
              // Case 1: both lines shown -> font size 1, Y = 22
              line3FontSize = 1;
              line3YPosition = 22;
            } else if (singleLineOrFunctionDisplayed) {
              // Case 2: only one line or "Denumire functie" -> font size 2, Y = 16
              line3FontSize = 2;
              line3YPosition = 16;
            }
          }
        }
        
        // Calculate text widths
        int line1Width_sc3 = strlen(line1) > 0 ? calcTextWidth(line1) : 0;
        int line2Width_sc3 = strlen(line2) > 0 ? calcTextWidth(line2) : 0;
        int line3Width_sc3 = strlen(line3) > 0 ? calcTextWidth(line3) : 0;
        
        // Recalculate line 3 width based on font size
        if (strlen(line3) > 0) {
          int16_t x, y;
          uint16_t w, h;
          dma_display->setTextSize(line3FontSize);
          dma_display->getTextBounds(line3, 0, 0, &x, &y, &w, &h);
          line3Width_sc3 = w;  // Exact width for current font size
          
          // If font size is 2, reduce width for tighter spacing (1px per char instead of 2px)
          if (line3FontSize == 2 && strlen(line3) > 1) {
            line3Width_sc3 = line3Width_sc3 - (strlen(line3) - 1);  // Reduce by (n-1) pixels for tighter spacing
          }
          
          dma_display->setTextSize(1);  // Reset to normal text size
        }
        
        // Check if each line needs scrolling (> 10 characters)
        // For "Denumire functie", always force scrolling
        bool line1NeedsScroll = (strlen(line1) > 10) || (line1Empty && line2Empty && strcmp(line1, "Denumire functie") == 0);
        bool line2NeedsScroll = (strlen(line2) > 10);
        
        // If display was inactive, or both scrolling lines are out of sync
        if (!displayActive) {
          displayActive = true;
          scrollX_line1_scenario3 = dma_display->width();
          scrollX_line2_scenario3 = dma_display->width();
        } else if (line1NeedsScroll && line2NeedsScroll) {
          // If both need scrolling, sync them to start together
          // Check if one was reset recently or if they are out of sync
          // If both are scrolling, align to the one further behind
          if (scrollX_line1_scenario3 == dma_display->width() || scrollX_line2_scenario3 == dma_display->width()) {
            // If one was reset, reset the other too
            scrollX_line1_scenario3 = dma_display->width();
            scrollX_line2_scenario3 = dma_display->width();
          }
        }
        
        // Update scroll positions if needed
        unsigned long currentTime = millis();
        if (currentTime - lastScrollUpdate >= SCROLL_UPDATE_INTERVAL) {
          if (line1NeedsScroll && line2NeedsScroll) {
            // If both need scrolling, use same offset for both (synced)
            scrollX_line1_scenario3--;
            scrollX_line2_scenario3 = scrollX_line1_scenario3; // Sync to same position
            
            // Reset both when both lines fully finish scrolling
            // A line fully passed when: scrollX + lineWidth < 0
            if (scrollX_line1_scenario3 + line1Width_sc3 < 0 && scrollX_line2_scenario3 + line2Width_sc3 < 0) {
              // Both lines finished scrolling, reset to start again
              scrollX_line1_scenario3 = dma_display->width();
              scrollX_line2_scenario3 = dma_display->width();
            }
          } else {
            // If only one needs scrolling, handle it independently
            if (line1NeedsScroll) {
              scrollX_line1_scenario3--;
              // Reset when all characters passed the screen
              if (scrollX_line1_scenario3 + line1Width_sc3 < 0) {
                scrollX_line1_scenario3 = dma_display->width();
              }
            }
            if (line2NeedsScroll) {
              scrollX_line2_scenario3--;
              // Reset when all characters passed the screen
              if (scrollX_line2_scenario3 + line2Width_sc3 < 0) {
                scrollX_line2_scenario3 = dma_display->width();
              }
            }
          }
          lastScrollUpdate = currentTime;
        }
        
        // Calculate X positions for centering (or scrolling)
        int posX_line1, posX_line2;
        if (line1NeedsScroll && strlen(line1) > 0) {
          posX_line1 = scrollX_line1_scenario3;
        } else {
          posX_line1 = (dma_display->width() - line1Width_sc3) / 2;
        }
        
        if (line2NeedsScroll && strlen(line2) > 0) {
          posX_line2 = scrollX_line2_scenario3;
        } else {
          posX_line2 = (dma_display->width() - line2Width_sc3) / 2;
        }
        
        int centerX_line3 = (dma_display->width() - line3Width_sc3) / 2;
        
        // Set Y positions based on which lines are shown
        int yLine1, yLine2;
        
        if (line1Empty && line2Empty) {
          // "Denumire functie" and line 3
          yLine1 = 5;  // "Denumire functie" vertically centered
        } else if (line1Empty) {
          // Only line 2 and line 3
          yLine2 = 5;  // Line 2 vertically centered
        } else if (line2Empty) {
          // Only line 1 and line 3
          yLine1 = 5;  // Line 1 vertically centered
        } else {
          // All 3 lines
          yLine1 = 2;
          yLine2 = 12;
        }
        
        // Y position for line 3 is set by format (line3YPosition)
        
        // Draw text
        dma_display->fillScreen(0);
        dma_display->setTextSize(1);
        
        // Scenario 3: line 3 has color (128, 128, 255), lines 1/2 and "FUNCTIA ?" use color by M71-M79
        uint16_t colorLine3_sc3 = dma_display->color565(128, 128, 255);  // Line 3: blue
        uint16_t colorLines12_sc3 = getColorForMIndex(activeMIndex);     // Lines 1/2 and "FUNCTIA ?": color by M index
        
        // Line 1 (if present)
        if (strlen(line1) > 0) {
          dma_display->setTextColor(colorLines12_sc3);
          dma_display->setCursor(posX_line1, yLine1);
          dma_display->print(line1);
        }
        
        // Line 2 (if present)
        if (strlen(line2) > 0) {
          dma_display->setTextColor(colorLines12_sc3);
          dma_display->setCursor(posX_line2, yLine2);
          dma_display->print(line2);
        }
        
        // Line 3 (x:x:x) if present - with font size and Y position based on rules
        if (strlen(line3) > 0) {
          dma_display->setTextColor(colorLine3_sc3);
          dma_display->setCursor(centerX_line3, line3YPosition);
          
          if (line3FontSize == 2) {
            // Font size 2: draw each char separately with tighter spacing
            dma_display->setTextSize(2);
            for (int i = 0; i < strlen(line3); i++) {
              dma_display->print(line3[i]);
              // Reduce spacing between chars (from 2px to 1px for text size 2)
              if (i < strlen(line3) - 1) {
                int16_t x, y;
                uint16_t w, h;
                dma_display->getTextBounds(String(line3[i]), 0, 0, &x, &y, &w, &h);
                // Adjust cursor for tight spacing (1px instead of 2px)
                dma_display->setCursor(dma_display->getCursorX() - 1, dma_display->getCursorY());
              }
            }
            dma_display->setTextSize(1);  // Reset to normal text size
          } else {
            // Font size 1: draw normally
            dma_display->setTextSize(1);
            dma_display->print(line3);
          }
        }
      } else if (shouldDisplayPause) {
        // SCENARIO 4: M97=1 and D0>0 -> show "PAUZA" with time
        const char *line1_pause = "PAUZA";
        char line2_pause[50] = "";
        
        // Line 2 variables: font size and Y position
        int line2FontSize_pause = 1;
        int line2YPosition_pause = 20;
        
        // Build line 2 with time from D59:D58:D57 as requested
        if (regValues->D_valid[7] && regValues->D_valid[6] && regValues->D_valid[5]) {
          uint16_t d59 = regValues->D59;
          uint16_t d58 = regValues->D58;
          uint16_t d57 = regValues->D57;
          
          if (d59 > 0) {
            // D59 > 0 -> always show D59:D58:D57 (hour:minute:second, x:x:x)
            char d59Str[10], d58Str[10], d57Str[10];
            formatTimeValue(d59, d59Str);
            strcat(line2_pause, d59Str);
            
            // D58 is always shown when D59 > 0 ("00" if D58 = 0)
            strcat(line2_pause, ":");
            if (d58 == 0) {
              strcat(line2_pause, "00");
            } else {
              formatTimeValue(d58, d58Str);
              strcat(line2_pause, d58Str);
            }
            
            // D57 is always shown when D59 > 0 ("00" if D57 = 0)
            strcat(line2_pause, ":");
            if (d57 == 0) {
              strcat(line2_pause, "00");
            } else {
              formatTimeValue(d57, d57Str);
              strcat(line2_pause, d57Str);
            }
            
            // Font size 1, Y = 20
            line2FontSize_pause = 1;
            line2YPosition_pause = 20;
          } else if (d58 > 0) {
            // D59 = 0, D58 > 0 -> show D58:D47
            char d58Str[10], d57Str[10];
            formatTimeValue(d58, d58Str);
            strcat(line2_pause, d58Str);
            
            // D57 is always shown when D58 > 0 ("00" if D57 = 0)
            strcat(line2_pause, ":");
            if (d57 == 0) {
              strcat(line2_pause, "00");
            } else {
              formatTimeValue(d57, d57Str);
              strcat(line2_pause, d57Str);
            }
            
            // Font size 2, Y = 18
            line2FontSize_pause = 2;
            line2YPosition_pause = 18;
          } else if (d57 > 0) {
            // D59 = 0, D58 = 0, D57 > 0 -> show only D57
            char d57Str[10];
            formatTimeValue(d57, d57Str);
            strcat(line2_pause, d57Str);
            
            // Font size 2, Y = 19
            line2FontSize_pause = 2;
            line2YPosition_pause = 18;
          }
        }
        
        // Calculate text widths
        int line1Width_pause = calcTextWidth(line1_pause);
        int line2Width_pause = strlen(line2_pause) > 0 ? calcTextWidth(line2_pause) : 0;
        
        // Recalculate line 1 width for text size 2 (larger font)
        int16_t x1, y1;
        uint16_t w1, h1;
        dma_display->setTextSize(2);
        dma_display->getTextBounds(line1_pause, 0, 0, &x1, &y1, &w1, &h1);
        line1Width_pause = w1;  // Exact width for text size 2
        dma_display->setTextSize(1);  // Reset to normal text size
        
        // Recalculate line 2 width based on font size
        if (strlen(line2_pause) > 0) {
          int16_t x, y;
          uint16_t w, h;
          dma_display->setTextSize(line2FontSize_pause);
          dma_display->getTextBounds(line2_pause, 0, 0, &x, &y, &w, &h);
          line2Width_pause = w;  // Exact width for current font size
          
          // If font size is 2, reduce width for tight spacing (1px per char instead of 2px)
          if (line2FontSize_pause == 2 && strlen(line2_pause) > 1) {
            line2Width_pause = line2Width_pause - (strlen(line2_pause) - 1);  // Reduce by (n-1) pixels for tight spacing
          }
          
          dma_display->setTextSize(1);  // Reset to normal text size
        }
        
        // If display was inactive, reset state
        if (!displayActive) {
          displayActive = true;
        }
        
        // Calculate X positions for centering
        int centerX_line1_pause = (dma_display->width() - line1Width_pause) / 2;
        int centerX_line2_pause = (dma_display->width() - line2Width_pause) / 2;
        
        // Draw text
        dma_display->fillScreen(0);
        dma_display->setTextSize(1);
        
        // Scenario 4: Line 1 (255, 128, 0) orange, line 2 (128, 128, 255) blue
        // Line 1: "PAUZA" centered - larger font
        dma_display->setTextColor(dma_display->color565(255, 128, 0));  // Orange
        dma_display->setCursor(centerX_line1_pause, 1);  // Adjusted for text size 2
        dma_display->setTextSize(2);  // Larger font
        dma_display->print(line1_pause);
        dma_display->setTextSize(1);  // Reset to normal text size
        
        // Line 2: time (x:x:x or x:x or x) centered - font size and Y based on rules
        if (strlen(line2_pause) > 0) {
          dma_display->setTextColor(dma_display->color565(128, 128, 255));  // Blue
          dma_display->setCursor(centerX_line2_pause, line2YPosition_pause);
          
          if (line2FontSize_pause == 2) {
            // Font size 2: draw each char separately with tighter spacing
            dma_display->setTextSize(2);
            for (int i = 0; i < strlen(line2_pause); i++) {
              dma_display->print(line2_pause[i]);
              // Reduce spacing between chars (from 2px to 1px for text size 2)
              if (i < strlen(line2_pause) - 1) {
                int16_t x, y;
                uint16_t w, h;
                dma_display->getTextBounds(String(line2_pause[i]), 0, 0, &x, &y, &w, &h);
                // Adjust cursor for tight spacing (1px instead of 2px)
                dma_display->setCursor(dma_display->getCursorX() - 1, dma_display->getCursorY());
              }
            }
            dma_display->setTextSize(1);  // Reset to normal text size
          } else {
            // Font size 1: draw normally
            dma_display->setTextSize(1);
            dma_display->print(line2_pause);
          }
        }
      } else if (shouldDisplayPauseFinished) {
        // SCENARIO 5: M99=1 and D0>0 -> show "PAUZA terminata" with time
        const char *line1_pause_finished = "PAUZA";
        const char *line2_pause_finished = "terminata";
        char line3_pause_finished[50] = "";
        
        // Line 3 variables: font size and Y position
        int line3FontSize_pause_finished = 1;
        int line3YPosition_pause_finished = 20;
        
        // Build line 3 with time from D49:D48:D47 as requested
        if (regValues->D_valid[4] && regValues->D_valid[3] && regValues->D_valid[2]) {
          uint16_t d49 = regValues->D49;
          uint16_t d48 = regValues->D48;
          uint16_t d47 = regValues->D47;
          
          if (d49 > 0) {
            // D49 > 0 -> always show D49:D48:D47 (hour:minute:second, x:x:x)
            char d49Str[10], d48Str[10], d47Str[10];
            formatTimeValue(d49, d49Str);
            strcat(line3_pause_finished, d49Str);
            
            // D48 is always shown when D49 > 0 ("00" if D48 = 0)
            strcat(line3_pause_finished, ":");
            if (d48 == 0) {
              strcat(line3_pause_finished, "00");
            } else {
              formatTimeValue(d48, d48Str);
              strcat(line3_pause_finished, d48Str);
            }
            
            // D47 is always shown when D49 > 0 ("00" if D47 = 0)
            strcat(line3_pause_finished, ":");
            if (d47 == 0) {
              strcat(line3_pause_finished, "00");
            } else {
              formatTimeValue(d47, d47Str);
              strcat(line3_pause_finished, d47Str);
            }
            
            // Font size 1, Y = 20
            line3FontSize_pause_finished = 1;
            line3YPosition_pause_finished = 20;
          } else if (d48 > 0) {
            // D49 = 0, D48 > 0 -> show D48:D47
            char d48Str[10], d47Str[10];
            formatTimeValue(d48, d48Str);
            strcat(line3_pause_finished, d48Str);
            
            // D47 is always shown when D48 > 0 ("00" if D47 = 0)
            strcat(line3_pause_finished, ":");
            if (d47 == 0) {
              strcat(line3_pause_finished, "00");
            } else {
              formatTimeValue(d47, d47Str);
              strcat(line3_pause_finished, d47Str);
            }
            
            // Font size 2, Y = 18
            line3FontSize_pause_finished = 2;
            line3YPosition_pause_finished = 18;
          } else if (d47 > 0) {
            // D49 = 0, D48 = 0, D47 > 0 -> show only D47
            char d47Str[10];
            formatTimeValue(d47, d47Str);
            strcat(line3_pause_finished, d47Str);
            
            // Font size 2, Y = 19
            line3FontSize_pause_finished = 2;
            line3YPosition_pause_finished = 18;
          }
        }
        
        // Calculate text widths
        int line1Width_pause_finished = calcTextWidth(line1_pause_finished);
        int line2Width_pause_finished = calcTextWidth(line2_pause_finished);
        int line3Width_pause_finished = strlen(line3_pause_finished) > 0 ? calcTextWidth(line3_pause_finished) : 0;
        
        // Recalculate line 3 width based on font size
        if (strlen(line3_pause_finished) > 0) {
          int16_t x, y;
          uint16_t w, h;
          dma_display->setTextSize(line3FontSize_pause_finished);
          dma_display->getTextBounds(line3_pause_finished, 0, 0, &x, &y, &w, &h);
          line3Width_pause_finished = w;  // Exact width for current font size
          
          // If font size is 2, reduce width for tighter spacing (1px per char instead of 2px)
          if (line3FontSize_pause_finished == 2 && strlen(line3_pause_finished) > 1) {
            line3Width_pause_finished = line3Width_pause_finished - (strlen(line3_pause_finished) - 1);  // Reduce by (n-1) pixels for tighter spacing
          }
          
          dma_display->setTextSize(1);  // Reset to normal text size
        }
        
        // If display was inactive, reset state
        if (!displayActive) {
          displayActive = true;
        }
        
        // Calculate X positions for centering (all lines are static and centered)
        int centerX_line1_pause_finished = (dma_display->width() - line1Width_pause_finished) / 2;
        int centerX_line2_pause_finished = (dma_display->width() - line2Width_pause_finished) / 2;
        int centerX_line3_pause_finished = (dma_display->width() - line3Width_pause_finished) / 2;
        
        // Draw text
        dma_display->fillScreen(0);
        dma_display->setTextSize(1);
        
        // Scenario 5: Lines 1 and 2 (255, 128, 128) pink, line 3 (128, 128, 255) blue
        // Line 1: "PAUZA" static and centered
        dma_display->setTextColor(dma_display->color565(255, 128, 128));  // Pink
        dma_display->setCursor(centerX_line1_pause_finished, 0);
        dma_display->print(line1_pause_finished);
        
        // Line 2: "terminata" static and centered
        dma_display->setTextColor(dma_display->color565(255, 128, 128));  // Pink
        dma_display->setCursor(centerX_line2_pause_finished, 9);
        dma_display->print(line2_pause_finished);
        
        // Line 3: time (x:x:x or x:x or x) static and centered - with font size and Y based on rules
        if (strlen(line3_pause_finished) > 0) {
          dma_display->setTextColor(dma_display->color565(128, 128, 255));  // Blue
          dma_display->setCursor(centerX_line3_pause_finished, line3YPosition_pause_finished);
          
          if (line3FontSize_pause_finished == 2) {
            // Font size 2: draw each char separately with tighter spacing
            dma_display->setTextSize(2);
            for (int i = 0; i < strlen(line3_pause_finished); i++) {
              dma_display->print(line3_pause_finished[i]);
              // Reduce spacing between chars (from 2px to 1px for text size 2)
              if (i < strlen(line3_pause_finished) - 1) {
                int16_t x, y;
                uint16_t w, h;
                dma_display->getTextBounds(String(line3_pause_finished[i]), 0, 0, &x, &y, &w, &h);
                // Adjust cursor for tight spacing (1px instead of 2px)
                dma_display->setCursor(dma_display->getCursorX() - 1, dma_display->getCursorY());
              }
            }
            dma_display->setTextSize(1);  // Reset to normal text size
          } else {
            // Font size 1: draw normally
            dma_display->setTextSize(1);
            dma_display->print(line3_pause_finished);
          }
        }
      } else if (shouldDisplayUnavailable) {
        // SCENARIO 7: M120=0 -> show "Acest post este INDISPONIBIL E1"
        const char *line1_unavailable = "Acest post";
        const char *line2_unavailable = "este";
        const char *line3_unavailable = "INDISPONIBIL E1";
        
        // Calculate text widths
        int line1Width_unavailable = calcTextWidth(line1_unavailable);
        int line2Width_unavailable = calcTextWidth(line2_unavailable);
        int line3Width_unavailable = calcTextWidth(line3_unavailable);
        
        // If display was inactive, reset line 3 scroll
        if (!displayActive) {
          displayActive = true;
          scrollX_line3_scenario7 = dma_display->width();
        }
        
        // Calculate X positions for centering (lines 1 and 2 static, line 3 scrolling)
        int centerX_line1_unavailable = (dma_display->width() - line1Width_unavailable) / 2;
        int centerX_line2_unavailable = (dma_display->width() - line2Width_unavailable) / 2;
        int posX_line3_unavailable = scrollX_line3_scenario7;
        
        // Update line 3 scroll
        if (currentTime - lastScrollUpdate >= SCROLL_UPDATE_INTERVAL) {
          lastScrollUpdate = currentTime;
          scrollX_line3_scenario7--;
          // Reset when all characters passed the screen
          if (scrollX_line3_scenario7 + line3Width_unavailable < 0) {
            // All characters passed the screen, reset to start again
            scrollX_line3_scenario7 = dma_display->width();
          }
        }
        
        // Draw text
        dma_display->fillScreen(0);
        // Scenario 7: (255, 0, 0) - red
        dma_display->setTextColor(dma_display->color565(255, 0, 0));
        dma_display->setTextSize(1);
        
        // Line 1: "Acest post" static and centered
        dma_display->setCursor(centerX_line1_unavailable, 2);
        dma_display->print(line1_unavailable);
        
        // Line 2: "este" static and centered
        dma_display->setCursor(centerX_line2_unavailable, 12);
        dma_display->print(line2_unavailable);
        
        // Line 3: "INDISPONIBIL E1" scrolling right to left
        dma_display->setCursor(posX_line3_unavailable, 22);
        dma_display->print(line3_unavailable);
      } else if (shouldDisplayCommError) {
        // SCENARIO 6: PLC communication error -> show "Acest post este INDISPONIBIL E0"
        const char *line1_comm_error = "Acest post";
        const char *line2_comm_error = "este";
        const char *line3_comm_error = "INDISPONIBIL E0";
        
        // Calculate text widths
        int line1Width_comm_error = calcTextWidth(line1_comm_error);
        int line2Width_comm_error = calcTextWidth(line2_comm_error);
        int line3Width_comm_error = calcTextWidth(line3_comm_error);
        
        // If display was inactive, reset line 3 scroll
        if (!displayActive) {
          displayActive = true;
          scrollX_line3_scenario6 = dma_display->width();
        }
        
        // Calculate X positions for centering (lines 1 and 2 static, line 3 scrolling)
        int centerX_line1_comm_error = (dma_display->width() - line1Width_comm_error) / 2;
        int centerX_line2_comm_error = (dma_display->width() - line2Width_comm_error) / 2;
        int posX_line3_comm_error = scrollX_line3_scenario6;
        
        // Update line 3 scroll
        if (currentTime - lastScrollUpdate >= SCROLL_UPDATE_INTERVAL) {
          lastScrollUpdate = currentTime;
          scrollX_line3_scenario6--;
          // Reset when all characters passed the screen
          if (scrollX_line3_scenario6 + line3Width_comm_error < 0) {
            // All characters passed the screen, reset to start again
            scrollX_line3_scenario6 = dma_display->width();
          }
        }
        
        // Draw text
        dma_display->fillScreen(0);
        // Scenario 6: (255, 0, 0) - red
        dma_display->setTextColor(dma_display->color565(255, 0, 0));
        dma_display->setTextSize(1);
        
        // Line 1: "Acest post" static and centered
        dma_display->setCursor(centerX_line1_comm_error, 2);
        dma_display->print(line1_comm_error);
        
        // Line 2: "este" static and centered
        dma_display->setCursor(centerX_line2_comm_error, 12);
        dma_display->print(line2_comm_error);
        
        // Line 3: "INDISPONIBIL E0" scrolling right to left
        dma_display->setCursor(posX_line3_comm_error, 22);
        dma_display->print(line3_comm_error);
      } else {
        // If conditions are not met, clear the screen
        if (displayActive) {
          dma_display->fillScreen(0);
          displayActive = false;
          scrollX = dma_display->width(); // Reset scroll for scenario 1
          scrollX_line1_scenario3 = dma_display->width(); // Reset scroll for scenario 3
          scrollX_line2_scenario3 = dma_display->width(); // Reset scroll for scenario 3
          scrollX_scenario5 = dma_display->width(); // Reset scroll for scenario 5
          scrollX_line3_scenario12 = dma_display->width(); // Reset scroll for scenario 1.2
          scrollX_line3_scenario6 = dma_display->width(); // Reset scroll for scenario 6 (PLC communication error)
          scrollX_line3_scenario7 = dma_display->width(); // Reset scroll for scenario 7 (M120=0 unavailable)
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));  // Yield control to FreeRTOS
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);  // Wait up to 5 seconds for Serial
  }
  
  // Create task for Modbus communication on Core 0
  xTaskCreatePinnedToCore(
    taskModbusCommunication,    // Task function
    "ModbusComm",               // Task name
    8192,                       // Stack size (bytes)
    NULL,                       // Task parameters
    1,                          // Priority (1 = normal)
    NULL,                       // Task handle (not needed)
    0                           // Core 0
  );
  
  // Create task for LED matrix display on Core 1
  xTaskCreatePinnedToCore(
    taskMatrixDisplay,          // Task function
    "MatrixDisplay",            // Task name
    8192,                       // Stack size (bytes)
    NULL,                       // Task parameters
    1,                          // Priority (1 = normal)
    NULL,                       // Task handle (not needed)
    1                           // Core 1
  );
  
  // Setup ends here, but tasks continue running
}

void loop() {
  // Main loop stays empty, all logic runs in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));  // Long delay, tasks do all the work
}
