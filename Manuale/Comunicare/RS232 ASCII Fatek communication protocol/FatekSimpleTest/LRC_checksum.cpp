#include "LRC_checksum.h"

/**
 * Calculate LRC (Longitudinal Redundancy Check) checksum
 * Converted from Python: LRC_checksum_calculator.py
 * 
 * Algorithm:
 * 1. Convert each character to hex
 * 2. Add STX (0x02) to the buffer
 * 3. Sum all hex values
 * 4. Take last 2 hex digits as LRC
 */
String LRC_calc(String data) {
    int sum = 0x02; // Start with STX value
    
    // Sum all character values
    for (int i = 0; i < data.length(); i++) {
        sum += (int)data[i];
    }
    
    // Get last 2 hex digits
    String lrc = String(sum, HEX);
    lrc.toUpperCase();
    
    // If more than 2 characters, take only last 2
    if (lrc.length() > 2) {
        lrc = lrc.substring(lrc.length() - 2);
    } else if (lrc.length() == 1) {
        // Pad with leading zero if needed
        lrc = "0" + lrc;
    }
    
    return lrc;
}
