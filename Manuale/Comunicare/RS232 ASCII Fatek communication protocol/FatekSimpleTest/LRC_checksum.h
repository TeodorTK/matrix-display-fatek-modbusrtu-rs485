#ifndef LRC_CHECKSUM_H
#define LRC_CHECKSUM_H

#include <Arduino.h>

/**
 * Calculate LRC (Longitudinal Redundancy Check) checksum
 * Converted from Python LRC_checksum_calculator.py
 * 
 * @param data The data string to calculate LRC for (without STX/ETX)
 * @return String containing the LRC checksum in hex format (2 characters)
 */
String LRC_calc(String data);

#endif
