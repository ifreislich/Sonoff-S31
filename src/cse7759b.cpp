/* 
 * Copyright (c) 2024-2025, Ian Freislich
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <Arduino.h>
#include <stdint.h>

#include "cse7759b.h"
#include "nvdata.h"
#include "config.h"
#include "states.h"

double          power = 0;
double          ave_power = 0;
uint8_t         ave_count = 0;
double          voltage = 0;
double          current = 0;
double          energy = 0;
extern struct nvHeader nvHeader;
extern uint8_t  state;
uint32_t        ovflow;
uint16_t        restoredPulses;
uint8_t         packet[24];
int             err;

// CSE77xx error codes.
#define CSE_ERROR_OK            0
#define CSE_ERROR_OUT_OF_RANGE  1
#define CSE_ERROR_WARM_UP       2
#define CSE_ERROR_TIMEOUT       3
#define CSE_ERROR_UNKNOWN_ID    4
#define CSE_ERROR_CRC           5
#define CSE_ERROR_CALIBRATION   8
#define CSE_ERROR_OTHER         99
#define V1R 1.0                     // 0.001R current shunt.
#define V2R 1.0                     // 1MR voltage divider.

#define H1_COEF_STORAGE_ABNORMAL    0x01
#define H1_POWER_CYCLE_EXCEEDED     0x02
#define H1_CURRENT_CYCLE_EXCEEDED   0x04
#define H1_VOLTAGE_CYCLE_EXCEEDED   0x08
#define H1_ABNORMAL                 0xF0
#define H1_UNCALIBRATED             0xAA
#define H1_CALIBRATED               0x55

#define ADJ_CAL_SEL_MASK            0x07
#define ADJ_COEF_LOAD_ERROR         0x08
#define ADJ_POWER_CYCLE_COMPLETE    0x10
#define ADJ_CURRENT_CYCLE_COMPLETE  0x20
#define ADJ_VOLTAGE_CYCLE_COMPLETE  0x40
#define ADJ_PULSE_OVERFLOW_MASK     0x80


static bool
checkSum(void) {
  unsigned char cksum = 0;

  for (uint8_t i = 2; i < 23; i++)
    cksum += packet[i];

  return cksum == packet[23];
}

static void
processPacket(void) {
  if (!checkSum()) {
    err = CSE_ERROR_CRC;
    return;
  }

  if (packet[0] == H1_UNCALIBRATED) {
    err = CSE_ERROR_CALIBRATION;
    return;
  }

  // Extract coefficients
  uint32_t kV = (packet[2]  << 16 | packet[3]  << 8 | packet[4]);
  uint32_t kI = (packet[8]  << 16 | packet[9]  << 8 | packet[10]);
  uint32_t kP = (packet[14] << 16 | packet[15] << 8 | packet[16]);

  uint8_t adj = packet[20];
  static uint8_t  lastAdj = adj;

  voltage = 0;
  if (!((packet[0] & H1_ABNORMAL ) && (packet[0] & H1_VOLTAGE_CYCLE_EXCEEDED)) && (adj & ADJ_VOLTAGE_CYCLE_COMPLETE)) {
    uint32_t tV = packet[5] << 16 | packet[6] << 8 | packet[7];
    voltage = cfg.calibration.V * (kV * V2R) / tV;
  }

  power = 0;
  current = 0;
  if (!((packet[0] & H1_ABNORMAL ) && (packet[0] & H1_POWER_CYCLE_EXCEEDED)) && (adj & ADJ_POWER_CYCLE_COMPLETE)) {
    uint32_t tP = packet[17] << 16 | packet[18] << 8 | packet[19];
    power = cfg.calibration.P * (kP * V2R) / (tP * V1R);

    if (adj & ADJ_CURRENT_CYCLE_COMPLETE) {
      uint32_t tI = packet[11] << 16 | packet[12] << 8 | packet[13];
      current = cfg.calibration.I * kI / (tI * V1R);
    }
  }

  // I think that kP is constant but calculate it anyway. kP = 5264000 
  uint16_t CFpulses = packet[21] << 8 | packet[22];
  double Fcf = 1000000000.0/kP;
  if ((adj & 0x80) != (lastAdj & 0x80)) { // adj:7 has toggled state
    ovflow++;
    lastAdj = adj;
  }
  if (state & STATE_FRAM_PRESENT) {
    nvHeader.ovflow = ovflow;
    nvHeader.pulses = CFpulses;
  }
  energy = (ovflow * 65536.0 + CFpulses + restoredPulses) / (Fcf * 3600);
}

void
readCse7759b(void) {
  static uint8_t index = 0;

  err = CSE_ERROR_OTHER;

  while (Serial.available() > 0) {
    uint8_t input = Serial.read();

    if (index == 0) {
      if ((input != 0x55) && (input < 0xF0))
        continue;
    }
    else if (index == 1) {
      if (input != 0x5A) {
        index = 0;
        continue;
      }
    }

    packet[index++] = input;

    if (index > 23) {
      Serial.flush();
      break;
    }
  }

  if (index == 24) {
    err = CSE_ERROR_OK;
    processPacket();
    index = 0;
  }
  if (state & STATE_FRAM_PRESENT) {
    ave_power += power;
    ave_count++;
  }
}