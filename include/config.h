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

#define STR32     32
#define STR64     64

struct calibration {
  float V;
  float I;
  float P;
} __attribute__((__packed__));

#define SCHED_ON_ENABLED  0x01
#define SCHED_OFF_ENABLED 0x02
#define SCHED_RANDOM      0x04

struct schedule {
  uint8_t   flags;
  uint8_t   h_on;
  uint8_t   m_on;
  uint8_t   h_off;
  uint8_t   m_off;
} __attribute__((__packed__));

#define CFG_RELAY_ON_BOOT   0x01
#define CFG_SCHEDULE        0x02

struct config {
  uint32_t            signature;
  char                hostname[STR32];
  char                ssid[STR64];
  char                psk[STR64];
  char                ntpserver[STR64];
  char                timezone[STR32];
  struct calibration  calibration;
  uint8_t             flags;
  uint8_t             onDelay;
  struct schedule     schedule[7];
} __attribute__((__packed__));