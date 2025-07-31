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
 * Pin header on the S31: GND D-RX:SDA:GPIO4 D-TX:SCL:GPIO5 GPIO1:TX GPIO3:RX 3.3V
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <coredecls.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <FastCRC.h>
#include <FRAM.h>
#include <LittleFS.h>
#include <SimpleTimer.h>
#include <stdint.h>
#include <sys/time.h>
#include <WiFiClient.h>

#include "cse7759b.h"
#include "config.h"
#include "nvdata.h"
#include "states.h"

#define NAME      "S31"
#define VERSION   1.0
#define SIGNATURE 0x1a2b3b4e
#define NVVERSION 1

struct config   cfg;
struct nvHeader nvHeader;
extern double   ave_power;      //cse7766.cpp
extern uint8_t  ave_count;      //cse7766.cpp
extern uint32_t ovflow;         //cse7766.cpp
extern uint16_t restoredPulses; //cse7766.cpp

#define BUTTON_PERIOD   100
#define BUTTON_TIMEOUT  10
void ntpCallBack(void);
void resetConfig(void);
void checkSchedule(void);
void APModeLED(void);
void buttonCheck(void);
void nvInit(void);
void saveNvHeader(void);
void saveNvLog(void);

void handleConfig(void);
void handleDygraphCSS(void);
void handleDygraphJS(void);
void handleFavIcon(void);
void handleNvData(void);
void handleOff(void);
void handleOn(void);
void handlePowerCycle(void);
void handleReboot(void);
void handleRoot(void);
void handleSave(void);
void handleSchedule(void);
void handleScheduleSave(void);

FRAM                fram;
ESP8266WebServer    web(80);
WiFiEventHandler	  eventConnected, eventDisconnected, eventGotIP;
static SimpleTimer  timer;
const char         *daysOfWeek[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

time_t  bootTime = 0;
uint8_t state;

#define BUTTON  0         // Sonoff pushbutton (LOW == pressed).
#define RELAY   12        // Sonoff relay (HIGH == ON).
#define LED     13        // Sonoff blue LED (LOW == ON).

void
setup(void)
{
  const char * headerkeys[] = {"Accept-Encoding"} ;

  state = 0;
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(0, cfg);
  if (cfg.signature != SIGNATURE)
    resetConfig();
  //memset(&cfg.schedule, '\0', sizeof(struct schedule) * 7);
  //EEPROM.put(0, cfg);
  //EEPROM.commit();
  state |= (cfg.flags & CFG_RELAY_ON_BOOT ? STATE_RELAY : 0);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, state & STATE_RELAY);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  restoredPulses = 0;
  ovflow = 0;
  Wire.begin();
  Wire.setClock(1000000);
  if (fram.begin(0x50) == FRAM_OK) {
		state |= STATE_FRAM_PRESENT;
    nvInit();
    ovflow = nvHeader.ovflow;
    restoredPulses = nvHeader.restoredPulses;
    // Restore the meter pulses on power-cycle
    if (ESP.getResetInfoPtr()->reason == REASON_DEFAULT_RST) {
      restoredPulses = nvHeader.restoredPulses + nvHeader.pulses;
      if (restoredPulses < nvHeader.restoredPulses)
        nvHeader.ovflow = ++ovflow;
      nvHeader.restoredPulses = restoredPulses;
    }
	}

  eventGotIP = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
    state |= STATE_GOT_IP_ADDRESS;
  });
  eventConnected = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected& event) {
  });
  eventDisconnected = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    state &= ~STATE_GOT_IP_ADDRESS;
    if (~state & STATE_OTA_OR_REBOOT && *cfg.ssid && *cfg.psk) {
      WiFi.begin(cfg.ssid, cfg.psk);
    }
  });
  LittleFS.begin();
  // Serial  - TX = GPIO1, RX = GPIO3 [CSE7766 and RX/TX]
  // Serial1 - TX = GPIO2, RX = GPIO8 Unused
  Serial.flush();
  Serial.begin(4800);

  // setup Web Server
  static String content;
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(cfg.hostname);
  settimeofday_cb(ntpCallBack);

  // ArduinoOTA.setPassword(F("admin"))
  ArduinoOTA.onStart([]() {
    state |= STATE_OTA_OR_REBOOT;
    switch (ArduinoOTA.getCommand()) {
      case U_FLASH:
    	break;
      case U_FS:
        LittleFS.end();
    	break;
      if (state & STATE_FRAM_PRESENT)
        saveNvHeader();
    }
  });
  ArduinoOTA.onEnd([]() {
    for (int i=20; i; i--) {
      digitalWrite(LED,!digitalRead(LED));
      delay(50);
    }
    digitalWrite(LED, 1);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t pwm = 8, direction = 1;
    analogWrite(LED, pwm);
    if (direction)
      pwm += 8;
    else
      pwm -= 8;
    
    if (pwm == 8 || pwm ==248)
      direction = !direction;
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%d]\r\n", error);
    state &= ~STATE_OTA_OR_REBOOT;
    switch (error) {
      case OTA_AUTH_ERROR:
    	  //Serial.println("Auth Failed");
    	  break;
      case OTA_BEGIN_ERROR:
    	  //Serial.println("Begin Failed");
    	  break;
      case OTA_CONNECT_ERROR:
    	  //Serial.println("Connect Failed");
    	  break;
      case OTA_RECEIVE_ERROR:
    	  //Serial.println("Receive Failed");
    	  break;
      case OTA_END_ERROR:
    	  //Serial.println("End Failed");
    	  break;
    }
  });
  web.on("/config", handleConfig);
  web.on("/data.txt", handleNvData);
  web.on("/dygraph.css", handleDygraphCSS);
  web.on("/dygraph.min.js", handleDygraphJS);
  web.on("/favicon.ico", handleFavIcon);
  web.on("/", handleRoot);
  web.on("/off", handleOff);
  web.on("/on", handleOn);
  web.on("/powercycle", handlePowerCycle);
  web.on("/reboot", handleReboot);
  web.on("/save", handleSave);
  web.on("/schedule", handleSchedule);
  web.on("/schedulesave", handleScheduleSave);
  web.collectHeaders(headerkeys, (size_t)1);

  WiFi.mode(WIFI_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.begin(cfg.ssid, cfg.psk);
  MDNS.begin(cfg.hostname);

  if (cfg.ntpserver[0])
    configTzTime(cfg.timezone, cfg.ntpserver);
  else
    setTZ(cfg.timezone);

  // Wait for connection
  for (int i = 20; i && WiFi.status() != WL_CONNECTED; i--) {
    digitalWrite(LED, LOW);
    delay(300);
    digitalWrite(LED, HIGH);
    delay(300);
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(NAME, "");
  }

  // Start a timer for checking button presses @ 100ms intervals.
  timer.setInterval(BUTTON_PERIOD, buttonCheck);
  timer.setInterval(1000, readCse7759b);
  timer.setInterval(1000, APModeLED);
  timer.setInterval(1000, checkSchedule);
  if (state & STATE_FRAM_PRESENT) {
    timer.setInterval(5000, saveNvHeader);
    timer.setInterval(10000, saveNvLog);
  }

  // Switch LED on to signal initialization complete.
  digitalWrite(LED, LOW);

  ArduinoOTA.begin();
  web.begin();
}

void
loop(void)
{
  timer.run();
  ArduinoOTA.handle();
  web.handleClient();
  state &= ~STATE_OTA_OR_REBOOT;
}

void
buttonCheck(void)
{
  bool button = (digitalRead(BUTTON));
  static int count = 0;

  if(!button) {
    if(++count >= BUTTON_TIMEOUT * 1000 / BUTTON_PERIOD)
      resetConfig();
  }
  else
    count = 0;

  if (!button && state & STATE_DEBOUNCE_TIMEOUT) {
    state = (state & ~STATE_RELAY) | (~state & STATE_RELAY);
    digitalWrite(RELAY, state & STATE_RELAY);
    state &= ~STATE_DEBOUNCE_TIMEOUT;
    delay(50);
  }
  else if (button) {
    state |= STATE_DEBOUNCE_TIMEOUT;
  }
}

void
checkSchedule(void)
{
  static int      offset_on = 0, offset_off = 0;
  static uint8_t  wday = 8;
  time_t          t = time(NULL);
  struct tm      *tm_off, tm_on;

  if (~cfg.flags & CFG_SCHEDULE)
    return;

  tm_off = localtime(&t);
  tm_on = *tm_off;
  if (wday != tm_off->tm_wday) {
    offset_on = floor(drand48() * 1800.0 - 900);
    offset_off = floor(drand48() * 1800.0 - 900);
    wday = tm_off->tm_wday;
  }

  if (cfg.schedule[tm_off->tm_wday].flags & SCHED_RANDOM) {
    t += offset_on;
    tm_off = localtime(&t);
    tm_on = *tm_off;
    t += offset_off - offset_on;
    tm_off = localtime(&t);
  }

  if (~state & STATE_RELAY && cfg.schedule[wday].flags & SCHED_ON_ENABLED && tm_on.tm_hour == cfg.schedule[wday].h_on && tm_on.tm_min == cfg.schedule[wday].m_on) {
    digitalWrite(RELAY, HIGH);
    state |= STATE_RELAY;
  }
  else if (state & STATE_RELAY && cfg.schedule[wday].flags & SCHED_OFF_ENABLED && tm_off->tm_hour == cfg.schedule[wday].h_off && tm_off->tm_min == cfg.schedule[wday].m_off) {
    digitalWrite(RELAY, LOW);
    state &= ~STATE_RELAY;
  }
}

void
APModeLED(void)
{
  if (WiFi.getMode() == WIFI_AP)
    digitalWrite(LED, !digitalRead(LED));
}

void
ntpCallBack(void)
{
  if (bootTime == 0 && ~state & STATE_NTP_GOT_TIME) {
    bootTime = time(NULL) - millis() / 1000;
    srand48(bootTime);
  }

  state |= STATE_NTP_GOT_TIME;
}

void
resetConfig(void)
{
  memset(&cfg, 0, sizeof(struct config));
  // Note: These strings and null termination must be less than 64 chars.
  strcpy(cfg.ssid, "none");
  strcpy(cfg.psk, "none");
  // cfg.ntpserver when empty will use the value from DHCP
  // Note: These strings and null termination must be less than 32 chars.
  strcpy(cfg.hostname, NAME);
  strcpy(cfg.timezone, "EST5EDT,M3.2.0,M11.1.0");
  cfg.calibration = {1.01, 0.995, 1.00};
  cfg.signature = SIGNATURE;
  EEPROM.put(0, cfg);
  EEPROM.commit();
  //ESP.restart();
}

void
nvInit(void)
{
  FastCRC16	  CRC16;
  uint16_t    crc;

  fram.read(0, (uint8_t *)&nvHeader, sizeof(nvHeader));
  crc = CRC16.ccitt((uint8_t *)&nvHeader, sizeof(struct nvHeader) - 2);
  if (nvHeader.version != NVVERSION || nvHeader.crc != crc) {
    memset(&nvHeader, '\0', sizeof(struct nvHeader));
    nvHeader.version = NVVERSION;
    nvHeader.crc = crc;
    saveNvHeader();
  }
}

void
saveNvHeader(void)
{
  FastCRC16	 CRC16;
  
  nvHeader.crc = CRC16.ccitt((uint8_t *)&nvHeader, sizeof(struct nvHeader) - 2);
  fram.write(NV_HEADER_OFFSET, (uint8_t *)&nvHeader, sizeof(nvHeader));
}

void
saveNvLog(void)
{
  struct nvLog    nvLog;

  if (state & STATE_NTP_GOT_TIME) {
    nvLog.time = time(NULL);
    if (ave_count)
      nvLog.power = ave_power / ave_count;
    else
      nvLog.power = power;
    ave_power = power;
    ave_count = 1;
    fram.write(NV_LOG_OFFSET + nvHeader.nvLogLast * sizeof(struct nvLog), (uint8_t *)&nvLog, sizeof(struct nvLog));
    nvHeader.nvLogLast++;
    nvHeader.nvLogLast %= NV_LOG_MAX;
    if (nvHeader.nvLogLast == nvHeader.nvLogFirst) {
      nvHeader.nvLogFirst++;
      nvHeader.nvLogFirst %= NV_LOG_MAX;
    }
    saveNvHeader();
  }
}

/*
 * Web Server
 */

void
handleRoot(void)
{
  WiFiClient client = web.client();
  struct tm	*tm;
  char		   timestr[20];
  double	   va, vars;
  time_t	   t = time(NULL), uptime = 0;
  int		     sec, min, hr, day;

  va = voltage * current;
  vars = va * va- power * power;
  vars = vars > 0 ? sqrt(vars) : 0;

  tm = localtime(&t);
  strftime(timestr, 20, "%F %T", tm);

  if (state & STATE_NTP_GOT_TIME)
      uptime = t - bootTime;
  
  sec = uptime % 60;
  min = (uptime / 60) % 60;
  hr = (uptime / 3600) % 24;
  day = uptime / 86400;

  client.print("HTTP/1.1 200 OK\nContent-Type: text/html\n\n");
  client.printf("<html lang='en'>"
    "<head>"
    "<meta http-equiv='Refresh' content='60; url=/'>"
    "<meta charset='UTF-8'>"
    "<title>%s</title>"
    "%s"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<style>"
      "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }"
      ".dygraph-legend {text-align: right;background: none;}"
    "</style>"
    "</head>"
    "<body>"
    "<h1>Switch %s</h1>"
    "%s<p>"
    "%.2fV %.3fA<br>"
    "%.2fW<br>"
    "%.2fVA<br>"
    "%.2fVAR<br>"
    "PF=%.1f<br>"
    "%.6lfkWh<br>"
    "<p>Plug is %s, turn %s"
    "%s"
    "%s"
    "<p><a href='/config'>Configuration</a>"
    "%s"
    "<p><font size=1>"
    "Uptime: %d days %02d:%02d:%02d"
    "<br>Firmware: %s"
    "<br>Boot reason: %s"
    "</font>"
    "%s"
    "</body>"
    "</html>",
    cfg.hostname, 
    state & STATE_FRAM_PRESENT ? "<script src='dygraph.min.js'></script><link rel='stylesheet' type='text/css' href='dygraph.css'>" : "",
    cfg.hostname, timestr, voltage, current, power, va, vars,
    voltage > 0 && current > 0 ? power / voltage / current : 1,
    energy,
    state & STATE_RELAY ? "on" : "off", state & STATE_RELAY ? "<a href='/off'>Off</a>" : "<a href='/on'>On</a>",
    state & STATE_RELAY ? "<p><a href='/powercycle'>Load Power Cycle</a>" : "",
    state & STATE_FRAM_PRESENT ? "<div id='history'></div>" : "",
    cfg.flags & CFG_SCHEDULE ? "<p><a href='/schedule'>Schedule</a>" : "",
    day, hr, min, sec, AUTO_VERSION, ESP.getResetReason().c_str(),
    state & STATE_FRAM_PRESENT ? R"(<script type="text/javascript">
      Dygraph.onDOMready(function onDOMready() {
        new Dygraph(document.getElementById('history'), 'data.txt', {
          title: 'Power history',
          width: 600,
          height: 300,
          legend: 'always',
          showRangeSelector: true,
        });
      });</script>)" : "");
  client.stop();
} 

void
handlePowerCycle(void)
{
  WiFiClient client = web.client();

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='1; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "%s<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname, state & STATE_RELAY ? "Power cycling" : "Not powercycling");
  client.stop();
  if (state & STATE_RELAY) {
    digitalWrite(RELAY, LOW);
    delay(1000);
    digitalWrite(RELAY, HIGH);
  }
}

void
handleOn(void)
{
  WiFiClient client = web.client();

  digitalWrite(RELAY, HIGH);
  state |= STATE_RELAY;
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='1; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "Relay is on<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname);
  client.stop();
}

void
handleOff(void)
{
  WiFiClient client = web.client();

  digitalWrite(RELAY, LOW);
  state &= ~STATE_RELAY;
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='1; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "Relay is off<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname);
  client.stop();
}

void
handleConfig()
{
  WiFiClient client = web.client();

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>\n"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "<form method='post' action='/save' name='Configuration'/>\n"
    "<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
    "<tr><td width='40%%'>Name:</td><td><input name='name' type='text' value='%s' size='31' maxlength='31'></td></tr>\n"
    "<tr><td width='40%%'>SSID:</td><td><input name='ssid' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
    "<tr><td width='40%%'>WPA Pass Phrase:</td><td><input name='psk' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
    "<tr><td width='40%%'>NTP Server:</td><td><input name='ntp' type='text' value='%s' size='31' maxlength='63' "
    	"pattern='^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\\-]*[a-zA-Z0-9])\\.)*([A-Za-z0-9]|[A-Za-z0-9][A-Za-z0-9\\-]*[A-Za-z0-9])$' title='A valid hostname'></td></tr>\n"
    "<tr><td width='40%%'>Timezone:</td><td><input name='tz' type='text' value='%s' size='31' maxlength='31'></td></tr>\n"
    "<tr><td width='40%%'>On at boot:</td><td><input name='relay' type='checkbox' value='true' %s></td></tr>\n"
    "<tr><td width='40%%'>Schedule:</td><td><input name='sched' type='checkbox' value='true' %s></td></tr>\n"
    "<tr><td width='40%%'>Correction factor V:</td><td><input name='vf' type='text' value='%5.3f' size='31' pattern='^[0-1]\\.[0-9]{1,3}$' title='float with up to 3 decimals'></td></tr>\n"
    "<tr><td width='40%%'>Correction factor I:</td><td><input name='if' type='text' value='%5.3f' size='31' pattern='^[0-1]\\.[0-9]{1,3}$' title='float with up to 3 decimals'></td></tr>\n"
    "<tr><td width='40%%'>Correction factor P:</td><td><input name='pf' type='text' value='%5.3f' size='31' pattern='^[0-1]\\.[0-9]{1,3}$' title='float with up to 3 decimals'></td></tr>\n"
    "</table><p>"
    "<input name='Save' type='submit' value='Save'/>\n"
    "<br></form>"
    "<form method='post' action='/reboot' name='Reboot'/>\n"
    "<input name='Reboot' type='submit' value='Reboot'/>\n"
    "<br></form>\n"
    "</body>\n"
    "</html>",
    cfg.hostname, cfg.hostname, cfg.hostname, cfg.ssid, cfg.psk, cfg.ntpserver, cfg.timezone,
    cfg.flags & CFG_RELAY_ON_BOOT ? "checked" : "",
    cfg.flags & CFG_SCHEDULE ? "checked" : "",
    cfg.calibration.V, cfg.calibration.I, cfg.calibration.P);
  client.stop();
}

void
handleSave(void)
{
  WiFiClient client = web.client();

  if (web.hasArg("vf"))
    cfg.calibration.V = web.arg("vf").toFloat();
  if (web.hasArg("if"))
    cfg.calibration.I = web.arg("if").toFloat();
  if (web.hasArg("ef"))
    cfg.calibration.P = web.arg("pf").toFloat();
  if (web.hasArg("name")) {
    strncpy(cfg.hostname, web.arg("name").c_str(), STR32);
    cfg.ssid[STR32 - 1] = '\0';
  }
  if (web.hasArg("ssid")) {
    strncpy(cfg.ssid, web.arg("ssid").c_str(), STR64);
    cfg.ssid[STR64 - 1] = '\0';
  }
  if (web.hasArg("psk")) {
    strncpy(cfg.psk, web.arg("psk").c_str(), STR64);
    cfg.psk[STR64 - 1] = '\0';
  }
  if (web.hasArg("relay"))
			cfg.flags |= CFG_RELAY_ON_BOOT;
		else
			cfg.flags &= ~CFG_RELAY_ON_BOOT;
  if (web.hasArg("sched"))
			cfg.flags |= CFG_SCHEDULE;
		else
			cfg.flags &= ~CFG_SCHEDULE;

  EEPROM.put(0, cfg);
  EEPROM.commit();

  if (cfg.ntpserver[0])
    configTzTime(cfg.timezone, cfg.ntpserver);
  else
    setTZ(cfg.timezone);

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='1; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "Saved<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname);
  client.stop();
  delay(100);
  WiFi.hostname(cfg.hostname);
  WiFi.begin(cfg.ssid, cfg.psk);
  MDNS.begin(cfg.hostname);
};

void
handleReboot(void)
{
  WiFiClient client = web.client();

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='10; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "Rebooting<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname);
  client.stop();
  delay(100);
  state |= STATE_OTA_OR_REBOOT;
  if (state & STATE_FRAM_PRESENT)
    saveNvHeader();
  ESP.restart();
}

void
handleSchedule(void)
{
  WiFiClient client = web.client();

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "</body>\n"
    "<html>"
    "<form method='post' action='/schedulesave' name='Schedule'>\n"
    "<table border=0 width='520' cellspacing=4 cellpadding=0>\n",
    cfg.hostname, cfg.hostname);

  for (int i = 0; i < 7; i++) {
    client.printf("<tr><td><b>%s:</b></td></tr>\n"
		"<tr><td>on:<input name='eon%d' type='checkbox' value='true' %s>"
    "<input name='on%d' type='time' value='%02d:%02d'></td>"
		"<td>off:<input name='eof%d' type='checkbox' value='true' %s>"
    "<input name='off%d' type='time' value='%02d:%02d'></td>"
    "<td>Randomize:<input name='r%d' type='checkbox' value='true' %s></td></tr>\n"
    "<tr><td>&nbsp</td></tr>",
    daysOfWeek[i], 
    i, cfg.schedule[i].flags & SCHED_ON_ENABLED ? "checked" : "",
    i, cfg.schedule[i].h_on, cfg.schedule[i].m_on,
    i, cfg.schedule[i].flags & SCHED_OFF_ENABLED ? "checked" : "",
    i, cfg.schedule[i].h_off, cfg.schedule[i].m_off,
    i, cfg.schedule[i].flags & SCHED_RANDOM ? "checked" : "");
  }

  client.printf("</table><p>"
    "<input name='Save' type='submit' value='Save'>\n"
    "</form>"
    "</body>"
    "</html>");
  client.stop();
}

void
handleScheduleSave(void)
{
  WiFiClient client = web.client();
  String  value;
  char    arg[5];

  for (int i = 0; i < 7; i++) {
    snprintf(arg, 5, "on%d", i);
    arg[4] = '\0';
    value = web.arg(arg);
    if (value.length()) {
      int h, m;
      if (sscanf(value.c_str(), "%d:%d", &h, &m) == 2 ) {
        if (h >= 0 && h <= 23)
          cfg.schedule[i].h_on = h;
        if (m >= 0 && m <= 59)
          cfg.schedule[i].m_on = m;
      }
    }

    snprintf(arg, 5, "off%d", i);
    arg[4] = '\0';
    value = web.arg(arg);
    if (value.length()) {
      int h, m;
      if (sscanf(value.c_str(), "%d:%d", &h, &m) == 2 ) {
        if (h >= 0 && h <= 23)
          cfg.schedule[i].h_off = h;
        if (m >= 0 && m <= 59)
          cfg.schedule[i].m_off = m;
      }
    }

    snprintf(arg, 5, "eon%d", i);
    if (web.hasArg(arg))
      cfg.schedule[i].flags |= SCHED_ON_ENABLED;
    else
      cfg.schedule[i].flags &= ~SCHED_ON_ENABLED;

    snprintf(arg, 5, "eof%d", i);
    if (web.hasArg(arg))
      cfg.schedule[i].flags |= SCHED_OFF_ENABLED;
    else
      cfg.schedule[i].flags &= ~SCHED_OFF_ENABLED;

    snprintf(arg, 5, "r%d", i);
    if (web.hasArg(arg))
      cfg.schedule[i].flags |= SCHED_RANDOM;
    else
      cfg.schedule[i].flags &= ~SCHED_RANDOM;
  }
  EEPROM.put(0, cfg);
  EEPROM.commit();

  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.printf("<html>"
    "<head>"
    "<title>%s</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "<link rel='icon' type='image/x-icon' href='/favicon.ico'>"
    "<meta http-equiv='Refresh' content='1; url=/'>"
    "</head>\n"
    "<body>\n"
    "<h1>Switch %s</h1>"
    "Saved<br>"
    "</body>\n"
    "</html>", cfg.hostname, cfg.hostname);
  client.stop();
}

void
handleDygraphJS(void)
{
  String encoding;
  File file;

  if (web.hasHeader("accept-encoding"))
    encoding = web.header("Accept-Encoding");

  if (encoding.startsWith("gzip"))
    file = LittleFS.open("/dygraph.min.js.gz", "r");
  else
    file = LittleFS.open("/dygraph.min.js", "r");

  web.sendHeader("X-Recieved", encoding, false);
  web.sendHeader("Cache-Control", "public, max-age=86400, immutable", false);
  web.streamFile(file, "application/javascript");
  file.close();
}

void
handleDygraphCSS(void)
{
  String encoding;
  File file;

  if (web.hasHeader("accept-encoding"))
    encoding = web.header("Accept-Encoding");

  if (encoding.startsWith("gzip"))
    file = LittleFS.open("/dygraph.css.gz", "r");
  else
    file = LittleFS.open("/dygraph.css", "r");
  
  web.sendHeader("Cache-Control", "public, max-age=86400, immutable", false);
  web.streamFile(file, "text/css");
  file.close();
}

void
handleFavIcon(void)
{
  String encoding;
  File file;

  if (web.hasHeader("accept-encoding"))
    encoding = web.header("Accept-Encoding");

  if (encoding.startsWith("gzip"))
    file = LittleFS.open("/favicon.ico.gz", "r");
  else
    file = LittleFS.open("/favicon.ico", "r");
  
  web.sendHeader("Cache-Control", "public, max-age=86400, immutable", false);
  web.streamFile(file, "image/x-icon");
  file.close();
}

void
handleNvData(void)
{
  WiFiClient    client = web.client();
  struct nvLog  log;
  struct tm     *tm;
  char          timestr[20], data[1460], *p;
  uint16_t      len;
  time_t        t;

  p = data;
  len = snprintf(p, 1460, "HTTP/1.1 200 OK\n"
    "Content-Type: text/plain\n"
    "Cache-Control: no-store\n"
    "\n"
    "Date,Power\n");
  p += len;
  for (uint16_t i = 0; i < NV_LOG_MAX; i++) {
    if (p - data > 1400) {
      client.write(data, p - data);
      p = data;
    }
    if ((i + nvHeader.nvLogFirst) % NV_LOG_MAX == nvHeader.nvLogLast)
      break;
    fram.read(NV_LOG_OFFSET + ((i + nvHeader.nvLogFirst) % NV_LOG_MAX) * sizeof(struct nvLog), (uint8_t *)&log, sizeof(struct nvLog));
    t = log.time;
    tm = localtime(&t);
    strftime(timestr, 20, "%F %T", tm);
    len = snprintf(p, 1460 - (p - data), "%s,%.2f\n", timestr, log.power);
    p += len;
  }
  if (p - data)
    client.write(data, p - data);
  client.stop();
}
