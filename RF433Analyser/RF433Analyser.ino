/*
*	RF433Analyser.ino
*	Copyright (c) 2019 Bob Tidey
*
*	Uses a RXB6 433MHz receiver as 433MHz analyser
*   C4 AGC capacitor reduced to 0.22uF to make it more responsive
*/

#define ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "SSD1306Spi.h"
#include "SH1106SPi.h"
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include "FS.h"

//put -1 s at end
int unusedPins[11] = {0,1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

/*
 Manual Web set up
*/
#define AP_SSID "ssid"
#define AP_PASSWORD "password"
#define AP_MAX_WAIT 10
#define AP_PORT 80

//uncomment next line to use static ip address instead of dhcp
//#define AP_IP 192,168,0,200
#define AP_DNS 192,168,0,1
#define AP_GATEWAY 192,168,0,1
#define AP_SUBNET 255,255,255,0

#define CONFIG_FILE "/esp433Config.txt"

/*
Wifi Manager Web set up
If WM_NAME defined then use WebManager
*/
#define WM_NAME "voltSetup"
#define WM_PASSWORD "password"
#ifdef WM_NAME
	WiFiManager wifiManager;
#endif

#define GPIO_DATA 4
#define GPIO_HOLD 5
#define GPIO_BUTTON 12
#define BUTTON_SHORT 100
#define BUTTON_MEDIUM 2000
#define BUTTON_LONG 4000

#define BUTTON_IDLE 0
#define BUTTON_DOWN 1
#define BUTTON_UP 2
#define BUTTON_SHORT 100
#define BUTTON_MEDIUM 2000
#define BUTTON_LONG 4000

int timeInterval = 10;
#define WIFI_CHECK_TIMEOUT 30000
#define POWERDOWN_DELAY 3000
unsigned long elapsedTime;
unsigned long wifiCheckTime;
unsigned long powerDownTimer;
int idleTimeout = 0;
int idleTimer;
#define DISPLAY_INTERVAL 500
unsigned long displayInterval = DISPLAY_INTERVAL;
unsigned long displayTime;
unsigned long buttonTime = 0;
int buttonDownTime = 0;
long buttonShort = BUTTON_SHORT;
long buttonMedium = BUTTON_MEDIUM;
long buttonLong = BUTTON_LONG;
int buttonIgnore = 0;
int volatile buttonState = 0;
int enableSleep = 0;
String rssiPrefix = "rssi";
String dataPrefix = "data";
String captureFile;
File fsUploadFile;
String adcCalString = "873,790,50,0";
int rssi;
int adcOffset;
int adcSlope;
int adcValue;
int adcRaw;

#define STARTUP_DELAY 3000
#define MAX_CAPTURELENGTH 2000
#define MAX_CAPTURETIME 10000
#define CAPTURESTATE_STARTUP 0
#define CAPTURESTATE_IDLE 1
#define CAPTURESTATE_DATAWAIT1 2
#define CAPTURESTATE_DATAWAIT2 3
#define CAPTURESTATE_DATAACTIVE 4
#define CAPTURESTATE_DATACOMPLETE 5
#define CAPTURESTATE_RSSIWAIT 6
#define CAPTURESTATE_RSSIACTIVE 7
#define CAPTURESTATE_RSSICOMPLETE 8
#define CAPTURESTATE_POWERDOWN 9
#define CAPTURETYPE_RSSI 1
#define CAPTURETYPE_DATA 2
#define CAPTURE_EXT "-cap.txt"

int captureOn;
int captureTransitions = 512;
int captureBuffer[MAX_CAPTURELENGTH];
int volatile captureCounter;
int captureRecord;
unsigned long dataTime;
unsigned long captureStartTime;
unsigned long captureTime;
int captureDataDuration;
int captureRSSIDuration;
int captureRSSIInterval;
int captureState;
int captureFileCount = 0;

String configNames[] = {"host","idleTimeout","timeInterval","adcCalString","buttonShort","buttonMedium","buttonLong",
						"displayInterval","rssiPrefix","dataPrefix","captureOn","captureDataDuration","captureTransitions","captureRSSIDuration","captureRSSIInterval"};

#define AP_AUTHID "2718"
//For update service
String host = "esp8266-433";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "password";

ESP8266WebServer server(AP_PORT);
ESP8266HTTPUpdateServer httpUpdater;
SSD1306Spi display(16, 2, 15);


/*
  Button Push interrupt handler
*/
void ICACHE_RAM_ATTR buttonInterrupt() {
	unsigned long m = millis();
	if(m-buttonTime > 30) {
		if(digitalRead(GPIO_BUTTON)) {
			buttonState = BUTTON_DOWN;
		} else if(buttonState == BUTTON_DOWN) {
			buttonDownTime = m - buttonTime;
			buttonState = BUTTON_UP;
		}
	}
	buttonTime = m;
}

/*
  Data interrupt handler
*/
void ICACHE_RAM_ATTR dataInterrupt() {
	unsigned long t = micros();
	if(captureState == CAPTURESTATE_DATAACTIVE) {
		if(captureCounter < captureTransitions) {
			//use positive value for low pulse and negative for high
			captureBuffer[captureCounter] = digitalRead(GPIO_DATA) ? t - dataTime : dataTime - t;
			captureCounter++;
		}
		dataTime = t;
	}
}

void ICACHE_RAM_ATTR  delaymSec(unsigned long mSec) {
	unsigned long ms = mSec;
	while(ms > 100) {
		delay(100);
		ms -= 100;
		ESP.wdtFeed();
	}
	delay(ms);
	ESP.wdtFeed();
	yield();
}

void ICACHE_RAM_ATTR  delayuSec(unsigned long uSec) {
	unsigned long us = uSec;
	while(us > 100000) {
		delay(100);
		us -= 100000;
		ESP.wdtFeed();
	}
	delayMicroseconds(us);
	ESP.wdtFeed();
	yield();
}

void unusedIO() {
	int i;
	
	for(i=0;i<11;i++) {
		if(unusedPins[i] < 0) {
			break;
		} else if(unusedPins[i] != 16) {
			pinMode(unusedPins[i],INPUT_PULLUP);
		} else {
			pinMode(16,INPUT_PULLDOWN_16);
		}
	}
}


void calibrateADC() {
	int v[4]; //mV2,adc2,mV1,adc1
	int j,k;
	String temp = adcCalString + ",1,2,3,4";
	
	j=0;
	for(k=0; k<4; k++) {
		j = temp.indexOf(',');
		v[k] = temp.substring(0,j).toInt();
		temp = temp.substring(j+1);
	}
	
	adcSlope = (1000 * (v[0] - v[2])) / (v[1] - v[3]);
	adcOffset = 1000 * v[2] - adcSlope * v[3];
	Serial.println("cal:" + String(adcSlope) + ":" + String(adcOffset));
}

/*
  Get config
*/
String getConfig() {
	String line = "";
	String strConfig;
	String strName;
	int config = 0;
	File f = SPIFFS.open(CONFIG_FILE, "r");
	if(f) {
		while(f.available()) {
			line =f.readStringUntil('\n');
			line.replace("\r","");
			if(line.length() > 0 && line.charAt(0) != '#') {
				switch(config) {
					case 0: host = line;break;
					case 1: idleTimeout = line.toInt();break;
					case 2: timeInterval = line.toInt();;break;
					case 3: adcCalString = line;break;
					case 4: buttonShort = line.toInt();;break;
					case 5: buttonMedium = line.toInt();break;
					case 6: buttonLong = line.toInt();break;
					case 7: displayInterval = line.toInt();break;
					case 8: rssiPrefix = line;break;
					case 9: dataPrefix = line;
					case 10: captureOn = line.toFloat() * 100;break;
					case 11: captureDataDuration = line.toInt();break;
					case 12: captureTransitions = line.toInt();break;
					case 13: captureRSSIDuration = line.toInt();break;
					case 14: captureRSSIInterval = line.toInt();
						Serial.println(F("Config loaded from file OK"));
						break;
				}
				strConfig += configNames[config] + ":" + line + "<BR>";
				config++;
			}
		}
		f.close();
		//enforce minimum idleTimeout of 60 seconds and maximum data transition count
		// idleTimeout of 0 is allowed and means no timeout
		if(idleTimeout && idleTimeout < 60000) idleTimeout = 60000;
		if(captureTransitions > MAX_CAPTURELENGTH) captureTransitions = MAX_CAPTURELENGTH;
		Serial.println("Config loaded");
		Serial.print(F("host:"));Serial.println(host);
		Serial.print(F("idleTimeout:"));Serial.println(idleTimeout);
		Serial.print(F("timeInterval:"));Serial.println(timeInterval);
		Serial.print(F("adcCalString:"));Serial.println(adcCalString);
		Serial.print(F("buttonShort:"));Serial.println(buttonShort);
		Serial.print(F("buttonMedium:"));Serial.println(buttonMedium);
		Serial.print(F("buttonLong:"));Serial.println(buttonLong);
		Serial.print(F("displayInterval:"));Serial.println(displayInterval);
		Serial.print(F("rssiPrefix:"));Serial.println(rssiPrefix);
		Serial.print(F("dataPrefix:"));Serial.println(dataPrefix);
		Serial.print(F("captureOn:"));Serial.println((float)captureOn / 100);
		Serial.print(F("captureDataDuration:"));Serial.println(captureDataDuration);
		Serial.print(F("captureTransitions:"));Serial.println(captureTransitions);
		Serial.print(F("captureRSSIDuration:"));Serial.println(captureRSSIDuration);
		Serial.print(F("captureRSSIInterval:"));Serial.println(captureRSSIInterval);
	} else {
		Serial.println(String(CONFIG_FILE) + " not found. Use default encoder");
	}
	calibrateADC();
	idleTimer = elapsedTime;
	return strConfig;
}


/*
  Connect to local wifi with retries
  If check is set then test the connection and re-establish if timed out
*/
int wifiConnect(int check) {
	if(check) {
		if((elapsedTime - wifiCheckTime) * timeInterval > WIFI_CHECK_TIMEOUT) {
			if(WiFi.status() != WL_CONNECTED) {
				Serial.println(F("Wifi connection timed out. Try to relink"));
			} else {
				wifiCheckTime = elapsedTime;
				return 1;
			}
		} else {
			return 0;
		}
	}
	wifiCheckTime = elapsedTime;
#ifdef WM_NAME
	Serial.println(F("Set up managed Web"));
	wifiManager.setConfigPortalTimeout(180);
	#ifdef AP_IP
		wifiManager.setSTAStaticIPConfig(IPAddress(AP_IP), IPAddress(AP_GATEWAY), IPAddress(AP_SUBNET));
	#endif
	wifiManager.autoConnect(WM_NAME, WM_PASSWORD);
	WiFi.mode(WIFI_STA);
#else
	Serial.println(F("Set up manual Web"));
	int retries = 0;
	Serial.print(F("Connecting to AP"));
	#ifdef AP_IP
		IPAddress addr1(AP_IP);
		IPAddress addr2(AP_DNS);
		IPAddress addr3(AP_GATEWAY);
		IPAddress addr4(AP_SUBNET);
		WiFi.config(addr1, addr2, addr3, addr4);
	#endif
	WiFi.begin(AP_SSID, AP_PASSWORD);
	while (WiFi.status() != WL_CONNECTED && retries < AP_MAX_WAIT) {
		delaymSec(1000);
		Serial.print(F("."));
		retries++;
	}
	Serial.println("");
	if(retries < AP_MAX_WAIT) {
		Serial.print(F("WiFi connected ip "));
		Serial.print(WiFi.localIP());
		Serial.printf_P(PSTR(":%d mac %s\r\n"), AP_PORT, WiFi.macAddress().c_str());
		return 1;
	} else {
		Serial.println(F("WiFi connection attempt failed"));
		return 0;
	} 
#endif
}

void initFS() {
	if(!SPIFFS.begin()) {
		Serial.println(F("No SIFFS found. Format it"));
		if(SPIFFS.format()) {
			SPIFFS.begin();
		} else {
			Serial.println(F("No SIFFS found. Format it"));
		}
	} else {
		Serial.println(F("SPIFFS file list"));
		Dir dir = SPIFFS.openDir("/");
		while (dir.next()) {
			Serial.print(dir.fileName());
			Serial.print(F(" - "));
			Serial.println(dir.fileSize());
		}
	}
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  Serial.printf_P(PSTR("handleFileRead: %s\r\n"), path.c_str());
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.printf_P(PSTR("handleFileUpload Name: %s\r\n"), filename.c_str());
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    Serial.printf_P(PSTR("handleFileUpload Data: %d\r\n"), upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    Serial.printf_P(PSTR("handleFileUpload Size: %d\r\n"), upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.printf_P(PSTR("handleFileDelete: %s\r\n"),path.c_str());
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.printf_P(PSTR("handleFileCreate: %s\r\n"),path.c_str());
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.printf_P(PSTR("handleFileList: %s\r\n"),path.c_str());
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  output += "]";
  server.send(200, "text/json", output);
}

void handleMinimalUpload() {
  char temp[700];

  snprintf ( temp, 700,
    "<!DOCTYPE html>\
    <html>\
      <head>\
        <title>ESP8266 Upload</title>\
        <meta charset=\"utf-8\">\
        <meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\">\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
      </head>\
      <body>\
        <form action=\"/edit\" method=\"post\" enctype=\"multipart/form-data\">\
          <input type=\"file\" name=\"data\">\
          <input type=\"text\" name=\"path\" value=\"/\">\
          <button>Upload</button>\
         </form>\
      </body>\
    </html>"
  );
  server.send ( 200, "text/html", temp );
}

void handleSpiffsFormat() {
	SPIFFS.format();
	server.send(200, "text/plain", "format complete");
}

//switch module into deep sleep
void powerOff() {
	WiFi.mode(WIFI_OFF);
	delaymSec(10);
	WiFi.forceSleepBegin();
	delaymSec(1000);
	pinMode(GPIO_HOLD, INPUT);
	ESP.deepSleep(0);
}

void startPowerDown() {
	Serial.println(F("start power down"));
	updateDisplay("Power Down","","");
	powerDownTimer = millis();
	detachInterrupt(GPIO_BUTTON);
	captureState = CAPTURESTATE_POWERDOWN;
}

void setCaptureFile(String filePrefix) {
	int fn;
	for(fn = 1; fn < 100; fn++) {
		captureFile = filePrefix + String(fn) + CAPTURE_EXT;
		if(!SPIFFS.exists("/" + captureFile)) return;
	}
	captureFile = filePrefix + CAPTURE_EXT;
}

void saveCapture(int type) {
	File f;
	int i = 0;
	unsigned long m = millis();
	
	if(captureRecord) 
		f = SPIFFS.open("/" + captureFile, "a");
	else
		f = SPIFFS.open("/" + captureFile, "w");
		
	if(f) {
		Serial.println("Save data to " + captureFile);
		if(captureRecord == 0) {
			//Add header info to file
			if(type == CAPTURETYPE_RSSI) {
				f.println("#RSSI:" + String(captureRSSIInterval) + " mSec");
			} else if(type == CAPTURETYPE_DATA){
				f.println("#DATA:");
			}
			f.println("#StartTime:" + String(captureStartTime));
			f.println("#EndTime:" + String(m));
			f.println("#Duration:" + String(m-captureStartTime));
		}
		while(i < captureCounter) {
			f.print(String(captureRecord) + ",");
			if(type == CAPTURETYPE_DATA) {
				if(captureBuffer[i] < 0)
					f.print("1,");
				else
					f.print("0,");
				f.println(String(abs(captureBuffer[i])));
			} else {
				f.println(String((float)(captureBuffer[i])/100));
			}
			captureRecord++;
			i++;
		}
		f.close();
	}
}

void readADC() {
	int i;
	adcRaw = analogRead(A0);;
	adcRaw = 0;
	for(i=0;i<16;i++) adcRaw += analogRead(A0);
	adcRaw = adcRaw >> 4;
	adcValue = (adcRaw * adcSlope + adcOffset) / 1000;
	calcRssi();
}

void checkButton() {
	if(captureState != CAPTURESTATE_POWERDOWN) {
		if(buttonState == BUTTON_DOWN) {
			if((millis() - buttonTime) > buttonLong) {
				Serial.println(F("start power down from button"));
				startPowerDown();
			}
		} else if(buttonState == BUTTON_UP && buttonDownTime < buttonLong) {
			if(buttonDownTime > buttonMedium) {
				if(captureState == CAPTURESTATE_IDLE) {
					Serial.println(F("initiate data capture"));
					captureFile = "";
					captureState = CAPTURESTATE_DATAWAIT1;
				}
			} else if(buttonDownTime > buttonShort) {
				if(captureState == CAPTURESTATE_IDLE) {
					Serial.println(F("initiate rssi capture"));
					captureFile = "";
					captureState = CAPTURESTATE_RSSIWAIT;
				} else if(captureState == CAPTURESTATE_RSSIACTIVE) {
					Serial.println(F("terminate rssi capture"));
					captureState = CAPTURESTATE_RSSICOMPLETE;
				}
			}
			buttonState = BUTTON_IDLE;
		}
	}
}

/*
*	Curve fit RSSI volts to power with cubic function
*	RSSI = -134.307 + 62.305 * V - 39.054 * V^2 + 13.247 * V^3
*	where V is RSSI voltage
*	in integer using mV
*	100*rssi==-13431+((623050-((3905430-mv*1325)/10000)*mv)*mv)/100000
*	old linear rssi = -110.0 + (adcValue - 0.25) * 66.7;
*/

void calcRssi() {
	int mv = 2 * adcValue;
	rssi = -13431 + ((623050 - ((3905430 - mv * 1325)/10000) * mv) * mv) / 100000;
}

void updateDisplay(String line1, String line2, String line3) {
	display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, line1);
    display.drawString(0, 22, line2);
    display.drawString(0, 44, line3);
	display.display();
}

void captureDataTransitions() {
	//Serial.println(F("Disable wifi during data capture"));
	unsigned long m = millis() + captureDataDuration;
	
	//WiFi.mode(WIFI_OFF);
	delaymSec(10);
	dataTime = micros();
	attachInterrupt(GPIO_DATA, dataInterrupt, CHANGE);
	ESP.wdtDisable();
	while(captureCounter < captureTransitions && millis() < m) {
		ESP.wdtFeed();
	}
	ESP.wdtEnable(1000);
	detachInterrupt(GPIO_DATA);
	//Serial.println(F("Reconnecting wifi"));
	//WiFi.mode(WIFI_STA);
	//WiFi.begin();
}

void stateMachine() {
	unsigned long m = millis();
	switch(captureState) {
		case CAPTURESTATE_STARTUP :
			if(elapsedTime * timeInterval > STARTUP_DELAY && digitalRead(GPIO_BUTTON) == 0)
				captureState = CAPTURESTATE_IDLE;
			break;
		case CAPTURESTATE_IDLE : 
			if((m - displayTime) > displayInterval) {
				readADC();
				updateDisplay("RSSI", "Real Time", String((float)rssi / 100) + " dB");
				displayTime = m;
			}
			break;
		case CAPTURESTATE_DATAWAIT1 : 
			if(captureFile == "") setCaptureFile(dataPrefix);
			updateDisplay(captureFile,"Data Wait", "for Trigger");
			captureStartTime = m;
			captureState = CAPTURESTATE_DATAWAIT2;
			break;
		case CAPTURESTATE_DATAWAIT2 : 
			if((m - captureStartTime) > (10 * captureDataDuration)) {
				Serial.println(F("Wait for trigger timed out"));
				captureState = CAPTURESTATE_IDLE;
			} else {
				readADC();
				if(rssi > captureOn) {
					updateDisplay(captureFile,"Capturing", "Data");
					captureStartTime = millis();
					captureCounter = 0;
					captureRecord = 0;
					captureState = CAPTURESTATE_DATAACTIVE;
				}
			}
			break;
		case CAPTURESTATE_DATAACTIVE :
			captureDataTransitions();
			captureState = CAPTURESTATE_DATACOMPLETE;
			break;
		case CAPTURESTATE_DATACOMPLETE : 
			saveCapture(CAPTURETYPE_DATA);
			captureFileCount++;
			captureState = CAPTURESTATE_IDLE;
			break;
		case CAPTURESTATE_RSSIWAIT :
			captureCounter = 0;
			captureRecord = 0;
			captureStartTime = m;
			captureTime = captureStartTime;
			captureState = CAPTURESTATE_RSSIACTIVE;
			if(captureFile == "") setCaptureFile(rssiPrefix);
			break;
		case CAPTURESTATE_RSSIACTIVE :
			if((m - captureTime) > captureRSSIInterval) {
				readADC();
				if(captureCounter < MAX_CAPTURELENGTH) {
					captureBuffer[captureCounter] = rssi;
					captureCounter++;
				}
				if((m - displayTime) > displayInterval) {
					updateDisplay(captureFile, "RSSI-Time:"+ String((m - captureStartTime) / 1000), String((float)rssi / 100) + " dB");
					displayTime = m;
				}
				if((m - captureStartTime) > captureRSSIDuration) {
					captureState = CAPTURESTATE_RSSICOMPLETE;
				}
				captureTime = m;
				if(captureState == CAPTURESTATE_RSSIACTIVE && captureCounter == MAX_CAPTURELENGTH) {
					//Empty the buffer
					saveCapture(CAPTURETYPE_RSSI);
					captureCounter = 0;
				}
			}
			break;
		case CAPTURESTATE_RSSICOMPLETE :
			Serial.println("RSSI Complete " + String(captureCounter));
			if(captureCounter) saveCapture(CAPTURETYPE_RSSI);
			captureFileCount++;
			captureState = CAPTURESTATE_IDLE;
			break;
		case CAPTURESTATE_POWERDOWN :
			if(millis() > (powerDownTimer + POWERDOWN_DELAY)) {
				powerOff();
				//should not get here
				captureState = CAPTURESTATE_IDLE;
			}
			break;
	}
	//check for idle timeout
	if(captureState == CAPTURESTATE_IDLE) {
		if(idleTimeout) {
			if(((elapsedTime - idleTimer) * timeInterval) > idleTimeout) {
				Serial.println("start power down from idle Timeout");
				startPowerDown();
			}
		}
	} else {
		idleTimer = elapsedTime;
	}
}

//action getcapturefiles
void handleGetCaptureFiles() {
	String fileList;
	String filename;
	Dir dir = SPIFFS.openDir("/");
	int i;
	captureFileCount = 0;
	while (dir.next()) {
		filename = dir.fileName();
		i = filename.indexOf(CAPTURE_EXT);
		if(i > 0 &&  i == (filename.length()-8)) {
			fileList += filename.substring(1) + "<BR>";
			captureFileCount++;
		}
	}
	server.send(200, "text/html", fileList);
}

//action request to capture data
void handleCapture() {
	String ret;
	captureFile = server.arg("filename");
	int captureType = server.arg("capturetype").toInt();
	Serial.println("handle capture:" + captureFile + " type:" + String(captureType));
	
	if(captureState == CAPTURESTATE_IDLE) {
		if(captureType == CAPTURETYPE_RSSI) {
			captureState = CAPTURESTATE_RSSIWAIT;
			ret = "capturing rssi";
		} else if(captureType == CAPTURETYPE_DATA) {
			captureState = CAPTURESTATE_DATAWAIT1;
			ret = "capturing data";
		} else {
			ret = "unknown capture type";
		}
	} else {
		ret = "already busy capturing";
	}
	server.send(200, "text/plain", ret);
}

//action request to reload config
void handleLoadConfig() {
	server.send(200, "text/html", getConfig());
}

//action request to save config
void handleSaveConfig() {
	File f;
	String config = server.arg("config");
	config.replace("<BR>","\n");
	f = SPIFFS.open(CONFIG_FILE, "w");
	if(f) {
		f.print(config);
		f.close();
		getConfig();
		server.send(200, "text/plain", "config saved");
	} else {
		server.send(200, "text/plain", "error saving config");
	}
}

//send response to status request
void handleStatus() {
	String status = "adcValue:" + String(adcValue) + "<BR>";
	status += "adcRaw:" + String(adcRaw) + "<BR>";
	status += "rssi:" + String((float)rssi / 100) + "<BR>";
	status += "captureFileCount:" + String(captureFileCount) + "<BR>";
	status += "buttonState:" + String(buttonState) + "<BR>";
	status += "buttonTime:" + String(buttonTime) + "<BR>";
	status += "captureState:" + String(captureState) + "<BR>";
	status += "captureCounter:" + String(captureCounter) + "<BR>";
	server.send(200, "text/html", status);
}

void setup() {
	unusedIO();
	display.init();
	display.flipScreenVertically();
	digitalWrite(GPIO_HOLD,1);
	pinMode(GPIO_HOLD, OUTPUT);
	pinMode(GPIO_BUTTON, INPUT);
	pinMode(GPIO_DATA, INPUT_PULLUP);
	Serial.begin(115200);
	Serial.println(F("\nStart up"));
	Serial.println(F("Set up filing system"));
	initFS();
	getConfig();
	wifiConnect(0);
	//Update service
	Serial.println("Set up Web update service");
	MDNS.begin(host.c_str());
	httpUpdater.setup(&server, update_path, update_username, update_password);
	MDNS.addService("http", "tcp", AP_PORT);
	Serial.println(F("Set up Web command handlers"));
	//Simple upload
	server.on("/upload", handleMinimalUpload);
	//SPIFFS format
	server.on("/format", handleSpiffsFormat);
	server.on("/list", HTTP_GET, handleFileList);
	//load editor
	server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");});
	//create file
	server.on("/edit", HTTP_PUT, handleFileCreate);
	//delete file
	server.on("/edit", HTTP_DELETE, handleFileDelete);
	//first callback is called after the request has ended with all parsed arguments
	//second callback handles file uploads at that location
	server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);
	server.on("/status",  handleStatus);
	server.on("/loadconfig", handleLoadConfig);
	server.on("/saveconfig", handleSaveConfig);
	server.on("/capture", handleCapture);
	server.on("/getcapturefiles", handleGetCaptureFiles);
	//called when the url is not defined here use it to load content from SPIFFS
	server.onNotFound([](){if(!handleFileRead(server.uri())) server.send(404, "text/plain", "FileNotFound");});
	server.begin();
	Serial.println();
	captureState = CAPTURESTATE_STARTUP;
	attachInterrupt(GPIO_BUTTON, buttonInterrupt, CHANGE);
	Serial.println(F("Set up complete"));
	updateDisplay("RF433 Analyser", WiFi.localIP().toString(),"");
}

void loop() {
	checkButton();
	stateMachine();
	server.handleClient();
	delay(timeInterval);
	elapsedTime++;
	wifiConnect(1);
}
