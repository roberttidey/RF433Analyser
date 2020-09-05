/*
*	RF433Analyser.ino
*	Copyright (c) 2019 Bob Tidey
*
*	Uses a RXB6 433MHz receiver as 433MHz analyser
*   C4 AGC capacitor reduced to 0.22uF to make it more responsive
*/

#define ESP8266
#include "BaseConfig.h"
#define FONT_SELECT
#define INC_FONT_ArialMT_Plain_16

#include "SSD1306Spi.h"
#include "SH1106SPi.h"
// if set to 1 allows disconnect request to remove wifi credentials
#define ALLOW_DISCONNECT 0

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
#define POWERDOWN_DELAY 3000
unsigned long elapsedTime;
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
String adcCalString = "873,790,50,0";
//cubic rssi fit c,x,x2,x3
String rssiCalString = "-13431,623050,3905430,1325";
int rssiCal[4];
int rssi;
int adcOffset;
int adcSlope;
int adcValue;
int adcRaw;

#define STARTUP_DELAY 3000
#define MAX_CAPTURELENGTH 8000
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
#define MAX_PULSEWIDTHS 8

int captureOn;
int captureTransitions = 512;
short captureBuffer[MAX_CAPTURELENGTH];
int volatile captureCounter;
int captureRecord;
unsigned long dataTime;
unsigned long captureStartTime;
unsigned long captureTime;
String pulseWidthsString;
String strConfig;

int pulseWidths[MAX_PULSEWIDTHS];
int captureDataDuration;
int captureRSSIDuration;
int captureRSSIInterval;
int captureState;
int captureFileCount = 0;

String configNames[] = {"host","idleTimeout","timeInterval","adcCalString","buttonShort","buttonMedium","buttonLong",
						"displayInterval","rssiPrefix","dataPrefix","captureOn","captureDataDuration","captureTransitions","captureRSSIDuration","captureRSSIInterval","rssiCalString","pulseWidths"};

//define cs as pin 17 so it is unused.
SSD1306Spi display(16, 2, 17);

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
  To encode into a short
  1 uSec basic resolution
  Divide by 32 for pulses longer than 16.4mSec
  Bit 15 signals high pulse
  Bit 14 indicates divided by 32
  0 indicates pulse longer than 524mSec
*/
void ICACHE_RAM_ATTR dataInterrupt() {
	unsigned long t = micros();
	if(captureState == CAPTURESTATE_DATAACTIVE) {
		if(captureCounter < captureTransitions) {
			int diff = (t - dataTime);
			short v;
			if(diff > 524287)
				v = 0;
			else if(diff>16383)
				v = 16384 + (diff >> 5);
			else
				v = diff;
			//Set Bit 15 to flag end of high pulse
			if(digitalRead(GPIO_DATA) == 0) v = v | 0x8000;
			captureBuffer[captureCounter] = v;
			captureCounter++;
		}
		dataTime = t;
	}
}

void calibrate() {
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

	temp = rssiCalString + ",1,2,3,4";
	j=0;
	for(k=0; k<4; k++) {
		j = temp.indexOf(',');
		rssiCal[k] = temp.substring(0,j).toInt();
		temp = temp.substring(j+1);
	}
}

void initPulseWidths() {
	int j,k;
	String temp = pulseWidthsString + ",0,0,0,0,0,0,0,500000";
	j=0;
	for(k=0; k<MAX_PULSEWIDTHS; k++) {
		j = temp.indexOf(',');
		pulseWidths[k] = temp.substring(0,j).toInt();
		temp = temp.substring(j+1);
	}
}

/*
  load config
*/
void loadConfig() {
	String line = "";
	String strName;
	int config = 0;
	File f = FILESYS.open(CONFIG_FILE, "r");
	if(f) {
		strConfig = "";
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
					case 14: captureRSSIInterval = line.toInt();break;
					case 15: rssiCalString = line;break;
					case 16: pulseWidthsString = line;
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
		Serial.print(F("rssiCalString:"));Serial.println(rssiCalString);
		Serial.print(F("pulseWidthsString:"));Serial.println(pulseWidthsString);
	} else {
		Serial.println(String(CONFIG_FILE) + " not found. Use default encoder");
	}
	initPulseWidths();
	calibrate();
	idleTimer = elapsedTime;
}

void configModeCallback (WiFiManager *myWiFiManager) {
	updateDisplay("Wifi config 120s",WiFi.softAPIP().toString(),String(myWiFiManager->getConfigPortalSSID()));
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
		if(!FILESYS.exists("/" + captureFile)) return;
	}
	captureFile = filePrefix + CAPTURE_EXT;
}

int getPulseWidth(int width) {
	int i;
	for(i=0;i < MAX_PULSEWIDTHS; i++) {
		if(width < pulseWidths[i]) {
			break;
		}
	}
	return i;
}

void saveCapture(int type) {
	File f;
	int i = 0;
	int v;
	unsigned long m = millis();
	
	if(captureRecord) 
		f = FILESYS.open("/" + captureFile, "a");
	else
		f = FILESYS.open("/" + captureFile, "w");
		
	if(f) {
		Serial.println("Save data to " + captureFile);
		if(captureRecord == 0) {
			//Add header info to file
			if(type == CAPTURETYPE_RSSI) {
				f.println("#RSSI:" + String(captureRSSIInterval) + " mSec");
			} else if(type == CAPTURETYPE_DATA){
				f.println("#DATA:");
				f.println("#Pulsewidths:" + pulseWidthsString);
			}
			f.println("#StartTime:" + String(captureStartTime));
			f.println("#EndTime:" + String(m));
			f.println("#Duration:" + String(m-captureStartTime));
		}
		while(i < captureCounter) {
			f.print(String(captureRecord) + ",");
			if(type == CAPTURETYPE_DATA) {
				v = captureBuffer[i];
				if(v < 0)
					f.print("1,");
				else
					f.print("0,");
				v = v & 0x7fff;
				if(v == 0)
					v = 524287;
				else if(v > 16383)
					 v = (v & 0x3fff) << 5;
				f.println(String(v) + "," + String(getPulseWidth(v)));
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
	rssi = rssiCal[0] + ((rssiCal[1] - ((rssiCal[2] - mv * rssiCal[3])/10000) * mv) * mv) / 100000;
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
	Dir dir = FILESYS.openDir("/");
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
	loadConfig();
	server.send(200, "text/html", strConfig);
}

//action request to save config
void handleSaveConfig() {
	File f;
	String config = server.arg("config");
	config.replace("<BR>","\n");
	f = FILESYS.open(CONFIG_FILE, "w");
	if(f) {
		f.print(config);
		f.close();
		loadConfig();
		server.send(200, "text/plain", "config saved");
	} else {
		server.send(200, "text/plain", "error saving config");
	}
}

//send response to status request
void handleStatus() {
	String status = "<head><link rel='shortcut icon' href='about:blank'></head>";
	status += "<body>adcValue:" + String(adcValue) + "<BR>";
	status += "adcRaw:" + String(adcRaw) + "<BR>";
	status += "rssi:" + String((float)rssi / 100) + "<BR>";
	status += "captureFileCount:" + String(captureFileCount) + "<BR>";
	status += "buttonState:" + String(buttonState) + "<BR>";
	status += "buttonTime:" + String(buttonTime) + "<BR>";
	status += "captureState:" + String(captureState) + "<BR>";
	status += "captureCounter:" + String(captureCounter) + "<BR>";
	status += "FreeHeap:" + String(ESP.getFreeHeap()) + "<BR></body>";
	server.send(200, "text/html", status);
}

void handleDisconnect() {
	if(ALLOW_DISCONNECT) {
		server.send(200, "text/html", "handleDisconnect");
		WiFi.disconnect();
		startPowerDown();
	} else {
		server.send(200, "text/html", "handleDisconnect ignored");
	}
}

void setupStart() {
	display.init();
	display.flipScreenVertically();
	digitalWrite(GPIO_HOLD,1);
	pinMode(GPIO_HOLD, OUTPUT);
	pinMode(GPIO_BUTTON, INPUT);
	pinMode(GPIO_DATA, INPUT_PULLUP);
	updateDisplay("RF433 Analyser", "Connecting","Up to 180s");
	wifiManager.setAPCallback(configModeCallback);
}

void extraHandlers() {
	server.on("/status",  handleStatus);
	server.on("/loadconfig", handleLoadConfig);
	server.on("/saveconfig", handleSaveConfig);
	server.on("/capture", handleCapture);
	server.on("/getcapturefiles", handleGetCaptureFiles);
	server.on("/disconnect", handleDisconnect);
}

void setupEnd() {
	if(WiFi.status() == WL_CONNECTED) {
		updateDisplay("RF433 Analyser", WiFi.localIP().toString(),"");
	} else {
		updateDisplay("RF433 Analyser", "No network","local mode");
	}
	captureState = CAPTURESTATE_STARTUP;
	attachInterrupt(GPIO_BUTTON, buttonInterrupt, CHANGE);
}

void loop() {
	checkButton();
	stateMachine();
	server.handleClient();
	delay(timeInterval);
	elapsedTime++;
}
