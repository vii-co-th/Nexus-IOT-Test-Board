#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define testcanbus  1
  
#define SW1 45
#define SW2 46
#define SW3 47
#define SW4 48
#define LED_STATUS  40 
#define BUZZER  41
#define CTRL4GWIFI  42
#define RX0 36
#define TX0 37
#define RX1 18
#define TX1 17
#define PMTX  20
#define PMRX  12
#define CO2TX 7
#define CO2RX 12
#define CTRX  38
#define CTTX  39
#define NTC 9
#define SDA 4
#define SCL 3
#define SDCS  10
#define SDMISO  11
#define SDSCK 12
#define SDMOSI  13
#define SDCD  16
#define EXCS  21
#define EXMISO  SDMISO
#define EXSCK SDSCK
#define EXMOSI  SDMOSI
#define SOLARADC  1
#define PARADC  2
#define RAINPULSE 5
#define WINDPULSE 6
#define ONEWIRE 15
#define TX485 14

//sdcard
#include "FS.h"
#include "SD.h"
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

#include <MHZCO2.h>
MHZ19E mhz19e;

#include <OneWire.h>
//OneWire  ow(ONEWIRE);  // on pin 10 (a 4.7K resistor is necessary)

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

void testdrawline();
void testdrawrect(void);
void testfillrect(void);
void testdrawcircle(void);
void testfillcircle(void);
void testdrawroundrect(void);
void testfillroundrect(void);
void testdrawtriangle(void);
void testfilltriangle(void);
void testdrawchar(void);
void testdrawstyles(void);
void testscrolltext(void);
void testdrawbitmap(void);
void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);

//NTC
#define THERMISTORPIN NTC
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3425
// the value of the 'other' resistor
#define SERIESRESISTOR 10000    
int samples[NUMSAMPLES];
float steinhart;
float average = 0;


//MODBUS RTU
#include <ModbusRTUSlave.h>
#define RS485RE_DE  TX485
#define MODBUSBASEADDR  0
const byte id = 1;
const unsigned long baud = 9600;
const uint32_t config = SERIAL_8N1;
const unsigned int bufSize = 256;
byte buf[bufSize];
#define MAXREGISTER_MODBUS  12
const unsigned int numInputRegisters = MAXREGISTER_MODBUS;
#if testcanbus == 0
ModbusRTUSlave modbus(Serial1, buf, bufSize,RS485RE_DE);
#else
#endif
int inputRegisterRead(short unsigned int address);
int rmodbus[MAXREGISTER_MODBUS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

//PM Sensor, softserial
#include <SoftwareSerial.h>
#define BAUD_RATE 9600
#define MAX_FRAMEBITS (1 + 8 + 1 + 2)
EspSoftwareSerial::UART sSerial;
boolean readPMSdata(Stream *s); 
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data pmsdata;
struct pms5003data pmsdata_realtime;
double aqi = 0;
bool first = true;

typedef struct AQITable_t {
  char quality_txt[50];
  int BL;
  int BH;
  double cL;
  double cH;  
} aqitab_t;

aqitab_t aqitab[8] = {
  { "Good"                    , 0, 50, 0, 12.0 },
  { "Moderate"                , 51, 100, 12.1, 35.4 },
  { "Unhealthy for sensitive" , 101, 150, 35.5, 55.4 },
  { "Unhealthy"               , 151, 200, 55.5, 150.4 },
  { "Very unhealthy"          , 201, 300, 150.5, 250.4 },
  { "Hazardous"               , 301, 400, 250.5, 350.4 },
  { "Hazardous"               , 401, 500, 350.5, 500.4 },
  { "Hazardous"               , 501, 999, 550.5, 99999.9 }
};

typedef struct TFTColorAQS_t {
  char pollution_txt[20];
  int good;
  int moderate;
  int unhealthy;
  int changecolor;
  char unit_txt[20];
} tftcoloraqs_t;

tftcoloraqs_t tfttab[7] = {
  { "IAQ VOC Index"    , 200,  300,  400,  1, ""  },
  { "CO2          "    , 700,  1000, 2500, 1, "ppm"  },
  { "AQI (PM 2.5) "    , 50,   100,  150,  1, ""  },
  { "PM 2.5       "    , 12,   35,   150,  1, "ug/m"  },
  { "PM 10        "    , 54,   154,  354,  1, "ug/m"  },
  { "Temperature  "     , 0,    0,    0,    0,"C"  },
  { "Humidity     "    , 0,    0,    0,    0, "%RH"  } 
};

int aqslevel[7] = {0,0,0,0,0,0,0};


// put function declarations here:
bool request_ntp_setdatetime(int wsec);
void test_sdcard();

#include "RTClib.h"
RTC_PCF8563 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
uint8_t dD, dM, dY;
uint8_t dh, dm, ds;
uint8_t dow;
uint8_t finsleep = 0;

long smilli = 0;
uint8_t osdcd = 2;
uint8_t orain = 2;
uint8_t owind = 2;

//SDCARD
#include <FS.h>
#include <FFat.h>
#include <esp_spi_flash.h>
void printDirectory(File dir, int numTabs = 3);
void printFile(File &file);

//MQTT
//#include <WiFiClient.h>
#include <WiFiClientSecure.h>
WiFiClient client;
#define MQTT_MAX_PACKET_SIZE 400
#include <PubSubClient.h>
PubSubClient MQTT_CLIENT(client);
String mqtt_id = "";
String mqtt_server = "";
String mqtt_port = "";
String mqtt_user = "";
String mqtt_pass = "";
String mqtt_topic = "";
String mqtt_iddf = "0001";
String mqtt_serverdf = "iot.vii.co.th";
String mqtt_portdf = "1883";
String mqtt_userdf = "jca";
String mqtt_passdf = "jcalambda";
String mqtt_topicdf = "/jca/aqs";

char cmqtt_topic[100];

static unsigned long last = millis();
static unsigned long last2 = millis();
static unsigned long lastmqtt = millis();

void send_mqtt(char *);
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void enable_mqtt();

//WiFi
#include <esp_wifi.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT WDTO_8S

#define WAITWIFI  60  //60 = 30s
String mac4 = "";
String mac6 = "";

#define WEB_TIME 60000  //default 60000,  0 = disable
#define SCREEN_TIME 3000
#define RETRYWEB  3
//#define HEARTBEAT_TIME  300000
#define RETRYMQTT  3
#define MQTT_TIME 60000 //default 60000,  0 = disable
#define __MQTT_ID_ADDR__  0x01  //10
#define __MQTT_SERVER_ADDR__  0x10  //30
#define __MQTT_PORT_ADDR__  0x30  //6
#define __MQTT_USER_ADDR__  0x36  //15
#define __MQTT_PASS_ADDR__  0x46  //15
#define __MQTT_TOPIC_ADDR__  0x56 //60
#define __SSID_ADDR__     0x80
#define __GSID_ADDR__  0x96 //80
#define __WIFIPASS_ADDR__ 0xf0
#define __IP_ADDR__     0x100
#define __SN_ADDR__     0x108
#define __GW_ADDR__     0x110
#define __DNS_ADDR__     0x118
#define __DHCP_ADDR__     0x120
#define __BAUD_RATE__     0x130
#define __MQTT_PERIOD__   0x136
#define __ADMINPASS_ADDR__     0x151
#define __SERVER_ADDR__     0x170 //0x170-0x1BF
#define __LOCK_ADDR__     0x1D0 
#define __LASTDATE_ADDR__     0x1D2 
uint  mqtt_stime = 60;  //0 = disable

#define VIP1  192
#define VIP2  168
#define VIP3  1
#define VIP4  100
HTTPClient http;
WiFiMulti wifiMulti;
uint8_t isWiFi = 0;
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

// SSID and PW for Config Portal
char host[] = "nexusiot";
String ssid = host + String(ESP_getChipId(), HEX).substring(String(ESP_getChipId(), HEX).length()-4);
const char* password = "nexus1234";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// Define NTP Client to get time
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//declare function
void send_web(const char *wdata, bool fallbacksms);
String urlencode(String str);
IPAddress myip(VIP1, VIP2, VIP3, VIP4); //ESP static ip
IPAddress mygw(VIP1, VIP2, VIP3, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress mysn(255, 255, 255, 0);  //Subnet mask
IPAddress mydns(VIP1, VIP2, VIP3, 1);  //DNS
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
IPAddress apip(192,168,4,1);
IPAddress apgw(192,168,4,1);
IPAddress apsn(255,255,255,0);
int wc = 0;
byte isDHCP = 1;
String remote_server = "iot.vii.co.th";
String tel1 = "0811710428";
String tel2 = "";
String tel3 = "";

char html[12000];
char shtml[2000];
char fns[40];
ulong wifistarttime = 0;

uint8_t pwd_c;
String wifissid = "LambdaNu";
String wifipassword = "0818090157";
String adminpassword = "admin";

char pwd_ch;
uint8_t cretry = 0;
uint8_t sendsucc = 0;

String deploy_id = "";
String deploy_iddf = "";

long tmillis;
uint32_t trun = 0;
uint32_t tsendweb = -WEB_TIME;
uint32_t tsendscreen = -SCREEN_TIME;
uint32_t tsec = 0;
int bx = 0;
int i = 0;

#define DEFAULT_INTERVALTIME  600

uint32_t intervalTime = DEFAULT_INTERVALTIME*1000;  //ms //heart beat
uint16_t preRequestTime = 800;  //ms
unsigned long preTime = millis();
unsigned long preTime1s = millis();
unsigned long curTime = millis();
bool hasreq = false;
char strdata[500];
char saveurl[500];
char st[80];

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

#define MIN_AP_PASSWORD_SIZE    8

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

WebServer server(80);
//WiFiServer serverAP(80);

// Variable to store the HTTP request
String header;

const char favicon[1406] = {
  0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x10, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x68, 0x05, 
  0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x20, 0x00, 
  0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC9, 0xDD, 
  0x00, 0x00, 0xBF, 0xE6, 0x00, 0x00, 0xB4, 0xEE, 0x00, 0x00, 0x8C, 0xF5, 0x00, 0x00, 0x98, 0xF5, 
  0x00, 0x00, 0x9F, 0xF5, 0x00, 0x00, 0xAA, 0xF5, 0x00, 0x00, 0x94, 0xF6, 0x00, 0x00, 0xD4, 0xD4, 
  0x01, 0x00, 0x9C, 0xD7, 0x01, 0x00, 0x89, 0xD8, 0x01, 0x00, 0x92, 0xD8, 0x01, 0x00, 0xA2, 0xF1, 
  0x01, 0x00, 0xDE, 0xCA, 0x02, 0x00, 0xA5, 0xD5, 0x02, 0x00, 0xB6, 0xC8, 0x03, 0x00, 0xAD, 0xCF, 
  0x03, 0x00, 0xF0, 0xB7, 0x04, 0x00, 0xBE, 0xC0, 0x04, 0x00, 0xAD, 0xE8, 0x04, 0x00, 0xD1, 0xAE, 
  0x05, 0x00, 0xC9, 0xB7, 0x05, 0x00, 0xE8, 0xC0, 0x05, 0x00, 0x75, 0xC3, 0x05, 0x00, 0x7E, 0xDD, 
  0x05, 0x00, 0xDE, 0x9D, 0x07, 0x00, 0xD8, 0xA6, 0x07, 0x00, 0x77, 0xDD, 0x08, 0x00, 0xB8, 0xDF, 
  0x09, 0x00, 0xDF, 0x95, 0x0A, 0x00, 0xDF, 0x8C, 0x0D, 0x00, 0xF9, 0xAD, 0x0E, 0x00, 0x86, 0xF6, 
  0x0E, 0x00, 0xC2, 0xD7, 0x0F, 0x00, 0xDF, 0x83, 0x10, 0x00, 0x91, 0xF3, 0x13, 0x00, 0x11, 0x10, 
  0x14, 0x00, 0x72, 0xDE, 0x14, 0x00, 0xC4, 0x6F, 0x18, 0x00, 0xFE, 0xA3, 0x1C, 0x00, 0xCD, 0xCE, 
  0x22, 0x00, 0x9B, 0xEC, 0x23, 0x00, 0xFD, 0x98, 0x25, 0x00, 0x25, 0x19, 0x28, 0x00, 0xFD, 0x8D, 
  0x2B, 0x00, 0xD7, 0xC4, 0x2C, 0x00, 0xDE, 0x76, 0x31, 0x00, 0xE0, 0xB9, 0x34, 0x00, 0xE9, 0xB0, 
  0x38, 0x00, 0x6B, 0xDE, 0x3B, 0x00, 0xA4, 0xE3, 0x3C, 0x00, 0xF2, 0xA6, 0x3E, 0x00, 0x7F, 0xF6, 
  0x3F, 0x00, 0xFC, 0x9D, 0x43, 0x00, 0xBC, 0xD1, 0x44, 0x00, 0x3A, 0x2C, 0x46, 0x00, 0x8A, 0xEE, 
  0x46, 0x00, 0xFE, 0x92, 0x47, 0x00, 0xDF, 0x71, 0x49, 0x00, 0xFE, 0x87, 0x4B, 0x00, 0xC6, 0xC8, 
  0x4C, 0x00, 0x4C, 0x3F, 0x50, 0x00, 0xD0, 0xBE, 0x51, 0x00, 0xDA, 0xB4, 0x54, 0x00, 0x67, 0xDE, 
  0x55, 0x00, 0x9E, 0xDF, 0x55, 0x00, 0xDE, 0x6D, 0x5B, 0x00, 0x7A, 0xF4, 0x5B, 0x00, 0xE1, 0xAD, 
  0x5C, 0x00, 0x90, 0xE8, 0x5C, 0x00, 0xF5, 0x97, 0x5E, 0x00, 0xFD, 0x8C, 0x60, 0x00, 0xB4, 0xCB, 
  0x61, 0x00, 0xFE, 0x82, 0x62, 0x00, 0x85, 0xEA, 0x62, 0x00, 0xC0, 0xC2, 0x67, 0x00, 0xCA, 0xB8, 
  0x6A, 0x00, 0xDF, 0x69, 0x6B, 0x00, 0xA5, 0xD7, 0x6C, 0x00, 0x62, 0xDE, 0x6D, 0x00, 0x6D, 0x41, 
  0x6F, 0x00, 0xEF, 0x91, 0x72, 0x00, 0xFE, 0x7D, 0x74, 0x00, 0xF9, 0x87, 0x74, 0x00, 0xAF, 0xC7, 
  0x76, 0x00, 0xDE, 0x67, 0x78, 0x00, 0x84, 0xE4, 0x79, 0x00, 0x74, 0xEF, 0x7A, 0x00, 0x47, 0x47, 
  0x7F, 0x00, 0x7F, 0xE5, 0x7F, 0x00, 0xE0, 0x97, 0x81, 0x00, 0x89, 0xDB, 0x82, 0x00, 0xEA, 0x8D, 
  0x83, 0x00, 0x5C, 0xDB, 0x83, 0x00, 0xF4, 0x83, 0x84, 0x00, 0x98, 0xD6, 0x84, 0x00, 0xFD, 0x7A, 
  0x85, 0x00, 0xDE, 0x63, 0x88, 0x00, 0x86, 0x7C, 0x88, 0x00, 0x59, 0x57, 0x8D, 0x00, 0x6F, 0x5E, 
  0x8D, 0x00, 0x8D, 0xD6, 0x91, 0x00, 0x6D, 0xEA, 0x92, 0x00, 0x57, 0xD7, 0x94, 0x00, 0xD9, 0x91, 
  0x95, 0x00, 0x78, 0xE0, 0x96, 0x00, 0xED, 0x7F, 0x97, 0x00, 0x7D, 0x84, 0x97, 0x00, 0xE3, 0x88, 
  0x97, 0x00, 0xBE, 0xB0, 0x97, 0x00, 0xA4, 0xC6, 0x97, 0x00, 0xDC, 0x60, 0x98, 0x00, 0x7D, 0x55, 
  0x99, 0x00, 0xF8, 0x75, 0x99, 0x00, 0x82, 0xD6, 0x99, 0x00, 0xCC, 0xA1, 0x9B, 0x00, 0x8D, 0xCD, 
  0x9C, 0x00, 0x6D, 0x54, 0x9D, 0x00, 0x69, 0x5A, 0x9D, 0x00, 0x97, 0xC3, 0x9D, 0x00, 0x98, 0x9D, 
  0x9E, 0x00, 0x7F, 0x7D, 0xA0, 0x00, 0xA1, 0xBB, 0xA0, 0x00, 0x8F, 0xD3, 0xA1, 0x00, 0xAC, 0xB1, 
  0xA3, 0x00, 0x97, 0xCF, 0xA3, 0x00, 0x8B, 0x53, 0xA4, 0x00, 0xD6, 0x5D, 0xA4, 0x00, 0x52, 0xD4, 
  0xA4, 0x00, 0x67, 0xE6, 0xA5, 0x00, 0xB5, 0xAB, 0xA7, 0x00, 0x72, 0xDC, 0xA7, 0x00, 0xE7, 0x7B, 
  0xAA, 0x00, 0x7D, 0xD2, 0xAA, 0x00, 0xF2, 0x71, 0xAB, 0x00, 0x87, 0xC9, 0xAC, 0x00, 0xD7, 0x8B, 
  0xAE, 0x00, 0xA4, 0xBB, 0xAE, 0x00, 0x91, 0xBF, 0xAE, 0x00, 0x9C, 0xB7, 0xB0, 0x00, 0xD3, 0x5C, 
  0xB1, 0x00, 0xBA, 0x9A, 0xB4, 0x00, 0x4C, 0xD0, 0xB4, 0x00, 0xC3, 0x92, 0xB6, 0x00, 0x6D, 0xDC, 
  0xB6, 0x00, 0x62, 0xE2, 0xB8, 0x00, 0xEC, 0x6E, 0xB9, 0x00, 0xE2, 0x78, 0xB9, 0x00, 0x6D, 0xD8, 
  0xBA, 0x00, 0xA3, 0xB4, 0xBB, 0x00, 0x78, 0xCE, 0xBC, 0x00, 0x83, 0xC5, 0xBE, 0x00, 0xCE, 0x5B, 
  0xBF, 0x00, 0x8C, 0xBB, 0xC0, 0x00, 0x95, 0xB3, 0xC1, 0x00, 0xBD, 0x95, 0xC3, 0x00, 0x48, 0xCC, 
  0xC4, 0x00, 0xD3, 0x7E, 0xC8, 0x00, 0x5B, 0xDD, 0xC8, 0x00, 0xE8, 0x6C, 0xC9, 0x00, 0xDD, 0x75, 
  0xC9, 0x00, 0x9D, 0x95, 0xCA, 0x00, 0x66, 0xD4, 0xCB, 0x00, 0xC9, 0x5A, 0xCC, 0x00, 0x71, 0xC9, 
  0xCC, 0x00, 0x67, 0xE1, 0xCC, 0x00, 0xA2, 0x4F, 0xCD, 0x00, 0x7B, 0xC0, 0xCE, 0x00, 0xD0, 0x6E, 
  0xCF, 0x00, 0x85, 0xB7, 0xCF, 0x00, 0x5D, 0xDC, 0xCF, 0x00, 0x83, 0xC4, 0xD0, 0x00, 0x70, 0xDD, 
  0xD0, 0x00, 0x40, 0xF4, 0xD0, 0x00, 0x98, 0xA3, 0xD1, 0x00, 0x30, 0xAA, 0xD1, 0x00, 0x8E, 0xAD, 
  0xD1, 0x00, 0x6D, 0x7F, 0xD3, 0x00, 0x42, 0xC7, 0xD3, 0x00, 0x5B, 0xDE, 0xD4, 0x00, 0xA4, 0xA5, 
  0xD6, 0x00, 0xE2, 0x6A, 0xD8, 0x00, 0x7D, 0x8D, 0xD8, 0x00, 0xC4, 0x59, 0xD9, 0x00, 0xA9, 0x82, 
  0xDB, 0x00, 0x32, 0xF5, 0xDB, 0x00, 0x55, 0xD8, 0xDC, 0x00, 0x60, 0xCF, 0xDD, 0x00, 0x6A, 0xC5, 
  0xDE, 0x00, 0xBE, 0x58, 0xDF, 0x00, 0x74, 0xBB, 0xDF, 0x00, 0x87, 0xA9, 0xE0, 0x00, 0x7D, 0xB2, 
  0xE0, 0x00, 0x3D, 0xC4, 0xE0, 0x00, 0xB7, 0x57, 0xE1, 0x00, 0x91, 0x9F, 0xE1, 0x00, 0x3E, 0xEF, 
  0xE2, 0x00, 0xA7, 0x8E, 0xE3, 0x00, 0x9D, 0x97, 0xE3, 0x00, 0x5A, 0xDC, 0xE3, 0x00, 0x4C, 0xA2, 
  0xE6, 0x00, 0x3A, 0xB2, 0xE6, 0x00, 0x36, 0xBF, 0xE6, 0x00, 0x4F, 0xD8, 0xE6, 0x00, 0xB0, 0x5A, 
  0xE7, 0x00, 0xA6, 0x5F, 0xE7, 0x00, 0x9C, 0x64, 0xE7, 0x00, 0x92, 0x69, 0xE7, 0x00, 0x88, 0x70, 
  0xE7, 0x00, 0xD0, 0x70, 0xE7, 0x00, 0x80, 0x75, 0xE7, 0x00, 0x76, 0x7C, 0xE7, 0x00, 0x6D, 0x83, 
  0xE7, 0x00, 0x63, 0x8A, 0xE7, 0x00, 0x5C, 0x92, 0xE7, 0x00, 0x53, 0x99, 0xE7, 0x00, 0x42, 0xAB, 
  0xE7, 0x00, 0xDC, 0x68, 0xE8, 0x00, 0x8C, 0x7C, 0xE8, 0x00, 0x37, 0xEE, 0xE8, 0x00, 0x4F, 0xD3, 
  0xEB, 0x00, 0x59, 0xCA, 0xEC, 0x00, 0x63, 0xC0, 0xED, 0x00, 0x6D, 0xB6, 0xEE, 0x00, 0x2B, 0xF5, 
  0xEE, 0x00, 0x80, 0xA4, 0xEF, 0x00, 0x77, 0xAD, 0xEF, 0x00, 0xB2, 0x7B, 0xF1, 0x00, 0xBF, 0x75, 
  0xF4, 0x00, 0xCA, 0x6D, 0xF6, 0x00, 0xD5, 0x67, 0xF7, 0x00, 0x49, 0xCF, 0xF9, 0x00, 0x5D, 0xBC, 
  0xFA, 0x00, 0x52, 0xC6, 0xFA, 0x00, 0x71, 0xA9, 0xFB, 0x00, 0x67, 0xB2, 0xFB, 0x00, 0xA3, 0x81, 
  0xFC, 0x00, 0x79, 0xA1, 0xFC, 0x00, 0xC4, 0x6B, 0xFD, 0x00, 0xAE, 0x79, 0xFD, 0x00, 0xC8, 0x64, 
  0xFE, 0x00, 0xCF, 0x65, 0xFE, 0x00, 0xBD, 0x6A, 0xFE, 0x00, 0xB2, 0x70, 0xFE, 0x00, 0xB9, 0x72, 
  0xFE, 0x00, 0xA7, 0x76, 0xFE, 0x00, 0x9C, 0x7D, 0xFE, 0x00, 0x92, 0x83, 0xFE, 0x00, 0x87, 0x8B, 
  0xFE, 0x00, 0x7D, 0x94, 0xFE, 0x00, 0x73, 0x9C, 0xFE, 0x00, 0x69, 0xA5, 0xFE, 0x00, 0x5F, 0xAE, 
  0xFE, 0x00, 0x56, 0xB7, 0xFE, 0x00, 0x4C, 0xC1, 0xFE, 0x00, 0x42, 0xCA, 0xFE, 0x00, 0x26, 0x2E, 
  0x3A, 0x42, 0x4D, 0x55, 0x61, 0x6F, 0x7F, 0x8C, 0x98, 0xA3, 0xB7, 0xBD, 0xC2, 0xA6, 0x22, 0x2C, 
  0x3B, 0x49, 0x52, 0x60, 0x71, 0x86, 0x92, 0x9F, 0xB5, 0xD9, 0xE6, 0xF1, 0xF0, 0xCC, 0x1E, 0x2A, 
  0x39, 0x47, 0x53, 0x5E, 0x6A, 0x84, 0x93, 0xA0, 0xA8, 0xD1, 0xE5, 0xEE, 0xF2, 0xCD, 0x1D, 0x27, 
  0x35, 0x46, 0x51, 0x5C, 0x6C, 0x88, 0x9B, 0x9D, 0x50, 0x7E, 0xE4, 0xF4, 0xF3, 0xCE, 0x19, 0x1F, 
  0x33, 0x6E, 0xB3, 0x5A, 0x68, 0xC7, 0xCB, 0xB4, 0x2B, 0x70, 0xE3, 0xEF, 0xF5, 0xCF, 0x1A, 0x11, 
  0x30, 0x44, 0xE0, 0x6D, 0x73, 0xAB, 0x8F, 0x79, 0x37, 0xB8, 0x75, 0xEC, 0xF6, 0xD0, 0x14, 0x16, 
  0x2F, 0x3F, 0x7D, 0xAA, 0x89, 0x95, 0x8D, 0x3D, 0x64, 0xC5, 0x76, 0xDA, 0xF7, 0xD2, 0x15, 0x0D, 
  0x2D, 0x3E, 0x4C, 0xC4, 0xAC, 0x82, 0x62, 0x24, 0xA1, 0xC6, 0x63, 0x58, 0xF8, 0xD3, 0x12, 0x08, 
  0x28, 0x3C, 0x4B, 0x7B, 0xDB, 0x7C, 0x78, 0x6B, 0xAE, 0xC3, 0xB6, 0xB1, 0xF9, 0xD4, 0x0F, 0x00, 
  0x21, 0x36, 0x48, 0x54, 0xA5, 0x7A, 0x8B, 0x9A, 0xB0, 0xBF, 0xE1, 0xED, 0xFA, 0xD5, 0x10, 0x01, 
  0x1C, 0x41, 0x4E, 0x5F, 0x90, 0x77, 0x8A, 0x99, 0xA9, 0xC0, 0xE2, 0xEA, 0xFB, 0xD6, 0x0E, 0x02, 
  0x13, 0x32, 0xAD, 0xB9, 0x65, 0x74, 0x87, 0x97, 0xA7, 0xBE, 0xDF, 0xEB, 0xFC, 0xD7, 0x09, 0x06, 
  0x0C, 0x29, 0x45, 0x56, 0x5B, 0x72, 0x85, 0x96, 0xA4, 0xBC, 0xDE, 0xE8, 0xFD, 0xC8, 0x0B, 0x05, 
  0x04, 0x23, 0x38, 0x4A, 0x59, 0x69, 0x83, 0x94, 0xA2, 0xBB, 0xDD, 0xE9, 0xFE, 0xD8, 0x0A, 0x07, 
  0x03, 0x20, 0x34, 0x43, 0x57, 0x66, 0x81, 0x91, 0x9E, 0xBA, 0xDC, 0xE7, 0xFF, 0xC9, 0x17, 0x18, 
  0x1B, 0x25, 0x31, 0x40, 0x4F, 0x5D, 0x67, 0x80, 0x8E, 0x9C, 0xB2, 0xC1, 0xCA, 0xAF, 0x00, 0x00, 
  0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
  0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
  0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
  0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF
};


/* Style */
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";


 const char* loginIndex1 = 
 "<html><head><title>Hello</title></head><body>"
 "Hello, it works. :D Please write your own web page!!"
 "</body></html>";

void handleCMD()
{

  sprintf(shtml,"<br><br>");
  
  if (server.hasArg("restart") && server.arg("restart") == "1") {
    sprintf(html,"Restart...<script>setTimeout(function(){ window.open('/','_self'); }, 5000);</script>");
  }
  else {
    strcpy(html,"unknown cmd");
  }

  sprintf(html,"%s",shtml);

  server.sendHeader("Connection", "close");
  server.send(200, "text/html", html); 

  if (server.hasArg("restart") && server.arg("restart") == "1") {
    delay(2000);
    ESP.restart();
  }
  else {
    wifistarttime = millis();    
  }

}

void write_adminpass_eeprom (char *s)
{
  for (pwd_c = 0; pwd_c <= strlen(s); ++pwd_c) {
    EEPROM.write(__ADMINPASS_ADDR__+pwd_c,s[pwd_c]);
  }
  EEPROM.commit();
}

void enable_wifi()
{

  //read wifi ssid & pass from EEPROM
  if (EEPROM.read(__SSID_ADDR__) != 0xff) {
    wifissid = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__SSID_ADDR__+pwd_c);
      wifissid += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM wifi-ssid=%s\r\n",wifissid.c_str());
  }
  else {
    Serial.printf("default wifi-ssid=%s\r\n",wifissid.c_str());    
  }

  if (EEPROM.read(__WIFIPASS_ADDR__) != 0xff) {
    wifipassword = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__WIFIPASS_ADDR__+pwd_c);
      wifipassword += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM wifi-pass=%s\r\n",wifipassword.c_str());
  }
  else {
    Serial.printf("default wifi-pass=%s\r\n",wifipassword.c_str());    
  }

  if (EEPROM.read(__ADMINPASS_ADDR__) != 0xff) {
    adminpassword = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__ADMINPASS_ADDR__+pwd_c);
      adminpassword += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM admin-pass=%s\r\n",adminpassword.c_str());
  }
  else {
    Serial.printf("default admin-pass=%s\r\n",adminpassword.c_str());    
  }

  if (EEPROM.read(__SERVER_ADDR__) != 0xff) {
    remote_server = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__SERVER_ADDR__+pwd_c);
      remote_server += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM remote-server=%s\r\n",remote_server.c_str());
  }
  else {
    Serial.printf("default remote-server=%s\r\n",remote_server.c_str());    
  }

  //init station info
  if (EEPROM.read(__IP_ADDR__+0) == 0xff && EEPROM.read(__IP_ADDR__+1) == 0xff && EEPROM.read(__IP_ADDR__+2) == 0xff && EEPROM.read(__IP_ADDR__+3) == 0xff) {
  }
  else {
    for(int i = 0; i < 4; ++i)
      myip[i] = EEPROM.read(__IP_ADDR__+i);
  }
  if (EEPROM.read(__SN_ADDR__+0) == 0xff && EEPROM.read(__SN_ADDR__+1) == 0xff && EEPROM.read(__SN_ADDR__+2) == 0xff && EEPROM.read(__SN_ADDR__+3) == 0xff) {
  }
  else {
    for(int i = 0; i < 4; ++i)
      mysn[i] = EEPROM.read(__SN_ADDR__+i);
  }
  if (EEPROM.read(__GW_ADDR__+0) == 0xff && EEPROM.read(__GW_ADDR__+1) == 0xff && EEPROM.read(__GW_ADDR__+2) == 0xff && EEPROM.read(__GW_ADDR__+3) == 0xff) {
  }
  else {
    for(int i = 0; i < 4; ++i)
      mygw[i] = EEPROM.read(__GW_ADDR__+i);
  }
  if (EEPROM.read(__DNS_ADDR__+0) == 0xff && EEPROM.read(__DNS_ADDR__+1) == 0xff && EEPROM.read(__DNS_ADDR__+2) == 0xff && EEPROM.read(__DNS_ADDR__+3) == 0xff) {
  }
  else {
    for(int i = 0; i < 4; ++i)
      mydns[i] = EEPROM.read(__DNS_ADDR__+i);
  }

  //read dhcp state
  if (EEPROM.read(__DHCP_ADDR__) != 0xff) {
    isDHCP = EEPROM.read(__DHCP_ADDR__);
  }
  else {
    isDHCP = 1;
  }

  Serial.printf("\r\n\r\nSaved IP Info:\r\nIP = %d.%d.%d.%d\r\nSub Netmask = %d.%d.%d.%d\r\nGateway IP = %d.%d.%d.%d\r\nDNS IP = %d.%d.%d.%d\r\nDHCP = %s\r\n",
    myip[0],myip[1],myip[2],myip[3],
    mysn[0],mysn[1],mysn[2],mysn[3],
    mygw[0],mygw[1],mygw[2],mygw[3],
    mydns[0],mydns[1],mydns[2],mydns[3],
    isDHCP==1?"Yes":"No"
  );

  isWiFi = 0;
  mac4 = WiFi.macAddress().substring(12,14)+WiFi.macAddress().substring(15,17);
  Serial.print("MAC :");
  Serial.print(WiFi.macAddress());  
  Serial.print(" ("+mac4+")");
  ssid = host + mac4;  
//*/
  if (isDHCP == 1) {
      Serial.println("\r\nStation uses DHCP");
  }
  else {
    if (!WiFi.config(myip, mygw, mysn, mydns, primaryDNS)) {
      Serial.println("STA Failed to configure");
    }
    else {
      Serial.printf("\r\nManual Station Info:\r\nIP = %d.%d.%d.%d\r\nSub Netmask = %d.%d.%d.%d\r\nGateway IP = %d.%d.%d.%d\r\nDNS IP = %d.%d.%d.%d\r\n",
        myip[0],myip[1],myip[2],myip[3],
        mysn[0],mysn[1],mysn[2],mysn[3],
        mygw[0],mygw[1],mygw[2],mygw[3],
        mydns[0],mydns[1],mydns[2],mydns[3]
      );
    }
  }

  // Configures static IP address
  WiFi.softAPConfig(apip,apgw,apsn);

  //wifiMulti.addAP(ssid.c_str(),password);
  WiFi.softAP(ssid.c_str(),password);

  WiFi.mode(WIFI_AP_STA);

  Serial.printf("\r\n#AP SSID = %s\r\n",ssid.c_str());
  Serial.printf("#AP Pass = %s\r\n",password);
  Serial.print("#AP IP = ");
  Serial.println(WiFi.softAPIP());

  WiFi.setHostname(host);
  WiFi.begin(wifissid.c_str(), wifipassword.c_str());

  Serial.printf("#WiFi SSID = %s\r\n",wifissid.c_str());
  Serial.printf("#WiFi Pass = %s\r\n",wifipassword.c_str());

  uint8_t twifi = 0;

  int wc = 0;
  while (WiFi.status() != WL_CONNECTED) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
    if (twifi == 0) {
      twifi = 1;
    }
    else {
      twifi = 0;
    }
    if (++wc >= WAITWIFI) {
      Serial.printf("\r\nBreak.. Can't connect WIFI SSID=%s with password '%s', please re-config via AP.\r\n",wifissid.c_str(),wifipassword.c_str());
      delay(5000);
      break;
    }
  }

  //serverAP.begin();  
  if (wc < 60) {
    isWiFi = 1;
    wifistarttime = millis();
    Serial.println("");
    Serial.println("#WiFi connected");
    Serial.print("#Client  IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("#Client  SN address: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("#Client  GW address: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("#Client DNS address: ");
    Serial.println(WiFi.dnsIP());


    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) { //http://{host}.local
      Serial.println("#Error setting up MDNS responder!");
      //while (1) {
      // delay(1000);
      //}
    }
    else {
      Serial.print("#mDNS responder started http://");    
      Serial.print(host);    
      Serial.println(".local");    
    }
  }

  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    sprintf(html,"%s",loginIndex1);
    server.send(200, "text/html", html);
  });

  server.on("/favicon.ico", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "image/x-icon", favicon);
  });


/*
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/global.css", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex2);
  });

  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex2);
  });

*/

  server.on("/cmd", HTTP_GET, []() {
    handleCMD();
  });
  
  server.begin();
}


void setup()
{
  EEPROM.begin(512);
  Serial.begin(115200);

  Wire.begin(SDA,SCL);
  
  //gpio_reset_pin(GPIO_NUM_40);
  //gpio_pulldown_dis(GPIO_NUM_40);
  //gpio_pullup_dis(GPIO_NUM_40);
  //gpio_iomux_out(GPIO_NUM_40,FUNC_MTDO_GPIO40,false);
  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  pinMode(SW3,INPUT);
  pinMode(SW4,INPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(LED_STATUS,OUTPUT);
  pinMode(SDCD,INPUT);
  pinMode(TX485,OUTPUT);
  pinMode(CTRL4GWIFI,OUTPUT);
  pinMode(SDCS,OUTPUT);
  pinMode(EXCS,OUTPUT);
  pinMode(ONEWIRE,OUTPUT);
  pinMode(RAINPULSE,INPUT);
  pinMode(WINDPULSE,INPUT);
  
  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);  
  Serial.println("#Start wifi...");
  enable_wifi();

  if (WiFi.status() == WL_CONNECTED) {
    mqtt_iddf = strtol(mac4.c_str(),NULL,16);
    mqtt_topicdf = "/iaq/" + mac6;
    enable_mqtt();
    lastmqtt = millis()-(uint32_t)(mqtt_stime*1000);
    if(WiFi.status() != WL_CONNECTED) {
      //lcdtft.println("#WiFi Connection Skipped");
    }
    else {
      //lcdtft.println("#WiFi Connection Ok");
    }
  }
  else {
    //lcdtft.println("#WiFi Connection Fail!!");
  }

  if (! rtc.begin()) {
    Serial.println("#Couldn't find RTC.");
  }
  else {
    Serial.println("#RTC init Ok.");
    rtc.start();
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!request_ntp_setdatetime(10)) {
      if (!request_ntp_setdatetime(10)) {
        if (!request_ntp_setdatetime(10)) {
          Serial.printf("#Too many retry NTP request, reboot...");
          ESP.restart();
        }
      }
    }
  }

 	SPI.setFrequency(2000000);
	SPI.begin(SDSCK,SDMISO,SDMOSI); //sck, miso, mosi, ss

  test_sdcard();

  Serial1.begin(baud,config,RX1,TX1);
  #if testcanbus == 0
  modbus.begin(id,baud);
  modbus.configureInputRegisters(numInputRegisters, inputRegisterRead);
  Serial.println("#Init MODBUS done.");
#else
#endif

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("#SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  else {
    Serial.println(F("#SSD1306 allocation ok"));
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("#Ready"));
  display.display();

  digitalWrite(BUZZER,HIGH);
  Serial.println("#Ready");
  delay(250);
  digitalWrite(BUZZER,LOW);
  digitalWrite(LED_STATUS,HIGH);
  digitalWrite(BUZZER,HIGH);
  delay(250);
  digitalWrite(BUZZER,LOW);
  //testdrawline();      // Draw many lines
}

void short_beep()
{
  digitalWrite(BUZZER,HIGH);
  delay(200);
  digitalWrite(BUZZER,LOW);
}

void calc_ntc()
{
    int Vo;
    float R1 = 10000;
    float logR2, R2, tKelvin, tCelsius, tFahrenheit;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

    average = 0;
    for (int i = 0; i < NUMSAMPLES; ++i) {
      average += analogRead(THERMISTORPIN);
      delay(10);
    }
    average /= NUMSAMPLES;
    Vo = average;
    R2 = R1 * (4095.0 / (float)Vo - 1.0); // resistance of the Thermistor
    logR2 = log(R2);
    tKelvin = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    tCelsius = tKelvin - 273.15;
    tFahrenheit = (tCelsius * 9.0) / 5.0 + 32.0;    
    steinhart = tCelsius;
}

void loop()
{
  esp_task_wdt_reset();
  server.handleClient();
#if testcanbus == 0
  modbus.poll();
#else
#endif  
  MQTT_CLIENT.loop();

  if(digitalRead(SDCD) != osdcd) {
    delay(20);
    if(digitalRead(SDCD) != osdcd) {
      osdcd = digitalRead(SDCD);
      Serial.printf("SDCard Detect = %d\n",osdcd);
    }
  }

  if(digitalRead(RAINPULSE) != orain) {
    delay(20);
    if(digitalRead(RAINPULSE) != orain) {
      orain = digitalRead(RAINPULSE);
      Serial.printf("Rain Pulse = %d\n",orain);
    }
  }

  if(digitalRead(WINDPULSE) != owind) {
    delay(20);
    if(digitalRead(WINDPULSE) != owind) {
      owind = digitalRead(WINDPULSE);
      Serial.printf("Wind Pulse = %d\n",owind);
    }
  }

  if(digitalRead(SW1)==LOW) {
    delay(20);
    if(digitalRead(SW1)==LOW) {
      Serial.println("#SW1 Press");
      short_beep();
      while(digitalRead(SW1)==LOW);
    }
  }

  if(digitalRead(SW2)==LOW) {
    delay(20);
    if(digitalRead(SW2)==LOW) {
      Serial.println("#SW2 Press");
      short_beep();        
      while(digitalRead(SW2)==LOW);
    }
  }

  if(digitalRead(SW3)==LOW) {
    delay(20);
    if(digitalRead(SW3)==LOW) {
      Serial.println("#SW3 Press");
      short_beep();        
      while(digitalRead(SW3)==LOW);
    }
  }

  if(digitalRead(SW4)==LOW) {
    delay(20);
    if(digitalRead(SW4)==LOW) {
      Serial.println("#SW4 Press");
      short_beep();        
      while(digitalRead(SW4)==LOW);
    }
  }

  //every sec
  if((uint32_t)(millis()-tsec) >= 1000) {
    tsec = millis();
    DateTime now = rtc.now();
    display.fillRect(0,18,128,15,SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.printf("%02d/%02d/%04d %02d:%02d %c%c%c",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),daysOfTheWeek[now.dayOfTheWeek()][0],daysOfTheWeek[now.dayOfTheWeek()][1],daysOfTheWeek[now.dayOfTheWeek()][2]);
    display.fillRoundRect(bx,28,8,4,1,SSD1306_WHITE);
    bx += 8;
    if (bx >= 128-8) bx = 0;
    display.display();    
  }

  if((uint32_t)(millis()-tsendscreen) >= SCREEN_TIME && SCREEN_TIME > 0) {
    DateTime now = rtc.now();
    Serial.printf("RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);

    tsendscreen = millis();
    digitalWrite(LED_STATUS,digitalRead(LED_STATUS)^1);
    //digitalWrite(BUZZER,digitalRead(BUZZER)^1);
    digitalWrite(CTRL4GWIFI,digitalRead(LED_STATUS));
    digitalWrite(EXCS,digitalRead(LED_STATUS));                                                                                                                                                                                                                                                                                ;
    digitalWrite(ONEWIRE,digitalRead(LED_STATUS));

    calc_ntc();

    //Serial.print("NTC ADC = "); 
    //Serial.println((int)average);
    Serial.print("Temperature "); 
    Serial.print(steinhart);
    Serial.println(" *C");
    display.fillRect(0,9,128,9,SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(0, 9);
    display.printf("#NTC Temp.",average);
    display.printf(" = %0.1f C",steinhart);
    display.display();

    //Serial.printf("Rain = %d\n",digitalRead(RAINPULSE));
    //Serial.printf("Wind = %d\n",digitalRead(WINDPULSE));

    //read solar sensor, par sensor
    int w2 = analogRead(SOLARADC);
    int w3 = analogRead(PARADC);
    Serial.printf("Solar Sensor ADC = %d\n",w2);
    Serial.printf("Par Sensor ADC = %d\n",w3);

    Serial2.begin(9600,SERIAL_8N1,CTRX,CTTX);
    Serial2.printf("#RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);
    Serial2.print("#CT ");
    Serial2.print("Temperature "); 
    Serial2.print(steinhart);
    Serial2.println(" *C\n");
    Serial2.flush();
    Serial2.end();    
    delay(200);

    Serial2.begin(9600,SERIAL_8N1,PMRX,PMTX);
    Serial2.printf("#RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);
    Serial2.print("#PM ");
    Serial2.print("Temperature "); 
    Serial2.print(steinhart);
    Serial2.println(" *C\n");
    Serial2.flush();
    Serial2.end();    
    delay(200);

    Serial2.begin(9600,SERIAL_8N1,CO2RX,CO2TX);
    Serial2.printf("#RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);
    Serial2.print("#CO2 ");
    Serial2.print("Temperature "); 
    Serial2.print(steinhart);
    Serial2.println(" *C\n");
    Serial2.flush();
    Serial2.end();    
    delay(200);

    Serial1.printf("#RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);
    Serial1.print("#Canbus ");
    Serial1.print("Temperature "); 
    Serial1.print(steinhart);
    Serial1.println(" *C\n");
    Serial1.flush();
    delay(200);

    Serial.println();
  }

}


// put function definitions here:
bool request_ntp_setdatetime(int wsec)
{
  ulong tstamp;

  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  timeClient.setTimeOffset(25200);  //GMT+7

  Serial.print("#Request NTP...");

  tstamp = millis();
  while(!timeClient.update() && ((ulong)(millis()-tstamp)) < wsec*1000) {
    timeClient.forceUpdate();
    esp_task_wdt_reset();
  }
  if(((ulong)(millis()-tstamp)) > wsec*1000) {
    Serial.println("Failed");
    return false;
  }
  Serial.println("Ok");

  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
  //Serial.printf("Date = %s/%s/%s\r\nTime = %s:%s:%s\r\n",
  //  formattedDate.substring(8,10).c_str(),
  //  formattedDate.substring(5,7).c_str(),
  //  formattedDate.substring(2,4).c_str(),
  //  formattedDate.substring(11,13).c_str(),
  //  formattedDate.substring(14,16).c_str(),
  //  formattedDate.substring(17,19).c_str()
  //);

  // Extract date  
  //2022-01-05T19:22:56Z
  dD = atoi(formattedDate.substring(8,10).c_str());
  dM = atoi(formattedDate.substring(5,7).c_str());
  dY = atoi(formattedDate.substring(2,4).c_str());
  dh = atoi(formattedDate.substring(11,13).c_str());
  dm = atoi(formattedDate.substring(14,16).c_str());
  ds = atoi(formattedDate.substring(17,19).c_str());
  Serial.printf("NTP Date = %02d/%02d/%04d Time = %02d:%02d:%02d\r\n",dD,dM,2000+dY,dh,dm,ds);

  rtc.adjust(DateTime(2000+dY,dM,dD,dh,dm,ds));
  DateTime now = rtc.now();
  Serial.printf("RTC Date = %02d/%02d/%04d Time = %02d:%02d:%02d Day=%s\r\n",now.day(),now.month(),now.year()+543,now.hour(),now.minute(),now.second(),daysOfTheWeek[now.dayOfTheWeek()]);

  return true;

}

void test_sdcard()
{
  if(!SD.begin(SDCS)){
    Serial.println("#Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("#No SD card attached");
    return;
  }

  Serial.print("#SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("#SD Card Size: %lluMB\n", cardSize);
  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
/*
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
*/  
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
/*
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  testFileIO(SD, "/test.txt");
*/  
  Serial.printf("#Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("#Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void printDirectory(File dir, int numTabs)
{
  while (true)
  {

    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    }
    else
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void printFile(File &file)
{
  Serial.println("Printing file contents of file: " + String(file.name()));
  while (file.available())
  {
    Serial.write(file.read());
  }
  Serial.println("\nPrinting file done.");
}

void send_gs(const char *wdata)
{
    Serial.printf("#report google sheet\n");
#ifdef USERTC  
    DateTime now = rtc.now();
    sprintf(strdata,"%02d%02d%02d,%02d%02d%02d,%s",
      now.day(),
      now.month(),
      now.year()-2000,
      now.hour(),
      now.minute(),
      now.second(),
      wdata
    );
#else
    sprintf(strdata,"010100,000000,%s",
      wdata
    );
#endif    
    Serial.printf("#data = %s\r\n",strdata);
    //Serial.println("-----------");

    HTTPClient http;
    String mac = WiFi.macAddress();
    //Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    //sprintf(saveurl,"https://www.lambda-nu.com/sfglog.php?data=%s*%s",strdata);
    //http.begin("https://www.howsmyssl.com/a/check", rootCACertificate); //HTTPS
    String Strdata = strdata;
    sprintf(saveurl,"https://script.google.com/macros/s/%s/exec?%s",deploy_id.c_str(),Strdata.c_str());
    http.begin(saveurl); //HTTP
    //Serial.print("[HTTP] GET... ");
    //Serial.println(saveurl);
    // start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if(httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        //Serial.printf("[HTTP] GET... code: %d\n", httpCode);

        // file found at server
        if(httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_FOUND) {
            cretry = 0;
            String payload = http.getString();
            Serial.println(payload);
        }
        else {
            Serial.printf("[HTTP] GET... failed, error: %d %s\r\n",httpCode, http.errorToString(httpCode).c_str());
            Serial.printf("[HTTP] URL : %s\r\n",saveurl);
            String payload = http.getString();
            Serial.println(payload);
            Serial.printf("Fail : %s\nretry\n", http.errorToString(httpCode).c_str());
            if (cretry < RETRYWEB) {
              preTime = curTime - intervalTime;
              delay(3000);
            }
            ++cretry;
        }
    } 
    http.end();

}

void write_mqttperiod_eeprom (uint16_t s)
{
  EEPROM.write(__MQTT_PERIOD__, s & 0x00ff);
  EEPROM.write(__MQTT_PERIOD__+1, s >> 8);
  EEPROM.commit();
}

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  //digitalWrite(LED_STATUS, HIGH);
  Serial.print("\r\n\r\n*** Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.print("\r\n\r\n");

  if (length > 0 && !strcmp(topic,mqtt_topic.c_str())) {
    if((char)payload[0] == 'p' || (char)payload[0] == 'P') {
      mqtt_stime = atoi((char *)(payload+1));
      if (mqtt_stime > 60000) mqtt_stime = 60000;
      write_mqttperiod_eeprom(mqtt_stime);
      Serial.printf("#set MQTT Period = %d s\n",mqtt_stime);
    } 
  }
  Serial.println();

  //digitalWrite(LED_STATUS, LOW);
}

void reconnect() {
  MQTT_CLIENT.setServer(mqtt_server.c_str(), atoi(mqtt_port.c_str()));  
  MQTT_CLIENT.setCallback(callback);
  MQTT_CLIENT.setClient(client);

  if(!mqtt_server.equals("") && !mqtt_port.equals("") && !mqtt_topic.equals("")) {
	  // Trying connect with broker.
	  Serial.printf("#Trying to connect MQTT %s:%s@%s:%s",mqtt_user.c_str(),mqtt_pass.c_str(),mqtt_server.c_str(),mqtt_port.c_str());
	  wc = 0;
	  while (!MQTT_CLIENT.connected() && wc++ < 5) {
	    esp_task_wdt_reset();
	    MQTT_CLIENT.connect(mqtt_id.c_str(),mqtt_user.c_str(),mqtt_pass.c_str()); 
	    // Wait to try to reconnect again...
	    delay(200);
	    Serial.print(".");
	  }
	
	  if(wc < 5) {
	    Serial.println("Conected");
	    MQTT_CLIENT.subscribe(mqtt_topic.c_str());
	  }
	  else
	    Serial.println("Abort");
  }
  else {
      Serial.println("#MQTT ignore test connect, no mqtt server, port, and topic information.\n");
      Serial.printf("mqtt server=%s, mqtt port=%s, mqtt topic=%s\n",mqtt_server.c_str(),mqtt_port.c_str(),mqtt_topic.c_str());
  }
}

void enable_mqtt()
{
  //read MQTT Settings
  if (mqtt_id.isEmpty() && EEPROM.read(__MQTT_ID_ADDR__) != 0xff) {
    mqtt_id = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_ID_ADDR__+pwd_c);
      mqtt_id += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-ID=%s\r\n",mqtt_id.c_str());
  }
  else {
    if (mqtt_id.isEmpty()) {
      mqtt_id = mqtt_iddf;
      Serial.printf("default MQTT-ID=%s\r\n",mqtt_id.c_str());    
    }
    else {
      Serial.printf("SDCard MQTT-ID=%s\r\n",mqtt_id.c_str());    
    }
  }

  if (mqtt_server.isEmpty() && EEPROM.read(__MQTT_SERVER_ADDR__) != 0xff) {
    mqtt_server = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_SERVER_ADDR__+pwd_c);
      mqtt_server += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-Server=%s\r\n",mqtt_server.c_str());
  }
  else {
    if(mqtt_server.isEmpty()) {
      mqtt_server = mqtt_serverdf;
      Serial.printf("default MQTT-Server=%s\r\n",mqtt_server.c_str());
    }
    else {
      Serial.printf("SDCard MQTT-Server=%s\r\n",mqtt_server.c_str());    
    }
  }

  if (mqtt_port.isEmpty() && EEPROM.read(__MQTT_PORT_ADDR__) != 0xff) {
    mqtt_port = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_PORT_ADDR__+pwd_c);
      mqtt_port += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-Port=%s\r\n",mqtt_port.c_str());
  }
  else {
    if (mqtt_port.isEmpty()) {
      mqtt_port = mqtt_portdf;
      Serial.printf("default MQTT-Port=%s\r\n",mqtt_port.c_str());    
    }
    else {
      Serial.printf("SDCard MQTT-Port=%s\r\n",mqtt_port.c_str());    
    }
  }

  if (mqtt_user.isEmpty() && EEPROM.read(__MQTT_USER_ADDR__) != 0xff) {
    mqtt_user = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_USER_ADDR__+pwd_c);
      mqtt_user += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-User=%s\r\n",mqtt_user.c_str());
  }
  else {
    if (mqtt_user.isEmpty()) {
      mqtt_user = mqtt_userdf;
      Serial.printf("default MQTT-User=%s\r\n",mqtt_user.c_str());    
    }
    else {
      Serial.printf("SDCard MQTT-User=%s\r\n",mqtt_user.c_str());    
    }
  }

  if (mqtt_pass.isEmpty() && EEPROM.read(__MQTT_PASS_ADDR__) != 0xff) {
    mqtt_pass = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_PASS_ADDR__+pwd_c);
      mqtt_pass += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-Password=%s\r\n",mqtt_pass.c_str());
  }
  else {
    if (mqtt_pass.isEmpty()) {
      mqtt_pass = mqtt_passdf;
      Serial.printf("default MQTT-Password=%s\r\n",mqtt_pass.c_str());    
    }
    else {
      Serial.printf("SDCard MQTT-Password=%s\r\n",mqtt_pass.c_str());    
    }
  }

  if (mqtt_topic.isEmpty() && EEPROM.read(__MQTT_TOPIC_ADDR__) != 0xff) {
    mqtt_topic = "";
    pwd_c=0;
    do {
      pwd_ch = EEPROM.read(__MQTT_TOPIC_ADDR__+pwd_c);
      mqtt_topic += pwd_ch;
      ++pwd_c;
    } while (pwd_ch != '\000');    
    Serial.printf("EEPROM MQTT-Topic=%s\r\n",mqtt_topic.c_str());
  }
  else {
    if (mqtt_topic.isEmpty()) {
      mqtt_topic = mqtt_topicdf;
      Serial.printf("default MQTT-Topic=%s\r\n",mqtt_topic.c_str());    
    }
    else {
      Serial.printf("SDCard MQTT-Topic=%s\r\n",mqtt_topic.c_str());    
    }
  }

  if (EEPROM.read(__MQTT_PERIOD__) == 0xff && EEPROM.read(__MQTT_PERIOD__+1) == 0xff) {
    mqtt_stime = MQTT_TIME/1000;
    Serial.printf("default MQTT-PERIOD=%d s\r\n",mqtt_stime);    
  }
  else {
    mqtt_stime = EEPROM.read(__MQTT_PERIOD__) + EEPROM.read(__MQTT_PERIOD__+1)*256;
    Serial.printf("EEPROM MQTT-PERIOD=%d s\r\n",mqtt_stime);    
  }


}

void send_mqtt(char *wdata)
{
  //if(mqtt_server != "" && mqtt_port != "" && mqtt_topic == "") {	
  if(!mqtt_server.equals("") && !mqtt_port.equals("") && !mqtt_topic.equals("")) {
	  //digitalWrite(LED_pin,HIGH);
#ifdef USERTC  
	  sprintf(strdata,"%02d%02d%02d,%02d%02d%02d,%s",
	    dD % 32,
	    dM % 13,
	    dY % 100,
	    dh % 25,
	    dm % 60,
	    ds % 60,
      wdata
	  );
#else
	  sprintf(strdata,"%s",
      wdata
	  );
#endif    
	
	  //char wdata2[301];
	  //for(int i = 0; i < 250; ++i) wdata2[i] = 0;
	  //strncpy(wdata2,strdata,strlen(wdata));
	  //String Payload = wdata2;
	  String mac = WiFi.macAddress();
	  sprintf(saveurl,"{\"mac\":\"%s\",\"data\":\"%s\",\"period\":\"%d\"}",mac.c_str(),strdata,mqtt_stime);
	  String PayloadTest = "{ \"data\":\"test\" }";
	  //mqtt_topic = "/jca/plug";
	  Serial.println("#Topic : "+mqtt_topic);
	  Serial.printf("#Payload (%d) : %s\r\n",strlen(saveurl),saveurl);
	
	  if (!MQTT_CLIENT.connected()) {
	    reconnect();
	  }
	
	  if (MQTT_CLIENT.publish(mqtt_topic.c_str(),saveurl)) {
	    Serial.println("#MQTT sent ok.\n");
	    sendsucc = 1;
	  }
	  else {
	    Serial.println("#MQTT sent failed.\n");
	  }
	
	  //MQTT_CLIENT.publish(mqtt_topic.c_str(),PayloadTest.c_str());
  }
  else {
      Serial.println("#MQTT ignore, no mqtt server, port, and topic information.\n");
      Serial.printf("mqtt server=%s, mqtt port=%s, mqtt topic=%s\n",mqtt_server.c_str(),mqtt_port.c_str(),mqtt_topic.c_str());
  }
}

int inputRegisterRead(short unsigned int address)
{
  Serial.printf("#RTU at %d\n",address);
  return address;
  if (address-MODBUSBASEADDR < MAXREGISTER_MODBUS)
    return rmodbus[address-MODBUSBASEADDR];
  else    
    return -1;
}

boolean readPMSdata(Stream *s) 
{
  uint8_t buffer[32];    
  uint16_t sum = 0;

  Serial.println("PMS wait serial");
  int fb;
  long cc = 0;
  do {
    fb = s->read();
    //Serial.printf("fb=%x\n",fb);
  } while (fb != 0x42 && ++cc < 2000000);
  if (cc >= 2000000) {
    Serial.println("PMS wait timeout");
    return false;
  }
  sum = fb;
  buffer[0] = fb;
  //Serial.print("Data[");
  //Serial.print(0);
  //Serial.print("] = ");
  //Serial.println(fb,HEX);

  for (int j = 0; j < 31; ++j) {
    do {
      fb = s->read();
    } while (fb == -1);
    //Serial.print("Data[");
    //Serial.print(j+1);
    //Serial.print("] = ");
    //Serial.println(fb,HEX);
    if (j < 29) sum += fb;
    buffer[j+1] = fb;
  }
  Serial.print("Sum = ");
  Serial.println((unsigned)(buffer[30]<<8)+buffer[31],HEX);

  //swap value for library
  for (int i = 0; i < 16; ++i)
  {
    uint8_t t = buffer[i*2];
    buffer[i*2] = buffer[i*2+1];
    buffer[i*2+1] = t;
  }
  memcpy((void *)&pmsdata_realtime, (void *)buffer+2, 30);
  Serial.print("Lib = ");
  Serial.println(pmsdata.checksum,HEX);

  if ((unsigned)sum == (unsigned)pmsdata_realtime.checksum) {
    pmsdata = pmsdata_realtime;
    return true;    
  }
  else {
    Serial.println("Checksum failure");
    return false;    
  }

  return true;

}

void pwm_out()
{
  //analogWriteRange(100);
  //analogWrite(IOPIN5, outputpwm);      
}

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
