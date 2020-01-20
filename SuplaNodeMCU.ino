
/**
 * Supla.org NodeMCU WiFi minimal example
 * Author: Programistyk - Kamil Kaminski <kamil@programistyk.pl>
 * 
 * This example shows how to configure SuplaDevice for building for NodeMCU within Arduino IDE
 */


#define SUPLADEVICE_CPP
#include <SuplaDevice.h>
#include <FS.h>
#include <SuplaImpulseCounter.h>
#include <io.h>
#include <Seeed_HM330X.h>   
#include <WiFiManager.h>
#include <ArduinoJson.h> //--------- https://github.com/bblanchon/ArduinoJson/tree/v5.13.2 ------

#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL_OUTPUT SerialUSB
#else
#define SERIAL_OUTPUT Serial
#endif

extern "C"
{
#include "user_interface.h"
} 

#define SLEEP_TIME (30 * 1000)
WiFiClient client;
HM330X sensor;
u8 buf[30];
uint32_t last_read_pm25 = 0;
double buf_pm_25 = 0;
double buf_pm_10 = 0;
double buf_pm_1 = 0;
// Setup Supla connection

int wificonfig_pin = 0; // D3
int C_W_state = HIGH; 
int last_C_W_state = HIGH;
unsigned long time_last_C_W_change = 0; 

long C_W_delay = 5000;               // ---------------------- config delay 5 seconds ---------------------------
char Supla_server[81];
char Email[81];
char Supla_name[51];
char Supla_status[51];
byte mac[6];
bool shouldSaveConfig = false;
bool initialConfig = false;
int npm;

WiFiManager wifiManager;
#include <supla/network/esp_wifi.h>
Supla::ESPWifi wifi("", "");  //------ Do not change----wifimanager takes care------

void saveConfigCallback () {          
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void ondemandwifiCallback () {
   
   WiFiManagerParameter custom_Supla_server("server", "supla server", Supla_server, 81,"required");
   WiFiManagerParameter custom_Email("email", "Email", Email, 81,"required");
   WiFiManagerParameter custom_Supla_name("name", "Supla Device Name", Supla_name, 51,"required");
   WiFiManagerParameter custom_Supla_status("status", "Supla Last State", Supla_status, 51,"readonly");
   
   wifiManager.setBreakAfterConfig(true);
   wifiManager.setSaveConfigCallback(saveConfigCallback);
 
  
   wifiManager.addParameter(&custom_Supla_server);
   wifiManager.addParameter(&custom_Email);
    wifiManager.addParameter(&custom_Supla_name);
    wifiManager.addParameter(&custom_Supla_status);

   wifiManager.setCustomHeadElement("<style>html{ background-color: #01DF3A;}</style><div class='s'><svg version='1.1' id='l' x='0' y='0' viewBox='0 0 200 200' xml:space='preserve'><path d='M59.3,2.5c18.1,0.6,31.8,8,40.2,23.5c3.1,5.7,4.3,11.9,4.1,18.3c-0.1,3.6-0.7,7.1-1.9,10.6c-0.2,0.7-0.1,1.1,0.6,1.5c12.8,7.7,25.5,15.4,38.3,23c2.9,1.7,5.8,3.4,8.7,5.3c1,0.6,1.6,0.6,2.5-0.1c4.5-3.6,9.8-5.3,15.7-5.4c12.5-0.1,22.9,7.9,25.2,19c1.9,9.2-2.9,19.2-11.8,23.9c-8.4,4.5-16.9,4.5-25.5,0.2c-0.7-0.3-1-0.2-1.5,0.3c-4.8,4.9-9.7,9.8-14.5,14.6c-5.3,5.3-10.6,10.7-15.9,16c-1.8,1.8-3.6,3.7-5.4,5.4c-0.7,0.6-0.6,1,0,1.6c3.6,3.4,5.8,7.5,6.2,12.2c0.7,7.7-2.2,14-8.8,18.5c-12.3,8.6-30.3,3.5-35-10.4c-2.8-8.4,0.6-17.7,8.6-22.8c0.9-0.6,1.1-1,0.8-2c-2-6.2-4.4-12.4-6.6-18.6c-6.3-17.6-12.7-35.1-19-52.7c-0.2-0.7-0.5-1-1.4-0.9c-12.5,0.7-23.6-2.6-33-10.4c-8-6.6-12.9-15-14.2-25c-1.5-11.5,1.7-21.9,9.6-30.7C32.5,8.9,42.2,4.2,53.7,2.7c0.7-0.1,1.5-0.2,2.2-0.2C57,2.4,58.2,2.5,59.3,2.5z M76.5,81c0,0.1,0.1,0.3,0.1,0.6c1.6,6.3,3.2,12.6,4.7,18.9c4.5,17.7,8.9,35.5,13.3,53.2c0.2,0.9,0.6,1.1,1.6,0.9c5.4-1.2,10.7-0.8,15.7,1.6c0.8,0.4,1.2,0.3,1.7-0.4c11.2-12.9,22.5-25.7,33.4-38.7c0.5-0.6,0.4-1,0-1.6c-5.6-7.9-6.1-16.1-1.3-24.5c0.5-0.8,0.3-1.1-0.5-1.6c-9.1-4.7-18.1-9.3-27.2-14c-6.8-3.5-13.5-7-20.3-10.5c-0.7-0.4-1.1-0.3-1.6,0.4c-1.3,1.8-2.7,3.5-4.3,5.1c-4.2,4.2-9.1,7.4-14.7,9.7C76.9,80.3,76.4,80.3,76.5,81z M89,42.6c0.1-2.5-0.4-5.4-1.5-8.1C83,23.1,74.2,16.9,61.7,15.8c-10-0.9-18.6,2.4-25.3,9.7c-8.4,9-9.3,22.4-2.2,32.4c6.8,9.6,19.1,14.2,31.4,11.9C79.2,67.1,89,55.9,89,42.6z M102.1,188.6c0.6,0.1,1.5-0.1,2.4-0.2c9.5-1.4,15.3-10.9,11.6-19.2c-2.6-5.9-9.4-9.6-16.8-8.6c-8.3,1.2-14.1,8.9-12.4,16.6C88.2,183.9,94.4,188.6,102.1,188.6z M167.7,88.5c-1,0-2.1,0.1-3.1,0.3c-9,1.7-14.2,10.6-10.8,18.6c2.9,6.8,11.4,10.3,19,7.8c7.1-2.3,11.1-9.1,9.6-15.9C180.9,93,174.8,88.5,167.7,88.5z'/></svg>");
   wifiManager.setMinimumSignalQuality(8);
   wifiManager.setConfigPortalTimeout(180);

   if (!wifiManager.startConfigPortal("Supla air quality sensor")) { Serial.println("Not connected to WiFi but continuing anyway.");} else { Serial.println("connected...yeey :)");}                
    strcpy(Supla_server, custom_Supla_server.getValue());
    strcpy(Email, custom_Email.getValue());
    strcpy(Supla_name, custom_Supla_name.getValue());
     WiFi.softAPdisconnect(true);   //  close AP
}

void status_func(int status, const char *msg) {    //    ------------------------ Status --------------------------
  if (status != 10){
     strcpy(Supla_status, msg); 
  }              
}

u16 get_pm1(u8 *data)
{
  return (u16)data[2*2]<<8|data[2*2+1];
}

u16 get_pm25(u8 *data)
{
  return (u16)data[3*2]<<8|data[3*2+1];
}

u16 get_pm10(u8 *data)
{
  return (u16)data[4*2]<<8|data[4*2+1];
}

HM330XErrorCode print_result(const char* str,u16 value)
{
    if(NULL==str)
        return ERROR_PARAM;
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);
    return NO_ERROR;
}

HM330XErrorCode parse_result_value(u8 *data)
{
    if(NULL==data)
        return ERROR_PARAM;
    for(int i=0;i<28;i++)
    {
        if((0==(i)%5)||(0==i))
        {
            SERIAL_OUTPUT.println(" ");
        }
    }
    u8 sum=0;
    for(int i=0;i<28;i++)
    {
        sum+=data[i];
    }
    if(sum!=data[28])
    {
        SERIAL_OUTPUT.println("wrong checkSum!!!!");
    }
    return NO_ERROR;
}


double get_norm_from_concentration_pm25(double pm)
{
  return pm*4;
}

double get_norm_from_concentration_pm10(double pm)
{
  return pm*2;
}

void get_temperature_and_humidity(int channelNumber, double *temp, double *humidity) {

  if (( millis() -last_read_pm25 )<SLEEP_TIME)
  {
    Serial.println("Skipping measurements");
  }
  else{
    Serial.println("measurig");
    if(sensor.read_sensor_value(buf,29))
    {
        Serial.println("HM330X read result failed!!!");
        *temp = 0;
        *humidity = 0;
        last_read_pm25 = millis();
    }
    else{
      parse_result_value(buf);
      buf_pm_25 = get_pm25(buf);
      buf_pm_10 = get_pm10(buf);
      buf_pm_1 = get_pm1(buf);
      *temp = get_norm_from_concentration_pm25(buf_pm_25);
      *humidity = get_norm_from_concentration_pm10(buf_pm_10);
      SERIAL_OUTPUT.print("callback measure pm2.5: ");
      SERIAL_OUTPUT.print(buf_pm_25);
      SERIAL_OUTPUT.print("callback measure pm10: ");
      SERIAL_OUTPUT.print(buf_pm_10);
      last_read_pm25 = millis();
    }
  }

}
  
double get_pm1_as_temp(int channelNumber, double temp)
{
  SERIAL_OUTPUT.print("callback measure pm1.0: ");
  SERIAL_OUTPUT.print(buf_pm_1);
  return buf_pm_1;
}

void setup() {
  wifi_set_sleep_type(NONE_SLEEP_T);
  Serial.begin(115200);
  delay(100);

    Serial.println("program start");
  if (WiFi.SSID()==""){ initialConfig = true;}  
  
  SERIAL_OUTPUT.println("Serial start");
  if(sensor.init())
  {
      SERIAL_OUTPUT.println("HM330X init failed!!!");
      while(1);
  }


  if (SPIFFS.begin()) {  // ------------------------- wificonfig read -----------------
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;         
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);   //print config data to serial on startup
        if (json.success()) {Serial.println("\nparsed json");         
          strcpy(Supla_server, json["Supla_server"]);
          strcpy(Email, json["Email"]);
          strcpy(Supla_name, json["Supla_name"]);         
        } else {
          Serial.println("failed to load json config");
          initialConfig = true;
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

//  char GUID[SUPLA_GUID_SIZE] = {0xB0,0xB0,0xB0,0xF3,0xD2,0xDF,0x53,0x09,0x74,0x73,0x9B,0x6C,0x12,0x9B,0xE7,0xBD};
//  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

//    
//  SuplaDevice.setDistanceCallback(&get_pm25);


   uint8_t mac[WL_MAC_ADDR_LENGTH];
   WiFi.macAddress(mac);

   char GUID[SUPLA_GUID_SIZE] = {mac[WL_MAC_ADDR_LENGTH - 6], mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 3],                                
                                 mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 2], 
                                 mac[WL_MAC_ADDR_LENGTH - 3], mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 6]};
   char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 2],mac[WL_MAC_ADDR_LENGTH - 3], mac[WL_MAC_ADDR_LENGTH - 4],
                                       mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 6],mac[WL_MAC_ADDR_LENGTH - 6], mac[WL_MAC_ADDR_LENGTH - 5],
                                       mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 3],mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1]};
  
//  SuplaDevice.addDistanceSensor();
//  SuplaDevice.addDS18B20Thermometer();
  SuplaDevice.setTemperatureHumidityCallback(&get_temperature_and_humidity);
  SuplaDevice.setTemperatureCallback(&get_pm1_as_temp);
  
  SuplaDevice.addDHT22();  
  SuplaDevice.addDS18B20Thermometer(); 
  
  SuplaDevice.setName(Supla_name);
  SuplaDevice.setStatusFuncImpl(&status_func);
  
 if(sensor.read_sensor_value(buf,29))
  {
      Serial.println("HM330X read result failed!!!");
  }
  last_read_pm25 = millis();
  parse_result_value(buf);
  buf_pm_25 = get_pm25(buf);
  buf_pm_10 = get_pm10(buf);
  buf_pm_1 = get_pm1(buf);
  SERIAL_OUTPUT.print("first measure pm2.5: ");
  SERIAL_OUTPUT.print(buf_pm_25);
  SERIAL_OUTPUT.print("first measure pm10: ");
  SERIAL_OUTPUT.print(buf_pm_10);
  SERIAL_OUTPUT.print("first measure pm1: ");
  SERIAL_OUTPUT.print(buf_pm_1);

  SuplaDevice.begin(GUID,              // Global Unique Identifier 
                    Supla_server,  // SUPLA server address
                    Email,   // Email address used to login to Supla Cloud
                    AUTHKEY);          // Authorization key
                    
}

void loop() {

  if (initialConfig == true){ondemandwifiCallback();}

  int C_W_read = digitalRead(wificonfig_pin);{  // ---------------------let's read the status of the Gpio to start wificonfig ---------------------
   if (C_W_read != last_C_W_state) {  time_last_C_W_change = millis();}      
    if ((millis() - time_last_C_W_change) > C_W_delay) {     
     if (C_W_read != C_W_state) {     
       C_W_state = C_W_read;       
       if (C_W_state == LOW) {
        ondemandwifiCallback () ;} } }         
     last_C_W_state = C_W_read;            
   }
      
  if (shouldSaveConfig == true) { // ------------------------ wificonfig save --------------
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["Supla_server"] = Supla_server;
    json["Email"] = Email;
    json["Supla_name"] = Supla_name;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {Serial.println("failed to open config file for writing"); }   
    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    Serial.println("config saved");    
    shouldSaveConfig = false;
    initialConfig = false; 
    WiFi.mode(WIFI_STA);   
    delay(5000);
    ESP.restart(); 
  }
  
  SuplaDevice.iterate();
}
