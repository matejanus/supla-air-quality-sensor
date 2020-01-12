
/**
 * Supla.org NodeMCU WiFi minimal example
 * Author: Programistyk - Kamil Kaminski <kamil@programistyk.pl>
 * 
 * This example shows how to configure SuplaDevice for building for NodeMCU within Arduino IDE
 */


#include <srpc.h>
#include <log.h>
#include <eh.h>
#include <proto.h>
#include <IEEE754tools.h>
// We define our own ethernet layer
#define SUPLADEVICE_CPP
#include <SuplaDevice.h>
#include <lck.h>

#include <WiFiClient.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiServer.h>
#include <ESP8266WiFiGeneric.h>
#include <WiFiClientSecure.h>
#include <ESP8266WiFiAP.h>  
#include <ESP8266WiFiSTA.h>
#include <WiFiUdp.h>
#include <Seeed_HM330X.h>   


#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL_OUTPUT SerialUSB
#else
#define SERIAL_OUTPUT Serial
#endif

#define SLEEP_TIME (30 * 1000)
WiFiClient client;
HM330X sensor;
u8 buf[30];
uint32_t last_read_pm25 = 0;
double buf_pm_25 = 0;
double buf_pm_10 = 0;
// Setup Supla connection
const char* ssid     = "coma_mierda";
const char* password = "KapitanPusheen1";


u16 get_pm25_AQI(double data)
{
  u16 AQI = 0;
  if (data <= 12)
  {
    AQI = (((50 - 0)/(12.0 - 0.0)) * (data - 0.0)) + 0;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
  }
  else if (data > 12 && data <= 35.4)
  {
    AQI = (((100 - 51)/(35.4-12.1)) * (data - 12.1)) + 51;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
    SERIAL_OUTPUT.print(" ");
  }
  else if (data > 35.4 && data <= 55.4)
  {
    AQI = (((150 - 101)/(55.4-35.5)) * (data - 35.5)) + 101;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
    SERIAL_OUTPUT.print(" ");
  }
  else if (data > 55.4 && data <= 150.4)
  {
    AQI = (((200 - 151)/(150.4-55.5)) * (data - 55.5)) + 151;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
    SERIAL_OUTPUT.print(" ");
  }
  else if (data > 150.4 && data <= 250.4)
  {
    AQI = (((300 - 201)/(250.4-150.5)) * (data - 150.5)) + 201;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
    SERIAL_OUTPUT.print(" ");
  }
  else if (data > 250.4 && data <= 500.4)
  {
    AQI = (((500 - 301)/(500.4-250.5)) * (data - 250.5)) + 301;
    SERIAL_OUTPUT.print("AQI: ");
    SERIAL_OUTPUT.print(AQI);
    SERIAL_OUTPUT.print(" ");
  }
  return AQI;
}

u16 get_pm10_AQI(u16 *data)
{
  return (u16)data[4*2]<<8|data[4*2+1];
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

double get_pm25(int channelNumber, double t) {

  if (( millis() -last_read_pm25 )<SLEEP_TIME)
  {
    Serial.println("Skipping measurements");
  }
  else{
    Serial.println("measurig");
    if(sensor.read_sensor_value(buf,29))
    {
        Serial.println("HM330X read result failed!!!");
        buf_pm_25 = 0;
        buf_pm_10 = 0;
        last_read_pm25 = millis();
    }
    else{
      parse_result_value(buf);
      buf_pm_25 = get_pm25(buf);
      buf_pm_10 = get_pm10(buf);
//      get_pm25_AQI(buf_pm_25);
      SERIAL_OUTPUT.print("callback measure pm2.5: ");
      SERIAL_OUTPUT.print(buf_pm_25);
      SERIAL_OUTPUT.print("callback measure pm10: ");
      SERIAL_OUTPUT.print(buf_pm_10);
      last_read_pm25 = millis();
    }
  }
  return buf_pm_25;
  }

double get_pm10(int channelNumber, double t) {
  return buf_pm_10;
  }

void setup() {
  Serial.begin(115200);
  delay(100);

  SERIAL_OUTPUT.println("Serial start");
  if(sensor.init())
  {
      SERIAL_OUTPUT.println("HM330X init failed!!!");
      while(1);
  }

  char GUID[SUPLA_GUID_SIZE] = {0xB0,0xB0,0xB0,0xF3,0xD2,0xDF,0x53,0x09,0x74,0x73,0x9B,0x6C,0x12,0x9B,0xE7,0xBD};
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

    
  SuplaDevice.setDistanceCallback(&get_pm25);
  SuplaDevice.setTemperatureCallback(&get_pm10);
  
  SuplaDevice.addDistanceSensor();
  SuplaDevice.addDS18B20Thermometer();
  
  SuplaDevice.begin(GUID,              // Global Unique Identifier 
                    mac,               // Ethernet MAC address
                    "svr4.supla.org",  // SUPLA server address
                    2453,                 // Location ID 
                    "e165");               // Location Password
                    
 if(sensor.read_sensor_value(buf,29))
  {
      Serial.println("HM330X read result failed!!!");
  }
  last_read_pm25 = millis();
  parse_result_value(buf);
  buf_pm_25 = get_pm25(buf);
  buf_pm_10 = get_pm10(buf);
  SERIAL_OUTPUT.print("first measure pm2.5: ");
  SERIAL_OUTPUT.print(buf_pm_25);
  SERIAL_OUTPUT.print("first measure pm10: ");
  SERIAL_OUTPUT.print(buf_pm_10);
  
}

void loop() {
  SuplaDevice.iterate();
}

// Supla.org ethernet layer
    int supla_arduino_tcp_read(void *buf, int count) {
        _supla_int_t size = client.available();
       
        if ( size > 0 ) {
            if ( size > count ) size = count;
            return client.read((uint8_t *)buf, size);
        };
    
        return -1;
    };
    
    int supla_arduino_tcp_write(void *buf, int count) {
        return client.write((const uint8_t *)buf, count);
    };
    
    bool supla_arduino_svr_connect(const char *server, int port) {
          return client.connect(server, 2015);
    }
    
    bool supla_arduino_svr_connected(void) {
          return client.connected();
    }
    
    void supla_arduino_svr_disconnect(void) {
         client.stop();
    }
    
    void supla_arduino_eth_setup(uint8_t mac[6], IPAddress *ip) {

       // Serial.println("WiFi init");
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.print("\nlocalIP: ");
        Serial.println(WiFi.localIP());
        Serial.print("subnetMask: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("gatewayIP: ");
        Serial.println(WiFi.gatewayIP());
    }

SuplaDeviceCallbacks supla_arduino_get_callbacks(void) {
          SuplaDeviceCallbacks cb;
          
          cb.tcp_read = &supla_arduino_tcp_read;
          cb.tcp_write = &supla_arduino_tcp_write;
          cb.eth_setup = &supla_arduino_eth_setup;
          cb.svr_connected = &supla_arduino_svr_connected;
          cb.svr_connect = &supla_arduino_svr_connect;
          cb.svr_disconnect = &supla_arduino_svr_disconnect;
          cb.get_temperature = NULL;
          cb.get_temperature_and_humidity = NULL;
          cb.get_rgbw_value = NULL;
          cb.set_rgbw_value = NULL;
          
          return cb;
}
