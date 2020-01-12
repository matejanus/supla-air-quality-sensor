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
double last_meas = 0;
// Setup Supla connection
const char* ssid     = "coma_mierda";
const char* password = "KapitanPusheen1";

const char *str[]={"sensor num: ","PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                   "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                   "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                   "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                   "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                   "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
};

u16 get_pm25(u8 *data)
{
  return (u16)data[3*2]<<8|data[3*2+1];
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
//        SERIAL_OUTPUT.print(data[i],HEX);
//        SERIAL_OUTPUT.print("  ");
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
    SERIAL_OUTPUT.println(" ");
    SERIAL_OUTPUT.println(" ");
    return NO_ERROR;
}

double get_distance(int channelNumber, double t) {

  if (( millis() -last_read_pm25 )<SLEEP_TIME)
  {
    Serial.println("Skipping measurements");
  }
  else{
    Serial.println("measurig");
    if(sensor.read_sensor_value(buf,29))
    {
        Serial.println("HM330X read result failed!!!");
        last_meas = 0;
        last_read_pm25 = millis();
    }
    else{
      parse_result_value(buf);
      last_meas = get_pm25(buf);
      SERIAL_OUTPUT.println("callback measure: ");
      SERIAL_OUTPUT.print(last_meas);
      last_read_pm25 = millis();
    }
  }
  return last_meas;
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


  // ﻿Replace the falowing GUID
  char GUID[SUPLA_GUID_SIZE] = {0xB0,0xB0,0xB0,0xF3,0xD2,0xDF,0x53,0x09,0x74,0x73,0x9B,0x6C,0x12,0x9B,0xE7,0xBD};
  // ﻿with GUID that you can retrieve from https://www.supla.org/arduino/get-guid

  // Ethernet MAC address
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

  /*
   * Having your device already registered at cloud.supla.org,
   * you want to change CHANNEL sequence or remove any of them,
   * then you must also remove the device itself from cloud.supla.org.
   * Otherwise you will get "Channel conflict!" error.
   */
    
  // CHANNEL0 - RELAY
  SuplaDevice.addRelay(44, true);           // ﻿44 - ﻿Pin number where the relay is connected      
                                      // Call SuplaDevice.addRelay(44, true) with an extra "true" parameter 
                                      // to enable "port value inversion"
                                      // where HIGH == LOW, and LOW == HIGH   

  // CHANNEL1 - RELAY
 // SuplaDevice.addRelay(45, true);           // 45 - ﻿﻿Pin number where the relay is connected   

  // CHANNEL3 - TWO RELAYS (Roller shutter operation)
  //SuplaDevice.addRollerShutterRelays(46,     // 46 - ﻿﻿Pin number where the 1st relay is connected   
//                                     47, true);    // 47 - ﻿Pin number where the 2nd relay is connected  

  // CHANNEL4 - Opening sensor (Normal Open)
  //SuplaDevice.addSensorNO(A0); // A0 - ﻿Pin number where the sensor is connected
                               // Call SuplaDevice.addSensorNO(A0, true) with an extra "true" parameter
                               // to enable the internal pull-up resistor


  // CHANNEL5 - Opening sensor (Normal Open)
//  SuplaDevice.addSensorNO(A1); // A1 - ﻿Pin number where the sensor is connected


  // CHANNEL6 - DHT22 Sensor
//   SuplaDevice.addDHT11();
  // SuplaDevice.addAM2302();
  // SuplaDevice.addDHT22();
  
  SuplaDevice.setDistanceCallback(&get_distance);
  SuplaDevice.addDistanceSensor();

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
  last_meas = get_pm25(buf);
  SERIAL_OUTPUT.print("first measuer: ");
  SERIAL_OUTPUT.print(last_meas);
  
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
