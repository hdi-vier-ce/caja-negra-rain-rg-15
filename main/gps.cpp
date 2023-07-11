/*

  GPS module

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <Arduino.h>
#include <TinyGPS++.h>
#include "configuration.h"
#include "gps.h"
#include <Wire.h>
#include "main.h"
#include <queue>
#include "FS.h"
#include "LittleFS.h"
#include <flash.h>
#include <iostream>
#include <sstream>
#include <iterator>

uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;
uint8_t ACC ; 
uint8_t EventACC ; 
uint8_t TotalACC ;


AXP20X_Class axp2;
std::queue<String> Queue;

RG15Arduino rg15; 


char t[32]; // used to sprintf for Serial output

TinyGPSPlus _gps;
HardwareSerial _serial_gps(GPS_SERIAL_NUM);
uint8_t BatPercent = axp2.getBattPercentage();
bool Remove = axp2.isVbusRemoveIRQ() ; 
uint8_t Status ;
void gps_time(char * buffer, uint8_t size) {
    snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude() {
    return _gps.location.lat();
}

float gps_longitude() {
    return _gps.location.lng();
}

float gps_altitude() {
    return _gps.altitude.meters();
}

float gps_hdop() {
    return _gps.hdop.hdop();
}

uint8_t gps_sats() {
    return _gps.satellites.value();
}

void gps_setup() {
    _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}
void Rain_setup()
{
    myserial.begin(GPS_BAUDRATE, SERIAL_8N1,GPS_RX_PIN, GPS_TX_PIN );
    rg15.setStream(&myserial) ; 
    
}

void gps_loop() {
    while (_serial_gps.available()) {
        _gps.encode(_serial_gps.read());
    }
}
void ReadData () {
   Queue = readFile(LittleFS, FAILED_DATA_FILE);
}

#if defined(PAYLOAD_USE_FULL)

    // More data than PAYLOAD_USE_CAYENNE
    void buildPacket(uint8_t txBuffer[33])
    {
        LatitudeBinary = ((_gps.location.lat() + 90) / 180.0) * 16777215;
        LongitudeBinary = ((_gps.location.lng() + 180) / 360.0) * 16777215;
        altitudeGps = _gps.altitude.meters();
        hdopGps = _gps.hdop.value() / 10;
        sats = _gps.satellites.value();

       //get time
       char  Buffertime[9] ;
       gps_time(Buffertime,sizeof(Buffertime));
       if (getBaChStatus()=="Charging")
       {
        Status = 1 ; 
       }else {
        Status =0 ; 
       }
       if(rg15.poll()) {
            Serial.print("Accumulation: ");
	        Serial.print(rg15.acc,3);
	        Serial.print(rg15.unit);
	        Serial.write(", Event Accumulation: ");
	        Serial.print(rg15.eventAcc,3);
	        Serial.print(rg15.unit);
	        Serial.write(", Total Accumulation: ");
	        Serial.print(rg15.totalAcc,3);          
	        Serial.print(rg15.unit);
	        Serial.write(", IPH: ");
	        Serial.println(rg15.rInt,3); 
            ACC = rg15.acc ;
            EventACC = rg15.eventAcc;
            TotalACC = rg15.totalAcc;
    }
       

        sprintf(t, "Lat: %f", _gps.location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps.location.lng());
        Serial.println(t);
        sprintf(t, "Alt: %d", altitudeGps);
        Serial.println(t);
        sprintf(t, "Hdop: %d", hdopGps);
        Serial.println(t);
        sprintf(t, "Sats: %d", sats);
        Serial.println(t);
        Serial.print("time: ");
        Serial.println(Buffertime);

       uint32_t timebinary = ((_gps.time.hour()*3600)+ (_gps.time.minute()*60)+_gps.time.second());
       Serial.print("Binary: ");
       Serial.println(timebinary);
       
       uint8_t BatPercentage = BatPercent;
       Serial.print("Battery pourcentage1: ");
       Serial.println(BatPercent);
       Serial.print("Battery pourcentage: ");
       Serial.println(BatPercentage);
       Serial.print("statu: ");
       Serial.println(Status);

       
       if (!Queue.empty())
    {   
        String data = Queue.front();
        
        // std::string data;
        Serial.println("sending data to LoRaWAN");
        Serial.println(data);

        const char *c = data.c_str();
        std::istringstream iss(c);
        std::vector<std ::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
        
        double latt = std::stod(tokens[0]);
        char Latdeeg[16];
        dtostrf(latt, 8, 6, Latdeeg);
        Serial.println(Latdeeg);
        double loong = std::stod(tokens[1]);
        uint16_t altt = std::stoi(tokens[2]);
        uint8_t hdp = std::stoi(tokens[3]);
        char HopS[16];
        itoa(hdp, HopS, 10);
        Serial.println(HopS);
        uint8_t sta = std::stoi(tokens[4]);
        unsigned int hr = std::stoi(tokens[5]);
        unsigned int Mi = std::stoi(tokens[6]);
        unsigned int Sec = std::stoi(tokens[7]);
        uint8_t BPc = std::stoi(tokens[8]);
        uint8_t Bs = std::stoi(tokens[9]);
        uint32_t LatB = ((latt + 90.0) / 180.0) * 16777215.0;
        uint32_t LongB = ((loong + 180.0) / 360.0) * 16777215.0;
        uint32_t Tb = ((hr * 3600) + (Mi * 60) + Sec);
        uint8_t BPB = (BPc / 100.0) * 255.0;


        txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
        txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;
        txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
        txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;
        txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;
        txBuffer[8] = hdopGps & 0xFF;
        txBuffer[9] = sats & 0xFF;
        txBuffer[10] = (timebinary >> 16) & 0xFF;
        txBuffer[11] = ( timebinary >> 8 ) & 0xFF;
        txBuffer[12] = timebinary & 0xFF;
        txBuffer[13] = BatPercentage & 0xFF;
        txBuffer[14] = Status & 0xFF;
        txBuffer[15] = ACC & 0xFF;
        txBuffer[16] = EventACC & 0xFF;
        txBuffer[17] = TotalACC & 0xFF;
        txBuffer[18] = (LatB >> 16) & 0xFF;
        txBuffer[19] = (LatB >> 8) & 0xFF;
        txBuffer[20] = LatB & 0xFF;
        txBuffer[21] = (LongB >> 16) & 0xFF;
        txBuffer[22] = (LongB >> 8) & 0xFF;
        txBuffer[23] = LongB & 0xFF;
        txBuffer[24] = (altt >> 8) & 0xFF;
        txBuffer[25] = altt & 0xFF;
        txBuffer[26] = hdp & 0xFF;
        txBuffer[27] = sta & 0xFF;
        txBuffer[28] = (Tb >> 16) & 0xFF;
        txBuffer[29] = (Tb >> 8) & 0xFF;
        txBuffer[30] = Tb & 0xFF;
        txBuffer[31] = BPB & 0xFF;
        txBuffer[32] = Bs & 0xFF;
        if (LMIC.txrxFlags & TXRX_ACK)
        { 
            Queue.pop();
        }
        



    } else {
        txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
        txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;
        txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
        txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;
        txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;
        txBuffer[8] = hdopGps & 0xFF;
        txBuffer[9] = sats & 0xFF;
        txBuffer[10] = (timebinary >> 16) & 0xFF;
        txBuffer[11] = ( timebinary >> 8 ) & 0xFF;
        txBuffer[12] = timebinary & 0xFF;
        txBuffer[13] = BatPercentage & 0xFF;
        txBuffer[14] = Status & 0xFF;
        txBuffer[15] = ACC & 0xFF;
        txBuffer[16] = EventACC & 0xFF;
        txBuffer[17] = TotalACC & 0xFF;
        txBuffer[18] = 0;
        txBuffer[19] = 0;
        txBuffer[20] = 0;
        txBuffer[21] = 0;
        txBuffer[22] = 0;
        txBuffer[23] = 0;
        txBuffer[24] = 0;
        txBuffer[25] = 0;
        txBuffer[26] = 0;
        txBuffer[27] = 0;
        txBuffer[28] = 0;
        txBuffer[29] = 0;
        txBuffer[30] = 0;
        txBuffer[31] = 0;
        txBuffer[32] = 0;
        }
    }

#elif defined(PAYLOAD_USE_CAYENNE)

    // CAYENNE DF
    void buildPacket(uint8_t txBuffer[11])
    {
        sprintf(t, "Lat: %f", _gps.location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps.location.lng());
        Serial.println(t);        
        sprintf(t, "Alt: %f", _gps.altitude.meters());
        Serial.println(t);        
        int32_t lat = _gps.location.lat() * 10000;
        int32_t lon = _gps.location.lng() * 10000;
        int32_t alt = _gps.altitude.meters() * 100;
        
        txBuffer[2] = lat >> 16;
        txBuffer[3] = lat >> 8;
        txBuffer[4] = lat;
        txBuffer[5] = lon >> 16;
        txBuffer[6] = lon >> 8;
        txBuffer[7] = lon;
        txBuffer[8] = alt >> 16;
        txBuffer[9] = alt >> 8;
        txBuffer[10] = alt;
    }

#endif
