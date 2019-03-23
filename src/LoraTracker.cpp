#include <LoRaWan.h>
#include <CayenneLPP.h>
#include "TinyGPS++.h"
#include "TheThingsNetwork.h"

char* DEV_EUI = (char *) "00AF354029FEE928";
char* APP_EUI = (char *) "70B3D57ED00198B1";
char* APP_KEY = (char *) "F557758A8F658A9C80C0932C1F878D38";

// settings
const uint32_t UPDATE_INTERVAL = 120000; // ms
const uint16_t GPS_WATCHDOG = 10000; // ms

enum {
    LOCATION_UPDATED,
    DATE_UPDATED,
    TIME_UPDATED,
    ALTITUDE_UPDATED,
    SATELLITES_UPDATED,
    SPEED_UPDATED,
    COURSE_UPDATED,
    HDOP_UPDATED,
//    RESPONSE_UPDATED
};

// GPS commands
const char GPS_COMMAND_STANDBY[] = "$PMTK161,0*28\r\n";
//const char GPS_RESPONSE_STANDBY[] = "$PMTK001,161,3*36\r\n";
//const char GPS_COMMAND_GPS_FIX_INTERVAL_1s[] = "$PMTK220,1000*1F\r\n";
//const char GPS_COMMAND_GPS_FIX_INTERVAL_10s[] = "$PMTK220,10000*2F\r\n";
//const char GPS_COMMAND_GPS_FIX_INTERVAL_RESPONSE[] = "$PMTK001,220,3*30\r\n";
// TODO try other standby modes as gps takes a long time to get valid data
const char GPS_COMMAD_HOT_START[] = "$PMTK101*32\r\n";
//const char GPS_RESPONSE_STARTUP[] = "$PMTK010,001*2E\r\n";
//const char GPS_RESPONSE_AIDING_EPO[] = "$PMTK010,002*2D\r\n";
//const char GPS_RESPONSE_MTKGPS[] = "$PMTK011,MTKGPS*08\r\n"; // a startup message?
//const char GPS_COMMAND_PERIODIC_STANDBY[] = "$PMTK225,2,10000,10000*29\r\n";

//#define FREQ_RX_WNDW_SCND_US  923.3
#define FREQ_RX_WNDW_SCND_EU  869.525
//#define FREQ_RX_WNDW_SCND_AU  923.3
//#define FREQ_RX_WNDW_SCND_CN  505.3
//#define FREQ_RX_WNDW_SCND_KR  923.3
//#define FREQ_RX_WNDW_SCND_IN  866.55
//#define FREQ_RX_WNDW_SCND_AS1 923.3
//#define FREQ_RX_WNDW_SCND_AS2 923.3

//const float US_hybrid_channels[8] = {903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3}; //rx 923.3
const float EU_hybrid_channels[8] = {868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9}; //rx 869.525
//const float AU_hybrid_channels[8] = {916.8, 917.0, 917.2, 917.4, 917.6, 917.8, 918.0, 918.2}; //rx 923.3
//const float CN_hybrid_channels[8] = {487.1, 487.3, 487.5, 487.7, 486.3, 486.5, 486.7, 486.9}; //rx 505.3
//const float KR_hybrid_channels[8] = {922.1, 922.3, 922.5, 922.7, 922.9, 923.1, 923.3, 0};     //rx 921.9
//const float IN_hybrid_channels[8] = {865.0625, 865.4025, 865.9850, 0, 0, 0, 0, 0};            //rx 866.55
//const float AS1_hybrid_channels[8] = {923.2, 923.4, 922.2, 922.4, 922.6, 922.8, 923.0, 922.1}; //rx 923.2
//const float AS2_hybrid_channels[8] = {923.2, 923.4, 923.6, 923.8, 924.0, 924.2, 924.4, 924.6}; //rx 923.2

//#define DOWNLINK_DATA_RATE_US DR8
#define DOWNLINK_DATA_RATE_EU DR8
//#define DOWNLINK_DATA_RATE_AU DR8
//#define DOWNLINK_DATA_RATE_CN DR0
//#define DOWNLINK_DATA_RATE_KR DR0
//#define DOWNLINK_DATA_RATE_IN DR2
//#define DOWNLINK_DATA_RATE_AS1 DR2
//#define DOWNLINK_DATA_RATE_AS2 DR2

//#define US_RX_DR DR8
//#define EU_RX_DR DR8
//#define AU_RX_DR DR8
//#define CN_RX_DR DR0
//#define KR_RX_DR DR0
//#define IN_RX_DR DR2
//#define AS1_RX_DR DR2
//#define AS2_RX_DR DR2

//#define UPLINK_DATA_RATE_MAX_US  DR3
#define UPLINK_DATA_RATE_MAX_EU  DR5
//#define UPLINK_DATA_RATE_MAX_AU  DR3
//#define UPLINK_DATA_RATE_MAX_CN  DR5
//#define UPLINK_DATA_RATE_MAX_KR  DR5
//#define UPLINK_DATA_RATE_MAX_IN  DR5
//#define UPLINK_DATA_RATE_MAX_AS1 DR5
//#define UPLINK_DATA_RATE_MAX_AS2 DR5

//#define MAX_EIRP_NDX_US 13
//#define MAX_EIRP_NDX_EU  2
//#define MAX_EIRP_NDX_AU 13
//#define MAX_EIRP_NDX_CN  7
//#define MAX_EIRP_NDX_KR  4
//#define MAX_EIRP_NDX_IN 13
//#define MAX_EIRP_NDX AS1 5
//#define MAX_EIRP_NDX_AS2 5

//The min uplink data rate for all countries / plans is DR0
#define UPLINK_DATA_RATE_MIN DR0

#define DEFAULT_RESPONSE_TIMEOUT 5

//vars
unsigned char frame_counter = 1;
//int loopcount = 0;
char buffer[256];
TinyGPSPlus gps;
uint32_t gps_alive_timestamp;
uint16_t allUpdated = 0;
boolean gps_enabled = false;
boolean gps_enabled_target_state = true;
CayenneLPP lpp(51);

void displayInfo() {
    if (gps.location.isUpdated()) {
        SerialUSB.print(F("Location: "));
        SerialUSB.print(gps.location.lat(), 6);
        SerialUSB.print(F(","));
        SerialUSB.println(gps.location.lng(), 6);
        allUpdated |= 1 << LOCATION_UPDATED;
    }

    if (gps.date.isUpdated()) {
        SerialUSB.print(F("Date: "));
        SerialUSB.print(gps.date.month());
        SerialUSB.print(F("/"));
        SerialUSB.print(gps.date.day());
        SerialUSB.print(F("/"));
        SerialUSB.println(gps.date.year());
        allUpdated |= 1 << DATE_UPDATED;
    }

    if (gps.time.isUpdated()) {
        SerialUSB.print("Time: ");
        if (gps.time.hour() < 10) SerialUSB.print(F("0"));
        SerialUSB.print(gps.time.hour());
        SerialUSB.print(F(":"));
        if (gps.time.minute() < 10) SerialUSB.print(F("0"));
        SerialUSB.print(gps.time.minute());
        SerialUSB.print(F(":"));
        if (gps.time.second() < 10) SerialUSB.print(F("0"));
        SerialUSB.print(gps.time.second());
        SerialUSB.print(F("."));
        if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
        SerialUSB.println(gps.time.centisecond());
        allUpdated |= 1 << TIME_UPDATED;
    }

    if (gps.altitude.isUpdated()) {
        SerialUSB.print("Altitude: ");
        SerialUSB.print(gps.altitude.meters());
        SerialUSB.println("m");
        allUpdated |= 1 << ALTITUDE_UPDATED;
    }

    if (gps.satellites.isUpdated()) {
        SerialUSB.print("Satelites: ");
        SerialUSB.println(gps.satellites.value());
        allUpdated |= 1 << SATELLITES_UPDATED;
    }

    if (gps.speed.isUpdated()) {
        SerialUSB.print("Speed: ");
        SerialUSB.println(gps.speed.kmph());
        allUpdated |= 1 << SPEED_UPDATED;
    }

    if (gps.course.isUpdated()) {
        SerialUSB.print("Course: ");
        SerialUSB.println(gps.course.deg());
        allUpdated |= 1 << COURSE_UPDATED;
    }

    if (gps.hdop.isUpdated()) {
        SerialUSB.print("Hdop: ");
        SerialUSB.print(gps.hdop.hdop());
        SerialUSB.print("/");
        SerialUSB.println(gps.hdop.value());
        allUpdated |= 1 << HDOP_UPDATED;
    }

    if (gps.response.isUpdated()) {
        int32_t v = gps.response.value();
        SerialUSB.print("Response: ");
        SerialUSB.println(v);
        if (v == _GPS_RESPONSE_STANDBY) {
            gps_enabled = false;
        } else if (v == _GPS_RESPONSE_STARTUP) {
            gps_enabled = true;
        }
    }

}

void setHybridForTTN(const float *channels) {
    for (int i = 0; i < 8; i++) {
        if (channels[i] != 0) {
            lora.setChannel((unsigned char) i, channels[i], UPLINK_DATA_RATE_MIN, UPLINK_DATA_RATE_MAX_EU);

        }
    }
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
void setup(void) {
    SerialUSB.begin(115200);
    while (!SerialUSB && millis() < 10000);
    SerialUSB.println("seeeduino is up...");

    // setup GPS
    Serial.begin(9600);     // open the GPS
    gps_alive_timestamp = millis();
    // TODO maybe set Fitness power mode PMTK262

    //setup lora
    lora.init();
//    lora.setDeviceDefault();
    lora.setDeviceReset();

    memset(buffer, 0, 256);
    lora.getVersion(buffer, 256, 1);
    SerialUSB.print(buffer);

    lora.setId(nullptr, DEV_EUI, APP_EUI);
    lora.setKey(nullptr, nullptr, APP_KEY);

    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);

    lora.setDeciveMode(LWOTAA);
    lora.setDataRate(DR5, EU868);
    lora.setAdaptiveDataRate(true);
    setHybridForTTN(EU_hybrid_channels);

    lora.setReceiceWindowFirst(0,  EU_hybrid_channels[0]);
    lora.setReceiceWindowSecond(FREQ_RX_WNDW_SCND_EU, DOWNLINK_DATA_RATE_EU);

    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
    lora.setPower(14);

//    SerialUSB.print("Starting OTTA Join.\n");
//    loopcount = 0;
//    while (!lora.setOTAAJoin(JOIN)) {
//        loopcount++;
//    }
//    SerialUSB.print("Took ");
//    SerialUSB.print(loopcount);
//    SerialUSB.println(" tries to join.");
}

void loop(void) {
    // TODO handle invalid data
    // TODO handel case, when gps doesn't find no satellites
    // get chars from GPS and process them in TinyGPSPlus
    while (Serial.available() > 0) {
        char c = (char) Serial.read();
        gps.encode(c);
        gps_alive_timestamp = millis();
//        SerialUSB.print(c);
    }

    // if some updates available, print them
    displayInfo();

    if (millis() > gps_alive_timestamp + UPDATE_INTERVAL && !gps_enabled && !gps_enabled_target_state) {
        SerialUSB.println("Waking GPS up.");
        // start gps module
        Serial.print(GPS_COMMAD_HOT_START);
        gps_enabled_target_state = true;
    }

    // if gps should be up, but didn't wake up, try again
    if (millis() > gps_alive_timestamp + GPS_WATCHDOG && !gps_enabled && gps_enabled_target_state) {
        SerialUSB.println("Trying to wake up gps again...");
        Serial.print(GPS_COMMAD_HOT_START);
        gps_alive_timestamp = millis();
    }

    if (allUpdated == 0x00FF && SerialUSB.read() == 'u') {
        // send all data to ttn
        while (!lora.setOTAAJoin(JOIN));
//        lpp.reset()mmmmmmmmmm;
//        lpp.addGPS((uint8_t) gps.satellites.value(), (float) gps.location.lat(), (float) gps.location.lng(),
//                   (float) gps.altitude.meters());
        auto lat_raw = (uint32_t)((float)gps.location.lat() * 10000);
        auto lng_raw = (uint32_t)((float)gps.location.lng() * 10000);
        char payload[8];
        payload[0] = (char) (lat_raw >> 24);
        payload[1] = (char) (lat_raw >> 16);
        payload[2] = (char) (lat_raw >> 8);
        payload[3] = (char) lat_raw;
        payload[4] = (char) (lng_raw >> 24);
        payload[5] = (char) (lng_raw >> 16);
        payload[6] = (char) (lng_raw >> 8);
        payload[7] = (char) lng_raw;
        bool result = lora.transferPacket(lpp.getBuffer(), lpp.getSize(), DEFAULT_RESPONSE_TIMEOUT);

        // see if there was a downlink
        if (result) {
            delay(50);
            frame_counter++;
            short length;
            short rssi;

            memset(buffer, 0, 256);
            length = lora.receivePacket(buffer, 256, &rssi);

            if (length) {
                SerialUSB.print("Length is: ");
                SerialUSB.println(length);
                SerialUSB.print("RSSI is: ");
                SerialUSB.println(rssi);
                SerialUSB.print("Data is: ");
                for (unsigned char i = 0; i < length; i++) {
                    SerialUSB.print("0x");
                    SerialUSB.print(buffer[i], HEX);
                    SerialUSB.print(" ");
                }
                SerialUSB.println();
            }
        }

        // send gps to sleep
        SerialUSB.println("Sending GPS to sleep.");
        Serial.print(GPS_COMMAND_STANDBY);
        gps_enabled_target_state = false;

        // reset flags here already, to avoid executing this while GPS is in standby
        allUpdated = 0;
    }

    // print debug infos from lora chip
    lora.loraDebug();
}
#pragma clang diagnostic pop
