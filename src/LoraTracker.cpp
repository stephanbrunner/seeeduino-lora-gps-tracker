#include "LoRaWan.h"
#include "TinyGPS++.h"

// settings
const uint16_t UPDATE_INTERVAL = 30000; // ms
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
    RESPONSE_UPDATED
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

TinyGPSPlus gps;
uint32_t gps_alive_timestamp;
uint16_t allUpdated = 0;
boolean gps_enabled = false;
boolean gps_enabled_target_state = true;

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
        Serial.println(gps.course.deg());
        allUpdated |= 1 << COURSE_UPDATED;
    }

    if (gps.hdop.isUpdated()) {
        SerialUSB.print("Hdop: ");
        Serial.print(gps.hdop.hdop());
        Serial.print("/");
        Serial.println(gps.hdop.value());
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

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
void setup() {
    SerialUSB.begin(115200);
    while (!SerialUSB && millis() < 10000);
    SerialUSB.println("seeeduino is up...");

    // setup Lora
    lora.init();
    lora.setDeviceReset();

    // setup GPS
    Serial.begin(9600);     // open the GPS
    gps_alive_timestamp = millis();
    // TODO maybe set Fitness power mode PMTK262
}


void loop() {
    // TODO handle invalid data
    // TODO handel case, when gps doesn't find no satellites
    // get chars from GPS and process them in TinyGPSPlus
    if (Serial.available() > 0) {
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

    if (allUpdated == 0x00FF) {
        SerialUSB.println("Sending GPS to sleep.");
        Serial.print(GPS_COMMAND_STANDBY);
        gps_enabled_target_state = false;

        // reset flags here already, to avoid executing this while GPS is in standby
        allUpdated = 0;
    }
}
#pragma clang diagnostic pop
