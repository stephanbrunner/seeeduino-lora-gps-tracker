#include "LoRaWan.h"
#include "TinyGPS++.h"

TinyGPSPlus gps;

void displayInfo() {
    if (gps.location.isUpdated()) {
        SerialUSB.print(F("Location: "));
        SerialUSB.print(gps.location.lat(), 6);
        SerialUSB.print(F(","));
        SerialUSB.println(gps.location.lng(), 6);
    }

    if (gps.date.isUpdated()) {
        SerialUSB.print(F("Date: "));
        SerialUSB.print(gps.date.month());
        SerialUSB.print(F("/"));
        SerialUSB.print(gps.date.day());
        SerialUSB.print(F("/"));
        SerialUSB.println(gps.date.year());
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
    }

    if (gps.altitude.isUpdated()) {
        SerialUSB.print("Altitude: ");
        SerialUSB.print(gps.altitude.meters());
        SerialUSB.println("m");
    }

    if (gps.satellites.isUpdated()) {
        SerialUSB.print("Satelites: ");
        SerialUSB.println(gps.satellites.value());
    }

    if (gps.speed.isUpdated()) {
        SerialUSB.print("Speed: ");
        SerialUSB.println(gps.speed.kmph());
    }

}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
void setup() {
    SerialUSB.begin(115200);
    while (!SerialUSB && millis() < 10000);
    SerialUSB.println("seeeduino is up...");

    lora.init();
    lora.setDeviceReset();

    Serial.begin(9600);     // open the GPS
}


void loop() {
    if (Serial.available() > 0) {
        gps.encode((char)Serial.read());
    }

    displayInfo();
}
#pragma clang diagnostic pop
