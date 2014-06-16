/*
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* 
 * Firmware for sensor board with HMC5883 magnetometer and MPU6050.
 *
 * Has two modes for returning data: json and binary
 * Binary protocol encodes data as:
 * cobs(crc(data))
 * data: id (1 byte), ax, ay, az, gx, gy, gz, mx, my, mz
 * all values are 2 bytes little endian
 * where crc() appends a 16-bit crc to the end of the message
 * and cobs() encodes the data using 
 * "Consistent Overhead Byte Stuffing" (Cheshire & Baker 1997)
 *
 * Output data:
 * accelerometer: divide by 32,768 to get to units (2gs)
 * gyro: divide by 32,768 to get to units (250deg/s)
 * magnetometer: divide by 1024 to get to units (gauss)
 *
 * Commands:
 * mode=idle             Stops reading data and sending output
 * mode=json             Start reading and sending json
 * mode=binary           Start reading and sending binary
 * output-interval-ms=   Set the output interval to the specified number of millis
 * debug                 Print debugging information
 */

#include <Wire.h>
#include <util/crc16.h>
#include "COBS.h"
#include "imu.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define IDLE 0
#define JSON 1
#define BINARY 2
#define ERROR_MODE 3
#define COMMAND_BUFFER_LEN 128

uint8_t id;
char commandBuffer[COMMAND_BUFFER_LEN];
uint8_t commandEnd;
uint8_t mode;
int outputIntervalMs;

long maxLoopTimeUs;
long maxReadTimeUs;
long maxOutputTimeUs;

MPU6050 mpu;
HMC5883L mag;

SensorReadings state;

void setup()
{
    id = 0;
    commandEnd = 0;
    mode = IDLE;
    maxLoopTimeUs = 0;
    maxReadTimeUs = 0;
    maxOutputTimeUs = 0;
    outputIntervalMs = 10;
  
    Serial.begin(9600);
    Wire.begin();

    // Do not call initialize() here, because that sets the device to SINGLE mode.
    // Once it's in single mode, you have to make a measurement before you 
    // can change the mode, otherwise it just ends up in IDLE mode.
    mag.setMode(HMC5883L_MODE_CONTINUOUS);
    // Set rate to 75Hz
    mag.setDataRate(HMC5883L_RATE_75);
    mag.setSampleAveraging(HMC5883L_AVERAGING_1);
    
    mpu.initialize();
    // Set rate to 8khz / (1 + rate) 
    mpu.setRate(7);
    // Set bandwidth to ~260Hz
    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    // Gyro range +/- 250deg/sec
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    // Accel up to 2g
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    // Enable interrupts
    mpu.setIntDataReadyEnabled(true);

    if (!mag.testConnection() || !mpu.testConnection()) {
        mode = ERROR_MODE;
    }
}

void loop()
{
    if (mode == ERROR_MODE) {
        Serial.println(F("Hardware failure"));
        return;
    }

    unsigned long start = micros();

    if (mode != IDLE) {
        unsigned long startRead = micros();
        readMagnetometer(&state);
        readAccelAndGyro(mpu, &state);
        unsigned long readTime = micros() - startRead;
        if (readTime > maxReadTimeUs) {
            maxReadTimeUs = readTime;
        }
        unsigned long startOutput = micros();
        output(id, state);
        unsigned long outputTime = micros() - startOutput;
        if (outputTime > maxOutputTimeUs) {
            maxOutputTimeUs = outputTime;
        }
    }

    unsigned long loopTime = micros() - start;
    if (loopTime > maxLoopTimeUs) {
        maxLoopTimeUs = loopTime;
    }
  
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            commandBuffer[commandEnd] = 0;
            processCommand(commandBuffer);
            commandEnd = 0;
        }
        else {
            commandBuffer[commandEnd] = c;
            commandEnd++;
        }
        
        if (commandEnd == COMMAND_BUFFER_LEN) {
            // Buffer overflow. Just throw out all the data we've received so far.
            Serial.print(F("ERROR: command longer than "));
            Serial.print(COMMAND_BUFFER_LEN - 1);
            Serial.println(F(" characters"));
            commandEnd = 0;
        }
    }
}

void readMagnetometer(SensorReadings *readings)
{
    if (mag.getReadyStatus()) {
        mag.getHeading(&(readings->mx), &(readings->my), &(readings->mz));
    }
}

void readAccelAndGyro(MPU6050 &mpu, SensorReadings *readings)
{
    if (mpu.getIntDataReadyStatus() == 1) {
        mpu.getAcceleration(&(readings->ax), &(readings->ay), &(readings->az));
        mpu.getRotation(&(readings->gx), &(readings->gy), &(readings->gz));
    }
}

void outputJson(int id, SensorReadings &readings) 
{
    Serial.print(F("{\"id\":"));
    Serial.print(id);
    Serial.print(F(",\"ax\":"));
    Serial.print(readings.ax);
    Serial.print(F(",\"ay\":"));
    Serial.print(readings.ay);
    Serial.print(F(",\"az\":"));
    Serial.print(readings.az);
    Serial.print(F(",\"gx\":"));
    Serial.print(readings.gx);
    Serial.print(F(",\"gy\":"));
    Serial.print(readings.gy);
    Serial.print(F(",\"gz\":"));
    Serial.print(readings.gz);
    Serial.print(F(",\"mx\":"));
    Serial.print(readings.mx);
    Serial.print(F(",\"my\":"));
    Serial.print(readings.my);
    Serial.print(F(",\"mz\":"));
    Serial.print(readings.mz);
    Serial.println("}");
}

void outputBinary(uint8_t id, SensorReadings &readings) 
{
    // id:1 byte, readings:18 bytes, crc: 2 bytes
    uint8_t buffer[1 + 9 * 2 + 2];
    buffer[0] = id;
    toLittleEndian(buffer + 1, readings.ax);
    toLittleEndian(buffer + 3, readings.ay);
    toLittleEndian(buffer + 5, readings.az);
    toLittleEndian(buffer + 7, readings.gx);
    toLittleEndian(buffer + 9, readings.gy);
    toLittleEndian(buffer + 11, readings.gz);
    toLittleEndian(buffer + 13, readings.mx);
    toLittleEndian(buffer + 15, readings.my);
    toLittleEndian(buffer + 17, readings.mz);
    toLittleEndian(buffer + 19, crcXmodem(buffer, 19));
    serialWriteCOBS(buffer, 21);
}

void output(uint8_t &id, SensorReadings &readings)
{
    static unsigned long lastUpdateMs = 0;

    unsigned long ms = millis();
    // Output at 100Hz
    if (ms - lastUpdateMs < outputIntervalMs) {
        return;
    }
    lastUpdateMs = ms;

    if (mode == JSON) {
        outputJson(id, readings);
    }
    else if (mode == BINARY) {
        outputBinary(id, readings);
    }
    id++;
}

uint16_t crcXmodem(uint8_t* data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = _crc_xmodem_update(crc, data[i]);
    }
    return crc;
}

void toLittleEndian(uint8_t* buffer, int value) 
{
    buffer[0] = (uint8_t) (0x00FF & value);
    buffer[1] = (uint8_t) (value >> 8);
}

void processCommand(char* c)
{
    if (mode == ERROR_MODE) {
        // Stay in error mode.
        return;
    }
    String command(c);
    if (command.equalsIgnoreCase("mode=json")) {
        mode = JSON;
    }
    else if (command.equalsIgnoreCase("mode=binary")) {
        mode = BINARY;
    }
    else if (command.equalsIgnoreCase("mode=idle")) {
        mode = IDLE;
    }
    else if (command.equalsIgnoreCase("debug")) {
        outputDebug();
    }
    else if (command.startsWith("output-interval-ms")) {
        int commandLength = String("output-interval-ms=").length();
        outputIntervalMs = command.substring(commandLength).toInt();
        if (outputIntervalMs == 0) {
            Serial.println(F("Invalid interval. Setting to 10ms"));
            outputIntervalMs = 10;
        }
    }
}

void outputDebug()
{
    Serial.print(F("Max loop time: "));
    Serial.print(maxLoopTimeUs);
    Serial.println(F("us"));
    Serial.print(F("Max read time: "));
    Serial.print(maxReadTimeUs);
    Serial.println(F("us"));
    Serial.print(F("Max output time: "));
    Serial.print(maxOutputTimeUs);
    Serial.println(F("us"));
    Serial.print(F("Output interval: "));
    Serial.print(outputIntervalMs);
    Serial.println(F("ms"));
}
