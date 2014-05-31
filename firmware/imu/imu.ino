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
 * Firmware for sensor board with HMC5883 magnetometer.
 *
 * Has two modes for returning data: json and binary
 * Binary protocol encodes data as:
 * cobs(crc(data))
 * data: id (1 byte), x (2 bytes), y (2 bytes), z (2 bytes) (all big endian)
 * where crc() appends a 16-bit crc to the end of the message
 * and cobs() encodes the data using 
 * "Consistent Overhead Byte Stuffing" (Cheshire & Baker 1997)
 */

#include <Wire.h>
#include <util/crc16.h>
#include "COBS.h"

#define address 0x1E // HMC5883 address

#define IDLE 0
#define JSON 1
#define BINARY 2
#define COMMAND_BUFFER_LEN 128

uint8_t id;
char commandBuffer[COMMAND_BUFFER_LEN];
uint8_t commandEnd;
uint8_t mode;

void setup()
{
    id = 0;
    commandEnd = 0;
    mode = IDLE;
  
    Serial.begin(9600);
    Wire.begin();
    
    // Set mode
    Wire.beginTransmission(address);
    // Mode register
    Wire.write(0x02);
    // Continuous mode
    Wire.write(0x00); 
    Wire.endTransmission();
    
    // Change update rate to 75Hz
    Wire.beginTransmission(address);
    // Config register 
    Wire.write(0x00);
    // Update rate 75Hz
    Wire.write(0x18);
    Wire.endTransmission();
}

void loop()
{
    if (mode != IDLE) {
        output();
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
            Serial.print("ERROR: command longer than ");
            Serial.print(COMMAND_BUFFER_LEN - 1);
            Serial.println(" characters");
            commandEnd = 0;
        }
    }

    delay(10);
}

void output()
{
    int x,y,z;
  
    // Set address pointer to first data register
    // register is auto advanced after each read
    Wire.beginTransmission(address);
    Wire.write(0x03);
    Wire.endTransmission();
    
    Wire.requestFrom(address, 6);
  
    if (6 <= Wire.available()){
        // X MSB
        x = Wire.read() << 8;
        // X LSB
        x |= Wire.read();
    
        // Z MSB 
        z = Wire.read() << 8;
        // Z LSB
        z |= Wire.read();
    
        // Y MSB
        y = Wire.read() << 8;
        // Y LSB 
        y |= Wire.read();
    }

    if (mode == JSON) {
        outputJson(id, x, y, z);
    }
    else if (mode == BINARY) {
        outputBinary(id, x, y, z);
    }
    id++;
}

void outputJson(int id, int x, int y, int z) 
{
    Serial.print("{\"id\":");
    Serial.print(id);
    Serial.print(",\"x\":");
    Serial.print(x);
    Serial.print(",\"y\":");
    Serial.print(y);
    Serial.print(",\"z\":");
    Serial.print(z);
    Serial.println("}");
}

void outputBinary(uint8_t id, int x, int y, int z) 
{
    uint8_t buffer[9];
    buffer[0] = id;
    toBigEndian(buffer + 1, x);
    toBigEndian(buffer + 3, y);
    toBigEndian(buffer + 5, z);
    toBigEndian(buffer + 7, crcXmodem(buffer, 7));
    serialWriteCOBS(buffer, 9);
}

uint16_t crcXmodem(uint8_t* data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = _crc_xmodem_update(crc, data[i]);
    }
    return crc;
}

void toBigEndian(uint8_t* buffer, int value) 
{
    buffer[0] = (uint8_t) (value >> 8);
    buffer[1] = (uint8_t) (0x00FF & value);
}

void processCommand(char* c)
{
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
}
