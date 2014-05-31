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
 * This library implements functions for writing frames to Serial,
 * in Consistent Overhead Byte Stuffing (COBS) encoding.
 * As well as a decoder for COBS.
 *
 * You must #define COBS_RECEIVE_MAX_FRAME_SIZE to use the decoder.
 */

#include "Arduino.h"

// Writes buffer to the serial port, using COBS encoding
void serialWriteCOBS(uint8_t* buffer, int length) {
    if (length == 0) {
        return;
    }

    // Send 0x00 as the frame delimiter
    Serial.write((uint8_t) 0x00);

    // The first element to send (inclusive)
    int first = 0;
    // Last element in the section (inclusive) (usually a zero)
    int last = 0;

    do {
        // Find the end of this section
        for (; buffer[last] != 0 && last < length; last++) {
            // sections can't be longer than 254 bytes
            if (last - first + 1 == 254) {
                break;
            }
        }

        if (last - first + 1 == 254 && buffer[last] != 0) {
            // 0xFF is the 254 section with no implicit zero
            Serial.write((uint8_t) 0xFF);
            // Write the data
            Serial.write(buffer + first, last - first + 1);
        }
        else {
            // Write the length of this section (including the zero)
            Serial.write((uint8_t) (last - first + 1));
            // Write the data, excluding the zero
            Serial.write(buffer + first, last - first);
        }

        last++;
        first = last;
    } while(first <= length); // Use '<=' because we want to write an implicit zero at the end
}

#ifndef COBS_RECEIVE_MAX_FRAME_SIZE
#define COBS_RECEIVE_MAX_FRAME_SIZE 0
#endif

// You must #define COBS_RECEIVE_MAX_FRAME_SIZE to use the decoder.
class COBSDecoder
{
private:
    // +1 so that there's room to write the implicit zero
    uint8_t buffer[COBS_RECEIVE_MAX_FRAME_SIZE + 1];
    int i;
    bool noImplicitZero;
    uint8_t remainingBytesInSection;
    bool error;

public:
    COBSDecoder()
    {
        reset();
    }

    /* 
     * Updates the decoding with the next byte in the stream
     */
    void update(uint8_t data)
    {
        if (error) {
            return;
        }
        if (data == 0) {
            // Frame is corrupt. Zero isn't allowed.
            error = true;
            return;
        }

        if (remainingBytesInSection == 0) {
            if (data + i >= COBS_RECEIVE_MAX_FRAME_SIZE + 1) {
                // Frame is too long.
                error = true;
                return;
            }
            if (data == 1) {
                buffer[i] = 0;
                i++;
                noImplicitZero = false;
            }
            else if (data == 0xFF) {
                remainingBytesInSection = 254;
                noImplicitZero = true;
            }
            else {
                remainingBytesInSection = data;
                noImplicitZero = false;
            }
            return;
        }
        
        buffer[i] = data;
        i++;
        remainingBytesInSection--;

        if (remainingBytesInSection == 1 && !noImplicitZero) {
            // Write implicit zero
            buffer[i] = 0;
            i++;
            remainingBytesInSection--;
        }
    }

    void reset()
    {
        i = 0;
        noImplicitZero = false;
        remainingBytesInSection = 0;
        error = false;
    }

    /*
     * Return: NULL if a valid frame has not been decoded.
     */
    uint8_t* getFrame()
    {
        if (error || remainingBytesInSection > 0) {
            return NULL;
        }
        else {
            return buffer;
        }
    }

    /*
     * Return: Length of the decoded frame, or -1 if a frame has not been decoded
     */
    int getFrameLength()
    {
        if (error || remainingBytesInSection > 0) {
            return -1;
        }

        if (noImplicitZero) {
            return i;
        }
        else {
            // Drop the zero from the end of the frame.
            return i - 1;
        }
    }
} COBS;
