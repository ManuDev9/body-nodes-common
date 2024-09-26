/**
* MIT License
* 
* Copyright (c) 2021-2024 Manuel Bottini
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include "BnConstants.h"

#ifndef __BN_DATATYPES_H
#define __BN_DATATYPES_H

#define MAX_RECEIVED_BYTES_LENGTH 150
#define MAX_LENGTH_SMALL_FIELDS 32

struct BnStatusLED {
  bool on;
  unsigned long lastToggle;
};

typedef struct BnKey_st {
    char name[MAX_LENGTH_SMALL_FIELDS];
} BnKey_t;

typedef struct BnType_st {
    char name[MAX_LENGTH_SMALL_FIELDS];
} BnType_t;

typedef struct BnIPAddress_st {
    uint8_t oct[4]; // four octates
} BnIPAddress_t;

typedef struct BnIPConnectionData_st {
  uint8_t conn_status = CONNECTION_STATUS_NOT_CONNECTED;
  BnIPAddress_t ip_address;
  bool has_ip_address;
  uint8_t received_bytes[MAX_RECEIVED_BYTES_LENGTH];
  uint16_t num_received_bytes = 0;
  unsigned long last_sent_time = 0;
  unsigned long last_rec_time = 0;
} BnIPConnectionData_t;

void BnIPConnectionData_setDisconnected(BnIPConnectionData_st *data);
void BnIPConnectionData_setWaitingACK(BnIPConnectionData_st *data);
void BnIPConnectionData_setConnected(BnIPConnectionData_st *data);
bool BnIPConnectionData_isWaitingACK(BnIPConnectionData_st *data);
bool BnIPConnectionData_isDisconnected(BnIPConnectionData_st *data);
bool BnIPConnectionData_isConnected(BnIPConnectionData_st *data);
void BnIPConnectionData_cleanBytes(BnIPConnectionData_st *data);

typedef struct BnSensorData_st {

    BnType sensortype;
    char values_str[MAX_RECEIVED_BYTES_LENGTH];  
    float values_float[5];
    int16_t values_int[5];
    uint8_t num_values;

} BnSensorData_t;

typedef struct BnAction_st {
    BnType_t type;
    char player[MAX_LENGTH_SMALL_FIELDS];
    char bodypart[MAX_LENGTH_SMALL_FIELDS];
    
    // Haptic parameters
    uint32_t duration_ms;
    uint32_t strength;

    // SetPlayer parameters
    char new_player[MAX_LENGTH_SMALL_FIELDS];

    // SetBodypart
    char new_bodypart[MAX_LENGTH_SMALL_FIELDS];

    // EnableSensor
    char sensortype[MAX_LENGTH_SMALL_FIELDS];
    bool enable;
    
    // SetWifi
    char ssid[MAX_LENGTH_SMALL_FIELDS];
    char ssid[MAX_LENGTH_SMALL_FIELDS];
    char multicast_group[MAX_LENGTH_SMALL_FIELDS];
    
} BnAction_t;

typedef struct BnMessage_st {
    char player[MAX_LENGTH_SMALL_FIELDS];
    char bodypart[MAX_LENGTH_SMALL_FIELDS];
    BnSensorData_t sensor_data; 
};

typedef union
{
    float number;
    unsigned char bytes[4];
} float_converter;

typedef union
{
    uint32_t numberU;
    int32_t numberS;
} int_converter;

#endif //__BN_DATATYPES_H
