/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 * 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "RPLidar_c.h"
#include <time.h>
#include "esp_timer.h"

// RPLidar::RPLidar()
//     : _bined_serialdev(NULL)
// {
//     _currentMeasurement.distance = 0;
//     _currentMeasurement.angle = 0;
//     _currentMeasurement.quality = 0;
//     _currentMeasurement.startBit = 0;
// }


// RPLidar::~RPLidar()
// {
//     end();
// }

// open the given serial interface and try to connect to the RPLIDAR
// bool RPLidar::begin(HardwareSerial &serialobj)
// {
//     if (isOpen()) {
//       end(); 
//     }
//     _bined_serialdev = &serialobj;
//     _bined_serialdev->end();
//     _bined_serialdev->begin(RPLIDAR_SERIAL_BAUDRATE); // setup UART
// }

// close the currently opened serial interface
// void RPLidar::end()
// {
//     if (isOpen()) {
//        _bined_serialdev->end();
//        _bined_serialdev = NULL;
//     }
// }


// check whether the serial interface is opened
// bool RPLidar::isOpen()
// {
//     return _bined_serialdev?true:false; 
// }

// ask the RPLIDAR for its health info
// u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
// {
//     _u32 currentTs = millis();
//     _u32 remainingtime;
  
//     _u8 *infobuf = (_u8 *)&healthinfo;
//     _u8 recvPos = 0;

//     rplidar_ans_header_t response_header;
//     u_result  ans;


//     if (!isOpen()) return RESULT_OPERATION_FAIL;

//     {
//         if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
//             return ans;
//         }

//         if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
//             return ans;
//         }

//         // verify whether we got a correct header
//         if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
//             return RESULT_INVALID_DATA;
//         }

//         if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
//             return RESULT_INVALID_DATA;
//         }
        
//         while ((remainingtime=millis() - currentTs) <= timeout) {
//             int currentbyte = _bined_serialdev->read();
//             if (currentbyte < 0) continue;
            
//             infobuf[recvPos++] = currentbyte;

//             if (recvPos == sizeof(rplidar_response_device_health_t)) {
//                 return RESULT_OK;
//             
//         }
//     }
//     return RESULT_OPERATION_TIMEOUT;
// }

// ask the RPLIDAR for its device info like the serial number
// u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout )
// {
//     _u8  recvPos = 0;
//     _u32 currentTs = millis();
//     _u32 remainingtime;
//     _u8 *infobuf = (_u8*)&info;
//     rplidar_ans_header_t response_header;
//     u_result  ans;

//     if (!isOpen()) return RESULT_OPERATION_FAIL;

//     {
//         if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
//             return ans;
//         }

//         if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
//             return ans;
//         }

//         // verify whether we got a correct header
//         if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
//             return RESULT_INVALID_DATA;
//         }

//         if (response_header.size < sizeof(rplidar_response_device_info_t)) {
//             return RESULT_INVALID_DATA;
//         }

//         while ((remainingtime=millis() - currentTs) <= timeout) {
//             int currentbyte = _bined_serialdev->read();
//             if (currentbyte<0) continue;    
//             infobuf[recvPos++] = currentbyte;

//             if (recvPos == sizeof(rplidar_response_device_info_t)) {
//                 return RESULT_OK;
//             }
//         }
//     }
    
//     return RESULT_OPERATION_TIMEOUT;
// }

// stop the measurement operation
u_result stop()
{
    u_result ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

// start the measurement operation
// SCAN MODE: Legacy mode
u_result startScan(bool force, _u32 timeout)
{
    u_result ans;
    
    stop(); //force the previous operation to stop

    {
        ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
        // force scan = do not check or rotational speed: if we don't force and the motor is not rotating, lidar won't send back data samples back

        if (IS_FAIL(ans)) return ans;

        // Wait to see if LiDAR accepted request packet
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // Verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }
    }
    return RESULT_OK;
}
#define NUMPOINTS 400
#define SIZEREAD 5 * NUMPOINTS
// wait for one sample point to arrive
u_result grabData(_u32 timeout, uint8_t * _currentMeasurement) {
    // rplidar_response_measurement_node_t node[400];
    int i = 0;

    int recvPos = 0;

    // _u8 *nodebuf = (_u8*)(_currentMeasurement);

    uint8_t currentbyte[SIZEREAD]; // sizeof(rplidar_response_measurement_node_t)


    int size = uart_read_bytes(UART_NUM_1, _currentMeasurement, SIZEREAD, timeout * 1000 / portTICK_RATE_MS);
    if (size != SIZEREAD) {
        return RESULT_OPERATION_TIMEOUT;
    } else {
        for (i = 0; i < NUMPOINTS; i++) {
            switch (recvPos % 5) {
                case 0: // expect the sync bit and its reverse in this byte          {
                    {
                        _u8 tmp = (_currentMeasurement[recvPos]>>1);
                        if ( (tmp ^ _currentMeasurement[recvPos]) & 0x1 ) {   // just checking if last 2 bits are opposite
                            // pass, we are goood
                        } else {
                            return RESULT_INVALID_DATA;;
                        }

                    }
                    break;
                case 1: // expect the highest bit to be 1
                    {
                        if (_currentMeasurement[recvPos] & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) { // check sum should be 1
                            // pass, we are good
                        } else {
                            recvPos = 0;
                            return RESULT_INVALID_DATA;;
                        }
                    }
                    break;
                recvPos++;
            }
        }
    }
    return RESULT_OK;
}


u_result waitPoint(_u32 timeout, RPLidarMeasurement * _currentMeasurement)
{
//    _u32 currentTs = millis();
    int64_t  currentTs = esp_timer_get_time();
    int64_t remainingtime;
    rplidar_response_measurement_node_t node;
    _u8 *nodebuf = (_u8*)&node;

    _u8 recvPos = 0;

    uint8_t currentbyte[5]; // sizeof(rplidar_response_measurement_node_t)

    while ((remainingtime=esp_timer_get_time() - currentTs) <= timeout * 1000) {
        // int currentbyte = _bined_serialdev->read(); //uart read (one byte)
        
        int size = uart_read_bytes(UART_NUM_1, &currentbyte, 5, 1000 / portTICK_RATE_MS);
        if (size != 5) {
            continue;
        } else {
            while (recvPos < 5) {
                switch (recvPos) {
                    case 0: // expect the sync bit and its reverse in this byte          {
                        {
                            _u8 tmp = (currentbyte[recvPos]>>1);
                            if ( (tmp ^ currentbyte[recvPos]) & 0x1 ) {   // just checking if last 2 bits are opposite
                                // pass, we are goood
                            } else {
                                continue;
                            }

                        }
                        break;
                    case 1: // expect the highest bit to be 1
                        {
                            if (currentbyte[recvPos] & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) { // check sum should be 1
                                // pass, we are good
                            } else {
                                recvPos = 0;
                                continue;
                            }
                        }
                        break;
                }
                nodebuf[recvPos] = currentbyte[recvPos];
                recvPos++;
            }
        }

        if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
            // store the data ... // at global object
            _currentMeasurement->distance = node.distance_q2/4.0f;
            _currentMeasurement->angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
            _currentMeasurement->quality = (node.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            _currentMeasurement->startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
            return RESULT_OK;
        }
        
   }

   return RESULT_OPERATION_TIMEOUT;
}


// Sends request packet
u_result _sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    // Set start byte and command for request packet
    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // Send request packet via UART
    // int8_t * tmp = (int8_t *) header;
    // _bined_serialdev->write( (uint8_t *)header, 2); // modify this with sendData from uart_async_rxtxtasks
    uart_write_bytes(UART_NUM_1, (const char *) header, 2);

    // If command includes payload information
    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        // _bined_serialdev->write((uint8_t *)&sizebyte, 1);
        uart_write_bytes(UART_NUM_1, (const char *) &sizebyte, 1);

        // send payload
        // _bined_serialdev->write((uint8_t *)&payload, sizebyte);
        uart_write_bytes(UART_NUM_1, (const char *) &payload, sizebyte);

        // send checksum
        // _bined_serialdev->write((uint8_t *)&checksum, 1);
        uart_write_bytes(UART_NUM_1, (const char *) &checksum, 1);

    }

    return RESULT_OK;
}

// Reads response descriptor packet from left to right (Start Flag1 --> Data Type)
u_result _waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    // _u32 currentTs = millis();
    int64_t  currentTs = esp_timer_get_time();
    int64_t remainingtime;
    // _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;


    while ((remainingtime=esp_timer_get_time() - currentTs) <= timeout * 1000) {
        
        // Reads single byte from current position in resp descr packet (uart)
        // int currentbyte = _bined_serialdev->read();
        uint8_t currentbyte;
        int size = uart_read_bytes(UART_NUM_1, &currentbyte, 1, 1000 / portTICK_PERIOD_MS);
        if (size != 1) continue;
        // Read 2 start flags
        switch (recvPos) {
            case 0: 
                if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                    continue;
                }
                break;
            case 1:
                if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
        }
        // Initializes header struct with stuff it reads from resp descr packet
        // For the data length & data type are merged (data length = first 3 bytes and the 6 rightmost bits of the 4th byte, data type = 2 leftmost bits of 4th byte)
        // Big endian (first byte that is read is the lowest)
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
  

    }

    return RESULT_OPERATION_TIMEOUT;
}

