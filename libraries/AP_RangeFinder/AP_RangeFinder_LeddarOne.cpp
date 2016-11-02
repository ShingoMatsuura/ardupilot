/*
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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_LeddarOne.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

/*
   The based on Arduino LeddarOne sample
   --------------------------------------------------------
       http://playground.arduino.cc/Code/Leddar
   --------------------------------------------------------
 */

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LeddarOne::AP_RangeFinder_LeddarOne(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/*
   detect if a LeddarOne rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LeddarOne::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LeddarOne::get_reading(uint16_t &reading_cm)
{
	uint8_t number_detections;
	LeddarOne_Status states;

uint32_t debug_ms = AP_HAL::millis();

    if (uart == nullptr) {
        return false;
    }

	switch (modbus_status) {

		case LEDDARONE_MODBUS_PRE_SEND_REQUEST:
			// clear buffer and buffer_len
			memset(read_buffer, 0, sizeof(read_buffer));
			read_len = 0;

			// send a request message for Modbus function 4
			if (send_request() != LEDDARONE_OK) {
				// TODO: handle LEDDARONE_ERR_SERIAL_PORT
				break;
			}
			modbus_status = LEDDARONE_MODBUS_SENT_REQUEST;
			last_sending_request_ms = AP_HAL::millis();
			break;

		case LEDDARONE_MODBUS_SENT_REQUEST:
			if (uart->available()) {
				modbus_status = LEDDARONE_MODBUS_AVAILABLE;
			} else {
				if (AP_HAL::millis() - last_sending_request_ms > 200) {
					modbus_status = LEDDARONE_MODBUS_PRE_SEND_REQUEST;
				}
			}
			break;

		case LEDDARONE_MODBUS_AVAILABLE:
			// parse a response message, set number_detections, detections and sum_distance
			states = parse_response(number_detections);

			if (states == LEDDARONE_OK) {
				reading_cm = sum_distance / number_detections;

				// reset mod_bus status to read new buffer
				modbus_status = LEDDARONE_MODBUS_PRE_SEND_REQUEST;

				gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "Leddar: %ucm, %u ms", reading_cm, AP_HAL::millis() - debug_ms);
				return true;
			}
			// keep reading next buffer
			else if (states == LEDDARONE_READING_BUFFER) {
				break;
			}
			// reset mod_bus status to read new buffer
			else {
				modbus_status = LEDDARONE_MODBUS_PRE_SEND_REQUEST;
			}

			break;
	}

gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "Leddar: get_reading %u ms", AP_HAL::millis() - debug_ms);
	return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LeddarOne::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

/*
   CRC16
   CRC-16-IBM(x16+x15+x2+1)
*/
bool AP_RangeFinder_LeddarOne::CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i=0; i<aLength; i++) {
        crc ^= aBuffer[i];
        for (uint32_t j=0; j<8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    uint8_t lCRCLo = LOWBYTE(crc);
    uint8_t lCRCHi = HIGHBYTE(crc);

    if (aCheck) {
        return (aBuffer[aLength] == lCRCLo) && (aBuffer[aLength+1] == lCRCHi);
    } else {
        aBuffer[aLength] = lCRCLo;
        aBuffer[aLength+1] = lCRCHi;
        return true;
    }
}

/*
   send a request message to execute ModBus function 0x04
 */
LeddarOne_Status AP_RangeFinder_LeddarOne::send_request(void)
{
    uint8_t send_buffer[10] = {0};
    uint8_t i = 0;

    uint32_t nbytes = uart->available();

    // clear buffer
    while (nbytes-- > 0) {
        uart->read();
        if (++i > 250) {
            return LEDDARONE_ERR_SERIAL_PORT;
        }
    }

    // Modbus read input register (function code 0x04)
    // send_buffer[3] = 20: Address of first register to read
    // send_buffer[5] = 10: The number of consecutive registers to read
    send_buffer[0] = LEDDARONE_DEFAULT_ADDRESS;
    send_buffer[1] = 0x04;
    send_buffer[2] = 0;
    send_buffer[3] = 20;
    send_buffer[4] = 0;
    send_buffer[5] = 10;

    // CRC16
    CRC16(send_buffer, 6, false);

    // write buffer data with CRC16 bits
    for (i=0; i<8; i++) {
        uart->write(send_buffer[i]);
    }
    uart->flush();

    return LEDDARONE_OK;
}

 /*
    parse a response message from Modbus
  */
LeddarOne_Status AP_RangeFinder_LeddarOne::parse_response(uint8_t &number_detections)
{
    uint8_t i;
    uint8_t index_offset = LEDDARONE_DATA_INDEX_OFFSET;

    // read serial
	uint32_t nbytes = uart->available();

	if (nbytes != 0)  {
		for (i=read_len; i<nbytes+read_len; i++) {
			if (i >= 25) {
				return LEDDARONE_ERR_BAD_RESPONSE;
			}
			read_buffer[i] = uart->read();
		}

		read_len += nbytes;

		if (read_len < 25) {
			return LEDDARONE_READING_BUFFER;
		}
	}

    if (read_len != 25 || read_buffer[1] != 0x04) {
    	return LEDDARONE_ERR_BAD_RESPONSE;
    }

    // CRC16
    if (!CRC16(read_buffer, read_len-2, true)) {
        return LEDDARONE_ERR_BAD_CRC;
    }

    // number of detections
    number_detections = read_buffer[10];

    // if the number of detection is over or zero , it is false
    if (number_detections > LEDDARONE_DETECTIONS_MAX || number_detections == 0) {
        return LEDDARONE_ERR_NUMBER_DETECTIONS;
    }

    memset(detections, 0, sizeof(detections));
    sum_distance = 0;
    for (i=0; i<number_detections; i++) {
        // construct data word from two bytes and convert mm to cm
        detections[i] =  (static_cast<uint16_t>(read_buffer[index_offset])*256 + read_buffer[index_offset+1]) / 10;
        sum_distance += detections[i];
        index_offset += 4;
    }

    return LEDDARONE_OK;
}

void AP_RangeFinder_LeddarOne::gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...)
{
    char str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)str, sizeof(str), fmt, arg_list);
    va_end(arg_list);
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}
