/*************************************************************
 *
 * $Workfile: serial_Protocol.cpp $
 *
 * $Description: implementation of arduino-robust-serial library $
 *               $https://github.com/araffin/arduino-robust-serial$
 *************************************************************/


#include "Serial_Protocol.h"

/***************************************************************************
 *
 * Serial Protocol Functions
 *
 ***************************************************************************/

//! This function initializes the protocol variables
void Serial_Protocol::begin(){
  // Init Serial
  Serial.begin(SERIAL_BAUD);
}

//! Sends 8-bits as order commands
Serial_Protocol::Order Serial_Protocol::read_order()
{
	return (Serial_Protocol::Order) Serial.read();
}

//! Packages float pressure data into uin16_t
int16_t Serial_Protocol::package_Pressure(float pressure){
  int16_t pressureTempo = (int16_t)(pressure*100.0f) + 500;   //to ensure data is positive
	return pressureTempo;
}

//! Packages float current data into uin16_t
int16_t Serial_Protocol::package_Current(float current){
  int16_t currentTempo = (int16_t)(current*1000.0f);      //Multiplier and offset
	return currentTempo;
}

//! Waits for response data
void Serial_Protocol::wait_for_bytes(int num_bytes, unsigned long timeout)
{
	unsigned long startTime = millis();
	//Wait for incoming bytes or exit if timeout
	while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void Serial_Protocol::read_signed_bytes(int8_t* buffer, size_t n)
{
	size_t i = 0;
	int c;
	while (i < n)
	{
		c = Serial.read();
		if (c < 0) break;
		*buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
		i++;
	}
}

//! Reads a byte from buffer
int8_t Serial_Protocol::read_i8()
{
	wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

//! Reads a 2-bytes from buffer
int16_t Serial_Protocol::read_i16()
{
  int8_t buffer[2];
	wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
	read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

//! Reads a 4-bytes from buffer
int32_t Serial_Protocol::read_i32()
{
  int8_t buffer[4];
	wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
	read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

//! Writes a byte to buffer
void Serial_Protocol::write_order(Order myOrder)
{
	uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

//! Writes a byte to buffer
void Serial_Protocol::write_i8(int8_t num)
{
  Serial.write(num);
}

//! Writes a 2-bytes to buffer
void Serial_Protocol::write_i16(int16_t num)
{
	int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

//! Writes a 4-byte to buffer
void Serial_Protocol::write_i32(int32_t num)
{
	int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
