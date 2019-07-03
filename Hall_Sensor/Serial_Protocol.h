/*************************************************************
 *
 * $Workfile: serial_Protocol.h $
 *
 * $Description: implementation of arduino-robust-serial library $
 *               $https://github.com/araffin/arduino-robust-serial$
 *************************************************************/

#include <Arduino.h>
#define SERIAL_BAUD 115200  // Baudrate


#ifndef Serial_Protocol_H_
#define Serial_Protocol_H_

class Serial_Protocol {
	protected:

	private:

	public:

    // Define the orders that can be sent and received
    enum Order {
    CALIBRATION = 49,
    START = 48,
    DATA = 50,
    
    };

    void begin();
    Order read_order();
    int16_t package_Pressure(float pressure);
    int16_t package_Current(float current);
    void wait_for_bytes(int num_bytes, unsigned long timeout);
    void read_signed_bytes(int8_t* buffer, size_t n);
    int8_t read_i8();
    int16_t read_i16();
    int32_t read_i32();
    void write_order(Order myOrder);
    void write_i8(int8_t num);
    void write_i16(int16_t num);
    void write_i32(int32_t num);
};
			 
#endif
/*****************************************************************************
 *
 * End of File: serial_Protocol.h
 *
 ****************************************************************************/
