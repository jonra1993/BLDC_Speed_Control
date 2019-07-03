#!/usr/bin/env python
"""
	Import lybraries

"""
from __future__ import print_function
import os
import os.path
import datetime
import sys
import argparse
import time
import serial
import serial.tools.list_ports
import struct
import numpy
from enum import Enum
from threading import Thread
import math
#Serial library
from robust_serial import write_order, write_i8, write_i16, read_i8, read_i16
from robust_serial.utils import open_serial_port
#csv libraries
import pandas

#List all available serial ports in pc
arduinoPorts = serial.tools.list_ports.comports(include_links=False)  
baudRate = 115200

csvHeaderTime = "Time"
csvHeaderRPM = "Speed [rpm]"
csvHeaderSetpoint = "Setpoint [rpm]"
csvHeaderControl = "Dutty cycle [ms]"
inputCSV = "../csv/Data_{}.csv".format(datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
timeDelay = 0.1

class Order(Enum):
	CALIBRATION = 49
	START = 48
	DATA = 50
	HELLO = 1

"""
	Functions definition

"""
def appeandNewRow(data):
	csvData = pandas.read_csv(inputCSV, delimiter = ',')
	serie =pandas.Series([data[csvHeaderTime], 
					data[csvHeaderRPM],
					data[csvHeaderSetpoint],
					data[csvHeaderControl]])
	# List of series 
	modDfObj = csvData.append(data, ignore_index=True)
	#csvModified = csvData.append(serie , ignore_index=True)
	modDfObj.to_csv(inputCSV, sep=',', index=False)

def createCSV():
	Measures = {csvHeaderTime: [],
				csvHeaderRPM: [],
				csvHeaderSetpoint: [],
				csvHeaderControl: []
				}
	df = pandas.DataFrame(Measures, columns= [csvHeaderTime, csvHeaderRPM, csvHeaderSetpoint, csvHeaderControl])
	df.to_csv(inputCSV, sep=',', index=False)

"""
	Main Function

"""
# list for saving serial ports of each Arduino
serial_file = []
def main():
	for arduino_port in arduinoPorts:
		try:
			ser = open_serial_port(serial_port=arduino_port.device ,baudrate=baudRate, timeout=1)
			serial_file.append(ser)
			
		except (OSError, serial.SerialException):
			pass
	# Wait 3 seconds until Arduino is ready because Arduino resets after serial USB connection
	time.sleep(3)
	# create a list of all no compatible Arduinos and no Arduinos
	serDel = []
	IDS = []
	# Send a command to verify if it is a compatible Arduino
	for ser in serial_file:
		write_order(ser, Order.HELLO)
		try:
			dataIn = read_i8(ser)
			if dataIn>0 and dataIn<10:
				# It is a compatible Arduino
				print("Compatible Arduino ID: "+str(dataIn)+" -> "+str(ser.port))	
				IDS.append(str(dataIn))
			else:
				# No return a valid Data so it is not a compatible arduino	
				ser.close()
				serDel.append(ser)	
				print("No compatible Arduino -> "+str(ser.port))
		except (struct.error, OverflowError):
    #There is no new data from slave, so it is not an Arduino
			ser.close()
			serDel.append(ser)
			#print("No Arduino -> "+str(ser.port))
			pass
	# Delete all no compatibe Arduinos and no Arduinos from serial_file
	for delete in serDel:
		serial_file.remove(delete)

	if not len(serial_file)>0:
		print ("There are not compatible Arduinos connected !!")
	else:
		while True:
			print("===============================================================")
			print("Arduinos Available: ", IDS)
			user_input = input("Choose Arduino ID: ")
			try:
				index_ids=IDS.index(str(user_input))					
				break				
			except ValueError:
				print("Invalid ID.")
		
		continuos=False	
		try:
			while True:
				print("===============================================================")
				print("	SPEED TEST")
				print("Caution: If you have already calibrated ESC and want another calibration, first desconnect ESC  from power source")
				print("0: Normal Operation, if you calibrated before")
				print("1: Calibration")
				print("2: Quit")
				print("===============================================================")
				user_input = input("Enter mode number and then enter: ")
				try:
					mode= int(user_input)
					if mode == 2:
						break
					elif mode == 0:
						write_order(serial_file[index_ids],Order.START)
						createCSV()
						time.sleep(5)
						continuos=True
						break
					elif mode == 1:
						write_order(serial_file[index_ids],Order.CALIBRATION)
						createCSV()
						print("Now connect the ESC to power and wait some seconds")
						time.sleep(12)
						continuos=True
						break
					else:
						print("No valid mode")
						continue
				except ValueError:
					print("Invalid Value.")
			pass
		except KeyboardInterrupt:
			pass

		try:
			print("Arduino data are been collected at "+inputCSV)
			print("Press KeyboardInterrupt Ctl+c for quit")
			while continuos==True:
				write_order(serial_file[index_ids], Order.DATA)
				speed = read_i16(serial_file[index_ids])
				setpoint = read_i16(serial_file[index_ids])	
				control = read_i16(serial_file[index_ids])		
				data = {csvHeaderTime: datetime.datetime.now(),
								csvHeaderRPM: speed,
								csvHeaderSetpoint: setpoint,
								csvHeaderControl: control
								}
				appeandNewRow(data)
				time.sleep(timeDelay)
			pass
		except KeyboardInterrupt:
			pass

"""
	Main Program

"""

if __name__ == '__main__':
	sys.exit(main())