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

arduinoPorts = ["COM9"]

csvHeaderTime = "Time"
csvHeaderRPM = "Speed [rpm]"
inputCSV = "../csv/inDatalogger.csv"
timeDelay = 1

class Order(Enum):
	CALIBRATION = 49
	START = 48
	DATA = 50

"""
	Functions definition

"""

def appeandNewRow(data):
	csvData = pandas.read_csv(inputCSV, delimiter = ',')
	serie =pandas.Series([data[csvHeaderTime], 
					data[csvHeaderRPM]])
	# List of series 

	modDfObj = csvData.append(data, ignore_index=True)
	#csvModified = csvData.append(serie , ignore_index=True)
	modDfObj.to_csv(inputCSV, sep=',', index=False)

"""
	Main Function

"""
def main():
	for arduino_port in arduinoPorts:
		try:
			serial_file = open_serial_port(serial_port=arduino_port ,baudrate=115200, timeout=10)
			print("Arduino found")
		except Exception as e:
			print("Arduino no found")
			raise e
		pass


	# Wait 3 seconds until Arduino is ready because Arduino resets after serial USB connection
	time.sleep(3)
	#Ensure that file exists
	if not os.path.exists(inputCSV):
		Measures = {csvHeaderTime: [],
						csvHeaderRPM: []
						}
		df = pandas.DataFrame(Measures, columns= [csvHeaderTime, csvHeaderRPM])
		df.to_csv(inputCSV, sep=',', index=False)
		pass
		
	continuos=False	
	try:
		while True:
			print("===============================================================")
			print("	TEST 1")
			print("0: Normal Operation, Continuos data reading and save at csv/inDatalogger.csv")
			print("1: Calibration")
			print("2: Quit")
			print("===============================================================")
			user_input = input("Enter mode number and then enter: ")
			try:
				mode= int(user_input)
				if mode == 2:
					break
				elif mode == 0:
					write_order(serial_file,Order.START)
					continuos=True
					break
				elif mode == 1:
					write_order(serial_file,Order.CALIBRATION)
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
		print("Arduino data are been collected at csv/inDatalogger.csv")
		print("Press KeyboardInterrupt Ctl+c for quit")
		while continuos==True:
			write_order(serial_file, Order.DATA)
			speed = read_i16(serial_file)			
			data = {csvHeaderTime: datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
							csvHeaderRPM: speed
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