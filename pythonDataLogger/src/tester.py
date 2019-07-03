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
import serial
import serial.tools.list_ports
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

# Create an empty dictionary, then it is fill with csv paths for each arduino connected
outputCSV = {}
point = {}
timeDelay = 0
pumsPerArduino = 14

#List all available serial ports in pc
arduinoPorts = serial.tools.list_ports.comports(include_links=False)  
baudRate = 115200
csvHeaders =[]


# Dictionary from all orders
Orders = {
	'CALIBRATION' : 49,
	'START' : 48,
	'DATA' : 50,
	'STOP' : 34,
}
"""
	Functions definition

"""
PWMn = 0
PUMPn = 0
idcho = 0

def ConvertData(_dutty, _pressure, _current):
	pressure = (_pressure-500)/100.0
	current = _current/1000.0
	return [pressure, _dutty,current]

def appeandNewRow(data, outputPath):
	# Read CSV file from folder csv
	csvData = pandas.read_csv(outputPath, parse_dates=['Time'], delimiter = ',')
	# Create a Dictionary from headers and data collected
	tempoDict = {}
	for i in range(len(csvHeaders)):
		tempoDict[csvHeaders[i]] = data[i]
	# Add new row to CSV object
	modDfObj = csvData.append(tempoDict, ignore_index=True)
	# This functions makes that pression have 2 point decimal and current 3
	modDfObj= modDfObj.round(point)
	# Export CSV file
	modDfObj.to_csv(outputPath, date_format="%Y-%m-%d %H:%M:%S", sep=',', index=False)

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
	index_ids = 0
	# Send a command to verify if it is a compatible Arduino
	for ser in serial_file:
		write_i8(ser, Orders.get("HELLO"))
		try:
			dataIn = read_i8(ser)
			if dataIn>0 and dataIn<50:
				# It is a compatible Arduino
				outputCSV[str(dataIn)] = "../csv/Arduino{}_{}.csv".format(dataIn,datetime.datetime.now().strftime("%Y%m%d_%H%M%S") )
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

	goClose= False
	if not len(serial_file)>0:
		print ("There are not compatible Arduinos connected !!")
	else:
		while goClose == False:
			print("READY!")
			while True:
				print("===============================================================")
				print("Arduinos Availables: ", IDS)
				user_input = input("Choose Arduino: ")
				try:
					index_ids=IDS.index(str(user_input))
					try:
						if "." in user_input:
							idcho = float(user_input)
							idcho = int(idcho)
						else: 
							idcho= int(user_input)
						break				
					except ValueError:
						print("Input string is not an Integer.")
				except ValueError:
					print("Invalid ID.")
			
			print("===============================================================")
			print("0: Normal Operation")
			print("1: Calibration")
			print("2: Quit")
			print("===============================================================")
			user_input = input("Enter mode number and then enter: ")
			try:
				mode= int(user_input)
				if mode == 2:
					print("Quit")
					goClose = True
				elif mode == 0:
					write_i8(serial_file[index_ids],Orders.get("START"))
					print("Data sent succesfully!")
				elif mode == 1:
					write_i8(serial_file[index_ids],Orders.get("CALIBRATION"))
					print("Data sent succesfully!")
				else:
					print("No valid mode")
					goClose = True
			except ValueError:
				print("Invalid Value.")
		

			print("Press KeyboardInterrupt Ctl+c to quit")
			# Starts main loop for secuence of requests and appeand new rows to csv files
			while goClose == False:
				try:
					
					# Create a sequence for request data from all arduinos
					for i in range(0,len(idsList)):
						data = []
						data.append(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
						for j in range(1,pumsPerArduino+1):
							write_i8(serial_file[i], Orders.get("PUMP{}_DATA".format(j)))
							time.sleep(0.05)
							dutty_cycle = read_i8(serial_file[i])
							pressure = read_i16(serial_file[i])
							current = read_i16(serial_file[i])
							tempoData = ConvertData(dutty_cycle, pressure, current)
							data.append(tempoData[0])
							data.append(tempoData[1])
							data.append(tempoData[2])
						# Appeand to csv file and export it
						appeandNewRow(data, outputCSV[idsList[i]])
					# Wait until the time has elapsed for request next reading
					#time.sleep(timeDelay)     #in seconds
					time.sleep(timeDelay*60)	#in minutes

				except KeyboardInterrupt:
					break		
"""
	Main Program

"""

if __name__ == '__main__':
	sys.exit(main())