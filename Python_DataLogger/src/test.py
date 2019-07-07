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
import matplotlib.pyplot as plt
from enum import Enum
from threading import Thread
import math
#Serial library
from robust_serial import write_order, write_i8, write_i16, read_i8, read_i16
from robust_serial.utils import open_serial_port
#csv libraries
import pandas
# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')
ymax = 2600
#List all available serial ports in pc
arduinoPorts = serial.tools.list_ports.comports(include_links=False)  
baudRate = 115200

csvHeaderIndex = "Index"
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
	STOP = 2

"""
	Functions definition

"""
def live_plotter(x_vec,y1_data,line,identifier='',pause_time=0.1):
	if line==[]:
		# this is the call to matplotlib that allows dynamic plotting
		plt.ion()
		fig = plt.figure(figsize=(10,5))
		ax = fig.add_subplot(111)
		# create a variable for the line so we can later update it
		line, = ax.plot(x_vec,y1_data,'-',alpha=0.8)        
		#update plot label/title
		plt.ylabel('Y Label')
		plt.title('Real Time: {}'.format(identifier))
		plt.show()

	# after the figure, axis, and line are created, we only need to update the y-data
	line.set_data(x_vec,y1_data)
	plt.axis((0,50+len(x_vec),0,ymax))
	plt.yticks(list(range(0,ymax,200)))
	# this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
	plt.pause(pause_time)
	# return line so we can update it again in the next iteration
	return line

def appeandNewRow(data, line):
	csvData = pandas.read_csv(inputCSV, delimiter = ',')
	serie =pandas.Series([data[csvHeaderIndex],
					data[csvHeaderTime], 
					data[csvHeaderRPM],
					data[csvHeaderSetpoint],
					data[csvHeaderControl]])
	# List of series 
	modDfObj = csvData.append(data, ignore_index=True)
	#csvModified = csvData.append(serie , ignore_index=True)
	modDfObj.to_csv(inputCSV, sep=',', index=False)
	return live_plotter(list(modDfObj[csvHeaderIndex]),list(modDfObj[csvHeaderRPM]),line, "RPM", timeDelay)


def createCSV():
	Measures = {csvHeaderIndex:[],
				csvHeaderTime: [],
				csvHeaderRPM: [],
				csvHeaderSetpoint: [],
				csvHeaderControl: []
				}
	df = pandas.DataFrame(Measures, columns= [csvHeaderIndex, csvHeaderTime, csvHeaderRPM, csvHeaderSetpoint, csvHeaderControl])
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
			count=0
			line1 = []
			while continuos==True:
				write_order(serial_file[index_ids], Order.DATA)
				speed = read_i16(serial_file[index_ids])
				setpoint = read_i16(serial_file[index_ids])	
				control = read_i16(serial_file[index_ids])		
				data = {csvHeaderIndex: count,
								csvHeaderTime: datetime.datetime.now(),
								csvHeaderRPM: speed,
								csvHeaderSetpoint: setpoint,
								csvHeaderControl: control
								}
				line1 = appeandNewRow(data, line1)
				count=count+1
			pass
		except KeyboardInterrupt:
			pass
		write_order(serial_file[index_ids],Order.STOP)
		serial_file[index_ids].close()

"""
	Main Program

"""

if __name__ == '__main__':
	sys.exit(main())