#!/usr/bin/python

from __future__ import print_function, division
from math import sin, cos, radians
import WalabotAPI
import serial
import sys
import time

MIN_AMPLITUDE = 0.005
WALABOT_OBJECT_DIRECTION_RANGE = 18
WALABOT_OBJECT_DISTANCE_RANGE = 40
WALABOT_OBJECT_DISTANCE = 80

try:
	SERIAL_PORT = sys.argv[1]
	SERIAL_BAUD = int(sys.argv[2])
except Exception:
	raise ValueError("Missing or incorrect command line arguments: (1.) serial port and (2.) baud rate")
	
class BeerTankApp:

	def __init__(self):
		self.wlbt = Walabot()  
		
		rMin = 10.0
		rMax = 200.0
		rRes = 2.0
		tMax = 20.0
		tRes = 10.0
		pMax = 45.0
		pRes = 2.0
		self.thld = 15.0
		self.mti = False
		
		self.rParams = (rMin, rMax, rRes)
		self.tParams = (-tMax, tMax, tRes)
		self.pParams = (-pMax, pMax, pRes)
		
	def run(self):
	
		try:
			print("Connecting to the MCU of the tank. port: " + SERIAL_PORT + ", baud rate: " + str(SERIAL_BAUD))
			self.mcu = SerialController(SERIAL_PORT, SERIAL_BAUD)
			print("MCU found")
			print("Identificating MCU")
			self.mcu.writeSerialData('s')
			time.sleep(1)
			response = self.mcu.readSerialData()
			
			print(response)
			if response != 'OK':
				print("Identification failed")
				return
			else:
				print("Identification successful")
				
		except serial.SerialException:
			print("Can't connect to the tank's MCU")
			return 

			
		
		if self.wlbt.isConnected():  # connection achieved
			print ("Connected to Walabot")
			try:
				self.wlbt.setParameters(self.rParams, self.tParams, self.pParams, self.thld, self.mti)
			except WalabotAPI.WalabotError as err:
				print(str(err))
				return
			#self.wlbt.calibrate()
			params = self.wlbt.getParameters()
			print("Status: " + self.wlbt.getStatusString())
			self.loop()
		else:
			print("Walabot is NOT connected")
	
	def loop(self):
		""" Triggers the Walabot, get the Sensor targets, and update the
			canvas and other components accordingly.
		"""
		while True:
			try:
				targets = self.wlbt.getTargets()
			except WalabotAPI.WalabotError as err:
				print(str(err))
				self.stopLoop()
				return

			if len(targets) > 0 and MIN_AMPLITUDE < targets[0].amplitude:
				target = targets[0]
				print(target)
				if abs(target.yPosCm) < WALABOT_OBJECT_DIRECTION_RANGE:
					print("irány ok")
					motorSpeedChange = 0;
				elif target.yPosCm > 0:
					motorSpeedChange = int(target.yPosCm - WALABOT_OBJECT_DIRECTION_RANGE) * 2
				else:
					motorSpeedChange = int(target.yPosCm + WALABOT_OBJECT_DIRECTION_RANGE) * 2

				if abs(target.zPosCm - WALABOT_OBJECT_DISTANCE) < WALABOT_OBJECT_DISTANCE_RANGE:
					print("távolság ok")
					motorSpeedBase = 0;
				elif target.zPosCm - WALABOT_OBJECT_DISTANCE < 0:
					motorSpeedBase = int((target.zPosCm - WALABOT_OBJECT_DISTANCE + WALABOT_OBJECT_DISTANCE_RANGE) * 5)
				else:
					motorSpeedBase = int((target.zPosCm - WALABOT_OBJECT_DISTANCE - WALABOT_OBJECT_DISTANCE_RANGE) * 5)
					
				print("speed " + str(motorSpeedBase))
				print("turn " + str(motorSpeedChange))
				self.mcu.writeSerialData('m ' + str(motorSpeedBase) + " " + str(motorSpeedChange))
		
	def stopLoop(self):
		""" Kills the loop function and reset the relevant app components.
		"""
		self.after_cancel(self.cyclesId)
		self.wlbt.stopAndDisconnect()
		print(self.wlbt.getStatusString())
		
			
class Walabot:
	""" Control the Walabot using the Walabot API.
	"""

	def __init__(self):
		""" Init the Walabot API.
		"""
		self.wlbt = WalabotAPI
		self.wlbt.Init()
		self.wlbt.SetSettingsFolder()
		
	def calibrate(self):
		""" Calibrates the Walabot.
		"""
		self.wlbt.StartCalibration()
		while self.wlbt.GetStatus()[0] == self.wlbt.STATUS_CALIBRATING:
			self.wlbt.Trigger()
			
	def isConnected(self):
		""" Try to connect the Walabot device. Return True/False accordingly.
		"""
		try:
			self.wlbt.ConnectAny()
		except self.wlbt.WalabotError as err:
			if err.code == 19:  # "WALABOT_INSTRUMENT_NOT_FOUND"
				return False
			else:
				raise err
		return True

	def getParameters(self):
		""" Get the arena parameters from the Walabot API.
		"""
		r = self.wlbt.GetArenaR()
		theta = self.wlbt.GetArenaTheta()
		phi = self.wlbt.GetArenaPhi()
		threshold = self.wlbt.GetThreshold()
		mti = self.wlbt.GetDynamicImageFilter()
		return r, theta, phi, threshold, mti

	def setParameters(self, r, theta, phi, threshold, mti):
		""" Set the arena Parameters according given ones.
		"""
		self.wlbt.SetProfile(self.wlbt.PROF_SENSOR)
		self.wlbt.SetArenaR(*r)
		self.wlbt.SetArenaTheta(*theta)
		self.wlbt.SetArenaPhi(*phi)
		self.wlbt.SetThreshold(threshold)
		self.wlbt.SetDynamicImageFilter(mti)
		self.wlbt.Start()

	def getStatusString(self):
		""" Return the Walabot status as a string.
		"""
		status = self.wlbt.GetStatus()[0]
		if status == 0:
			return "STATUS_DISCONNECTED"
		elif status == 1:
			return "STATUS_CONNECTED"
		elif status == 2:
			return "STATUS_IDLE"
		elif status == 3:
			return "STATUS_SCANNING"

	def getTargets(self):
		""" Trigger the Walabot, retrive the targets according to the desired
			tracker given.
		"""
		self.wlbt.Trigger()
		return self.wlbt.GetSensorTargets()

	def stopAndDisconnect(self):
		""" Stop and disconnect from the Walabot.
		"""
		self.wlbt.Stop()
		self.wlbt.Disconnect()
		
class SerialController:
	
	def __init__(self, port, baud):
		""" Initialize and open serial communication
		"""
		self.openSerial(port, baud)
		
	def getPortBaudRate(self):
		""" getter
		"""
		return self.port, self.baud
		
	def writeSerialData(self, data):
		self.serial.write((data + '\r\n').encode())
		
	def readSerialData(self):
		data = ''
		while self.serial.inWaiting() > 0:
			data += self.serial.read(1).decode("utf-8")
			
		return data.rstrip("\r").strip()
	
	def openSerial(self, port, baud):
		""" open serial communication
		"""
		self.port = port
		self.baud = baud
		self.serial = serial.Serial(
			port=port, 
			baudrate=baud,
			parity=serial.PARITY_ODD,
			stopbits=serial.STOPBITS_TWO,
			bytesize=serial.SEVENBITS
		)
	
	def closeSerial(self):
		""" close serial communication
		"""
		if(self.serial.isOpen()):
			self.serial.close()
		
if __name__ == "__main__":
	BeerTankApp().run()