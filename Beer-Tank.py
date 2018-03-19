#!/usr/bin/python

from __future__ import print_function, division
from math import sin, cos, radians
import serial
import sys
import time
from os.path import join
from imp import load_source

if sys.platform == 'win32':
	modulePath = join('C:/', 'Program Files', 'Walabot', 'WalabotSDK',
		'python', 'WalabotAPI.py')
elif sys.platform.startswith('linux'):
	modulePath = join('/usr', 'share', 'walabot', 'python', 'WalabotAPI.py')	 

WalabotAPI = load_source('WalabotAPI', modulePath)
WalabotAPI.Init()

__author__ = "Balazs Simon"
__license__ = "GPL"
__version__ = "1.0.0"
__details__ = "https://www.hackster.io/Abysmal/beer-tank-20a2ed"

MIN_AMPLITUDE = 0.004
WALABOT_OBJECT_DIRECTION_RANGE = 18
WALABOT_OBJECT_DISTANCE_RANGE = 25
WALABOT_OBJECT_DISTANCE = 80

""" Command line arguments are used to get the correct serial port and baud rate
"""
try:
	SERIAL_PORT = sys.argv[1]
	SERIAL_BAUD = int(sys.argv[2])
except Exception:
	raise ValueError("Missing or incorrect command line arguments: (1.) serial port and (2.) baud rate")
	
class BeerTankApp:

	def __init__(self):
		self.wlbt = Walabot()  
		
		self.rMin = 10.0
		self.rMax = 150.0
		self.rRes = 2.0
		self.tMax = 20.0
		self.tRes = 10.0
		self.pMax = 50.0
		self.pRes = 2.0
		self.thld = 15.0
		self.mti = 2	# enable MTI
		
		self.rParams = (self.rMin, self.rMax, self.rRes)
		self.tParams = (-self.tMax, self.tMax, self.tRes)
		self.pParams = (-self.pMax, self.pMax, self.pRes)
		
	def run(self):
		""" Initializing the system. 
			Connecting to the Arduino --> Identifying it -> Connecting to the Walabot -> Run loop
		"""
		try:
			print("Connecting to the MCU of the tank. port: " + SERIAL_PORT + ", baud rate: " + str(SERIAL_BAUD))
			self.mcu = SerialController(SERIAL_PORT, SERIAL_BAUD)
			print("MCU found")
			""" The validation is quite simple. The Raspberry will send the letter 's' to the Arduino.
				If it answers with 'OK' then it should use the Beer-Tank.ino or other compatible sketch
			"""
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
			print("Status: " + self.wlbt.getStatusString())
			self.loop()
		else:
			print("Walabot is NOT connected")
	
	def loop(self):
		""" Giving moving commands to the Arduino based on the targets received from the Walabot
		"""
		while True:
			try:
				targets = self.wlbt.getTargets()
			except WalabotAPI.WalabotError as err:
				print(str(err))
				self.stopLoop()
				return

			if not targets:
				continue
			""" There might be multiple targets. Most likely the real target is the one that is the 
				closest to the goal (the human's desired position) and has a high enough amplitude
			"""
			goalZ = self.rMax / 2
			goalY = 0
			closest = targets[0]
			for t in targets:
				""" Calculating the distance of 't' and the goal
				"""
				if ((closest.yPosCm - goalY) ** 2 + (closest.zPosCm - goalZ) ** 2) ** 0.5 > ((t.yPosCm - goalY) ** 2 + (t.zPosCm - goalZ) ** 2) ** 0.5 and t.amplitude > MIN_AMPLITUDE:
					closest = t				

			""" Due to possible false targets and the slow update speed of the Walabot, I used fix 
				speeds for moving to avoid very fast movement. A minimum speed was also required to
				make the tank move.
			"""
			if 'closest' in locals() and MIN_AMPLITUDE < closest.amplitude:
				if closest.yPosCm > WALABOT_OBJECT_DIRECTION_RANGE:
					motorSpeedChange = 75
					motorSpeedBase = 0
				elif closest.yPosCm < -WALABOT_OBJECT_DIRECTION_RANGE:
					motorSpeedChange = -75
					motorSpeedBase = 0
				elif closest.zPosCm - WALABOT_OBJECT_DISTANCE > WALABOT_OBJECT_DISTANCE_RANGE:
					motorSpeedChange = 0
					motorSpeedBase = 50
				elif closest.zPosCm - WALABOT_OBJECT_DISTANCE < -WALABOT_OBJECT_DISTANCE_RANGE:
					motorSpeedChange = 0
					motorSpeedBase = -50
			else:
				motorSpeedChange = 0
				motorSpeedBase = 0
					
			print("y: " + str(closest.yPosCm) + " z: " + str(closest.zPosCm)  + " speed: " + str(motorSpeedBase) + " turn: " + str(motorSpeedChange) + " a: " + str(closest.amplitude))
			""" sending a movement command to the Arduino
			"""
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
		""" Getter for port and baud
		"""
		return self.port, self.baud
		
	def writeSerialData(self, data):
		""" Sending data to the Arduino through serial communication. Line ending is '\r\n'
		"""
		self.serial.write((data + '\r\n').encode())
		
	def readSerialData(self):
		""" Reading the data, received from the Arduino
		"""
		data = ''
		while self.serial.inWaiting() > 0:
			data += self.serial.read(1).decode("utf-8")
			
		return data.rstrip("\r").strip()
	
	def openSerial(self, port, baud):
		""" Open serial communication
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
	""" Starting the application
	"""
	BeerTankApp().run()