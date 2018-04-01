#!/usr/bin/python
 
from __future__ import print_function, division
from collections import deque
from os.path import join
from imp import load_source
import paho.mqtt.client as paho
import os
import serial
import sys
import time
import ssl
import json
 
__author__ = "Balazs Simon"
__license__ = "GPL"
__version__ = "1.0.0"
__details__ = "https://www.hackster.io/Abysmal/beer-tank-20a2ed"
 
if sys.platform == 'win32':
	modulePath = join('C:/', 'Program Files', 'Walabot', 'WalabotSDK',
		'python', 'WalabotAPI.py')
elif sys.platform.startswith('linux'):
	modulePath = join('/usr', 'share', 'walabot', 'python', 'WalabotAPI.py')	 
 
WalabotAPI = load_source('WalabotAPI', modulePath)
WalabotAPI.Init()
 
""" Movement and Walabot related settings
"""
MAX_MOVEMENT = 25;
OBJECT_DIRECTION_RANGE = 18
OBJECT_DISTANCE_RANGE = 25
OBJECT_DISTANCE = 80

""" AWS IoT related settings
""" 
awshost = "a16zyi8b5y4ebz.iot.us-west-2.amazonaws.com"
awsport = 8883
cert_path = os.path.realpath(__file__).rstrip(os.path.basename(__file__)) + "cert/"
caPath = cert_path + "root-CA.crt"
certPath = cert_path + "certificate.pem.crt"
keyPath = cert_path + "private.pem.key"

lastPosition = None
alexaCommands = deque()
 
""" Command line arguments are used to get the correct serial port and baud rate
"""
try:
	SERIAL_PORT = sys.argv[1]
	SERIAL_BAUD = int(sys.argv[2])
except Exception:
	raise ValueError("Missing or incorrect command line arguments: (1.) serial port and (2.) baud rate")
   
class BeerTankApp:
 
	def __init__(self, port, baud):
		self.wlbt = Walabot()  
	   
		self.port = port
		self.baud = baud
		self.rMin = 30.0
		self.rMax = 150.0
		self.rRes = 5.0
		self.tMax = 20.0
		self.tRes = 10.0
		self.pMax = 60.0
		self.pRes = 4.0
		self.thld = 15.0
		self.mti = 2	# enable MTI
		self.movingEnabled = True
	   
		self.rParams = (self.rMin, self.rMax, self.rRes)
		self.tParams = (-self.tMax, self.tMax, self.tRes)
		self.pParams = (-self.pMax, self.pMax, self.pRes)
	   
	def run(self):
		""" Initializing the system.
			Connecting to the Arduino --> Identifying it -> Connecting to the Walabot -> Run loop
		"""
		try:
			print("Connecting to the MCU of the tank. port: " + self.port + ", baud rate: " + str(self.baud))
			self.mcu = SerialController(self.port, self.baud)
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
		global alexaCommands
		global lastPosition
	   
		while True:
			while alexaCommands:
				command = alexaCommands.popleft()
				print(command)
				if command == "enable":
					self.movingEnabled = True
				elif command == "disable":
					self.movingEnabled = False
					lastPosition = None
				else:
					self.mcu.writeSerialData(command)
	   
			try:
				targets = self.wlbt.getTargets()
			except WalabotAPI.WalabotError as err:
				print(str(err))
				self.stopLoop()
				return
 
			if not targets:
				self.mcu.writeSerialData('m 0 0')
				continue
			""" There might be multiple targets. Most likely the real target is the one that is the
				closest to the goal (the human's desired position) and has a high enough amplitude
			"""
		   
			if lastPosition == None:
				lastPosition, _ = self.findClosestTarget(0, OBJECT_DISTANCE, targets)
				continue
		   
			human, distance = self.findClosestTarget(lastPosition.yPosCm, lastPosition.zPosCm, targets)
		   
			if distance > MAX_MOVEMENT: #any bigger movement than MAX_MOVEMENT is considered as false target or not human.
				print("human not found")
				motorSpeedChange = 0
				motorSpeedBase = 0
			else:
				""" Due to possible false targets and the slow update speed of the Walabot, I used slow, fix
					speeds for moving the tank to prevent it from doing crazy things. A minimum speed was also
					required to make the tank move.
				"""
				if human.yPosCm > OBJECT_DIRECTION_RANGE:
					motorSpeedChange = 85
					motorSpeedBase = 0
				elif human.yPosCm < -OBJECT_DIRECTION_RANGE:
					motorSpeedChange = -85
					motorSpeedBase = 0
				elif human.zPosCm - OBJECT_DISTANCE > OBJECT_DISTANCE_RANGE:
					motorSpeedChange = 0
					motorSpeedBase = 50
				elif human.zPosCm - OBJECT_DISTANCE < -OBJECT_DISTANCE_RANGE:
					motorSpeedChange = 0
					motorSpeedBase = -50
				else:
					motorSpeedChange = 0
					motorSpeedBase = 0
				   
				lastPosition = human
				print("y: " + str(human.yPosCm) + " z: " + str(human.zPosCm)  + " speed: " + str(motorSpeedBase) + " turn: " + str(motorSpeedChange) + " d: " + str(distance))
			   
			if self.movingEnabled:
				""" sending a movement command to the Arduino
				"""
				self.mcu.writeSerialData('m ' + str(motorSpeedBase) + " " + str(motorSpeedChange))
	   
	def stopLoop(self):
		""" Kills the loop function and reset the relevant app components.
		"""
		self.after_cancel(self.cyclesId)
		self.wlbt.stopAndDisconnect()
		print(self.wlbt.getStatusString())
	   
	def findClosestTarget(self, goalY, goalZ, targets):
		smallestDistance = float('inf')
		for t in targets:
			""" Calculating the distance of t and goal
			"""
			distance = ((t.yPosCm - goalY) ** 2 + (t.zPosCm - goalZ) ** 2) ** 0.5
			if distance < smallestDistance:
				closest = t
				smallestDistance = distance
		return closest, smallestDistance
	   
		   
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

class MQTTSub(object):
	""" This class is used to communicate with AWS IoT/AWS Lambda/Alexa
	"""
 
	def __init__(self, listener = False, topic = "default"):
		self.connect = False
		self.listener = listener
		self.topic = topic
 
	def __on_connect(self, client, userdata, flags, rc):
		self.connect = True
	   
		if self.listener:
			self.mqttc.subscribe(self.topic)
 
		print("{0}".format(rc))
 
	def __on_message(self, client, userdata, msg):
		""" This function will be called when the system receives a message on
			the topic that it is subscribed to
		"""
		global alexaCommands
	   
		data = json.loads(msg.payload)
		if data["state"]["desired"]["led"] == "on":
			alexaCommands.append('l')
			print("Turning lighting on")
		else:
			alexaCommands.append('o')
			print("Turning lighting off")
		if data["state"]["desired"]["crane"] == "on":
			alexaCommands.append('e')
			print("Elevating beer")
		else:
			alexaCommands.append('d')
			print("Closing the cargo hold")
		if data["state"]["desired"]["follow"] == "on":
			alexaCommands.append('enable')
			print("Beer Tank will follow you")
		else:
			alexaCommands.append('disable')
			alexaCommands.append('m 0 0')
			print("Beer Tank stopped")
		   
	def __on_log(self, client, userdata, level, buf):
		print("{0}, {1}, {2}, {3}".format(client, userdata, level, buf))
 
	def bootstrap_mqtt(self):
		""" Configuring
		"""
		global awshost, awsport, cert_path, caPath, certPath, keyPath
 
		self.mqttc = paho.Client()
		self.mqttc.on_connect = self.__on_connect
		self.mqttc.on_message = self.__on_message
		self.mqttc.on_log = self.__on_log
 
		self.mqttc.tls_set(caPath,
			certfile=certPath,
			keyfile=keyPath,
			cert_reqs=ssl.CERT_REQUIRED,
			tls_version=ssl.PROTOCOL_TLSv1_2,
			ciphers=None)
 
		result_of_connection = self.mqttc.connect(awshost, awsport, keepalive=120)
 
		if result_of_connection == 0:
			self.connect = True
 
		return self

	def start(self):
		self.mqttc.loop_start()
		   
				   
if __name__ == "__main__":
	""" Subscribing to wbt topic
	"""
	MQTTSub(listener = True, topic = "wbt").bootstrap_mqtt().start()
	""" Starting the application
	"""
	BeerTankApp(SERIAL_PORT, SERIAL_BAUD).run()