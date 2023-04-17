from evdev import InputDevice, categorize, ecodes
import select
import queue 		# for serial command queue
import threading 	# for multiple threads
import os
import pygame		# for sound
import serial 		# for Arduino serial access
import serial.tools.list_ports
import subprocess 	# for shell commands
import time

##### VARIABLES WHICH YOU CAN MODIFY #####
arduinoPort = "/dev/ttyACM0"                                        # Default port which will be selected
soundFolder = "/home/walle/walle-control/sounds/"                   # Location of the folder containing all audio files
controller = "/dev/input/event1"

exitFlag = 0
arduinoActive = 0
queueLock = threading.Lock()
workQueue = queue.Queue()
threads = []
head_last = {
    "G": 50,
    "B": 50,
    "T": 50
}
arm_last = {
	"L": 50,
	"R": 50
}


# Define button mappings
button_map = {
    305: lambda x: print("Button A pressed") if x > 0 else print("Button A released"),
    304: lambda x: print("Button B pressed") if x > 0 else print("Button B released"),
    307: lambda x: print("Button X pressed") if x > 0 else print("Button X released"),
    306: lambda x: print("Button Y pressed") if x > 0 else print("Button Y released"),
    308: lambda x: shoulder_control("L", event.value),
    309: lambda x: shoulder_control("R", event.value),
    310: lambda x: print("Select pressed") if x > 0 else print("Select released"),
    311: lambda x: print("Start pressed") if x > 0 else print("Start released"),
    139: lambda x: subprocess.call(['shutdown', '-h', 'now'], shell=False),
}

# Define joystick mappings
joystick_map = {
    ecodes.ABS_X: lambda value: motor_control("Y",event.value) if value >= 1 else (),
    ecodes.ABS_Y: lambda value: motor_control("X",event.value) if value >= 1 else (),
    ecodes.ABS_RX: lambda value: joystick_control("G",event.value),
    ecodes.ABS_RY: lambda value: (joystick_control("B",event.value), joystick_control("T",event.value)),
    ecodes.ABS_HAT0X: lambda value: print(f"D-Pad X: {event.value}"),
    ecodes.ABS_HAT0Y: lambda value: print(f"D-Pad Y: {event.value}"),
    ecodes.ABS_Z: lambda value: shoulder_control("L", event.value),
    ecodes.ABS_RZ: lambda value: shoulder_control("R", event.value),
}

### Start of Program ###
class arduino (threading.Thread):
	##
	# Constructor
	#
	# @param  threadID  The thread identification number
	# @param  name      Name of the thread
	# @param  q         Queue containing the message to be sent
	# @param  port      The serial port where the Arduino is connected
	#
	def __init__(self, threadID, name, q, arduinoPort):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.q = q
		self.port = arduinoPort

	##
	# Run the thread
	#
	def run(self):
		print("Starting Arduino Thread", self.name)
		process_data(self.name, self.q, self.port)
		print("Exiting Arduino Thread", self.name)

""" End of class: Arduino """

##
# Send data to the Arduino from a buffer queue
#
# @param  threadName Name of the thread
# @param  q          Queue containing the messages to be sent
# @param  port       The serial port where the Arduino is connected
#
def process_data(threadName, q, arduinoPort):
	global exitFlag
	
	ser = serial.Serial(arduinoPort,115200)
	ser.flushInput()
	dataString = ""

	# Keep this thread running until the exitFlag changes
	while not exitFlag:
		try:
			# If there are any messages in the queue, send them
			queueLock.acquire()
			if not workQueue.empty():
				data = q.get() + '\n'
				queueLock.release()
				ser.write(data.encode())
			else:
				queueLock.release()

			# Read any incomming messages
			while (ser.inWaiting() > 0):
				data = ser.read()
				if (data.decode() == '\n' or data.decode() == '\r'):
					parseArduinoMessage(dataString)
					dataString = ""
				else:
					dataString += data.decode()

			time.sleep(0.01)

		# If an error occured in the Arduino Communication
		except Exception as e: 
			print(f"Exception: {e}")
			exitFlag = 1
	ser.close()

##
# Parse messages received from the Arduino
#
# @param  dataString  String containing the serial message to be parsed
#
def parseArduinoMessage(dataString):
	global batteryLevel
	
	# Battery level message
	if "Battery" in dataString:
		dataList = dataString.split('_')
		if len(dataList) > 1 and dataList[1].isdigit():
			batteryLevel = dataList[1]


##
# Turn on/off the Arduino background communications thread
#
# @param  q    Queue object containing the messages to be sent
# @param  port The serial port where the Arduino is connected
#
def onoff_arduino(q, arduinoPort):
	global arduinoActive
	global exitFlag
	global threads
	global batteryLevel
	
	# Set up thread and connect to Arduino
	if not arduinoActive:
		exitFlag = 0

		thread = arduino(1, "Arduino", q, arduinoPort)
		thread.start()
		threads.append(thread)

		arduinoActive = 1

	# Disconnect Arduino and exit thread
	else:
		exitFlag = 1
		batteryLevel = -999

		# Clear the queue
		queueLock.acquire()
		while not workQueue.empty():
			q.get()
		queueLock.release()

		# Join any active threads up
		for t in threads:
			t.join()

		threads = []
		arduinoActive = 0

	return 0

##
# Test whether the Arduino connection is still active
#
def test_arduino():
	global arduinoActive
	global exitFlag
	global workQueue
	
	if arduinoActive and not exitFlag:
		return 1
	elif exitFlag and arduinoActive:
		onoff_arduino(workQueue, 0)
	else:
		return 0

## 
# Shoulder Button Control
#
def shoulder_control(servoID,value):
    global arm_last
    
    if value == 1 and arm_last[servoID] < 100:
        arm_current_value = arm_last[servoID] + 5
        arm_current = servoID + str(arm_current_value)
        sendMsg(arm_current)
        arm_last[servoID] = arm_current_value
    elif value == 1023 and arm_last[servoID] > 0:
        arm_current_value = arm_last[servoID] - 5
        arm_current = servoID + str(arm_current_value)
        sendMsg(arm_current)
        arm_last[servoID] = arm_current_value
     
##
# Right Joystick Control
#
def joystick_control(servoID,value):
    global head_last
    
    position = int(value / 655.35)
    
    if (position < 50 and position < head_last[servoID]) or (position > 50 and position > head_last[servoID]):
        head_current = servoID + str(position)
        sendMsg(head_current)
        head_last[servoID] = position

##
# Left Joystick Control of Motors
#
def motor_control(servoID,value):
    if servoID == "X":
        speed = -int((value - 32768) / 327.68)
    else:
        speed = int((value - 32768) / 327.68)
    sendMsg(f"{servoID}{speed}")    


def sendMsg(value):
    if test_arduino() == 1:
        queueLock.acquire()
        workQueue.put(value)
        queueLock.release()
        return
    else:
        print("Can't find Arduino")
        return



##  Initialize Everything

##  Set up the Bluetooth controller
#   Connects to bluetooth controller
#   if available, if not will wait
#
while True:
    try:
        gamepad = InputDevice(controller)
        print('Bluetooth controller connected')
        break
    except OSError:
        print('Bluetooth controller not found. Please turn on your controller and try again in 10 seconds...')
        time.sleep(10)

##  Connect to Arduino
arduino = onoff_arduino(workQueue, arduinoPort)

##  Reset Servos to Default
sendMsg("G50")
sendMsg("T0")
sendMsg("B50")
sendMsg("L0")
sendMsg("R0")
sendMsg("E50")
sendMsg("U50")

# Listen input from the gamepad
while True:
    r, w, x = select.select([gamepad], [], [])
    for event in gamepad.read():
        if event.type == ecodes.EV_KEY:
            # Button event
            button_func = button_map.get(event.code)
            if button_func:
                button_func(event.value)
        elif event.type == ecodes.EV_ABS:
            # Joystick event
            joystick_func = joystick_map.get(event.code)
            if joystick_func:
                joystick_func(event.value)
