#!/usr/bin/python

import time
import math
import re
import smbus


#Adafruit Libraries Copyright (c) 2012-2013 Limor Fried, Kevin Townsend and Mikey Sklar for Adafruit #Industries. All rights reserved.

#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. * Neither the name of the nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ===========================================================================
# Adafruit_I2C Class
# ===========================================================================

class Adafruit_I2C(object):
	
	@staticmethod
	def getPiRevision():
		"Gets the version number of the Raspberry Pi board"
		# Revision list available at: http://elinux.org/RPi_HardwareHistory#Board_Revision_History
		try:
			with open('/proc/cpuinfo', 'r') as infile:
				for line in infile:
					# Match a line of the form "Revision : 0002" while ignoring extra
					# info in front of the revsion (like 1000 when the Pi was over-volted).
					match = re.match('Revision\s+:\s+.*(\w{4})$', line)
					if match and match.group(1) in ['0000', '0002', '0003']:
						# Return revision 1 if revision ends with 0000, 0002 or 0003.
						return 1
					elif match:
						# Assume revision 2 if revision ends with any other 4 chars.
						return 2
				# Couldn't find the revision, assume revision 0 like older code for compatibility.
				return 0
		except:
			return 0

	@staticmethod
	def getPiI2CBusNumber():
		# Gets the I2C bus number /dev/i2c#
		return 1 if Adafruit_I2C.getPiRevision() > 1 else 0

	def __init__(self, address, busnum=-1, debug=False):
		self.address = address
		# By default, the correct I2C bus is auto-detected using /proc/cpuinfo
		# Alternatively, you can hard-code the bus version below:
		# self.bus = smbus.SMBus(0); # Force I2C0 (early 256MB Pi's)
		# self.bus = smbus.SMBus(1); # Force I2C1 (512MB Pi's)
		self.bus = smbus.SMBus(busnum if busnum >= 0 else Adafruit_I2C.getPiI2CBusNumber())
		self.debug = debug

	def reverseByteOrder(self, data):
		"Reverses the byte order of an int (16-bit) or long (32-bit) value"
		# Courtesy Vishal Sapre
		byteCount = len(hex(data)[2:].replace('L','')[::2])
		val			 = 0
		for i in range(byteCount):
			val		= (val << 8) | (data & 0xff)
			data >>= 8
		return val

	def errMsg(self):
		print "Error accessing 0x%02X: Check your I2C address" % self.address
		return -1

	def write8(self, reg, value):
		"Writes an 8-bit value to the specified register/address"
		try:
			self.bus.write_byte_data(self.address, reg, value)
			if self.debug:
				print "I2C: Wrote 0x%02X to register 0x%02X" % (value, reg)
		except IOError, err:
			return self.errMsg()

	def write16(self, reg, value):
		"Writes a 16-bit value to the specified register/address pair"
		try:
			self.bus.write_word_data(self.address, reg, value)
			if self.debug:
				print ("I2C: Wrote 0x%02X to register pair 0x%02X,0x%02X" %
				 (value, reg, reg+1))
		except IOError, err:
			return self.errMsg()

	def writeRaw8(self, value):
		"Writes an 8-bit value on the bus"
		try:
			self.bus.write_byte(self.address, value)
			if self.debug:
				print "I2C: Wrote 0x%02X" % value
		except IOError, err:
			return self.errMsg()

	def writeList(self, reg, list):
		"Writes an array of bytes using I2C format"
		try:
			if self.debug:
				print "I2C: Writing list to register 0x%02X:" % reg
				print list
			self.bus.write_i2c_block_data(self.address, reg, list)
		except IOError, err:
			return self.errMsg()

	def readList(self, reg, length):
		"Read a list of bytes from the I2C device"
		try:
			results = self.bus.read_i2c_block_data(self.address, reg, length)
			if self.debug:
				print ("I2C: Device 0x%02X returned the following from reg 0x%02X" %
				 (self.address, reg))
				print results
			return results
		except IOError, err:
			return self.errMsg()

	def readU8(self, reg):
		"Read an unsigned byte from the I2C device"
		try:
			result = self.bus.read_byte_data(self.address, reg)
			if self.debug:
				print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
				 (self.address, result & 0xFF, reg))
			return result
		except IOError, err:
			return self.errMsg()

	def readS8(self, reg):
		"Reads a signed byte from the I2C device"
		try:
			result = self.bus.read_byte_data(self.address, reg)
			if result > 127: result -= 256
			if self.debug:
				print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
				 (self.address, result & 0xFF, reg))
			return result
		except IOError, err:
			return self.errMsg()

	def readU16(self, reg, little_endian=True):
		"Reads an unsigned 16-bit value from the I2C device"
		try:
			result = self.bus.read_word_data(self.address,reg)
			# Swap bytes if using big endian because read_word_data assumes little 
			# endian on ARM (little endian) systems.
			if not little_endian:
				result = ((result << 8) & 0xFF00) + (result >> 8)
			if (self.debug):
				print "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X" % (self.address, result & 0xFFFF, reg)
			return result
		except IOError, err:
			return self.errMsg()

	def readS16(self, reg, little_endian=True):
		"Reads a signed 16-bit value from the I2C device"
		try:
			result = self.readU16(reg,little_endian)
			if result > 32767: result -= 65536
			return result
		except IOError, err:
			return self.errMsg()

if __name__ == '__main__':
	try:
		bus = Adafruit_I2C(address=0)
		print "Default I2C bus is accessible"
	except:
		print "Error accessing default I2C bus"

# ============================================================================
# Adafruit PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PWM :
	# Registers/etc.
	__MODE1							= 0x00
	__MODE2							= 0x01
	__SUBADR1						= 0x02
	__SUBADR2						= 0x03
	__SUBADR3						= 0x04
	__PRESCALE					 = 0xFE
	__LED0_ON_L					= 0x06
	__LED0_ON_H					= 0x07
	__LED0_OFF_L				 = 0x08
	__LED0_OFF_H				 = 0x09
	__ALL_LED_ON_L			 = 0xFA
	__ALL_LED_ON_H			 = 0xFB
	__ALL_LED_OFF_L			= 0xFC
	__ALL_LED_OFF_H			= 0xFD

	# Bits
	__RESTART						= 0x80
	__SLEEP							= 0x10
	__ALLCALL						= 0x01
	__INVRT							= 0x10
	__OUTDRV						 = 0x04

	general_call_i2c = Adafruit_I2C(0x00)

	@classmethod
	def softwareReset(cls):
		"Sends a software reset (SWRST) command to all the servo drivers on the bus"
		cls.general_call_i2c.writeRaw8(0x06)				# SWRST

	def __init__(self, address=0x40, debug=False):
		self.i2c = Adafruit_I2C(address)
		self.i2c.debug = debug
		self.address = address
		self.debug = debug
		if (self.debug):
			print "Reseting PCA9685 MODE1 (without SLEEP) and MODE2"
		self.setAllPWM(0, 0)
		self.i2c.write8(self.__MODE2, self.__OUTDRV)
		self.i2c.write8(self.__MODE1, self.__ALLCALL)
		time.sleep(0.005)																			 # wait for oscillator
		
		mode1 = self.i2c.readU8(self.__MODE1)
		mode1 = mode1 & ~self.__SLEEP								 # wake up (reset sleep)
		self.i2c.write8(self.__MODE1, mode1)
		time.sleep(0.005)														 # wait for oscillator

	def setPWMFreq(self, freq):
		"Sets the PWM frequency"
		prescaleval = 25000000.0		# 25MHz
		prescaleval /= 4096.0			 # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		if (self.debug):
			print "Setting PWM frequency to %d Hz" % freq
			print "Estimated pre-scale: %d" % prescaleval
		prescale = math.floor(prescaleval + 0.5)
		if (self.debug):
			print "Final pre-scale: %d" % prescale

		oldmode = self.i2c.readU8(self.__MODE1);
		newmode = (oldmode & 0x7F) | 0x10						 # sleep
		self.i2c.write8(self.__MODE1, newmode)				# go to sleep
		self.i2c.write8(self.__PRESCALE, int(math.floor(prescale)))
		self.i2c.write8(self.__MODE1, oldmode)
		time.sleep(0.005)
		self.i2c.write8(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):
		"Sets a single PWM channel"
		self.i2c.write8(self.__LED0_ON_L+4*channel, on & 0xFF)
		self.i2c.write8(self.__LED0_ON_H+4*channel, on >> 8)
		self.i2c.write8(self.__LED0_OFF_L+4*channel, off & 0xFF)
		self.i2c.write8(self.__LED0_OFF_H+4*channel, off >> 8)

	def setAllPWM(self, on, off):
		"Sets a all PWM channels"
		self.i2c.write8(self.__ALL_LED_ON_L, on & 0xFF)
		self.i2c.write8(self.__ALL_LED_ON_H, on >> 8)
		self.i2c.write8(self.__ALL_LED_OFF_L, off & 0xFF)
		self.i2c.write8(self.__ALL_LED_OFF_H, off >> 8)


def printStat(description, value, debug):
	if debug:
		if value.is_integer():
			print "%s: %d" % (description, value)
		else:
			print "%s: %f" % (description, value)

class stdServo:
	def __init__(self, pwm, frequency, channel, servoRange, minPulse, maxPulse, debug=False):
		self.frequency = float(frequency)		#Set frequency in Hertz
		self.channel = int(channel)			#channel of this servo
		self.servoRange = float(servoRange)	#The range of the servo in degrees
		self.minPulse = float(minPulse)		#The pulse required for 0 degrees in us(microseconds)
		self.maxPulse = float(maxPulse)		#The pulse required for max degrees in us
		self.timePerTick = (1000000/self.frequency)/4096				#Time per tick in us
		self.timePerDegree = (self.maxPulse-self.minPulse)/self.servoRange	#Also in us
		self.pwm = pwm
		self.pwm.setPWMFreq(frequency)		#set the frequency to send in Hertz
		self.debug = debug
		self.curDegree = 0
		self.maxDegree = self.servoRange/2
		self.minDegree = self.maxDegree*-1
		self.curPulse = (self.maxPulse-self.minPulse)/2
		self.setPulse(self.curPulse)
		printStat("Channel", self.channel, self.debug)
		printStat("Time per tick", self.timePerTick, self.debug)
		printStat("Time per degree", self.timePerDegree, self.debug)

	def moveToDegree(self, desiredDegree):
		desiredDegree = int(desiredDegree)
		printStat("Input degree", desiredDegree, self.debug)
		if desiredDegree > self.maxDegree:
			desiredDegree = self.maxDegree
		elif desiredDegree < self.minDegree:
			desiredDegree = self.minDegree
		realDegree = desiredDegree+self.maxDegree
		printStat("Translated degree", realDegree, self.debug)
		desiredPulse = (realDegree*self.timePerDegree)+self.minPulse		#Also in us
		printStat("Desired pulse", desiredPulse, self.debug)
		endTick = int((desiredPulse)/self.timePerTick)	#Final calculation
		printStat("Pulse ending tick", endTick, self.debug)
		self.curDegree = desiredDegree
		self.curPulse = desiredPulse
		self.pwm.setPWM(self.channel, 0, endTick)		#Sends the signal
		return True

	def setPulse(self, desiredPulse):
		printStat("Input pulse", desiredPulse, self.debug)
		if desiredPulse > self.maxPulse:
			desiredPulse = self.maxPulse
		elif desiredPulse < self.minPulse:
			desiredPulse = self.minPulse
		endTick = int(desiredPulse/self.timePerTick)	#convert to ticks
		desiredDegree = (((endTick*self.timePerTick)-self.minPulse)/self.timePerTick)-self.maxDegree
		printStat("Desired degree", desiredDegree, self.debug)
		printStat("Pulse ending tick", endTick, self.debug)
		self.curDegree = desiredDegree
		self.curPulse = desiredPulse
		self.pwm.setPWM(self.channel, 0, endTick)			#Sends the signal
		return True
		
	def backAndForth(self, degreeBuffer, degreePerIteration, timePerSweep):
		minDegree = self.minDegree+degreeBuffer
		maxDegree = self.maxDegree-degreeBuffer
		timePerSweep = float(timePerSweep)
		sleepTime = (timePerSweep/(maxDegree*2))/1000	#time between each step in loop in ms
		printStat("Time per step(sec)", sleepTime, self.debug)
		minDegree = int(minDegree)
		maxDegree = int(maxDegree)
		for degree in range(minDegree, maxDegree):
			self.moveToDegree(degree)
			time.sleep(sleepTime)
		for degree in range(maxDegree, minDegree, -1):
			self.moveToDegree(degree)
			time.sleep(sleepTime)
		return True
	
	def manualPulse(self):
		pulse = int(raw_input("Desired pulse in us:"))
		while(pulse!=0):
			pan.setPulse(pulse)
			pulse = int(raw_input("Desired pulse in us:"))
		return True
	
	def stop(self):
		self.pwm.setPWM(self.channel, 0, 4096)
		return True
		
	def addDegree(self, degree):
		self.moveToDegree(self.curDegree+degree)
		

class contServo:
	def __init__(self, pwm, frequency, channel, neutralPulse, minPulse, maxPulse, revTime, Trim, debug=False):
		self.frequency = float(frequency)		#Set frequency in Hertz
		self.channel = int(channel)			#channel of this servo
		self.neutralPulse = float(neutralPulse)	#The range of the servo in degrees
		self.minPulse = float(minPulse)		#The pulse required for 0 degrees in us(microseconds)
		self.maxPulse = float(maxPulse)		#The pulse required for max degrees in us
		self.revTime = float(revTime)		#time in seconds for servo to move 360 degrees at 100% speed
		self.Trim = float(Trim)
		self.neutralTick = int(round((self.neutralPulse*4096)/(1000000/self.frequency))) #convert neutral to ticks
		self.minTick = int(round((self.minPulse*4096)/(1000000/self.frequency))) #convert minimum
		self.maxTick = int(round((self.maxPulse*4096)/(1000000/self.frequency))) #convert maximum
		self.pwm = pwm
		self.pwm.setPWMFreq(frequency)		#set the frequency to send in Hertz
		self.debug = debug
		
	def setSpeed(self, direction, speed):
		#sanitize speed
		speed = int(speed)
		if speed > 100:
			speed = 100
		elif speed < 0:
			speed = 0
		#check direction and calculate pulse
		if direction == 'cw':
			pulse = int(round(self.neutralTick-((speed*(self.neutralTick-self.minTick))/100)))
		else:
			pulse = int(round(self.neutralTick+((speed*(self.maxTick-self.neutralTick))/100)))
		#send the signal
		#print 'Set channel %d to pulse %d.' % (channel, pulse)
		self.pwm.setPWM(self.channel, 0, pulse)
	
	def servoSweep(self, speed):
		sweepTime = round(((100/speed)*self.revTime)/4, 2)
		print 'Sweep Time %f seconds' % sweepTime
		self.setSpeed('cw', speed)
		time.sleep(sweepTime*(1-self.Trim))
		self.setSpeed('ccw', speed)
		time.sleep(sweepTime*(1+self.Trim))

	def servoRotateLeft(self, angle, speed):
		angle = int(angle)
		#sanitize speed
		speed = int(speed)
		if speed > 100:
			speed = 100
		elif speed < 0:
			speed = 0
		sweepTime = round(((100/speed)*self.revTime)/(360/angle), 2)
		print "Setting laser to the left of target."
		print "Sweep Time %f seconds at %d%%" % (sweepTime, speed)
		self.setSpeed('ccw', speed)
		time.sleep(sweepTime)
		self.stop()

	def runSweep(self, sweeps, minspeed):
		print 'Executing Channel %d with %d number of sweeps.' % (channel, sweeps)
		sweeps = int(sweeps)
		if(sweeps > 50):
			sweeps = 50	
		if(sweeps <= 1):
			print 'Last sweep, %d%% speed' % minspeed
			self.servoSweep(minspeed)
		elif(sweeps == 2):
			print 'Sweep 1, 100%% speed'
			self.servoSweep(100)
			print 'Last sweep, %d%% speed' % minspeed
			self.servoSweep(minspeed)
		else:
			print 'Sweep 1, 100%% speed'
			self.servoSweep(100)
			increment = int((100-minspeed)/(sweeps-1))
			speed = increment
			for sweep in range(2, sweeps):
				if(speed < minspeed):
					speed = minspeed
				elif(speed > 100):
					speed = 100
				print 'Sweep %d, %d%% speed' % (sweep, speed)
				self.servoSweep(speed)
				speed += increment
			print 'Last sweep, %d%% speed' % minspeed
			self.servoSweep(minspeed)
		print 'Sweeping complete, resetting Channel %d.' % channel
		self.stop()
		
		
	def stop():
		self.pwm.setPWM(self.channel, 0, 4096)
			
	def manual():
		print 'Starting Servo Check...'
		direction = raw_input('Direction (cw or ccw): ')
		speed = int(raw_input('Speed (1-100): '))
		while(speed != 0):
			print 'Setting Channel %d to %d%% speed...' % (self.channel, speed)
			self.setSpeed(direction, speed)
			print 'Enter 0 to quit.'
			speed = int(raw_input('Speed (1-100): '))
		self.stop()

	def center():
		self.setSpeed(self.channel, 'cw', 10)
		raw_input('Press ENTER when centered.')
		self.stop()
	
	def scan():
		print 'Starting Scan...'
		self.center()
		self.servoRotateLeft(45, 10)
		print 'Ready for scanning.'
		sweeps = int(raw_input('How many sweeps per scan? (0-50): '))
		scans = int(raw_input('How many scans per angle?: '))
		rotations = int(raw_input('How many angles to scan?: '))
		if(rotations >= 1):
			moveangle = int(360/rotations)
			for rotation in range(0, rotations):
				for scan in range(0, scans):
					self.runSweep(sweeps, 10)
				print 'Scan Complete.'
				if(rotation+1 < rotations):
					moveangle += int(360/rotations)
		self.stop()
		print 'LaserScan is finished.'
