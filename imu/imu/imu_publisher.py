#!/usr/bin/python
#This code was written by Caleb G. Teague in 2017

"""To do:
	Add more error handling?
	Add more variable setup on initiation
	Update the timing for my accurate polling
	Create getYaw() and getAngle() functions
"""

import smbus
import time
import math
# import thread

class MinIMU_v5_pi:
	"""
	Init function
	Valid values for    aFullScale are 2, 4, 8, and 16 [g]
						gFullScale are 125, 245, 500, 1000, and 2000 [dps]
						mFullScale are 4, 8, 12, and 16 [guass]
	"""
	def __init__(self, SMBusNum = 5, aFullScale=2, gFullScale=500, mFullScale=4):

		#Accelerometer and Gyro Register addresses
		self.Accel_Gyro_REG = dict(
			WHO_AM_I            = 0x0F,
			CTRL1_XL            = 0x10,
			CTRL2_G             = 0x11,
			CTRL3_C             = 0x12,
			STATUS_REG          = 0x1E,
			OUTX_L_XL           = 0x28,
			OUTX_H_XL           = 0x29,
			OUTY_L_XL           = 0x2A,
			OUTY_H_XL           = 0x2B,
			OUTZ_L_XL           = 0x2C,
			OUTZ_H_XL           = 0x2D)


		#Unit scales
		self.aScale = 0 #default: aScale = 2g/2^15,
		self.gScale = 0 #default: gScale = 500dps/2^15
		self.mScale = 0 #default: mScale = 4guass/2^15
		
		#Variables for updateAngle and updateYaw
		self.prevYaw = [0]
		self.tau = 0.04 #Want this roughly 10x the dt
		self.lastTimeAngle = [0]
		self.lastTimeYaw = [0]
	
		#i2c addresses
		self.accel_gyro = 0x6b

		#Connect i2c bus
		self.bus = smbus.SMBus(SMBusNum)
		
		#Enable Accel, and Gyro
		self.enableAccel_Gyro(aFullScale, gFullScale)
		

	"""Setup the needed registers for the Accelerometer and Gyro"""
	def enableAccel_Gyro(self, aFullScale, gFullScale):
		#Accelerometer
		
		g = 9.806
		#the gravitational constant for a latitude of 45 degrees at sea level is 9.80665
		#g for altitude is g(6,371.0088 km / (6,371.0088 km + altitude))^2
		#9.80600 is a good approximation for Tulsa, OK

		#default: 0b10000000
		#ODR = 1.66 kHz; +/-2g; BW = 400Hz
		b0_3 = 0b1000 #1.66 kHz
		
		#full-scale selection; 2**15 = 32768
		if aFullScale == 4:
			b4_5 = 0b10
			self.aScale = 4*g/32768
		elif aFullScale == 8:
			b4_5 = 0b11
			self.aScale = 8*g/32768
		elif aFullScale == 16:
			b4_5 = '01'
			self.aScale = 16*g/32768
		else: #default to 2g if no valid value is given
			b4_5 = '00'
			self.aScale = 2*g/32768
			
		b6_7 = '00' #0b00; 400Hz anti-aliasing filter bandwidth
		
		# self.bus.write_byte_data(self.accel_gyro, self.Accel_Gyro_REG['CTRL1_XL'], 0b10000000)
		self.bus.write_byte_data(self.accel_gyro, self.Accel_Gyro_REG['CTRL1_XL'], self.binConcat([b0_3, b4_5, b6_7]))

		#Gyro

		#default: 0b010000000
		#ODR = 1.66 kHz; 500dps
		b0_3 = 0b1000 #1.66 kHz
		
		#full-scale selection
		if gFullScale == 254:
			b4_6 = '000'
			self.gScale = 254/32768.0
		elif gFullScale == 1000:
			b4_6 = 0b100
			self.gScale = 1000/32768.0
		elif gFullScale == 2000:
			b4_6 = 0b110
			self.gScale = 2000/32768.0
		elif gFullScale == 125:
			b4_6 = '001'
			self.gScale = 125/32768.0
		else: #default to 500 dps if no valid value is given
			b4_6 = '010'
			self.gScale = 500/32768.0
			
		# self.bus.write_byte_data(self.accel_gyro, self.Accel_Gyro_REG['CTRL2_G'], 0b10000100)
		self.bus.write_byte_data(self.accel_gyro, self.Accel_Gyro_REG['CTRL2_G'], self.binConcat([b0_3, b4_6, 0]))

		#Accelerometer and Gyro

		#default: 0b00000100
		#IF_INC = 1 (automatically increment register address)
		self.bus.write_byte_data(self.accel_gyro, self.Accel_Gyro_REG['CTRL3_C'], 0b00000100)

	"""Read values from accelerometer and scale them to m/s^2, returns 0's if unable to read"""
	def readAccelerometer(self):
		#   Reading low and high 8-bit register and converting the 16-bit
		#two's complement number to decimal.
		try:
			AX = self.byteToNumber(self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTX_L_XL']),
									self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTX_H_XL']))

			AY = self.byteToNumber(self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTY_L_XL']),
									self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTY_H_XL']))

			AZ = self.byteToNumber(self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTZ_L_XL']),
									self.bus.read_byte_data(self.accel_gyro, self.Accel_Gyro_REG['OUTZ_H_XL']))
		except:
			#print "Error!"
			return 0, 0, 0

		#Scaling the decimal number to understandable units
		AX *= self.aScale; AY *= self.aScale; AZ *= self.aScale;
						   
		return [AX, AY, AZ]

	"""Combines Hi and Low 8-bit values to a 16-bit two's complement and
	converts to decimal"""
	def byteToNumber(self, val_Low, val_Hi):
		number = 256 * val_Hi + val_Low #2^8 = 256
		if number >= 32768: #2^7 = 32768
			number= number - 65536 #For two's complement
		return number

			
	"""Concatonate a list of values (integer, boolean, or string) into a single binary number
	e.g. binaryConcatenation([True, 0, 2, 0xA]) returns 170, e.g. 0b1010 1010
	Can't distinguish between 0b00 and 0b0 as an input since python interprets both as 0.
	
	Can return as a string or an integer
	"""
	@staticmethod
	# def binaryConcatenation(lst):
	def binConcat(lst, retStr=False):
		#return '0b' + ''.join(['1' if x else '0' for x in lst])
		#return '0b' + ''.join([str(int(x)) for x in lst])
		#return int('0b' + ''.join([str(int(x)) if type(x) is bool else bin(x)[2:] for x in lst]), 2)
		strValue = '0b' + ''.join([x if type(x) is str else bin(x)[2:] for x in lst])
		if retStr == False:
			return int(strValue, 2)
		else:
			return strValue

			
def main():
	IMU = MinIMU_v5_pi()
	last_reading = time.time()
	while True:
		
		IMU.readAccelerometer()
		this_reading = time.time()
		print(this_reading-last_reading)
		last_reading = this_reading
		time.sleep(.01)


		"""while True:
				i = 0
				while i < 30:
					i += 1
					IMU.updateYaw()
					time.sleep(0.004)           
				print IMU.prevYaw[0]
				#print  IMU.readAccelerometer() + IMU.readGyro() + IMU.readMagnetometer()
				time.sleep(0.004)"""


if __name__ == "__main__":
	print("MinIMU is main")
	main()
	