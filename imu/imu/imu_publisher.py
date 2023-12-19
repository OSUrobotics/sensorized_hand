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
			FUNC_CFG_ACCESS     = 0x01,
								\
			FIFO_CTRL1          = 0x06,
			FIFO_CTRL2          = 0x07,
			FIFO_CTRL3          = 0x08,
			FIFO_CTRL4          = 0x09,
			FIFO_CTRL5          = 0x0A,
			ORIENT_CFG_G        = 0x0B,
								\
			INT1_CTRL           = 0x0D,
			INT2_CTRL           = 0x0E,
			WHO_AM_I            = 0x0F,
			CTRL1_XL            = 0x10,
			CTRL2_G             = 0x11,
			CTRL3_C             = 0x12,
			CTRL4_C             = 0x13,
			CTRL5_C             = 0x14,
			CTRL6_C             = 0x15,
			CTRL7_G             = 0x16,
			CTRL8_XL            = 0x17,
			CTRL9_XL            = 0x18,
			CTRL10_C            = 0x19,
								\
			WAKE_UP_SRC         = 0x1B,
			TAP_SRC             = 0x1C,
			D6D_SRC             = 0x1D,
			STATUS_REG          = 0x1E,
								\
			OUT_TEMP_L          = 0x20,
			OUT_TEMP_H          = 0x21,
			OUTX_L_G            = 0x22,
			OUTX_H_G            = 0x23,
			OUTY_L_G            = 0x24,
			OUTY_H_G            = 0x25,
			OUTZ_L_G            = 0x26,
			OUTZ_H_G            = 0x27,
			OUTX_L_XL           = 0x28,
			OUTX_H_XL           = 0x29,
			OUTY_L_XL           = 0x2A,
			OUTY_H_XL           = 0x2B,
			OUTZ_L_XL           = 0x2C,
			OUTZ_H_XL           = 0x2D,
								\
			FIFO_STATUS1        = 0x3A,
			FIFO_STATUS2        = 0x3B,
			FIFO_STATUS3        = 0x3C,
			FIFO_STATUS4        = 0x3D,
			FIFO_DATA_OUT_L     = 0x3E,
			FIFO_DATA_OUT_H     = 0x3F,
			TIMESTAMP0_REG      = 0x40,
			TIMESTAMP1_REG      = 0x41,
			TIMESTAMP2_REG      = 0x42,
								\
			STEP_TIMESTAMP_L    = 0x49,
			STEP_TIMESTAMP_H    = 0x4A,
			STEP_COUNTER_L      = 0x4B,
			STEP_COUNTER_H      = 0x4C,
								\
			FUNC_SRC            = 0x53,
								\
			TAP_CFG             = 0x58,
			TAP_THS_6D          = 0x59,
			INT_DUR2            = 0x5A,
			WAKE_UP_THS         = 0x5B,
			WAKE_UP_DUR         = 0x5C,
			FREE_FALL           = 0x5D,
			MD1_CFG             = 0x5E,
			MD2_CFG             = 0x5F)

		#Magnemometer addresses
		self.Mag_REG= dict(
			WHO_AM_I    = 0x0F,
						\
			CTRL_REG1   = 0x20,
			CTRL_REG2   = 0x21,
			CTRL_REG3   = 0x22,
			CTRL_REG4   = 0x23,
			CTRL_REG5   = 0x24,
						\
			STATUS_REG  = 0x27,
			OUT_X_L     = 0x28,
			OUT_X_H     = 0x29,
			OUT_Y_L     = 0x2A,
			OUT_Y_H     = 0x2B,
			OUT_Z_L     = 0x2C,
			OUT_Z_H     = 0x2D,
			TEMP_OUT_L  = 0x2E,
			TEMP_OUT_H  = 0x2F,
			INT_CFG     = 0x30,
			INT_SRC     = 0x31,
			INT_THS_L   = 0x32,
			INT_THS_H   = 0x33)

		#Unit scales
		self.aScale = 0 #default: aScale = 2g/2^15,
		self.gScale = 0 #default: gScale = 500dps/2^15
		self.mScale = 0 #default: mScale = 4guass/2^15
		
		#Variables for updateAngle and updateYaw
		self.prevAngle = [[0,0,0]] #x, y, z (roll, pitch, yaw)
		self.prevYaw = [0]
		self.tau = 0.04 #Want this roughly 10x the dt
		self.lastTimeAngle = [0]
		self.lastTimeYaw = [0]
	
		#i2c addresses
		self.mag = 0x1e #0011110 (from docs)
		self.accel_gyro = 0x6b

		#Connect i2c bus
		self.bus = smbus.SMBus(SMBusNum)
		
		#Enable Mag, Accel, and Gyro
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
	