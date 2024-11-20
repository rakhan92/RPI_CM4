#!/usr/bin/env python3
#
#  HR-SC05.py
#  
#  Copyright 2024 Rizwan <Rizwan@DESKTOP-IHQJ99F>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  


import sys


def main(args):
	import RPi.GPIO as GPIO
	import time
	GPIO.setmode(GPIO.BCM)
	from gpiozero import PWMLED, MCP3008
	from time import sleep
	#create an object called pot that refers to MCP3008 channel 0
	PHSENSOR = MCP3008(0)
	# MCP3008 has eight channels 
	
	
	from w1thermsensor import W1ThermSensor
	# Imports for one wire sensor 

#requires python-smbus to be installed:  #
# sudo apt-get install python-smbus       #   
###########################################

#import the required libraries
	import smbus
	import time
	
#for SHT31

# Start the i2c bus and label as 'bus'
	bus = smbus.SMBus(0) # Using I2C (0) interface  GPIO 0 and GPIO 1
	#bus=smbus.SMBus(1)  # using I2C (1) interface  GPIO 2 and GPIO 3

while True:
 
# Send the start conversion command to the SHT31
		bus.write_i2c_block_data(0x44, 0x2C, [0x06])

# wait for the conversion to complete
		time.sleep(0.5)
 
# Read the data from the SHT31 containing
# the temperature (16-bits + CRC) and humidity (16bits + crc)
		data = bus.read_i2c_block_data(0x44, 0x00, 6)
 
# Convert the data
		temp = data[0] * 256 + data[1]
		cTemp = -45 + (175 * temp / 65535.0)
		humidity = 100 * (data[3] * 256 + data[4]) / 65535.0
 
	# Output data to the terminal
		print ("Temperature in Celsius is : %.2f C" %cTemp)
		print ("Relative Humidity is : %.2f %%RH" %humidity)
	
	########################## Reading PH values and converting it#########################################
	
		PHSENSORVALUE=map_range(PHSENSOR.value,0,1023,1,14) # Reading analog value from MCp3008
		print(PHSENSOR.value)
		sleep(0.1)


	########################## Reading PH values and converting it#########################################
	

	####################### Reading one wire temperature sensor ###########################################
		temperature = sensor.get_temperature()
		print("The temperature is %s celsius" % temperature)
		time.sleep(1)
    ##########################################################################################################
	
if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
    
    
def ReadHRSC05 ():
	
	TRIG = 23  # GPIO for Trigger output

	ECHO = 24  # GPIO for ECHO listening input

	print ("Distance Measurement In Progress")
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	GPIO.output(TRIG, False)
	print ("Waiting For Sensor To Settle")
	time.sleep(2)
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)
	while GPIO.input(ECHO)==0:
		pulse_start = time.time()
	while GPIO.input(ECHO)==1:
		pulse_end = time.time()      
	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150
	distance = round(distance, 2)
	print ("Distance:",distance,"cm")
	GPIO.cleanup()
	return distance
	
#def ReadSHT31() :
#	pass


def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min	
	
	
	    
