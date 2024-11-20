#!/usr/bin/env python3
#

import sys

def main(args):
	import RPi.GPIO as GPIO
	import time
	GPIO.setmode(GPIO.BCM)
	from gpiozero import PWMLED, MCP3008
	from time import sleep
	from w1thermsensor import W1ThermSensor
	from bmp280 import BMP280
	import bme280

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
	
####################### Time definition ####################################################
	Lights_OFF_TIME=14*60*60 ## 14 hours ## can be changed as required
	Lights_ON_TIME=10*60*60 ## 10 hours ## can be changed as required
	Start_time=time.time()
	
	

####################### Reservoir Limits ##################################################
	Max_Depth_of_Reservoir=100	



####################### Heat Lamps Temp Setpoint ###########################################
	Heat_Lamp_Start_Temp=15   # Temp in celsius
	Heat_Lamp_Stop_Temp=30    # Temp in celsius
	



####################### Himudity Setpoint ###########################################
	Humidifier_Start_Setpoint= 40
	Humidifier_Stop_Setpoint= 70
####################### Inlet and Exhaust Fans Setpoint ###########################################	
	
	Air_Pressure_Low_Set_Point=700
	Air_Pressure_High_Set_Point=1000
	
####################### PH and TDS Limits ##################################################

	PH_SENSOR_PUMP1_START_VALUE=9
	PH_SENSOR_PUMP1_STOP_VALUE = 7
	
	PH_SENSOR_PUMP2_START_VALUE=3
	PH_SENSOR_PUMP2_STOP_VALUE = 7
	
	Ratio_Value_TDS_AND_EC=1
	
	EC_Pump_Start_Limit=1000
	EC_Pump_Stop_Limit=500
	
#creating object called that refers to MCP3008 channel 0
	PHSENSOR = MCP3008(1)
	TDSSENSOR=MCP3008(2) # Channel 2 as per Pin Connections
	# MCP3008 has eight channels 
	
#### Updated Pumps Pins #################
	Pump1_pin=26 	
	Pump2_pin=11 
	Pump3_pin=13 
	Pump4_pin=6	 
	Pump5_pin=10 
	Pump6_pin=22 
	Pump7_pin=27  
	Pump8_pin=17 
	DCfan_IN=24  
	Light_IN_1=9
	Light_IN_2=23
	Humidifier_Power_Pin=15
	Lamp_heater_power_pin=3
	Ac_Heater_Power_pin=25
	Ventilation_Power_Pin=2
	Soil_Sensor_1_Switch=16
	Soil_Sensor_2_Switch=12
	PH502C_Digital_Output=8
	

### Setting GPIOs as Input Output ###############
	GPIO.setup(Pump1_pin,GPIO.OUT)
	GPIO.setup(Pump2_pin,GPIO.OUT)
	GPIO.setup(Pump3_pin,GPIO.OUT)
	GPIO.setup(Pump4_pin,GPIO.OUT)
	GPIO.setup(Pump5_pin,GPIO.OUT)
	GPIO.setup(Pump6_pin,GPIO.OUT)
	GPIO.setup(Pump7_pin,GPIO.OUT)
	GPIO.setup(Pump8_pin,GPIO.OUT)
	GPIO.setup(DCfan_IN,GPIO.OUT)
	GPIO.setup(Light_IN_1,GPIO.OUT)
	GPIO.setup(Light_IN_2,GPIO.OUT)
	GPIO.setup(Humidifier_Power_Pin,GPIO.OUT)
	GPIO.setup(Lamp_heater_power_pin,GPIO.OUT)
	GPIO.setup(Ac_Heater_Power_pin,GPIO.OUT)
	GPIO.setup(Ventilation_Power_Pin,GPIO.OUT)
	GPIO.setup(Soil_Sensor_1_Switch,GPIO.IN)
	GPIO.setup(Soil_Sensor_2_Switch,GPIO.IN)
	GPIO.setup(PH502C_Digital_Output,GPIO.IN)
	  
	
	
	
	
	
	GPIO.output(Light_IN_1,True)	 #### Initial condition for Light Output
	
	

	
	

#### Setting pwm instance for GPIOs   #############################
	Pump1PWM=GPIO.PWM(pump1_pin, 1000)
	Pump2PWM=GPIO.PWM(pump2_pin, 1000)
	Pump3PWM=GPIO.PWM(pump3_pin, 1000)
	Pump4PWM=GPIO.PWM(pump4_pin, 1000)
	Pump5PWM=GPIO.PWM(pump5_pin, 1000)
	Pump6PWM=GPIO.PWM(pump6_pin, 1000)
	Pump7PWM=GPIO.PWM(pump7_pin, 1000)
	Pump8PWM=GPIO.PWM(pump8_pin, 1000)
	
	
	#start (Duty Cycle)

#	It is used to start PWM generation of specified Duty Cycle.

#	ChangeDutyCycle(Duty Cycle)

#	This function is used to change the Duty Cycle of signal. We have to provide Duty Cycle in the range of 0-100.

#	ChangeFrequency(frequency)

#	This function is used to change the frequency (in Hz) of PWM. This function we have not used in above program. But, we can use it for changing the frequency.

#	stop()

# 	This function is used to stop the PWM generation.
	
	
	
	while True:
 
################## Reading SHT31 ##########################################################################
		seconds = time.time() ## Reading time from raspberry pi
		
	# Send the start conversion command to the SHT31
		bus.write_i2c_block_data(0x44, 0x2C, [0x06])

	# wait for the conversion to complete
		time.sleep(0.5)
 
	# Read the data from the SHT31 containing
	# the temperature (16-bits + CRC) and humidity (16bits + crc)
		SHT31_data = bus.read_i2c_block_data(0x44, 0x00, 6)
 
	# Convert the data
		SHT31_temp = SHT31_data[0] * 256 + SHT31_data[1]
		SHT31_cTemp = -45 + (175 * SHT31_temp / 65535.0)
		SHT31_humidity = 100 * (SHT31_data[3] * 256 + data[4]) / 65535.0
 
	# Output data to the terminal
		print ("Temperature in Celsius is : %.2f C" %SHT31_cTemp)
		print ("Relative Humidity is : %.2f %%RH" %SHT31_humidity)



########################## Reading PH values and converting it#########################################
	
		PHSENSORVALUE=map_range(PHSENSOR.value,0,1023,1,14) # Reading analog value from MCp3008
		print ("PH value is")
		print(PHSENSOR.value)
		sleep(0.1)



########################## Reading TDS values and converting it#########################################
	
		TDSSENSORVALUE=map_range(T ,0,1023,0,1000) # Reading analog value from MCp3008
		print ("TDS Sensor value is")
		print(TDSSENSOR.value)
		sleep(0.1)


####################### Reading one wire temperature sensor ###########################################
		DS18B20_temperature = sensor.get_temperature()
		print("The temperature of water in the system is %s celsius" % DS18B20_temperature)
		time.sleep(1)



####################### Reading bmp 280 sensor ###########################################

#Copy and paste the below command

#sudo pip install bmp280
#Now run the below command to get the folders from GitHub
#git clone https://github.com/pimoroni/bmp280-python


		# Initialise the BMP280
		bmp280 = BMP280(i2c_dev=bus)
		bmp280_temperature = bmp280.get_temperature()
		bmp280_pressure = bmp280.get_pressure()
		bmp280_degree_sign = u"\N{DEGREE SIGN}"
		bmp280_format_temp = "{:.2f}".format(bmp280_temperature)
		print('BMP 280 Temperature = ' + bmp280_format_temp + bmp280_degree_sign + 'C')
		format_press = "{:.2f}".format(bmp280_pressure)
		print('BMP 280 Pressure = ' + format_press + ' hPa \n')
		time.sleep(4)




####################### Reading GY-bme280 sensor ###########################################
		# BME280 sensor address (default address)
		bme_280_address = 0x76
		# Load calibration parameters
		bme_280_calibration_params = bme280.load_calibration_params(bus, bme_280_address)
        # Read sensor data
		print('GY bme 280 is Running')
		bme_280_data = bme280.sample(bus, bme_280_address, bme_280_calibration_params)

        # Extract temperature, pressure, humidity, and corresponding timestamp
		bme_280_temperature_celsius = bme_280_data.temperature
		bme_280_humidity = bme_280_data.humidity
		bme_280_pressure = bme_280_data.pressure
		bme_280_timestamp = bme_280_data.timestamp
		time.sleep(1)



############## Calculating EC with TDS ######################################################
		EC_Value=TDSSENSORVALUE/Ratio_Value_TDS_AND_EC # us/cm


############## Logic for pump Operation with PH ######################################################

		if PHSENSORVALUE > PH_SENSOR_PUMP1_START_VALUE:  # Pump one logic to reduce ph
			Pump1PWM.start(100)
		if PHSENSORVALUE < PH_SENSOR_PUMP1_STOP_VALUE:
			Pump1PWM.ChangeDutyCycle(0)



		if PHSENSORVALUE < PH_SENSOR_PUMP2_START_VALUE:  # Pump two logic to increase ph
			Pump2PWM.start(100)
		if PHSENSORVALUE > PH_SENSOR_PUMP2_START_VALUE:
			Pump2PWM.ChangeDutyCycle(0)





############## Logic for pump Operation with TDS ######################################################

		if EC_Value > EC_Pump_Start_Limit: # Pump logic for TDS Control
			Pump3PWM.start(100)
			Pump4PWM.start(100)
			Pump5PWM.start(100)
			Pump6PWM.start(100)
		if EC_Value < EC_Pump_Stop_Limit:
			Pump3PWM.ChangeDutyCycle(0)
			Pump4PWM.ChangeDutyCycle(0)
			Pump5PWM.ChangeDutyCycle(0)
			Pump6PWM.ChangeDutyCycle(0)



################ Reading Ultrasonic Sensor HC-SR04 ####################################################
		HCSR04_distance=ReadHRSC05
		print ("The depth of the water is measured :",Max_Depth_of_Reservoir-HCSR04_distance,"cm")
		sleep(0.1)	


################ Average value of SHDT/GY-BM280 ###############################################

		SHDT_GYBM280_Average_humidity= (SHT31_humidity+bme_280_humidity)/2
		SHDT_GYBM280_Average_temperature=(SHT31_cTemp+bme_280_temperature_celsius)/2
		print ("Average humidity of SHDT and GYBM280 is ",SHDT_GYBM280_Average_humidity)
		print ("Average temperature of SHDT and GYBM280 is ",SHDT_GYBM280_Average_temperature)			


################ Average value of BMP280/GY-BM280 ###############################################
		bme280_bmp280_average_pressure=(bme_280_pressure+bmp280_pressure)/2


############## Logic for heat lamp operation ######################################################
		if SHDT_GYBM280_Average_temperature< Heat_Lamp_Start_Temp:
			GPIO.output(Lamp_heater_power_pin, True)
			HL_STATUS=1
		if SHDT_GYBM280_Average_temperature< Heat_Lamp_Start_Temp:
			GPIO.output(Lamp_heater_power_pin, False)
			HL_STATUS=0


############## Logic for inlet and exhaust fan operation ######################################################

		if bme280_bmp280_average_pressure < Air_Pressure_Low_Set_Point:
			GPIO.output(Ac_Heater_Power_pin,True)
		else:
			GPIO.output(Ac_Heater_Power_pin,False)
			
		if bme280_bmp280_average_pressure > Air_Pressure_High_Set_Point:
			GPIO.output(Ventilation_Power_Pin,True)
		else: 
			GPIO.output(Ventilation_Power_Pin,False)
		


############## Logic for Humidifier operation ######################################################
		if SHDT_GYBM280_Average_humidity < Humidifier_Start_Setpoint:
			GPIO.output(Humidifier_Power_Pin,True)
		if SHDT_GYBM280_Average_humidity > Humidifier_Stop_Setpoint:
			GPIO.output(Humidifier_Power_Pin,False)





############## Logic for Lights on/off operation #################################################

		if GPIO.input(Light_IN_1)==0 & (time.time()-Start_time >= Lights_OFF_TIME): 
			GPIO.output(Light_IN_1,True)
			Start_time=time.time()
		if GPIO.input(Light_IN_1)==1 & (time.time()-Start_time >= Lights_ON_TIME):
			GPIO.output(Light_IN_2,False)
			Start_time=time.time() 



############## Indication for Soil Sensor  #################################################
		if GPIO.input(Soil_Sensor_1_Switch)==1:
			print("Soil Sensor 1 is activated ")
		if GPIO.input(Soil_Sensor_2_Switch)==1:		
			print("Soil Sensor 2 is activated ")



if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
    
    
def ReadHRSC05 ():
	
	TRIG = 21  # GPIO for Trigger output

	ECHO = 20  # GPIO for ECHO listening input

	print ("Distance Measurement In Progress")
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	GPIO.output(TRIG, False)
	print ("Waiting For Sensor To Settle")
	time.sleep(2) # Sleep for 2 seconds
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
	print ("The distance measured from Water is :",distance,"cm")
	GPIO.cleanup()
	return distance
	


def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min	
	
	
	    
