###########################################
# Created by The Machine Shop 2019        #
# Visit our website TheMachineShop.uk     #
# This script interfaces the Zio Qwiic    #
# Temperature and Humidity sensor (SHT31) #
# to a Raspberry Pi over i2c and converts #
# the data to celcius and percentage      #
# requires python-smbus to be installed:  #
# sudo apt-get install python-smbus       #   
###########################################

#import the required libraries
import smbus
import time
 
# Start the i2c bus and label as 'bus'
bus = smbus.SMBus(1)
 
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
