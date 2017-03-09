import csv
import os.path
import time
import serial

file_name = 'AlcoholReadings_blowing_no_alcohol-1.csv'
file_exists = False
port = 'COM5'
start_time = time.time()

while os.path.exists(file_name):
	file_split = file_name.split('.')[0].split('-')
	name = file_split[0]
	file_number = int(file_split[1]) + 1
	file_name = name + '-' + str(file_number) + '.csv'

ser = serial.Serial(port, 9600)

with open(file_name,'wb') as f:
	writer = csv.writer(f)
	writer.writerows([[ 'time',
						'date',
						'time_since_start(s)',
						'sensor_voltage', 
						'sensor_voltage_bl', 
						'R0', 
						'R0_baseline', 
						'RS_air', 
						'RS_air_baseline',
						'ratio_baseline', 
						'Dynamic RS_AIR/R0',
						'RS_air/.45',
						'RS_air/.60',
						'RS_air/.697',
						'RS_air/.75']])
	
	while ser.is_open:
		sensorFlow = ser.readline()
		sensorFlow = sensorFlow.split(',')	
		dataToWrite = [[ time.strftime('%X'),		#time
							time.strftime('%x'),		#date
							(time.time() - start_time), #time value
							sensorFlow[0],				#sensor_voltage
							sensorFlow[1],				#sensor_voltage_baseline
							sensorFlow[2],				#R0
							sensorFlow[3],				#R0 Baseline
							sensorFlow[4],				#RS_air
							sensorFlow[5],				#RS_air_baseline
							sensorFlow[6],				#ratio_baseline
							sensorFlow[7],				#Dynamic RS_AIR/R0
							sensorFlow[8],				#RS_air/.45
							sensorFlow[9],				#RS_air/.60
							sensorFlow[10],				#RS_air/.697
							float(sensorFlow[11])]]		#RS_air/.70
						
		writer.writerows(dataToWrite)			

		
		if (time.time() - start_time) > 30.0 and (time.time() - start_time) < 39.0:
			print '===========================BREATH NOW==========================='
		elif (time.time() - start_time) > 100.0:
			ser.close()
			print 'Wrote to ', file_name
		else:
			print dataToWrite
			
		
		
		
		
		
		
		
	
	
	
