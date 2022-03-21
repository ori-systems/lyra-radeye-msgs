#!/usr/bin/env python


#v1.0 By Bart Garcia Nathan, April 2020 
#v1.1 Editted by Thomas Wright, April 2020


import serial, string, time, rospy
from epd2.msg import EPD2_Message


class EPD2(object):
	def __init__(self,serial_port,rate):

		rospy.init_node("EPD")


		self._ser_port_id = serial_port
		self._ser  = None
		self.output = ""
		self._r = rospy.Rate(rate)
		#Commands to send
		#Written in hex as it should be sent: ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffc0b3696c0000564cc1
		self._heartbeat_epd_cmd =chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0xc6)+chr(0xbb)+chr(0x02)+chr(0x00)+chr(0x00)+chr(0xa9)+chr(0xab)+chr(0xc1)

		#Written in hex as it should be sent:ffffffc0b3696c04030501087de0d8c1
		self._request_dose_cmd = chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0xc6)+chr(0xbb)+chr(0x02)+chr(0x04)+chr(0x03)+chr(0x05)+chr(0x01)+chr(0x08)+chr(0xa2)+chr(0xba)+chr(0xc1)
		## Expected Answers from device
		#Written in hex as it should be received:ffffffc0b3696c0200e67fc1
		self._heartbeat_epd_ans = ['0xff', '0xff', '0xff', '0xc0', '0xc6', '0xbb', '0x2', '0x2', '0x0', '0x19', '0x98', '0xc1']
		self._conv_indx = 0.015622 #This value is an average obtained using previous values

		self._Hp10_dose_decoded = 0.0
		self._Hp07_dose_decoded = 0.0

		self._pub = rospy.Publisher("EPD", EPD2_Message, queue_size=2)
			

		self.open_comms()
		self.run()


	def open_comms(self):
		ser_connected = False
		#Open Serial Port
		print "Opening serial port for communication in Port /dev/ttyUSB0..."
		self._ser = serial.Serial(self._ser_port_id,9600, timeout=1)
		self._ser.flushInput()
		self._ser.flushOutput()

		#Search for the EPD using the heartbeat command
		print "Searching for EPD...."
		count = 0
		while ser_connected == False:
			self._ser.write(self._heartbeat_epd_cmd)
			time.sleep(0.2)
			output=self._ser.read(12)


			answer = map(hex,map(ord,output))
			if answer == self._heartbeat_epd_ans:
				print "EPD communication opened"
				ser_connected = True
			elif count > 100:
				print "EPD not found..."
				exit()
			else: 
				count +=1

	def request_measurement(self):
		try:
			#Read values
			print "Requesting Dose values..."
			self._ser.write(self._request_dose_cmd)
			time.sleep(0.2)
			output=self._ser.read(66)#Amount of bytes to read, size of the expected answer

			return map(hex,map(ord,output))
		except:
			return None

	def decode_answer(self,answer):

		#Clear values:
		Hp10_dose_string = ""
		Hp10_dose_reversed = ""
		
		Hp07_dose_string = ""
		Hp07_dose_reversed = ""

		#Obtain desired values from the message, there are always in the same index:
		Hp10_dose_vector = answer[11 : 15]
		Hp07_dose_vector = answer[15 : 19]

		#Reverse each byte since each byte is sent in LSbit first
		for element in Hp10_dose_vector:
			Hp10_dose_string+="{:02x}".format(int(element,16))[::-1]
		for element in Hp07_dose_vector:
			Hp07_dose_string+="{:02x}".format(int(element,16))[::-1]

		#Reverse full string as number is sent int LSByte first
		Hp10_dose_reversed = Hp10_dose_string[::-1]
		Hp07_dose_reversed = Hp07_dose_string[::-1]

		#Multiply values by the Convertion Index
		self._Hp10_dose_decoded = int(Hp10_dose_reversed,16)*self._conv_indx
		self._Hp07_dose_decoded = int(Hp07_dose_reversed,16)*self._conv_indx


	def assemble_message(self):
		msg = EPD2_Message()
		msg.header.stamp = rospy.Time.now()
		msg.data0 = self._Hp10_dose_decoded
		msg.unit0 = "Sv/h"
		msg.measurement_type0 = "Hp10"
		msg.data1 = self._Hp07_dose_decoded
		msg.unit1 = "Sv/h"
		msg.measurement_type1 = "Hp07"
		return msg

	def run(self):
		while not rospy.is_shutdown():
		#while(1):

			answer = self.request_measurement()
			if answer is not None:
				try:
					self.decode_answer(answer)
					msg = self.assemble_message()
					self._pub.publish(msg)
					#Print Result
					print "Hp10 Dose: \t" + str(round(self._Hp10_dose_decoded,2))
					print "Hp07 Dose: \t" + str(round(self._Hp07_dose_decoded,2))
					print "*********************"
					self._r.sleep()
				except:
					pass
			else:
				time.sleep(0.01)


def main():

	epd = EPD2('/dev/ttyUSB0',1)

if __name__ == "__main__":
	# execute only if run as a script
	main()

