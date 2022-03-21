#!/usr/bin/env python


#v1.0 By Bart Garcia Nathan, April 2020 
#v1.1 Editted by Thomas Wright, April 2020


import serial, string, time, rospy
from epd2.msg import EPD2_Message


class N2(object):
	def __init__(self):
		self._Hp0_dose_decoded = 0.0
		self._Hp1_dose_decoded = 0.0
		self._Hp0_meaning = "HpG"
		self._Hp1_meaning = "HpN"
		#Written in hex as it should be sent: ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffc0b3696c0000564cc1
		self._heartbeat_epd_cmd =chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0xe5)+chr(0x85)+chr(0x6c)+chr(0x00)+chr(0x00)+chr(0x12)+chr(0x21)+chr(0xc1)
		#Written in hex as it should be sent:ffffffc0b3696c04030501087de0d8c1
		self._request_dose_cmd = chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0xe5)+chr(0x85)+chr(0x6c)+chr(0x04)+chr(0x03)+chr(0x05)+chr(0x01)+chr(0x08)+chr(0xfa)+chr(0x68)+chr(0xc1)
		## Expected Answers from device
		#Written in hex as it should be received:ffffffc0b3696c0200e67fc1
		self._heartbeat_epd_ans = ['0xff', '0xff', '0xff', '0xc0', '0xe5', '0x85', '0x6c', '0x2', '0x0', '0xa2', '0x12', '0xc1']


class MK25(object):
	def __init__(self):
		self._Hp0_dose_decoded = 0.0
		self._Hp1_dose_decoded = 0.0
		self._Hp0_meaning = "Hp10"
		self._Hp1_meaning = "Hp07"
		#Written in hex as it should be sent: ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffc0b3696c0000564cc1
		self._heartbeat_epd_cmd =chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x3b)+chr(0x6a)+chr(0x00)+chr(0x00)+chr(0x00)+chr(0x00)+chr(0x19)+chr(0xc1)
		#Written in hex as it should be sent:ffffffc0b3696c04030501087de0d8c1
		self._request_dose_cmd = chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x3b)+chr(0x6a)+chr(0x00)+chr(0x04)+chr(0x03)+chr(0x05)+chr(0x01)+chr(0x08)+chr(0xb5)+chr(0x20)+chr(0xc1)
		## Expected Answers from device
		#Written in hex as it should be received:ffffffc0b3696c0200e67fc1
		self._heartbeat_epd_ans = ['0xff', '0xff', '0xff', '0xc0', '0x3b', '0x6a', '0x0', '0x2', '0x0', '0xb0', '0x2a', '0xc1']



class EPD2(object):
	def __init__(self,serial_port,device,rate):

		if device == "N2":
			self._sensor = N2()
		elif device == "MK25":
			self._sensor = MK25()


		self._ser_port_id = serial_port
		self._ser  = None
		self.output = ""
		self._r = rospy.Rate(rate)
		#Commands to send
		self._conv_indx = 0.015622 #This value is an average obtained using previous values
		self._pub = rospy.Publisher("EPD_"+device, EPD2_Message, queue_size=2)
		self.open_comms()
		self.run()

	def open_comms(self):
		ser_connected = False
		#Open Serial Port
		print "Opening serial port for communication in Port {}".format(self._ser_port_id)
		self._ser = serial.Serial(self._ser_port_id,9600, timeout=1)
		self._ser.flushInput()
		self._ser.flushOutput()

		#Search for the EPD using the heartbeat command
		print "Searching for EPD...."
		count = 0
		while ser_connected == False:
			self._ser.write(self._sensor._heartbeat_epd_cmd)
			time.sleep(0.2)
			output=self._ser.read(12)


			answer = map(hex,map(ord,output))
			if answer == self._sensor._heartbeat_epd_ans:
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
			self._ser.write(self._sensor._request_dose_cmd)
			time.sleep(0.2)
			output=self._ser.read(66)#Amount of bytes to read, size of the expected answer
			#print output
			return map(hex,map(ord,output))
		except:
			return None

	def decode_answer(self,answer):

		#Clear values:
		Hp0_dose_string = ""
		Hp0_dose_reversed = ""
		
		Hp1_dose_string = ""
		Hp1_dose_reversed = ""

		#Obtain desired values from the message, there are always in the same index:
		Hp0_dose_vector = answer[11 : 15]
		Hp1_dose_vector = answer[15 : 19]

		#Reverse each byte since each byte is sent in LSbit first
		for element in Hp0_dose_vector:
			Hp0_dose_string+="{:02x}".format(int(element,16))[::-1]
		for element in Hp1_dose_vector:
			Hp1_dose_string+="{:02x}".format(int(element,16))[::-1]

		#Reverse full string as number is sent int LSByte first
		Hp0_dose_reversed = Hp0_dose_string[::-1]
		Hp1_dose_reversed = Hp1_dose_string[::-1]

		#Multiply values by the Convertion Index
		self._sensor._Hp0_dose_decoded = int(Hp0_dose_reversed,16)*self._conv_indx
		self._sensor._Hp1_dose_decoded = int(Hp1_dose_reversed,16)*self._conv_indx

		previousHp0 = self._sensor._Hp0_dose_decoded
		dose_rateHp0 = (previousHp0 - self._sensor._Hp0_dose_decoded)*1800
		previousHp1 = self._sensor._Hp1_dose_decoded
		dose_rateHp1 = (previousHp1 - self._sensor._Hp1_dose_decoded)*1800

		print self._sensor._Hp0_meaning, "=" , self._sensor._Hp0_dose_decoded, "Dose-rate:", dose_rateHp0, "/h"
		print self._sensor._Hp1_meaning, "=" , self._sensor._Hp1_dose_decoded, "Dose-rate:", dose_rateHp1, "/h"

	def assemble_message(self):
		msg = EPD2_Message()
		msg.header.stamp = rospy.Time.now()
		msg.data0 = self._sensor._Hp0_dose_decoded
		msg.unit0 = "Sv/h"
		msg.measurement_type0 = self._sensor._Hp0_meaning
		msg.data1 = self._sensor._Hp1_dose_decoded
		msg.unit1 = "Sv/h"
		msg.measurement_type1 = self._sensor._Hp1_meaning
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
					print self._sensor._Hp0_meaning+ " Dose: \t" + str(round(self._sensor._Hp0_dose_decoded,2))
					print self._sensor._Hp1_meaning+ " Dose: \t" + str(round(self._sensor._Hp1_dose_decoded,2))
					print "*********************"
					self._r.sleep()
				except:
					pass
			else:
				time.sleep(0.01)


def main():

	rospy.init_node("EPDMK25")
	port = rospy.get_param('/EPD/serial_port', '/dev/EPDMK25')
	device = rospy.get_param('/EPD/device_id', 'MK25')
	rate = rospy.get_param('/EPD/sample_rate', 1)

	epd = EPD2(port,device,rate)

if __name__ == "__main__":
	# execute only if run as a script
	main()

