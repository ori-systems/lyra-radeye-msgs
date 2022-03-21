#!/usr/bin/env python


#v1.0 By Bart Garcia Nathan, April 2020 
#v1.1 Editted by Thomas Wright, April 2020


import serial, string, time, rospy
from epd2.msg import EPD2_Message


class N2(object):
	def __init__(self):
		self._model = "N2"
		self._Hp0_dose_decoded = 0.0
		self._Hp1_dose_decoded = 0.0
		self._Hp0_meaning = "HpG"
		self._Hp1_meaning = "HpN"
		#Written in hex as it should be sent: ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffc0b3696c0000564cc1
		self._heartbeat_epd_cmd =chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x36)+chr(0x84)+chr(0x6c)+chr(0x00)+chr(0x00)+chr(0x52)+chr(0xdf)+chr(0xc1)
		#Written in hex as it should be sent:ffffffc0b3696c04030501087de0d8c1
		self._request_dose_cmd = chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x36)+chr(0x84)+chr(0x6c)+chr(0x04)+chr(0x03)+chr(0x05)+chr(0x01)+chr(0x08)+chr(0x94)+chr(0x88)+chr(0xc1)
		## Expected Answers from device
		#Written in hex as it should be received:ffffffc0b3696c0200e67fc1
		self._heartbeat_epd_ans = ['0xff', '0xff', '0xff', '0xc0', '0x36', '0x84', '0x6c', '0x2', '0x0', '0xe2', '0xec', '0xc1']


class MK25(object):
	def __init__(self):
		self._model = "MK25"
		self._Hp0_dose_decoded = 0.0
		self._Hp1_dose_decoded = 0.0
		self._Hp0_meaning = "Hp10"
		self._Hp1_meaning = "Hp07"
		#Written in hex as it should be sent: ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffc0b3696c0000564cc1
		self._heartbeat_epd_cmd =chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x39)+chr(0xea)+chr(0x04)+chr(0x00)+chr(0x00)+chr(0x87)+chr(0x41)+chr(0xc1)
		#Written in hex as it should be sent:ffffffc0b3696c04030501087de0d8c1
		self._request_dose_cmd = chr(0xff)+chr(0xff)+chr(0xff)+chr(0xc0)+chr(0x39)+chr(0xea)+chr(0x04)+chr(0x04)+chr(0x03)+chr(0x05)+chr(0x01)+chr(0x08)+chr(0x6b)+chr(0xbd)+chr(0xc1)
		## Expected Answers from device
		#Written in hex as it should be received:ffffffc0b3696c0200e67fc1
		self._heartbeat_epd_ans = ['0xff', '0xff', '0xff', '0xc0', '0x39', '0xea', '0x4', '0x2', '0x0', '0x37', '0x72', '0xc1']



class EPD2(object):
	def __init__(self,serial_port,rate):


		self._sensors = [N2(), MK25()]

		self._ser_port_id = serial_port
		self._ser  = None
		self.output = ""
		self._r = rospy.Rate(rate)
		#Commands to send
		self._conv_indx = 0.015622 #This value is an average obtained using previous values
		self._publishers = [rospy.Publisher("EPD_N2", EPD2_Message, queue_size=2),rospy.Publisher("EPD_MK25", EPD2_Message, queue_size=2)]
		self.open_comms()
		self.run()

	def open_comms(self):
		ser_connected = 0
		#Open Serial Port
		print "Opening serial port for communication in Port {}".format(self._ser_port_id)
		self._ser = serial.Serial(self._ser_port_id,9600, timeout=1)
		self._ser.flushInput()
		self._ser.flushOutput()

		#Search for the EPD using the heartbeat command
		print "Searching for EPD...."
		count = 0
		while ser_connected < len(self._sensors):
			self._ser.write(self._sensors[ser_connected]._heartbeat_epd_cmd)
			time.sleep(0.2)
			output=self._ser.read(12)


			answer = map(hex,map(ord,output))
			if answer == self._sensors[ser_connected]._heartbeat_epd_ans:
				print "EPD communication opened with sensor {}".format(self._sensors[ser_connected]._model)
				ser_connected += 1
				count = 0

			elif count > 100:
				print "EPD not found..."
				exit()
			else: 
				count +=1

	def request_measurement(self,i):
		try:
			#Read values
			print "Requesting Dose values from {}".format(self._sensors[i]._model)
			self._ser.write(self._sensors[i]._request_dose_cmd)
			time.sleep(0.2)
			output=self._ser.read(66)#Amount of bytes to read, size of the expected answer

			return map(hex,map(ord,output))
		except:
			return None

	def decode_answer(self,answer,i):

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
		self._sensors[i]._Hp0_dose_decoded = int(Hp0_dose_reversed,16)*self._conv_indx
		self._sensors[i]._Hp1_dose_decoded = int(Hp1_dose_reversed,16)*self._conv_indx
		#dose_rate = (self._sensors[i]._Hp0_dose_decoded-previousHp0)*1800
		#print dose_rate		
		previousHp0 = self._sensors[i]._Hp0_dose_decoded
		dose_rateHp0 = (previousHp0 - self._sensors[i]._Hp0_dose_decoded)*1800
		previousHp1 = self._sensors[i]._Hp1_dose_decoded
		dose_rateHp1 = (previousHp1 - self._sensors[i]._Hp1_dose_decoded)*1800
		#print dose_rate
		#previousHp1 = 
		#print self._sensors[i]._Hp0_dose_decoded
		#print self._sensors[i]._Hp1_dose_decoded
		print self._sensors[i]._Hp0_meaning, "=" , self._sensors[i]._Hp0_dose_decoded, "Dose-rate:", dose_rateHp0, "/h"
		print self._sensors[i]._Hp1_meaning, "=" , self._sensors[i]._Hp1_dose_decoded, "Dose-rate:", dose_rateHp0, "/h"

	def assemble_message(self,i):
		msg = EPD2_Message()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame = "epd"
		msg.data0 = self._sensors[i]._Hp0_dose_decoded
		msg.unit0 = "Sv/h"
		msg.measurement_type0 = self._sensors[i]._Hp0_meaning
		msg.data1 = self._sensors[i]._Hp1_dose_decoded
		msg.unit1 = "Sv/h"
		msg.measurement_type1 = self._sensors[i]._Hp1_meaning
		return msg

	def run(self):
		while not rospy.is_shutdown():
		#while(1):
			for i in range(0,len(self._sensors)):
				answer = self.request_measurement(i)
				if answer is not None:
					try:
						self.decode_answer(answer,i)
						msg = self.assemble_message(i)
						self._publishers[i].publish(msg)
						#Print Result
						print self._sensors[i]._Hp0_meaning+ " Dose: \t" + str(round(self._sensors[i]._Hp0_dose_decoded,2))
						print self._sensors[i]._Hp1_meaning+ " Dose: \t" + str(round(self._sensors[i]._Hp1_dose_decoded,2))
						print "*********************"
						self._r.sleep()
					except:
						pass

				
				self._r.sleep()


def main():

	rospy.init_node("EPD")
	port = rospy.get_param('/EPD/serial_port', '/dev/epd')
	rate = rospy.get_param('/EPD/sample_rate', 0.5)
	#previousHp0 = 0
	#previousHp1 = 0

	epd = EPD2(port,rate)

if __name__ == "__main__":
	# execute only if run as a script
	main()

