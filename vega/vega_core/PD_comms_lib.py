# /*
#   Power Distribution (PD_control) V1 comms python implementation
#
#   Implementation of the comms protocol with the PD_Control V1 board
#
#   Code by:
#   Bart Garcia-Nathan<bart.garcia.nathan@gmail.com>
#   Oct 2020
# */
#Imports------------------------------------------------------------------------
import serial
import time

#Constants definitions----------------------------------------------------------
get = "GET"
set = "SET"
set_ackno = "{ACK}"
get_ackno = ",ACK}"
who_I_am_string = "PD_CONTROL"

#Register address---------------------------------------------------------------
Who_I_am_addrs            =1 #//Address used to read and ensure correct comms
Front_Light_PWM_addrs     =2 #//Controlable from a SET type of command
Back_Light_PWM_addrs      =3 #//Controlable from a SET type of command
CAM1_Light_PWM_addrs      =4 #//Controlable from a SET type of command
CAM2_Light_PWM_addrs      =5 #//Controlable from a SET type of command
CAM1_12V_enable_addrs     =6 #//Controlable from a SET type of command
CAM2_12V_enable_addrs     =7 #//Controlable from a SET type of command
USB1_power_enable_addrs   =8 #//Controlable from a SET type of command
USB2_power_enable_addrs   =9 #//Controlable from a SET type of command
USB3_power_enable_addrs   =10#//Controlable from a SET type of command
USB5_power_enable_addrs   =11#//Controlable from a SET type of command
USB6_power_enable_addrs   =12#//Controlable from a SET type of command
LED_onboard_addrs         =13#//Controlable from a SET type of command
current_12V_instant_addrs =14#//Controlable from a GET type of command
current_12V_total_addrs   =15#//Controlable from a GET type of command
current_5V_instant_addrs  =16#//Controlable from a GET type of command
current_5V_total_addrs    =17#//Controlable from a GET type of command



#PD_Comms protocol descriptor---------------------------------------------------
class PD_comms_protocol:
    def __init__(self,usb_address,debug):
        self.usb_address = usb_address
        self.debug = debug
        self.serial_baudrate = 115200
        #Serial string_object
        self.ser = serial.Serial(self.usb_address,self.serial_baudrate,timeout=0.5)  # open serial port

    def test_comms(self):
        self.send_cmd(get,Who_I_am_addrs,0)
        time.sleep(0.01)
        msg=self.receive_response_from_get()
        if (msg == who_I_am_string):
            if(self.debug):
                print "Found PD_Control board!"
            return True
        else:
            if(self.debug):
                print "Error! No PD_Control found"
            return False
    def set_pwm_value(self, reg_addrs, payload):
        self.send_cmd(set,reg_addrs,payload)
        self.receive_response_from_set()

    def power_enable(self, reg_addrs):
        self.send_cmd(set,reg_addrs,1)
        self.receive_response_from_set()

    def power_disable(self, reg_addrs):
        self.send_cmd(set,reg_addrs,0)
        self.receive_response_from_set()

    def read_register(self,reg_addrs):
        self.send_cmd(get,reg_addrs,0)
        return self.receive_response_from_get()


    def send_cmd(self,type,reg_address,payload):
        if(type == get):
            message = "{"+get+","+(format(reg_address, '02'))+"}"
            self.ser.write(message)
            if(self.debug):
                print "Sent >>"+message
        if(type == set):
            message = "{"+set+","+(format(reg_address, '02'))+","+str(payload)+"}"
            self.ser.write(message)
            if(self.debug):
                print "Sent >>"+message

    def receive_response_from_set(self):
        resp = self.ser.readline()
        if(resp[0:5]!=set_ackno):
            print "ERROR!"
        elif(self.debug):
            print "Received >> ACK"

    def receive_response_from_get(self):
        resp = self.ser.readline()
        #check for errors
        if(resp.find(get_ackno)):
            start=resp.find("{")+1 #1 is the lenght of the pattern searched for
            end = resp.find(",")
            if(self.debug):
                print "Received payload >>"+resp[start:end]
            return resp[start:end]
        else :
            print "ERROR!" #TODO : an error code handling could be added

