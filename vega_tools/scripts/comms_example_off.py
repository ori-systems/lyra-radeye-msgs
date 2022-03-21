import PD_comms_lib as comms_lib


comms = comms_lib.PD_comms_protocol('/dev/PD',False)
if (not(comms.test_comms())):
    print "The board was not found, double check the connection and the usb port"
    exit()

#Set the PWM value of one of the lights.The valid value is between 0 and 255.
#The possible registers to control are:
#comms_lib.Front_Light_PWM_addrs
#comms_lib.Back_Light_PWM_addrs
#comms_lib.CAM1_Light_PWM_addrs
#comms_lib.CAM2_Light_PWM_addrs
comms.set_pwm_value(comms_lib.Front_Light_PWM_addrs,0)
comms.set_pwm_value(comms_lib.Back_Light_PWM_addrs,0)
comms.set_pwm_value(comms_lib.CAM1_Light_PWM_addrs,0)
comms.set_pwm_value(comms_lib.CAM2_Light_PWM_addrs,0)

#Enable the power to one of the power Controlable devices. The list is:
#comms_lib.CAM1_12V_enable_addrs
#comms_lib.CAM2_12V_enable_addrs
#comms_lib.USB1_power_enable_addrs
#comms_lib.USB2_power_enable_addrs
#comms_lib.USB3_power_enable_addrs
#comms_lib.USB5_power_enable_addrs
#comms_lib.USB6_power_enable_addrs
#comms_lib.LED_onboard_addrs
#comms.power_enable(comms_lib.CAM1_12V_enable_addrs);

#Disable power from one of the power controlable devices, The list is:
#comms_lib.CAM1_12V_enable_addrs
#comms_lib.CAM2_12V_enable_addrs
#comms_lib.USB1_power_enable_addrs
#comms_lib.USB2_power_enable_addrs
#comms_lib.USB3_power_enable_addrs
#comms_lib.USB5_power_enable_addrs
#comms_lib.USB6_power_enable_addrs
#comms_lib.LED_onboard_addrs
#comms.power_disable(comms_lib.CAM1_12V_enable_addrs);

#Read the value of a register, used to read the power consumption registers
#The possible list is:
#comms_lib.current_12V_instant_addrs
#comms_lib.current_12V_total_addrs
#comms_lib.current_5V_instant_addrs
#comms_lib.current_5V_total_addrs
print "The value for the instant current usage in the 12V rail is:"
print(comms.read_register(comms_lib.current_12V_instant_addrs) + " mA")
