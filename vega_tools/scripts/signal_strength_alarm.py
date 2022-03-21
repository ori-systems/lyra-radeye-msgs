#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from vega_tools.msg import SignalStrength
import rospkg
import sys
import subprocess
import time
import argparse as ap

class Signal_Strength(object):

    def get_name(self,cell):
        return self.matching_line(cell,"ESSID:")[1:-1]

    def get_quality(self,cell):
        try:
		quality = self.matching_line(cell,"Quality=").split()[0].split('/')
        	return str(int(round(float(quality[0]) / float(quality[1]) * 100))).rjust(3) + " %"
	except:
		return self.signal_strength.quality


    def get_channel(self,cell):
        return self.matching_line(cell,"Channel:")

    def get_signal_level(self,cell):
        # Signal level is on same line as Quality data so a bit of ugly
        # hacking needed...
	try:
        	return self.matching_line(cell,"Quality=").split("Signal level=")[1]
	except:
		return self.signal_strength.signal

    def get_encryption(self,cell):
        enc=""
        if self.matching_line(cell,"Encryption key:") == "off":
            enc="Open"
        else:
            for line in cell:
                matching = self.match(line,"IE:")
                if matching!=None:
                    wpa=self.match(matching,"WPA Version ")
                    if wpa!=None:
                        enc="WPA v."+wpa
            if enc=="":
                enc="WEP"
        return enc

    def get_address(self,cell):
        return self.matching_line(cell,"Address: ")

    # Here's a dictionary of rules that will be applied to the description of each
    # cell. The key will be the name of the column in the table. The value is a
    # function defined above.

    def sort_cells(self,cells):
        sortby = "Quality"
        reverse = True
        cells.sort(None, lambda el:el[sortby], reverse)


    # Below here goes the boring stuff. You shouldn't have to edit anything below
    # this point

    def matching_line(self,lines, keyword):
        """Returns the first matching line in a list of lines. See match()"""
        for line in lines:
            matching=self.match(line,keyword)
            if matching!=None:
                return matching
        return None

    def match(self,line,keyword):
        """If the first part of line (modulo blanks) matches keyword,
        returns the end of that line. Otherwise returns None"""
        line=line.lstrip()
        length=len(keyword)
        if line[:length] == keyword:
            return line[length:]
        else:
            return None

    def parse_cell(self,cell):
        """Applies the rules to the bunch of text describing a cell and returns the
        corresponding dictionary"""
        parsed_cell={}
        for key in self._rules:
            rule=self._rules[key]
            parsed_cell.update({key:rule(cell)})
        return parsed_cell

    def print_table(self,table):
        widths=map(max,map(lambda l:map(len,l),zip(*table))) #functional magic

        justified_table = []
        for line in table:
            justified_line=[]
            for i,el in enumerate(line):
                justified_line.append(el.ljust(widths[i]+2))
            justified_table.append(justified_line)
        
        for line in justified_table:
            for el in line:
                print el,
            print

    def print_cells(self,cells):
        table=[]
        for cell in cells:
            cell_properties=[]
            for column in self._columns:
                cell_properties.append(cell[column])
            table.append(cell_properties)
        self.print_table(table)


    def __init__(self,interface,password,nn):

        self._network_name = nn
        self._all_networks = None
        self._my_networks = None
        self._pw = password
        self._interface = interface
        self._columns = ["Name","Address","Quality","Signal", "Channel","Encryption"]
        self._cmd = "sudo -S iwlist " + self._interface + " scan"
        self._rules={"Name":self.get_name,
        "Quality":self.get_quality,
        "Channel":self.get_channel,
        "Encryption":self.get_encryption,
        "Address":self.get_address,
        "Signal":self.get_signal_level
        }
        self.signal_strength = SignalStrength()
        self.signal_pub = rospy.Publisher("/signal_strength",SignalStrength,queue_size=1)

        #self.print_table([self._columns])

    def find_my_network(self):
        self._my_networks = []
        if self._all_networks is not None:
            for i in self._all_networks:
                if self._network_name in i['Name']:
                    self._my_networks.append(i)


    def decode(self):
        for i in self._my_networks:
            try:
                if i["Name"] == self._network_name:
                    self.signal_strength.name = i["Name"]
                    self.signal_strength.quality = int(i["Quality"].split("%")[0])
                    self.signal_strength.signal = int(i["Signal"].split("/")[0])
                    if i["Channel"] is None:
                        self.signal_strength.channel = -1
                    else:
                        self.signal_strength.channel = i["Channel"] 
                    self.signal_strength.header.stamp = rospy.get_rostime()

            except:
                pass




    def run(self):
        
        cells=[[]]
        parsed_cells=[]

        
        proc = subprocess.Popen([self._cmd],shell=True,stdin = subprocess.PIPE,stdout=subprocess.PIPE, universal_newlines=True)
        out, err = proc.communicate('{}\n'.format(self._pw) ) 

        for line in out.split("\n"):
            cell_line = self.match(line,"Cell ")
            if cell_line != None:
                cells.append([])
                line = cell_line[-27:]
            cells[-1].append(line.rstrip())

        cells=cells[1:]

        for cell in cells:
            parsed_cells.append(self.parse_cell(cell))

        self.sort_cells(parsed_cells)

        #self.print_cells(parsed_cells)
        self._all_networks = parsed_cells

        self.find_my_network()
        

        print self._my_networks

        self.decode()

        self.signal_pub.publish(self.signal_strength)


        """
        ADD LOGIC TO CHECK IF SIGNAL IS BAD AND PUBLISH TRUE OR FALSE BASED ON OUTPUT
        """
        #self.link_quality_check()

def main():

    #parser = ap.ArgumentParser()
    #parser.add_argument("pw", help="sudo password required for network checker")
    #args = parser.parse_args()
    #if not args.pw:
    #    print "argument pw (password) must be provided at runtime"
    #    exit(0)
    
    rospy.init_node('signal_strength_alarm', anonymous=True)

    interface = rospy.get_param('network_interface', "wlx74ee2af53258")
    network_name = rospy.get_param('network_id', "Pedro network")

    pw = "ubuntu"

    ss = Signal_Strength(interface,pw,network_name)

    r = rospy.Rate(1) # 1hz 
    while not rospy.is_shutdown():
        ss.run()
        r.sleep()
    

if __name__ == "__main__":
    main()
