import socket
import struct
import time
import numpy
import math
import numpy as np
from skyfield.api import load, wgs84

class satellite_packet():
    def __init__(self, satellite_num):
        self.satellite_num = satellite_num

    def time_packet(self,year, month, day, hour, min, sec):

        return struct.pack('4Bi5B',0x96, 0x00, self.satellite_num, 0x00, year, month, day, hour, min, sec)

    def attitude_packet(self,quaternion):
        q1, q2, q3, q4 = quaternion

        return struct.pack('4B4f',0x96,0x00, self.satellite_num,0x02, q1, q2, q3, q4)

    def location_packet(self,location):
        x, y, z = location
        return struct.pack('4B3f',0x96,0x00,self.satellite_num,0x01,x,y,z)

    def vector_packet(self,name, color, vector):
        x, y, z = vector
        name, color = name.encode(), color.encode()

        return struct.pack(('6B3f%ds%ds' % (len(name), len(color))),0x96,0x00,self.satellite_num,0x03,len(name),len(color),x,y,z,name, color)

    def tle_packet(self,line1, line2, line3):
        line1, line2, line3 = line1.encode(), line2.encode(), line3.encode()
        
        return struct.pack(('4B3B%ds%ds%ds' % (len(line1),len(line2), len(line3))),0x96,0x00,self.satellite_num,0x05,len(line1),len(line2),len(line3),line1,line2,line3)

    def view_degree_packet(self,degree):
        
        return struct.pack('5B',0x96,0x00,self.satellite_num,0x06, degree)

    def tle_end_packet(self):
        return struct.pack('4B',0x96,0x00,self.satellite_num,0x07)

    def satellite_end_packet(self):
        return struct.pack('4B',0x96, 0x00,self.satellite_num, 0x04)