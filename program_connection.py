from re import T
import socket
import struct
import math
import numpy as np
from skyfield.api import load, wgs84
from astropy import units as u
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5  # Low-level frames

class SatellitePacketManage():
    def __init__(self, satellite_num, ip, port, cord = 'icrs'):
        self.satellite_num = satellite_num

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((ip, port))

        self.cord_system = cord

        transform = SkyCoord(x=1, y=1, z=1, frame = self.cord_system, representation_type = 'cartesian', unit = 'kpc').transform_to('itrs')
        vec1 = np.array([1.,1.,1.])
        vec2 = np.array([float(transform.x / u.kpc),float(transform.y / u.kpc),float(transform.z / u.kpc)])
        
        self.dcm_eci_to_ecef = self.rotation_matrix_from_vectors(vec1, vec2)  
        #print(self.dcm_eci_to_ecef)

        self.send(self.time_packet(2022,8,19,10,30,0))
        self.send(self.attitude_packet([0,0,0,1]))
        self.send(self.location_packet([0,0,0]))
        self.send(self.satellite_end_packet())

    def send(self,data):
        self.client_socket.send(data)
    
    def close(self):
        self.client_socket.close()

    def time_packet(self,year, month, day, hour, min, sec):

        return struct.pack('4Bi5B',0x96, 0x00, self.satellite_num, 0x00, year, month, day, hour, min, sec)

    def attitude_packet(self,quaternion):

        if self.cord_system == 'itrs':
            q1, q2, q3, q4 = quaternion

        else:
            dcm = self.q_t_d(quaternion)

            result_dcm = dcm @ self.dcm_eci_to_ecef

            quaternion = self.d_t_q(result_dcm)
            quaternion = quaternion / np.linalg.norm(quaternion)
            q1, q2, q3, q4 = quaternion

        return struct.pack('4B4f',0x96,0x00, self.satellite_num,0x02, q1, q2, q3, q4)

    def location_packet(self,location):
        x, y, z = location
        
        transform = SkyCoord(x=x, y=y, z=z, frame = self.cord_system, representation_type = 'cartesian', unit = 'kpc').transform_to('itrs')
        x = float(transform.x / u.kpc)
        y = float(transform.y / u.kpc)
        z = float(transform.z / u.kpc)
        print(x, y, z)
        print(location)

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

    def q_t_d(self, quaternion):
        q1, q2, q3,q4 = quaternion

        return np.array([[(q4**2 + q1**2 -q2**2-q3**2), 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)],
                        [2*(q1*q2-q3*q4), (q4**2-q1**2 + q2**2 - q3**2), 2*(q2*q3 + q1*q4)],
                        [2*(q1*q3+q2*q4), 2*(q2*q3 - q1*q4), (q4**2 - q1**2 - q2**2 + q3**2)]])

    def d_t_q(self, dcm):
        li = [1+dcm[0,0]+dcm[1,1]+dcm[2,2], 1+dcm[0,0]-dcm[1,1]-dcm[2,2], 1-dcm[0,0]+dcm[1,1]-dcm[2,2], 1-dcm[0,0]-dcm[1,1]+dcm[2,2]]

        idx, max_num = 0, -1
        for i in range(len(li)):
            if li[i] > max_num:
                idx, max_num = i, li[i]

        if idx == 0:
            q4 = 0.5*math.sqrt(1+dcm[0,0]+dcm[1,1]+dcm[2,2])
            q1 = 0.25*(dcm[1,2] - dcm[2,1]) / q4
            q2 = 0.25*(dcm[2,0] - dcm[0,2]) / q4
            q3 = 0.25*(dcm[0,1] - dcm[1,0]) / q4

        elif idx == 1:
            q1 = 0.5*math.sqrt(1+dcm[0,0]-dcm[1,1]-dcm[2,2])
            q2 = 0.25*(dcm[0,1] + dcm[1,0]) / q1 
            q3 = 0.25*(dcm[2,0] + dcm[0,2]) / q1
            q4 = 0.25*(dcm[1,2] - dcm[2,1]) / q1
            
        elif idx == 2:
            q2 = 0.5*math.sqrt(1-dcm[0,0]+dcm[1,1]-dcm[2,2])
            q3 = 0.25*(dcm[1,2] + dcm[2,1]) / q2
            q4 = 0.25*(dcm[2,0] - dcm[0,2]) / q2
            q1 = 0.25*(dcm[0,1] + dcm[1,0]) / q2

        elif idx == 3:
            q3 = 0.5*math.sqrt(1-dcm[0,0]-dcm[1,1]+dcm[2,2])
            q1 = 0.25*(dcm[0,2] + dcm[2,0]) / q3
            q2 = 0.25*(dcm[2,1] + dcm[1,2]) / q3
            q4 = 0.25*(dcm[0,1] - dcm[1,0]) / q3  

        return np.array([q1,q2,q3,q4])  


    def rotation_matrix_from_vectors(self,vec1, vec2):
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix