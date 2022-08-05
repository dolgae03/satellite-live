import socket
import time

from skyfield.api import Loader, wgs84, EarthSatellite

import pyvista as pv
import math
import numpy as np
import struct

from threading import Thread, Lock

class satrac_info:
    def __init__(self, t, location, attitude, vector):
        self.t = t
        self.location = location
        self.attitude = attitude
        self.vector = vector
        
        
class program_manage:
    def __init__(self, gs_location, sat_location):
        self.program_mode = 'live'

        self.load = Loader('./data')
        self.gs_location = gs_location
        self.sat_location = sat_location
        self.actor = {'satellite_axis':[],'earth_axis':[], 'satellite_vector':{}, 'satellite_dark_trajectory':[], 'satellite_bright_trajectory':[]}
        self.polydata = {}

        for key in self.actor.keys():
            self.actor[key]

        self.earth_radius = 6378.1

        self.eph = self.load('de421.bsp')

        self.tle = ['','','']
        self.lock = Lock()
        self.ts = self.load.timescale()

        self.trajectory_active = False
        self.satellite_active = False
        self.satellite_vector_active = False
        self.earth_axis_active = False
        self.time_gap = None
        self.now_idx = -1
        

    def start_program(self):
        global gs_mesh
        global recieve_time, recieve_location, recieve_attitude, recieve_vec
        global data_idx, live_idx

        sphere = pv.Sphere(radius=self.earth_radius, theta_resolution=240, phi_resolution=240, start_theta=270.0001, end_theta=270)
        sphere.active_t_coords = np.zeros((sphere.points.shape[0], 2))

        for i in range(sphere.points.shape[0]):
            x, y, z = sphere.points[i, 0]/self.earth_radius, sphere.points[i, 1]/self.earth_radius, sphere.points[i, 2]/self.earth_radius
            x, y, z = self.normalize(x), self.normalize(y), self.normalize(z)
                
            sphere.active_t_coords[i] = [0.5 + math.atan2(-x, y)/(2 * math.pi), 0.5 + math.asin(z)/math.pi]
            
        sphere.rotate_z(-90)

        self.pl = pv.Plotter()
        self.actor['backgroud'] = self.pl.add_background_image('./data/starry-night-sky-fit.jpg', scale=1.001)
        self.actor['earth'] = self.pl.add_mesh(sphere, texture=pv.read_texture("./data/earth.jpg"), smooth_shading=False)
        
        self.color = ['red','green','blue','yellow','purple','orange', 'white', 'skyblue']
        self.sat_dcm = np.array([[1,0,0],[0,1,0],[0,0,1]])

        basis = np.array([[500,0,0],
                            [0,500,0],
                            [0,0,500]])
        sat_att = (self.sat_dcm @ basis).T

        reader = pv.get_reader('./data/antenna.stl')
        gs_mesh = reader.read()
        gs_scale = 2
        gs_dcm = rotation_matrix_from_vectors(np.array([0,1,0]), np.array(gs_location))
        
        '''
        reader = pv.get_reader('./data/satellite.stl')
        self.polydata['satellite'] = reader.read()
        self.sat_scale = 30
        '''

        for i in range(gs_mesh.points.shape[0]):
            gs_mesh.points[i] = (gs_dcm @ (gs_mesh.points[i].T)).T * gs_scale + gs_location
        '''
        for i in range(self.polydata['satellite'].points.shape[0]):
            self.polydata['satellite'].points[i] = ((self.sat_dcm @ (self.polydata['satellite'].points[i].T)).T)*self.sat_scale + self.sat_location
        '''
        self.actor['groundstation'] = self.pl.add_mesh(gs_mesh)
        #self.actor['satellite'] = self.pl.add_mesh(self.polydata['satellite'], name = 'satellite')

        for i in range(3):
            self.polydata['satellite_axis%d' % i] = pv.PolyData(np.array([self.sat_location + sat_att[i],self.sat_location]), lines=np.array([2,0,1]))
            self.actor['satellite_axis'].append(self.pl.add_mesh(self.polydata['satellite_axis%d' % i], line_width = 1.5, color = self.color[i], name = 'satellite_axis%d' % i))

        for i in range(3):
            self.polydata['earth_axis%d' % i] = pv.PolyData(np.array([[0,0,0], basis[i]/500*self.earth_radius*1.3]), lines=np.array([2,0,1]))
            self.actor['earth_axis'].append(self.pl.add_mesh(self.polydata['earth_axis%d' % i],line_width = 1.5, color = self.color[i], name = 'earth_axis%d' % i))

        actor_bright = []
        actor_dark = []

        for i in range(5):
            self.polydata['satellite_bright_trajectory%d' % i] = pv.PolyData(np.array([[0,0,0],[0,0,0]]), lines = np.array([2,0,1]))
            actor_bright.append(self.pl.add_mesh(self.polydata['satellite_bright_trajectory%d' % i], line_width = 1.5, color = 'white'))

        for i in range(5):
            self.polydata['satellite_dark_trajectory%d' % i] = pv.PolyData(np.array([[0,0,0],[0,0,0]]), lines = np.array([2,0,1]))
            actor_dark.append(self.pl.add_mesh(self.polydata['satellite_dark_trajectory%d' % i], line_width = 1.5, color = 'gray'))

        self.actor['satellite_dark_trajectory'] = actor_dark
        self.actor['satellite_bright_trajectory'] = actor_bright

        self.pl.add_key_event('t',self.trajectory_callback)
        self.pl.add_key_event('s',self.satellite_callback)
        self.pl.add_key_event('v',self.satellite_vector_callback)
        self.pl.add_key_event('e',self.earth_axis_callback)
        self.pl.add_key_event('n',self.prev_callback)
        self.pl.add_key_event('m',self.next_callback)
        self.pl.add_key_event('l',self.live_update_mode)
        self.pl.add_key_event('k',self.view_mode)

        self.pl.show(interactive_update=True)

        prev_time = time.time()

        while True:
            if live_idx < len(data_queue)-1 and self.program_mode == 'live':
                live_idx += 1
                self.now_idx = data_idx = live_idx
                self.update_satellite()
                print(live_idx)

            if tle_update != None and self.program_mode == 'live':
                print('hi')
                self.update_trajectory()
            
            if self.program_mode == 'view' and self.now_idx != data_idx:
                live_idx = self.now_idx = data_idx
                
                self.update_satellite()


            self.pl.update(stime = 1, force_redraw = False)

            if time.time() - prev_time > 1:
            
                prev_time = time.time()
                self.pl.update()

    def trajectory_callback(self):
        print('key pushed t')
        
        if self.trajectory_active:
            self.lock.acquire()
            for ele in self.actor['satellite_dark_trajectory']:
                self.pl.remove_actor(ele)
            for ele in self.actor['satellite_bright_trajectory']:
                self.pl.remove_actor(ele)
            self.lock.release()
            self.trajectory_active = False
        else :
            self.lock.acquire()
            for ele in self.actor['satellite_dark_trajectory']:
                self.pl.add_actor(ele)
            for ele in self.actor['satellite_bright_trajectory']:
                self.pl.add_actor(ele)
            self.lock.release()
            self.trajectory_active = True
    def satellite_vector_callback(self):
        if self.satellite_vector_active :
            self.lock.acquire()
            for ele in self.actor['satellite_vector'].values():
                self.pl.remove_actor(ele)
            self.lock.release()
            self.satellite_vector_active = False
        else :
            self.lock.acquire()
            for ele in self.actor['satellite_vector'].values():
                self.pl.add_actor(ele)
            self.lock.release()
            self.satellite_vector_active = True
    def satellite_callback(self):
        if self.satellite_active :
            for ele in self.actor['satellite_axis']:
                self.pl.remove_actor(ele)
            self.satellite_active = False
        else :
            for ele in self.actor['satellite_axis']:
                self.pl.add_actor(ele)
            self.satellite_active = True
    def earth_axis_callback(self):
        if self.earth_axis_active:
            for ele in self.actor['earth_axis']:
                self.pl.remove_actor(ele)
            self.earth_axis_active = False
        else :
            for ele in self.actor['earth_axis']:
                self.pl.add_actor(ele)
            self.earth_axis_active = True
    def prev_callback(self):
        if self.view_mode == 'live':
            return

        global data_idx
        
        data_idx = max(data_idx-1, 0)
        print(data_idx)

    def next_callback(self):
        if self.view_mode == 'live':
            return
        
        global data_idx
        
        data_idx = min(data_idx+1, len(data_queue)-1)
        print(data_idx)

    def live_update_mode(self):
        self.program_mode = 'live'
    def view_mode(self):
        self.program_mode = 'view'
            

    def update_satellite(self):
        global data_update, data_queue, data_idx

        if self.program_mode == 'live':
            data_update = data_queue[live_idx]
        else :
            data_update = data_queue[data_idx]
        #all_data = {}

        #print('hi')
        basis = np.array([[500,0,0],
                          [0,500,0],
                          [0,0,500]])

        #all_data['time'] = self.ts.utc(*tuple(data_update.t))
    
        self.now_time = self.ts.utc(*tuple(data_update.t))
        self.end_time = self.ts.utc(*tuple(data_update.t[:5] + [data_update.t[5] + 10000]))
        self.time_gap = self.ts.utc(*tuple(data_update.t[:5] + [range(data_update.t[5],data_update.t[5] + 10000)]))
        
        self.sat_location = data_update.location
        self.sat_vector = data_update.vector

        self.sat_dcm = self.q_t_d(data_update.attitude)
        sat_att = (self.sat_dcm @ basis).T

        '''
        for i in range(self.polydata['satellite'].points.shape[0]):
            self.polydata['satellite'].points[i] = ((self.sat_dcm @ (self.polydata['satellite'].points[i].T)).T)*self.sat_scale + self.sat_location
        '''

        for i in range(3):
            self.polydata['satellite_axis%d' % i].points = np.array([self.sat_location + sat_att[i],self.sat_location])
            #all_data['satellite_axis%d' % i] = np.array([self.sat_location + sat_att[i],self.sat_location])
        for (name, color, vector), i in zip(self.sat_vector, range(len(self.sat_vector))):
            if 'satellite_vector(%s)' % name in self.polydata:
                self.polydata['satellite_vector(%s)' % name].points = np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T])
                #all_data['satellite_vector(%s)' % name] = np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T])
            else:    
                self.polydata['satellite_vector(%s)' % name] = pv.PolyData(np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T]), lines=np.array([2,0,1]))
                self.actor['satellite_vector'][name] = (self.pl.add_mesh(self.polydata['satellite_vector(%s)' % name], line_width = 1.5, color = color, name = 'satellite_vector%d' % i))
                #all_data['satellite_vector(%s)' % name] = np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T])
        
    
    def update_trajectory(self):
        global tle_update
        if self.time_gap == None:
            return

        self.satellite = EarthSatellite(tle_update[1], tle_update[2], tle_update[0], self.ts)
        print(self.satellite)

        sat_trac = [[],[]]
        sat_trac_bright = []
        sat_trac_dark = []

        key = True

        for ti in self.time_gap:
            loc = self.lat_lon_at(self.satellite, ti, self.earth_radius)
            sunlit = self.satellite.at(ti).is_sunlit(self.eph)
        
            if sunlit :
                if key == False:
                    if len(sat_trac_dark) : 
                        sat_trac_dark.append(loc)
                        sat_trac[1].append(np.array(sat_trac_dark))
                        sat_trac_dark = []
                    
                    key = True
                sat_trac_bright.append(loc)
        
            else:
                if key == True:
                    if len(sat_trac_bright) :
                        sat_trac_bright.append(loc)
                        sat_trac[0].append(np.array(sat_trac_bright))
                        sat_trac_bright = []

                    key = False
                sat_trac_dark.append(loc)

        if key == True:
            sat_trac[0].append(np.array(sat_trac_bright))
        else :
            sat_trac[1].append(np.array(sat_trac_dark))

        actor_dark, actor_bright = [], []

        for ele in self.actor['satellite_dark_trajectory']:
            self.pl.remove_actor(ele)

        for ele in self.actor['satellite_bright_trajectory']:
            self.pl.remove_actor(ele)

        for ele, i in zip(sat_trac[0],range(len(sat_trac[0]))):
            self.polydata['satellite_bright_trajectory%d' % i].points = ele
            self.polydata['satellite_bright_trajectory%d' % i].lines = [[2,j,j+1] for j in range(len(ele)-1)]

        for ele, i in zip(sat_trac[1],range(len(sat_trac[1]))):
            self.polydata['satellite_dark_trajectory%d' % i].points = ele
            self.polydata['satellite_dark_trajectory%d' % i].lines = [[2,j,j+1] for j in range(len(ele)-1)]

        tle_update = None

    def normalize(self, value):
        if value > 1:
            return 1
        elif value < -1:
            return -1
        
        return value

    def q_t_d(self, quaternion):
        q1, q2, q3,q4 = quaternion

        return np.array([[(q4**2 + q1**2 -q2**2-q3**2), 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)],
                        [2*(q1*q2-q3*q4), (q4**2-q1**2 + q2**2 - q3**2), 2*(q2*q3 + q1*q4)],
                        [2*(q1*q3+q2*q4), 2*(q2*q3 - q1*q4), (q4**2 - q1**2 - q2**2 + q3**2)]])

    def lat_lon_at(self, satellite, ti, earth_radius):
        geo = satellite.at(ti)

        lat, lon = wgs84.latlon_of(geo)
        lat, lon = lat.arcminutes()/60, lon.arcminutes()/60
        height = wgs84.height_of(geo).km

        return self.lat_lon_rotation(lat, lon, np.array([earth_radius + height, 0, 0]).T)

    def lat_lon_rotation(self, lat, lon, vector):
        s = math.sin(lat * math.pi / 180)
        c = math.cos(lat * math.pi / 180)
        dcm_y = np.array([[c, 0, s],
                        [0, 1, 0],
                        [-s, 0, c]])

        s = math.sin(lon * math.pi / 180)
        c = math.cos(lon * math.pi / 180)
        dcm_z = np.array([[c, -s, 0],
                        [s, c, 0],
                        [0, 0, 1]])

        res = (dcm_z @ (dcm_y @ vector))

        return res.T

def rotation_matrix_from_vectors(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def data_receiving(client_socket,adress):
    #buf = [0 for i in range(1000)]
    
    while True:
        try: 
            buf = client_socket.recv(1024)
        except:
            client_socket.close()
            return 

        if buf == b'':
            client_socket.close()
            return

        data_processing(buf)


#0, 1 시작 개시 2번 통신 코드
#0번은 time
#1번은 location
#2번은 attitude
#3번은 vector
#4번은 이를 적용
#0번은 

def data_processing(buf):
    global recieve_time, recieve_location, recieve_attitude, recieve_vec, recieve_tle
    global data_queue, data_update, tle_update
    

    while len(buf) > 1:
        #print(buf)
        if buf[0] != 0x96 or buf[1] !=0x00:
            return

        if buf[2] == 0x00:
            _, _, _, year, month, day, hour, min, sec = struct.unpack('3Bi5B', buf[:13])
            recieve_time = [year, month, day, hour, min, sec]
            #print(len(buf),buf,recieve_time)
            buf = buf[13:]
        elif buf[2] == 0x01:
            _, _, _, x, y, z = struct.unpack('3B3f',buf[:16])
            recieve_location = np.array([x, y ,z])
            #print(len(buf),buf, recieve_location)
            buf = buf[16:]
        elif buf[2] == 0x02:
            _, _, _, q1, q2, q3, q4 = struct.unpack('3B4f',buf[:20])
            recieve_attitude = np.array([q1, q2, q3, q4])
            #print(len(buf),buf, recieve_attitude)
            buf = buf[20:]
        elif buf[2] == 0x03:
            #print(buf,len(buf[:18 + buf[3] + buf[4]]))
            _, _, _, _, _,  x, y, z, name, color = struct.unpack(('5B3f%ds%ds' % (buf[3],buf[4])),buf[:20 + buf[3] + buf[4]])
            
            recieve_vec.append((name.decode(),color.decode(),np.array([x, y ,z])))
            #print(len(buf),buf, recieve_vec)
            buf = buf[20 + buf[3] + buf[4]:]
        elif buf[2] == 0x04:
            data_queue.append(satrac_info(recieve_time, recieve_location, recieve_attitude, recieve_vec))
            
            #print(data_queue, data_update)
            
            recieve_time = None
            recieve_location = None
            recieve_attitude = None
            recieve_vec = []
            buf = buf[3:]

        elif buf[2] == 0x05:
            _, _, _, _, _, _, line1, line2, line3 = struct.unpack(('3B3B%ds%ds%ds' % (buf[3],buf[4],buf[5])),buf[:6+buf[3]+buf[4]+buf[5]])
            tle_update = [line1.decode(), line2.decode(), line3.decode()]
            buf = buf[6+buf[3]+buf[4]+buf[5]:]
        
        else :
            buf = []

def take_client_connection():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 30000))
    server_socket.listen(5)


    while True:
        (client_socket, address) = server_socket.accept()

        th1 = Thread(target=data_receiving, args=(client_socket, address), daemon = True)
        th1.start()


recieve_time = None
recieve_location = None
recieve_attitude = None
recieve_vec = []
recieve_tle = None

tle_update = None

data_queue = []
data_idx = -1
live_idx = -1

gs_location = [6378.1,0,0]

#reader = pv.get_reader('./data/satellite.stl')
#sat_mesh = reader.read()
#sat_scale = 30

th1 = Thread(target=take_client_connection, daemon= True)
th1.daemon = True
th1.start()


new_program = program_manage(gs_location, np.array([6378.1*1.3, 0, 0]))
new_program.start_program()