from http import client
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

class program_satellite:
    def __init__(self,satellite_num, sat_location, pl):
        self.sat_location = sat_location
        self.satellite = None
        self.satellite_num = satellite_num
        self.pl = pl

        self.actor = {'satellite_axis':[],'earth_axis':[], 'satellite_vector':{}, 'satellite_dark_trajectory':[], 'satellite_bright_trajectory':[]}
        self.polydata = {}

        self.tle = ['','','']

        self.now_idx = -1

        self.trajectory_active = False
        self.satellite_active = False
        self.satellite_vector_active = False
        self.earth_axis_active = True
        self.communcation_active = False

        actor_bright = []
        actor_dark = []
        actor_communication = []

        self.color = ['red','green','blue','yellow','purple','orange', 'white', 'skyblue']
        self.sat_dcm = np.array([[1,0,0],[0,1,0],[0,0,1]])

        basis = np.array([[500,0,0],
                            [0,500,0],
                            [0,0,500]])
        sat_att = (self.sat_dcm @ basis).T


        for i in range(3):
            self.polydata['satellite_axis%d' % i] = pv.PolyData(np.array([self.sat_location + sat_att[i],self.sat_location]), lines=np.array([2,0,1]))
            self.actor['satellite_axis'].append(self.pl.add_mesh(self.polydata['satellite_axis%d' % i], line_width = 1.5, color = self.color[i]))

        for i in range(5):
            self.polydata['satellite_bright_trajectory%d' % i] = pv.PolyData(np.array([[0,0,0],[0,0,0]]), lines = np.array([2,0,1]))
            actor_bright.append(self.pl.add_mesh(self.polydata['satellite_bright_trajectory%d' % i], line_width = 1.5, color = 'white'))

        for i in range(5):
            self.polydata['satellite_dark_trajectory%d' % i] = pv.PolyData(np.array([[0,0,0],[0,0,0]]), lines = np.array([2,0,1]))
            actor_dark.append(self.pl.add_mesh(self.polydata['satellite_dark_trajectory%d' % i], line_width = 1.5, color = 'gray'))

        for i in range(5):
            self.polydata['satellite_communication_trajectory%d' % i] = pv.PolyData(np.array([[0,0,0],[0,0,0]]), lines = np.array([2,0,1]))
            actor_communication.append(self.pl.add_mesh(self.polydata['satellite_communication_trajectory%d' % i], line_width = 4.0, color = 'yellow'))

        self.actor['satellite_dark_trajectory'] = actor_dark
        self.actor['satellite_bright_trajectory'] = actor_bright
        self.actor['satellite_communication_trajectory'] = actor_communication

        self.time_gap = None
        self.load = Loader('./data')
        self.ts = self.load.timescale()

class internal_function:
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

    def rotation_matrix_from_vectors(self,vec1, vec2):
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix
    
        
        
class program_manage(internal_function):
    def __init__(self, gs_location, sat_location):
        self.program_mode = 'live'
        self.client_recieve = {}

        self.load = Loader('./data')
        self.gs_location = gs_location
        self.earth_radius = 6378.1
        self.eph = self.load('de421.bsp')
        
        self.data_idx = -1

        self.control_satellite_num = 1


        ##############
        self.sat_location = sat_location
        self.actor = {'satellite_axis':[],'earth_axis':[], 'satellite_vector':{}, 'satellite_dark_trajectory':[], 'satellite_bright_trajectory':[]}
        self.polydata = {}

        self.tle = ['','','']

        self.now_idx = -1

        self.added_satellite = {}

        self.trajectory_active = False
        self.satellite_active = False
        self.satellite_vector_active = False
        self.earth_axis_active = True
        self.communcation_active = False
        ##############

        self.lock = Lock()
        self.ts = self.load.timescale()

        self.working = False
        self.time_gap = None

    def start_program(self):
        global gs_mesh

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
        gs_dcm = self.rotation_matrix_from_vectors(np.array([0,1,0]), np.array(gs_location))
        
        self.actor['main_text'] = self.pl.add_text('-',name = 'main_text')

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
            self.polydata['earth_axis%d' % i] = pv.PolyData(np.array([[0,0,0], basis[i]/500*self.earth_radius*1.3]), lines=np.array([2,0,1]))
            self.actor['earth_axis'].append(self.pl.add_mesh(self.polydata['earth_axis%d' % i],line_width = 1.5, color = self.color[i], name = 'earth_axis%d' % i))

        self.pl.add_key_event('t',self.trajectory_callback)
        self.pl.add_key_event('s',self.satellite_callback)
        self.pl.add_key_event('v',self.satellite_vector_callback)
        self.pl.add_key_event('e',self.earth_axis_callback)
        self.pl.add_key_event('c',self.communication_callback)
        self.pl.add_key_event('n',self.prev_callback)
        self.pl.add_key_event('m',self.next_callback)
        self.pl.add_key_event('l',self.live_mode)
        self.pl.add_key_event('k',self.static_mode)

        self.pl.show(interactive_update=True)

        prev_time = time.time()

        while True:
            a = list(self.client_recieve.keys()).copy()
            b = list(self.client_recieve.values()).copy()
            for satellite_num, each_client in zip(a,b):
                if satellite_num not in self.added_satellite:
                    self.added_satellite[satellite_num] = program_satellite(satellite_num,np.array([0,0,0]),self.pl)
                    print(satellite_num)
                if each_client['live_idx'] < len(each_client['data_queue'])-1 and self.program_mode == 'live':
                    each_client['live_idx'] = len(each_client['data_queue'])-1
                    self.now_idx = self.data_idx = each_client['live_idx']
                    self.update_satellite(satellite_num)

                if each_client['tle_update'] != None and self.program_mode == 'live':
                    print('hi', satellite_num)
                    self.update_trajectory(satellite_num)
                    print('ddd')
                    self.update_communication(satellite_num)
                
                if self.program_mode == 'view' and self.now_idx != self.data_idx:
                    self.now_idx = self.data_idx
                    self.update_satellite(satellite_num)

            if time.time() - prev_time > 0.05:           
                prev_time = time.time()
                self.pl.update()

    def trajectory_callback(self):
        print('key pushed t')
        
        if self.trajectory_active:
            #self.working = True
            for ele1, ele2 in zip(self.actor['satellite_dark_trajectory'],self.actor['satellite_bright_trajectory']):
                self.pl.remove_actor(ele1)
                self.pl.remove_actor(ele2)
            #self.working = False
            self.trajectory_active = False
        else :
            #self.working = True
            for ele1, ele2 in zip(self.actor['satellite_dark_trajectory'],self.actor['satellite_bright_trajectory']):
                self.pl.add_actor(ele1)
                self.pl.add_actor(ele2)
            #self.working = False
            self.trajectory_active = True
    def satellite_vector_callback(self):
        if self.satellite_vector_active :
            for ele in self.actor['satellite_vector'].values():
                self.pl.remove_actor(ele)
            self.satellite_vector_active = False
        else :
            for ele in self.actor['satellite_vector'].values():
                self.pl.add_actor(ele)
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
    def communication_callback(self):
        
        if self.communcation_active:
            for ele in self.actor['satellite_communication_trajectory']:
                self.pl.remove_actor(ele)
            self.communcation_active = False
        else :
            for ele in self.actor['satellite_communication_trajectory']:
                self.pl.add_actor(ele)
            self.communcation_active = True    
    def prev_callback(self):
        if self.program_mode == 'live':
            return

        self.data_idx = max(self.data_idx-1, 0)
    def next_callback(self):
        if self.program_mode == 'live':
            return
        
        self.data_idx = self.data_idx + 1
    def live_mode(self):
        self.program_mode = 'live'
    def static_mode(self):
        self.program_mode = 'view'


    def update_satellite(self, satellite_num):
        each_client = self.client_recieve[satellite_num]
        now_sat = self.added_satellite[satellite_num]

        print(id(now_sat),id(self.added_satellite[satellite_num]))
        
        live_idx = each_client['live_idx']
        data_idx = min(len(each_client['data_queue'])-1,self.data_idx+1)
        data_queue = each_client['data_queue']

        if self.program_mode == 'live':
            data_update = data_queue[live_idx]
        else :
            data_update = data_queue[data_idx]
        #all_data = {}


        now_sat.actor['main_text'] = self.pl.add_text(("%d/%d/%d    %d:%d:%d" % tuple(data_update.t)),name = 'main_text')
        print(now_sat.satellite)

        #print('hi')
        basis = np.array([[500,0,0],
                          [0,500,0],
                          [0,0,500]])

        #all_data['time'] = self.ts.utc(*tuple(data_update.t))
    
        now_sat.now_time = self.ts.utc(*tuple(data_update.t))
        now_sat.end_time = self.ts.utc(*tuple(data_update.t[:5] + [data_update.t[5] + 10000]))
        now_sat.time_gap = self.ts.utc(*tuple(data_update.t[:5] + [range(data_update.t[5],data_update.t[5] + 6000)]))
        
        now_sat.sat_location = data_update.location
        now_sat.sat_vector = data_update.vector

        now_sat.sat_dcm = self.q_t_d(data_update.attitude)
        sat_att = (now_sat.sat_dcm @ basis).T

        '''
        for i in range(self.polydata['satellite'].points.shape[0]):
            self.polydata['satellite'].points[i] = ((self.sat_dcm @ (self.polydata['satellite'].points[i].T)).T)*self.sat_scale + self.sat_location
        '''

        for i in range(3):
            now_sat.polydata['satellite_axis%d' % i].points = np.array([now_sat.sat_location + sat_att[i],now_sat.sat_location])
            #all_data['satellite_axis%d' % i] = np.array([self.sat_location + sat_att[i],self.sat_location])
        
        for (name, color, vector), i in zip(now_sat.sat_vector, range(len(now_sat.sat_vector))):
            if 'satellite_vector(%s)' % name in now_sat.polydata:
                now_sat.polydata['satellite_vector(%s)' % name].points = np.array([now_sat.sat_location , now_sat.sat_location + 400*(now_sat.sat_dcm@((vector).T)).T])
                #all_data['satellite_vector(%s)' % name] = np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T])
            else:    
                now_sat.polydata['satellite_vector(%s)' % name] = pv.PolyData(np.array([now_sat.sat_location , now_sat.sat_location + 400*(now_sat.sat_dcm@((vector).T)).T]), lines=np.array([2,0,1]))
                now_sat.actor['satellite_vector'][name] = (self.pl.add_mesh(now_sat.polydata['satellite_vector(%s)' % name], line_width = 1.5, color = color, name = 'satellite_vector%d' % i))
                #all_data['satellite_vector(%s)' % name] = np.array([self.sat_location , self.sat_location + 400*(self.sat_dcm@((vector).T)).T])
        
        print(id(now_sat),id(self.added_satellite[satellite_num]))

    def update_trajectory(self,satellite_num):
        now_sat = self.added_satellite[satellite_num]
        each_client = self.client_recieve[satellite_num]

        tle_update = each_client['tle_update']
        if now_sat.time_gap == None:
            return

        print(tle_update)
        print(EarthSatellite(tle_update[0][1], tle_update[0][2], tle_update[0][0], now_sat.ts))
        now_sat.satellite = EarthSatellite(tle_update[0][1], tle_update[0][2], tle_update[0][0], now_sat.ts)
        print(now_sat.satellite)

        sat_trac = [[],[]]
        sat_trac_bright = []
        sat_trac_dark = []

        key = True

        for ti in now_sat.time_gap:
            loc = self.lat_lon_at(now_sat.satellite, ti, self.earth_radius)
            sunlit = now_sat.satellite.at(ti).is_sunlit(self.eph)
        
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

        for ele, i in zip(sat_trac[0],range(len(sat_trac[0]))):
            now_sat.polydata['satellite_bright_trajectory%d' % i].points = ele
            now_sat.polydata['satellite_bright_trajectory%d' % i].lines = [[2,j,j+1] for j in range(len(ele)-1)]

        for ele, i in zip(sat_trac[1],range(len(sat_trac[1]))):
            now_sat.polydata['satellite_dark_trajectory%d' % i].points = ele
            now_sat.polydata['satellite_dark_trajectory%d' % i].lines = [[2,j,j+1] for j in range(len(ele)-1)]

    def update_communication(self, satellite_num):
        now_sat = self.added_satellite[satellite_num]
        each_client = self.client_recieve[satellite_num]

        tle_update = each_client['tle_update']
        now_sat.view_degree = tle_update[1]

        lat, lon = math.asin(self.normalize(self.gs_location[2]/self.earth_radius)), math.atan2(self.gs_location[1], self.gs_location[0])
        
        crisp = wgs84.latlon(lat, lon)
        t_deg, events_deg = now_sat.satellite.find_events(crisp, now_sat.now_time, now_sat.end_time, altitude_degrees=now_sat.view_degree)

        two_time_set = []

        taos = now_sat.now_time
        aos_flag = False
        for ti, event in zip(t_deg, events_deg):
            if event == 0:
                taos = ti

            elif event == 2:
                two_time_set.append([taos, ti])

        if aos_flag == True:
            two_time_set.append([taos, now_sat.end_time])

        time_gap = 1/(24*60*60)
        now_sat.sat_trac_communication = []

        for (start_utc, end_utc), i in zip(two_time_set, range(len(two_time_set))):
            li = []
            while end_utc - start_utc >= 0:
                start_utc += time_gap

                li.append(self.lat_lon_at(now_sat.satellite, start_utc, self.earth_radius))

            now_sat.polydata['satellite_communication_trajectory%d' % i].points = np.array(li)
            now_sat.polydata['satellite_communication_trajectory%d' % i].lines = [[2,j,j+1] for j in range(len(li)-1)]

        each_client['tle_update'] = None

def data_receiving(client_socket,adress):
    #buf = [0 for i in range(1000)]
    
    while True:
        try: 
            buf = client_socket.recv(1024*9)
        except:
            client_socket.close()
            return 

        if buf == b'':
            client_socket.close()
            return

        data_processing(buf, adress)


#0, 1 시작 개시 2번 통신 코드
#0번은 time
#1번은 location
#2번은 attitude
#3번은 vector
#4번은 이를 적용
#0번은 

def data_processing(buf, adress):
    global new_program

    while len(buf) > 1:
        #print(buf)
        if buf[0] != 0x96 or buf[1] !=0x00:
            return
    
        new_program.lock.acquire()
        if buf[2] not in new_program.client_recieve:
            new_program.client_recieve[buf[2]] = {'time':None,
                                                  'location':None,
                                                  'attitude':None,
                                                  'vector':[],
                                                  'data_queue':[],
                                                  'data_idx':-1,
                                                  'live_idx':-1,
                                                  'tle_update':None,
                                                  'tle':None,
                                                  'degree':None}

        if buf[3] == 0x00:
            length = struct.calcsize('4Bi5B')
            _, _, _, _, year, month, day, hour, min, sec = struct.unpack('4Bi5B', buf[:length])
            new_program.client_recieve[buf[2]]['time'] = [year, month, day, hour, min, sec]
            #print(len(buf),buf,recieve_time)
            buf = buf[length:]
        elif buf[3] == 0x01:
            #print(buf)
            length = struct.calcsize('4B3f')
            _, _, _, _, x, y, z = struct.unpack('4B3f',buf[:length])
            new_program.client_recieve[buf[2]]['location'] = np.array([x, y ,z])
            #print(len(buf),buf, client_recieve[buf[2]]['location'])
            buf = buf[length:]
        elif buf[3] == 0x02:
            length = struct.calcsize('4B4f')
            _, _, _, _, q1, q2, q3, q4 = struct.unpack('4B4f',buf[:length])
            new_program.client_recieve[buf[2]]['attitude'] = np.array([q1, q2, q3, q4])
            #print(len(buf),buf, client_recieve[buf[2]]['attitude'])
            buf = buf[length:]
        elif buf[3] == 0x03:
            length = struct.calcsize('6B3f%ds%ds' % (buf[4],buf[5]))
            print(buf,buf[:length])
            _, _, _, _, _, _,  x, y, z, name, color = struct.unpack(('6B3f%ds%ds' % (buf[4],buf[5])),buf[:length])
            
            new_program.client_recieve[buf[2]]['vector'].append((name.decode(),color.decode(),np.array([x, y ,z])))
            #print(len(buf),buf, recieve_vec)
            buf = buf[length:]
        elif buf[3] == 0x04:
            length = struct.calcsize('4B')
            a = new_program.client_recieve[buf[2]]
            new_program.client_recieve[buf[2]]['data_queue'].append(satrac_info(a['time'], a['location'], a['attitude'],a['vector']))
            #print(data_queue, data_update)
            buf = buf[length:]

        elif buf[3] == 0x05:
            length = struct.calcsize('4B3B%ds%ds%ds' % (buf[4],buf[5],buf[6]))
            _, _, _, _, _, _, _, line1, line2, line3 = struct.unpack(('4B3B%ds%ds%ds' % (buf[4],buf[5],buf[6])),buf[:length])
            new_program.client_recieve[buf[2]]['tle'] = [line1.decode(), line2.decode(), line3.decode()]
            
            buf = buf[length:]
           
        
        elif buf[3] == 0x06:
            length = struct.calcsize('5B')
            _, _, _, _, degree = struct.unpack('5B',buf[:length])
            new_program.client_recieve[buf[2]]['degree'] = degree
            buf = buf[length:]

        elif buf[3] == 0x07:
            length = struct.calcsize('5B')
            new_program.client_recieve[buf[2]]['tle_update'] = (new_program.client_recieve[buf[2]]['tle'], new_program.client_recieve[buf[2]]['degree'])
            
            buf = buf[length:]

        else :
            buf = []
        new_program.lock.release()

def take_client_connection():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 30000))
    server_socket.listen(5)

    while True:
        (client_socket, address) = server_socket.accept()

        th1 = Thread(target=data_receiving, args=(client_socket, address), daemon = True)
        th1.start()

gs_location = [6378.1,0,0]

#reader = pv.get_reader('./data/satellite.stl')
#sat_mesh = reader.read()
#sat_scale = 30

th1 = Thread(target=take_client_connection, daemon= True)
th1.daemon = True
th1.start()

new_program = program_manage(gs_location, np.array([6378.1*1.3, 0, 0]))
new_program.start_program()