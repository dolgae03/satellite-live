import socket
import struct
import time
import numpy
import math
import numpy as np
from skyfield.api import load, wgs84, EarthSatellite
from program_connection import *

def lat_lon_at(satellite, ti, earth_radius):
    geo = satellite.at(ti)

    lat, lon = wgs84.latlon_of(geo)
    lat, lon = lat.arcminutes()/60, lon.arcminutes()/60
    height = wgs84.height_of(geo).km

    return lat_lon_rotation(lat, lon, np.array([earth_radius + height, 0, 0]).T)

def lat_lon_rotation(lat, lon, vector):
    s = math.sin(lat * math.pi / 180)
    c = math.cos(lat * math.pi / 180)
    dcm_y = np.array([[c, 0, -s],
                      [0, 1, 0],
                      [s, 0, c]])

    s = math.sin(lon * math.pi / 180)
    c = math.cos(lon * math.pi / 180)
    dcm_z = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])

    res = (dcm_z @ (dcm_y @ vector))

    return res.T

earth_radius = 6378.1
loca = [np.array([earth_radius*1.3, 0, 0]), np.array([0, 8000, 0])]
quaternion = [0,(1/2)**(1/2),0,(1/2)**(1/2)]

stations_url = 'https://celestrak.org/NORAD/elements/gp.php?INTDES=2022-072'
satellites = load.tle_file(stations_url, reload=True)
print('Loaded', len(satellites), 'satellites')

line1 = 'DS-EO'
line2 = '1 52935U 22072A   22216.15420356 -.00001113  00000+0 -14437-3 0  9995'
line3 = '2 52935   9.9958 183.1620 000691  87.5122 272.5811 14.99607335  5188'

ts = load.timescale()
t = ts.now()
i=0
time_gap = ts.utc(2022,8,19,10,28,range(0,120))
packet_list = []
sat = SatellitePacketManage(1,'127.0.0.1',30000, 'itrs')


sat.send(sat.tle_packet(line1, line2, line3))
satellite = EarthSatellite(line2, line3, line1, ts)
sat.send(sat.view_degree_packet(5))
sat.send(sat.tle_end_packet())
#time.sleep(10)

print(satellite)

for ti in time_gap:
    loc = lat_lon_at(satellite, ti, earth_radius)
    a = np.random.randint(10, size=4) 
    a = a / np.linalg.norm(a)

    y, m, d = int(ti.utc.year), int(ti.utc.month), int(ti.utc.day)
    h, mi, se = int(ti.utc.hour), int(ti.utc.minute), int(ti.utc.second)
    p1 = sat.time_packet(y, m, d, h, mi, se)
    p2 = sat.attitude_packet(a)
    p3 = sat.location_packet(loc)
    p4 = sat.vector_packet('helasdlo','purple',[1,1,1])
    p5 = sat.vector_packet('heldlo','yellow',[1,1,0])
    p6 = sat.satellite_end_packet()
    print(ti.utc_strftime(),loc)

    for pac in (p1, p2, p3, p4,p5,p6):
        sat.send(pac)

    time.sleep(1)

sat.close()