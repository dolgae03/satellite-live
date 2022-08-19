import pandas as pd
import time
from program_connection import *

######
data = pd.read_csv('./orbit_and_attitude.csv')

t = data.loc[:,'Current Satellite Time'].to_numpy()
quaternion = data.loc[:,'q_1 of Estimated Quaternion':'q_4 of Estimated Quaternion'].to_numpy()
location = data.loc[:,'X axis of Estimated Position':'Z axis of Estimated Position'].to_numpy()

sat = SatellitePacketManage(1,'127.0.0.1',30000, 'icrs')

packet_list = []

######

for ti, i in zip(t, range(len(t))):
    res = np.array(ti[:10].split('-'))
    y, m, d = int(res[0]), int(res[1]), int(res[2])
    res = np.array(ti[10:].replace(' ','').split(':'))
    h, mi, se = int(res[0]), int(res[1]), 30*(i%2)

    p1 = sat.time_packet(y, m, d, h, mi, se)
    p2 = sat.attitude_packet(quaternion[i,:])
    p3 = sat.location_packet(location[i,:])
    p4 = sat.vector_packet('helasdlo','purple',[1,1,1])
    p5 = sat.satellite_end_packet()

    packet_list.append((p1, p2, p3, p4, p5))



for ele in packet_list:
    #print(ele[0])
    for pac in ele:
        sat.send(pac)

    time.sleep(0.1)

sat.close()