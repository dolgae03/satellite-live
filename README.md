Program for Initial Operation of the Satellite
===================

## **개요**
  본 프로그램은 위성의 초기 운용시에 위성의 위치 및 정보를 보여주기 위한 프로그램이다. 위성의 위치를 표현하기 위해서 ECEF 좌표계를 이용했으며, 프로그램이 지원하는 기능은 다음과 같다.

1. 여러 위성의 Trajectory 및 Attitude 표현 
2. 여러 위성의 벡터(안테나, 카메라) 표현
3. 여러 위성의 궤도 및 궤도 상의 통신 가능 영역 표현
4. 여러 위성의 태양 노출 여부 표시
5. 지상국 위치 표시
6. 각 Component 표시 여부 선택 가능<br/><br/>

## **사용된 모듈**
### **Pyvista**
  Pyvista는 3D 그래픽을 쉽게 나타내도록 하는 도구이다. 따라서 이를 이용하면 원하는 그림을 쉽게 그릴 수 있다. 본 프로그램에서 사용하는 모든 기능은 pyvista의 visualization 기능을 이용해서 구현하였다. Pyvista는 Plotter를 이용해서 모든 그래픽을 표현한다. 따라서 PolyData를 Plotter에 추가하여 원하는 그래픽을 나타낼 수 있다. Plotter에 add를 이용해서 추가하게 되면 Actor가 형성되게 된다. 이러한 Actor는 Polydata의 값을 기반으로 그래픽을 표현한다. Plotter는 interactive_update를 True로 설정하면 업데이트가 가능하며, 본 프로그램에서는 True로 되어있다. 따라서 **Plotter에 Add 되어있는 Actor의 PolyData의 값을 수정함으로써 그래픽을 변화시켰다.**
### **Skyfield**
  Skyfield는 wgs84, EarthSatellite는 tlefm 데이터를 받음으로써 이러한 문제를 해결하는데 활용하였다.
### **Etc**
  그 외에는 math, numpy, struct, socket, time 등이 사용되었다. 자세한 버전 정보는 requirements.txt에서 확인할 수 있다.<br/><br/>


## **프로그램 구성 및 구현**
  본 프로그램은 서버, 클라이언트로 구성되어 있다. 서버는 이러한 그래픽을 제공하고, 클라이언트는 위성을 나타내기 위한 여러가지 정보를 서버에 전송한다. 어떤 방식을 사용하더라도, 편하게 이용하기 위해서 socket통신을 이용해서 구현하였다. 기본적으로 30000번 포트와 localhost를 통해서 접속이 가능하다. 본 프로그램에서 제공하는 program_connection의 내장 class를 이용해서 클라이언트의 기능을 제공한다. 프로그램의 구현에 관련된 자세한 설명은 아래에 분리하여 서술한다.<br/>
### **서버** 
서버가 실행될 때 program_manage라는 class를 만든다. 해당 class가 프로그램의 모든 것을 담당한다. class가 init될 때 take_client_connection 함수를 새로운 Thread로 시작한다. 이후 take_client_connection 함수는 해당 주소로 접속하는 여러 클라이언트에서 들어오는 연결을 받아주는 역할을 한다.
```python
gs_location = (1.2833, 103.85)

new_program = program_manage(gs_location)
new_program.start_program()
```
```python

def take_client_connection(self):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 30000))
    server_socket.listen(5)

    while True:
        (client_socket, address) = server_socket.accept()

        th1 = Thread(target=self.data_receiving, args=(client_socket, address), daemon = True)
        th1.start()
```
이후 해당 class의 start_program을 실행하면 각 요소들을 생성한다. 이러한 요소들은 update가 가능해야하기 때문에 program_manage에서 저장해놓고 관리한다. 따라서 본 class의 actor, polydata는 dictionary type으로 각각 actor와 polydata를 저장하여 관리한다.

아래는 지구 객체를 만드는 과정이다. 지구의 Polydata를 만든 후 plotter를 선언하고, 배경과 지구를 추가한다. 이후에 antenna.stl 객체를 불러온 후에 지상국의 위치에 안테나 객체를 표시한다. 실제로 add_mesh등이 actor를 반환하므로, 이를 따로 저장해서 관리한다.

**지구**
```python
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
```

**지상국**
```python
gs_mesh = reader.read()
gs_scale = 2
gs_dcm = self.rotation_matrix_from_vectors(np.array([0,1,0]), self.gs_location)

for i in range(gs_mesh.points.shape[0]):
    gs_mesh.points[i] = (gs_dcm @ (gs_mesh.points[i].T)).T * gs_scale + self.gs_location

self.actor['groundstation'] = self.pl.add_mesh(gs_mesh)

self.actor['main_text'] = self.pl.add_text('-',name = 'main_text')
```

**지구 Axis**
```python
for i in range(3):
    self.polydata['earth_axis%d' % i] = pv.PolyData(np.array([[0,0,0], basis[i]/500*self.earth_radius*1.3]), lines=np.array([2,0,1]))
    self.actor['earth_axis'].append(self.pl.add_mesh(self.polydata['earth_axis%d' % i],line_width = 1.5, color = self.color[i], name = 'earth_axis%d' % i))
```

이후 add_key_event를 추가한 후에 Callback 함수를 연동시켜 다양한 기능을 제공할 수 있도록 하였다. 아래는 Callback 함수에서 추가된 actor를 나타냈다가 지우는 로직이다.

```python
def satellite_vector_callback(self):
    if self.control_satellite_num not in self.added_satellite:
        return 

    sat = self.added_satellite[self.control_satellite_num]

    if sat.satellite_vector_active :
        self.remove_actor_list(self.pl, sat.actor['satellite_vector'].values())
        sat.satellite_vector_active = False
    else :
        self.add_actor_list(self.pl, sat.actor['satellite_vector'].values())
        sat.satellite_vector_active = True
```

이후에 기본 설정이 끝난 이후에는 무한 루프에 들어가게 된다. interactive_update가 True로 설정되어 있으면, 주기적으로 plotter를 업데이트 해주어야만 프로그램이 정상적으로 실행될 수 있다. **여러 위성이 동시에 plot될 수 있기 때문에 control_satellite_num을 기준으로 모든 위성이 plot된다. 즉, control_satellite_num의 시간을 기준으로 다른 위성들의 위치, 자세 정보가 모두 동기화 된다.** 

해당 반복문에서는 지속적으로 데이터 업데이트를 확인한 후 그래픽을 업데이트 한다. **이러한 데이터 업데이트는 객체에 존재하는 client_recieve에 저장된다. client_recieve는 satellite_num을 key로 가지며 아래의 dictionary를 value로 가지는 수신 buffer이다.** 
```python
{'time':None, 'location':None, 'attitude':None,
'vector':[], 'data_queue':[],
'data_idx':-1, 'live_idx':-1,
'tle_update':None, 'tle':None,
'degree':None}
```
이후 무한루프는 위 수신 buffer를 확인하면서 여러가지 기능을 수행한다.

공통적으로는 
1. 현재 기준 satellite_num이 존재하는 satellite_num인지 여부
2. tle_update에 값이 있는지 여부
3. 새로운 satellite_num이 있는지 여부

view_mode가 live라면 
1. 새로운 data set이 들어왔는지 여부(data_queue의 길이가 바뀌었는지 여부)

view_mode가 static이라면 
1. key press에 따른 satellite 정보가 반영 되었는지 여부(idx로 판단)

먼저 새로운 satellite_num이 들어온 경우에는 program_satellite 객체를 만들어서 added_satellite에 추가한다. program_satellite 객체는 각자 satellite가 그래픽으로 구현되는데 필요한 actor, polydata를 관리하며, init될 때 이러한 객체를 미리 plotter에 추가하여, Polydata를 변경함으로써 데이터 변화를 나타낸다.

이후에 각 상황에 맞추어서 update_satellite, update_trajectory, update_communication 함수를 실행하여 Ploydata를 수정하는 함수를 호출한다. 어떤 상황에서 어떤 함수를 호출하는지 아래 코드에서 확인하면 이해할 수 있다.

```Python
while True:
    if self.control_satellite_num in self.client_recieve:
        a = list(self.client_recieve.keys()).copy()
        b = list(self.client_recieve.values()).copy()

        now_sat = self.client_recieve[self.control_satellite_num]

        for satellite_num, each_client in zip(a,b):
            if satellite_num not in self.added_satellite:
                self.added_satellite[satellite_num] = program_satellite(satellite_num,np.array([0,0,0]),self.pl)
            
            if each_client['tle_update'] != None:
                self.update_trajectory(satellite_num)
                self.update_communication(satellite_num)

        if now_sat['live_idx'] < len(now_sat['data_queue'])-1 and self.program_mode == 'live':
            self.now_idx = self.data_idx = now_sat['live_idx']
            
            for satellite_num, each_client in zip(a,b):
                each_client['live_idx'] = len(each_client['data_queue'])-1
                self.update_satellite(satellite_num)
            
        if self.program_mode == 'view' and self.now_idx != self.data_idx:
            for satellite_num, each_client in zip(a,b):
                self.update_satellite(satellite_num)
            
            self.now_idx = self.data_idx

    if time.time() - prev_time > 0.05:           
        prev_time = time.time()
        self.pl.update()
```

**위성 자세, 위치 업데이트(update_satellite)**</br>
update_satellite에서는 먼저 data_update를 설정한다. data_update는 satrac_info를 가진다. satrac_info는 아래와 같이 정의되어 있는 객체이다. 시간, 위치, 자세, 백터(list) 정보를 가지고 있다.
```python
class satrac_info:
    def __init__(self, t, location, attitude, vector):
        self.t = t
        self.location = location
        self.attitude = attitude
        self.vector = vector
```
t는 (Year,Month,Day,Hour,Minute,Second)인 Tuple을 데이터 형이다. location은 1x3 ndarray, 단위는 km이다. attitude는 1x4 ndarray, quaternion으로 저장한다. vector는 list로 각 element는 (벡터 이름,벡터 색깔, 1x3 ndarray)를 가진다. 벡터의 방향은 위성의 벡터를 기준으로 표현한다.</br>
이후에 이 데이터를 토대로 program_satellite가 가지고 있는 polydata dictionary에서 각각의 Polydata를 바꾸어줌으로써 위성을 표현한다.</br></br>
**위성 궤도 업데이트(update_trajectory)**</br>
마찬가지의 원리로 구현되어 있는데, 위성의 궤도를 그리면서 해에 노출된 영역과 노출되지 않은 영역이 계속해서 발생하므로, program_satellite 객체에서 각각 5개의 actor를 만들고, polydata의 값을 변화시켜서 선을 표현하는 방식으로 구현하였다. 나머지는 skyfiled의 satellite가 제공하는 기능을 이용하여 시간에 따른 위성의 위치를 계산하고, 이를 표현한다.</br>
**위성 통신 가능 영역 업데이트(update_communication)**</br>
위성 궤도 업데이트와 마찬가지로 skyfield에 구현되어 있는 find_evnets함수를 활용하여 latlon의 시점을 찾고 이를 바탕으로 다른 색으로 궤도를 색칠하여 구현하였다.

### **클라이언트** 
클라이언트는 사용자의 필요에 따라서 Live로 데이터를 입력할 수 있도록 socket통신을 
이용하였다. socekt 통신을 이용함으로써 어떤 프로그램에서도 모두 이 server를 이용할 수 있도록 하였으며, Live로 데이터를 전송하는 것도 가능하다. 이를 편하게 하기 위해서 program_connection.py를 이용하면 된다.
```Python
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

        self.send(self.time_packet(2022,8,5,0,0,0))
        self.send(self.attitude_packet([0,0,0,1]))
        self.send(self.location_packet([0,0,0]))
        self.send(self.satellite_end_packet())
```
해당 객체는 socket 통신을 이용해서 정보를 전송한다. 또한 좌표계 변환도 자동으로 진행하게 되는데, cord에 원하는 좌표계를 넣게 되면, Astropy.coordinates의 SkyCoord 모듈을 이용해서 이를 자동으로 변환하여 위치나 자세를 연산한다. program_connection 모듈에는 정보를 입력하면 자동으로 패킷을 만들어주는 method가 많이 구현되어 있다. 예를 들면 아래와 같다.
```python
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

    return struct.pack('4B3f',0x96,0x00,self.satellite_num,0x01,x,y,z)
```
해당 모듈을 보면 location과 quaternion을 전송하게 되면, 처음 init할 때 설정했던 좌표계를 인식하여 이를 itrs로 바꾸게 된다. 각각의 method를 이용해서 패킷을 형성해줄 수 있고, 이를 서버에 전송한 이후에 end_packet을 만들어서 보내면 이를 업데이트 하게 된다. **여기서 중요한 것은 end_packet을 보내기 전까지는 각자의  data가 업데이트 되지 않는다. tle_end_packet은 tle 업데이트를 진행하며, satellite_end_packet은 시간, 위치, 자세, 벡터 업데이트를 진행한다.**


### **데이터 프로토콜**
서버와 클라이언트 사이에 사용하는 데이터 프로토콜이다. 

**시간 전송**
|Byte|0|1|2|3|4~7|9|10|11|12|13|
|-|-|-|-|-|-|-|-|-|-|-|
|Num|0x96|0x00|0x00|Num|Val|Val|Val|Val|Val|Val|
|의미|시작|시작|시간|위성 번호|년|월|일|시|분|초

**위치 전송**
|Byte|0|1|2|3|4~7|8~11|12~15
|-|-|-|-|-|-|-|-|
|Num|0x96|0x00|0x01|Num|Val|Val|Val|
|의미|시작|시작|위치|위성 번호|x|y|z|

**자세 전송**
|Byte|0|1|2|3|4~7|8~11|12~15|16~19|
|-|-|-|-|-|-|-|-|-|
|Num|0x96|0x00|0x02|Num|Val|Val|Val|Val|
|의미|시작|시작|자세|위성 번호|q1|q2|q3|q4|

**벡터 전송**
|Byte|0|1|2|3|4|5|6~9|10~13|14~17|18~18+A-1|18+A~18+A+B-1|
|-|-|-|-|-|-|-|-|-|-|-|-|
|Num|0x96|0x00|0x03|Num|A|B|Val|Val|Val|Val|Val|Val|
|의미|시작|시작|자세|위성 번호|Name size|Color size|u|v|w|Name|Color|

이 외의 백터는 program_connection.py에 있는 struct 함수를 통해서 이해할 수 있다. 이를 참고하면 된다.

## **프로그램 사용법**
![Alt text](./data/example.png)
먼저 프로그램 실행 화면은 아래와 같다. 해당 프로그램에서 RGB 순으로 x, y, z축을 나타낸다. 현재 프로그램으로는 2대 이상의 위성 표현이 가능하다. 왼쪽 위에서 현재 날짜와 시간을 알 수 있다. 하얀 선은 태양에 있을 때, 회색 선은 태양에 가려질 때를 나타낸다. 좌표는 x축은 위도와 경도가 0인 지점, z축은 지구의 자전축, y축은 x축과 y축의 벡터 곱이다. 해당 프로그램은 마우스를 이용해서 시점을 변경할 수 있으며 Shift와 좌측 마우스를 클릭하여 평행이동, 스크롤로 크기를 변경할 수 있다.

1, 2, 3, 4, 5를 입력해서 자신이 원하는 위성에 timescale을 동기화할 수 있다. 예를 들어 2개의 위성 중 1번 위성의 시간에 맞추어서 위 프로그램이 업데이트 되고 있다면, 2를 눌러서 2번 위성의 시간에 맞추어 프로그램이 업데이트 되도록 할 수 있다.

T: 위성의 궤도를 숨기고 나타나개 할 수 있다.</br>
C: 통신 가능 범위를 숨기고 나타나게 할 수 있다.</br>
S: 위성의 축을 숨기고 나타나게 할 수 있다.</br>
V: 위성의 벡터를 숨기고 나타나게 할 수 있다.</br>
E: 지구의 축을 숨기고 나나타게 할 수 있다.</br>
K, L: K는 static mode, L은 live mode로 설정한다.</br>
N, M: N은 prev state로 M은 next state로 이동한다.(static 상태에서만 유효)

해당 프로그램을 다른 것 없이 사용해보기 위한 과정은 다음과 같다.
1. 해당 repository에 들어가서 satellite_program.py 실행
2. server_client.py 실행

제작자: 9574m@kaist.ac.kr