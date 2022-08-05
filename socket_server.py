import socket
import struct


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('127.0.0.1', 30000))
print('listendaf')
server_socket.listen(5)



(client_socket, address) = server_socket.accept()
print('client is connected (%s,%s)' % (address[0], address[1]))

buf = client_socket.recv(1024)
print(len(buf), buf)
print(struct.unpack('BBBffff',buf))
buf = client_socket.recv(1024)
print(len(buf), buf)
print(struct.unpack('BBBfff',buf))


