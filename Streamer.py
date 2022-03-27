import socket

path = 'D:/FFOutput/binary/Frame{}'

server = socket.socket(type=socket.SOCK_DGRAM)
server.bind(('0.0.0.0', 2233))
client = socket.socket(type=socket.SOCK_DGRAM)

for i in range(5478):
    f = open(path.format(i + 1), 'rb')
    for j in range(320):
        server.recv(64)
        data = j.to_bytes(2, 'big') + f.read(480)
        client.sendto(data, ('169.254.82.233', 6666))
    f.close()
