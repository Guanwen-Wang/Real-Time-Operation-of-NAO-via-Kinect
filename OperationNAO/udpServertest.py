#!/usr/bin/env python
import socket
from time import ctime
 
server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server.bind(("127.0.0.1",8888))
print("udp server start")
while True:
    data,addr = server.recvfrom(1024)
    text = str(data)
    print addr
    print text
    server.sendto("done", addr)
    if text == "quit":
        break
print("udp server finish")
server.close()
