#!/usr/bin/python
# -*- coding: UTF-8 -*-

from socket import *
import time

client = socket(AF_INET, SOCK_STREAM)
ip_port = ('192.168.2.156', 3629)
client.connect(ip_port)
while True:
    msg = "Espressif_IDF_".strip()
    client.send(msg.encode('utf-8'))
    data = client.recv(2048).decode()
    if not data:
        break
    print(data)
    time.sleep(0.1)
client.close()
