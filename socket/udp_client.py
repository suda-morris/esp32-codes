#!/usr/bin/python
# -*- coding: UTF-8 -*-

from socket import *

client = socket(AF_INET, SOCK_DGRAM)
ip_port = ('192.168.2.156', 3629)
while True:
    msg = raw_input(">> ").strip()
    if not msg:
        break
    client.sendto(msg.encode('utf-8'), ip_port)
    if msg.startswith("quit"):
        break
client.close()
