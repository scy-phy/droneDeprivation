import uhd
import numpy as np
import time
import socket

address = '127.0.0.1' #replace with your server address

# create an INET, STREAMing socket
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# bind the socket to a public host, and a well-known port
serversocket.bind((address, 8040))
# become a server socket
serversocket.listen(5)

usrp = uhd.usrp.MultiUSRP()
samples = 0.1*np.random.randn(10_000) + 0.1j*np.random.randn(10_000)
print(samples.shape)

samples = 1*np.ones(10_000) + 0j*np.zeros(10_000)


bits = 5
duration = (160e-6)*bits
center_freq = 160e6
center_freqs = [160, 320, 870]
center_freqs= [126]

sample_rate = 1e6
gain = 1
gains = [60]
while True:
    for center_freq in center_freqs:   
        center_freq =  center_freq*1e6
        for gain in gains:
            print(center_freq/1e6, gain)
            # accept connections from outside
            (clientsocket, address) = serversocket.accept()
            usrp.send_waveform(samples, duration, center_freq, sample_rate, [0], gain)
