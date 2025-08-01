import uhd
import numpy as np
import time

usrp = uhd.usrp.MultiUSRP()
samples = 0.1*np.random.randn(10_000) + 0.1j*np.random.randn(10_000)
print(samples.shape)

samples = 1*np.ones(10_000) + 0j*np.zeros(10_000)


duration = 1600e-6 # secs
center_freq = 160e6

center_freqs = range(120,150,1)

#center_freqs= [126] #tuned param

sample_rate = 1e6
gain = 1
gains = range(40, 70, 10) #tuned param 60
while True:
    for center_freq in center_freqs:   
        center_freq =  center_freq*1e6#/factor
        for gain in gains:
            print(center_freq/1e6, gain)
            usrp.send_waveform(samples, duration, center_freq, sample_rate, [0], gain)
            time.sleep(1)
