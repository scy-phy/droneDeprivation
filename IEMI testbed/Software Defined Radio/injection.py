import uhd
import numpy as np

usrp = uhd.usrp.MultiUSRP()
samples = 0.1*np.random.randn(10_000) + 0.1j*np.random.randn(10_000)
print(samples.shape)

samples = 1*np.ones(10_000) + 0j*np.zeros(10_000)

bits = 5
duration = (160e-6)*bits
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
            usrp.send_waveform(samples, duration, center_freq, sample_rate, [0], gain)
