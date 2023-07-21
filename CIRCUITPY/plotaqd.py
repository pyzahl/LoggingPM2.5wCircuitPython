#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

aqd=np.transpose(np.loadtxt('aqd.log', comments='#', usecols=(0, 2,3,4, 6,7,8,9,10,11)))
print (aqd)

secs = aqd[0]

t0 = secs[0]
secs = secs-t0
ti=0

tm = np.zeros(secs.shape[0])
sec = np.zeros(secs.shape[0])

tp=0
to=0
i=0
print(secs)
print (tm)
tl = 0
for t in secs:
    if tl > t:
        to = tp
        print(to)
        sec[i]=500
    tm[i] = t+to
    tp = tm[i]
    tl=t
    i=i+1

tm = (tm/60./60.+0) #%24
    
pm01  = aqd[1]
pm025 = aqd[2]
pm10  = aqd[3]
aq03  = aqd[4]
aq05  = aqd[5]
aq10  = aqd[6]
aq25  = aqd[7]
aq50  = aqd[8]
aq100 = aqd[9]

def strided_app(a, L, S ):  # Window len = L, Stride len/stepsize = S
    nrows = ((a.size-L)//S)+1
    n = a.strides[0]
    return np.lib.stride_tricks.as_strided(a, shape=(nrows,L), strides=(S*n,n))

#np.median(strided_app(data, window_len,1),axis=1)

medlen=60

fig, axs = plt.subplots(2, 1)
#axs[0].scatter(tm, pm01, label='PM 1.0μ')
axs[0].scatter(np.median(strided_app(tm, medlen,1),axis=1), np.median(strided_app(pm01, medlen,1),axis=1),label='PM 1.0μ med')
axs[0].scatter(tm, pm025,label='PM 2.5μ raw')
axs[0].scatter(np.median(strided_app(tm, medlen,1),axis=1), np.median(strided_app(pm025, medlen,1),axis=1),label='PM 2.5μ med')
#axs[0].scatter(tm, pm10, label='PM 10μ')
axs[0].scatter(np.median(strided_app(tm, medlen,1),axis=1), np.median(strided_app(pm10, medlen,1),axis=1),label='PM 10μ med')
axs[0].scatter(tm, sec/50, label='Section')
#axs[0].set_xlim(0, 2)
axs[0].set_xlabel('Time in hours')
axs[0].set_ylabel('PM ** standart in μg/m³')
axs[0].grid(True)
axs[0].legend()



#axs[1].scatter(tm, aq03, label='> 0.3μm')
#axs[1].scatter(tm, aq05, label='> 0.5μm')
#axs[1].scatter(tm, aq10, label='> 1.0μm')
#axs[1].scatter(tm, aq25, label='> 2.5μm')
#axs[1].scatter(tm, aq50, label='> 5.0μm')
#axs[1].scatter(tm, aq100, label='> 10μm')

aq50_100 = aq50-aq100
aq25_50  = aq25-aq50
aq10_25  = aq10-aq25
aq05_10  = aq05-aq10
aq03_05  = aq03-aq05

axs[1].scatter(tm, aq03_05, label='0.3-0.5μm')
axs[1].scatter(tm, aq05_10, label='0.5-1.0μm')
axs[1].scatter(tm, aq10_25, label='1.0-2.5μm')
axs[1].scatter(tm, aq25_50, label='2.5-5.0μm')
axs[1].scatter(tm, aq50_100, label='5.0-10μm')

axs[1].scatter(tm, sec, label='Section')
#axs[0].set_xlim(0, 2)
axs[1].set_xlabel('Time in hours')
axs[1].set_ylabel('AQD in #particels > ** μm/0.1L air')
axs[1].grid(True)
axs[1].legend()

fig.tight_layout()
plt.show()
