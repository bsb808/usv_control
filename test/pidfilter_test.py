# To be run with ipython --pylab
from numpy import *
import pid
reload (pid)

w1 = 1.0  # rad/s
dt = 1.0/100.0

# test signal
tt = arange(0,2*pi/w1*10,dt)
i1 = 1*sin(w1*tt)+0.1*randn(len(tt))
wc = w1*5.

# pid obj
cont0 = pid.Pid(1,0,1)
#cont0.set_inputfilter(2,wc)
cont0.set_derivfilter(2,wc)

o0 = zeros((len(tt),4))
o1 = zeros((len(tt),4))
o2 = zeros((len(tt),4))


for ii in range(len(tt)):
    o0[ii,:] = cont0.execute(dt,i1[ii])
    #p1[ii] = cont0.execute(i1[ii],dt)[1]
    #ii1[ii] = cont0.execute(i1[ii],dt)[2]
    #d1[ii] = cont0.execute(i1[ii],dt)[3]


figure(1)
clf()
plot(tt,i1,label='i1')
hold(True)
plot(tt,o0[:,0],label='PID')
plot(tt,o0[:,1],label='P')
plot(tt,o0[:,2],label='I')
plot(tt,o0[:,3],label='D')
#plot(tt,o2,linewidth=2,label='o2')
legend()
title('PID w1=%.2f rad/s'%w1)
xlim([0,2*2.0*pi/w1])
show()

