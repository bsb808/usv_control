# To be run with ipython --pylab

import pid
reload (pid)

w1 = 1.0  # rad/s
dt = 1.0/100.0
tt = arange(0,2*pi/w1*10,dt)
i1 = 1*sin(w1*tt)+0.1*randn(len(tt))
wc = w1
first = pid.Firstlowpass(wc)
second = pid.Secondbutter(wc)

o1 = zeros(len(tt))
o2 = zeros(len(tt))
for ii in range(len(tt)):
    o1[ii] = first.execute(dt,i1[ii])
    o2[ii] = second.execute(dt,i1[ii])


figure(1)
clf()
plot(tt,i1,label='i1')
hold(True)
plot(tt,o1,linewidth=2,label='o1')
plot(tt,o2,linewidth=2,label='o2')
legend()
title('w1=%.2f rad/s'%w1)
xlim([0,2*2.0*pi/w1])
show()

