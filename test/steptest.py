# To be run with ipython --pylab
from pylab import *
import pypid
reload (pypid)

close('all')

w1 = 1.0  # rad/s
dt = 1.0/100.0

# Define the plant
# Continuous time
#  G(s) = K/(s((tau)s+1))
# state is {vel,pos}
K = 0.1
tau = 1.0
A = array([[-1/tau,0],[1,0]])
B = array([K/tau,0])
# Discrete time
AA = A*dt+eye(A.shape[0])
BB = B*dt

# Simulate
tt = arange(0,20,dt)

# pid obj
wc = 100/(2*pi);  # cut-off freq for filters

Kp = 100.0
Ki = 1.0
Kd = 50.0

# dict list of controllers to test
Conts={}
# Compare standard with deriv feedback
Conts['Pstandard'] = pypid.Pid(Kp,Ki,Kd)
Conts['Pderivfeedback'] = pypid.Pid(Kp,Ki,Kd)
Conts['Pderivfeedback'].set_derivfeedback(True)
Conts['Pderivfeedbackdfilt'] = pypid.Pid(Kp,Ki,Kd)
Conts['Pderivfeedbackdfilt'].set_derivfeedback(True)
Conts['Pderivfeedbackdfilt'].set_derivfilter(2,wc)
Conts['Pderivfeedbackbothfilt'] = pypid.Pid(Kp,Ki,Kd)
Conts['Pderivfeedbackbothfilt'].set_derivfeedback(True)
Conts['Pderivfeedbackbothfilt'].set_derivfilter(2,wc)
Conts['Pderivfeedbackbothfilt'].set_inputfilter(2,wc)

'''
# Velocity Controllers
Kp = 10.0
Ki = 10.0
Kd = 1.0

Conts['Vstandard'] = pypid.Pid(Kp,Ki,Kd)
Conts['Vderivfeedback'] = pypid.Pid(Kp,Ki,Kd)
Conts['Vderivfeedback'].set_derivfeedback(True)
Conts['Vderivfeedbackdfilt'] = pypid.Pid(Kp,Ki,Kd)
Conts['Vderivfeedbackdfilt'].set_derivfeedback(True)
Conts['Vderivfeedbackdfilt'].set_derivfilter(2,wc)
Conts['Vderivfeedbackbothfilt'] = pypid.Pid(Kp,Ki,Kd)
Conts['Vderivfeedbackbothfilt'].set_derivfeedback(True)
Conts['Vderivfeedbackbothfilt'].set_derivfilter(2,wc)
Conts['Vderivfeedbackbothfilt'].set_inputfilter(2,wc)

Conts['Vwindup'] = pypid.Pid(Kp,Ki,Kd)
Conts['Vwindup'].set_derivfeedback(True)
Conts['Vwindup'].set_derivfilter(2,wc)
Conts['Vwindup'].set_inputfilter(2,wc)
Conts['Vwindup'].set_maxIout(2.0)
'''

#cont0.set_inputfilter(2,wc)
#cont0.set_derivfilter(2,wc)
    
# State allocation
XXol = zeros((len(tt),AA.shape[0]))
XXX = {}  # states for each controller
CCC = {}  # PID outputs for each
# Initial states
X0 = 0.0
V0 = 0.0
for kk in Conts.keys():
    XXX[kk]=zeros((len(tt),AA.shape[0]))
    XXX[kk][0,0]=V0
    XXX[kk][0,1]=X0
    CCC[kk]=zeros((len(tt),4))
    XXX[kk]
# Input allocation
UU = zeros(len(tt))

# Unit step - setpoint
t0 = 1.0
t1 = 10.0
# Amplitude of steps
U0 = X0  # inital setpoint
U1 = 1.0  # step 1
U2 = -1.0 # step 2
for ii in range(1,len(tt)):
    u = U0
    if tt[ii] > t0:
        u = U1
    if tt[ii] > t1:
        u = U2
    UU[ii] = u
    # Open loop
    XXol[ii,:] = dot(AA,XXol[ii-1,:])+dot(BB,u)
    # Closed loop

    for jj in Conts.keys():
        Conts[jj].set_setpoint(u)
        x = XXX[jj][ii-1,1]  # position feedback
        x = x+0.1*randn()
        v = XXX[jj][ii-1,0]  # velocity feedback
        v += 0.1*randn()
        # Which variable to control: posiiton or velocity
        if jj[0] == 'V':
            xx = v
        else:
            xx = x
        CCC[jj][ii,:]=Conts[jj].execute(dt,xx)
        #CCC[jj][ii,:]=Conts[jj].execute(dt,x,v)  # test inputs for pos and vel
        XXX[jj][ii,:] = dot(AA,XXX[jj][ii-1,:])+dot(BB,CCC[jj][ii,0])

for jj in Conts.keys():
    tstr = 'Controller %s: Step Response'%jj
    fig=figure(tstr,figsize=(8.125,10.5))
    #fig.get_size_inches()*fig.dpi
    #fig.set_size_inches([8.125,10.5])
    clf()
    subplot(311)
    plot(tt,XXol[:,1],label='OL pos')
    hold(True)
    plot(tt,XXX[jj][:,1],label='CL pos')
    legend()
    title(tstr)
    subplot(312)
    plot(tt,XXol[:,0],label='OL vel')
    hold(True)
    plot(tt,XXX[jj][:,0],label='CL vel')
    legend()
    title('PID w1=%.2f rad/s'%w1)

    #tstr = 'Controller %s: PID Outputs'%jj
    #figure(tstr)
    #clf()
    subplot(313)
    plot(tt,CCC[jj][:,0],label='PID')
    plot(tt,CCC[jj][:,1],label='P')
    plot(tt,CCC[jj][:,2],label='I')
    plot(tt,CCC[jj][:,3],label='D')
    title(tstr)
    legend()

show()
