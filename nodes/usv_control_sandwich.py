#!/usr/bin/env python3
'''
Using instances of the pypid Pid class to control yaw and velocity

This is a modified version of usv_control_diff_drive.py specifically for Matt Heubach's oswarm application.

* Setpoints are yaw-rate and forward velocity which arrive through subscrption to a Twist message
* Yaw feedback comes from Imu messages
* Surge is open-loop, using a lookup table.

'''
# Python
import sys
from math import pi
import numpy as np

# ROS
import rospy
import tf
from dynamic_reconfigure.server import Server
from usv_control.cfg import YawDynamicConfig
from usv_control.cfg import TwistDynamicConfig

from usv_control.msg import PidDiagnose

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# From this package
from usv_pid.pypid import Pid


class Node():
    def __init__(self):
        # Setup Yaw Pid
        self.ypid = Pid(0.0, 0.0, 0.0)
        self.ypid.set_setpoint(0.0)
        #self.pid.set_inputisangle(True,pi)
        self.ypid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.ypid.set_derivfilter(1,wc)
        self.ypid.set_maxIout(1.0)
        
        # Initialize some bits as none - for now
        self.drivemsg = None
        self.publisher = None
        self.lasttime = None

        # For diagnosing/tuning PID
        self.ypubdebug = None

        # Initialize velocity setpoint
        self.vsetpoint = 0
        
    def twist_callback(self,msg):
        '''
        Setpoints arrive as Twist messages
        '''
        self.ypid.set_setpoint(msg.angular.z)
        self.vsetpoint = msg.linear.x

    def imu_callback(self,msg):
        # Yaw feedback control
        dyaw = msg.angular_velocity.z  # measured rate (process variable)
        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now-self.lasttime
        self.lasttime = now
        #print("dt: %.6f"%dt)
        if dt < 1.0e-6:
            rospy.logwarn("USV Control dt too small <%f>"%dt)
            return
        
        yout = self.ypid.execute(dt,dyaw)
        torque = yout[0]

        # Velocity open-loop control
        q = [8.787516741419185e-07,-1.797707086229711e-06,0.010252082562086,0.001525282390623,-8.217698480861404e-07]
        thrust = np.polyval(q, self.vsetpoint)

        # I believe drive messages are scaled to -1.0 to 1.0
        # Scale so that no one output saturates
        '''
        mag = abs(torque)+abs(thrust)
        if mag > 1.0:

            torque = torque/mag
            thrust = thrust/mag
        '''

        #rospy.loginfo('Torque: %.3f, Thrust: %.3f'%(torque,thrust))
        '''
        self.drivemsg.left=-1*torque + thrust
        self.drivemsg.right=torque + thrust
        self.publisher.publish(self.drivemsg)
        '''
        self.left_cmd.data = -1.0*torque + thrust
        self.right_cmd.data = torque + thrust
        self.left_publisher.publish(self.left_cmd)
        self.right_publisher.publish(self.right_cmd)

        if not (self.ypubdebug is None):
            self.ydebugmsg.PID = yout[0]
            self.ydebugmsg.P = yout[1]
            self.ydebugmsg.I = yout[2]
            self.ydebugmsg.D = yout[3]
            self.ydebugmsg.Error = yout[4]
            self.ydebugmsg.Setpoint = yout[5]
            self.ydebugmsg.Derivative= yout[6]
            self.ydebugmsg.Integral = yout[7]
            self.ypubdebug.publish(self.ydebugmsg)

    def dynamic_callback(self,config,level):
        #rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
        #  {str_param}, {bool_param}, {size}""".format(**config))
        rospy.loginfo("Reconfigure request...")
        #print config.keys()
        #print config['yawKp']
        self.ypid.Kp = config['yawKp']
        #self.ypid.Ki = config['yawKi']
        # Use method to zero the integrator
        Ki = config['yawKi']
        tol = 1e-6
        if abs(abs(Ki)-abs(self.ypid.Ki)) > tol:
            rospy.loginfo("Setting yaw Ki to %.3f"%Ki)
            self.ypid.set_Ki(Ki)
        self.ypid.Kd = config['yawKd']

        return config
        

if __name__ == '__main__':
    
    rospy.init_node('usv_control_sandwich', anonymous=True)
    
    # ROS Parameters
    yawKp = rospy.get_param('~yawKp',0.0)
    yawKd = rospy.get_param('~yawKd',0.0)
    yawKi = rospy.get_param('~yawKi',0.0)

    # Initiate node object - creates PID object
    node=Node()
    
    # Set initial gains from parameters
    node.ypid.Kp = yawKp
    node.ypid.Kd = yawKd
    node.ypid.Ki = yawKi

    # Setup outbound messages
    node.left_cmd = Float32()
    node.right_cmd = Float32()
    
    # Setup publisher
    node.left_publisher = rospy.Publisher('left_thrust_cmd',
                                          Float32,queue_size=1)
    node.right_publisher = rospy.Publisher('right_thrust_cmd',
                                           Float32,queue_size=1)
    rospy.loginfo("Publishing to %s and %s"%
                  (node.left_publisher.name, node.right_publisher.name))
    node.ypubdebug = rospy.Publisher("yaw_pid_debug",PidDiagnose,queue_size=10)
    node.ydebugmsg = PidDiagnose()

    # Setup subscribers
    s1 = rospy.Subscriber('imu/data', Imu, node.imu_callback)
    s2 = rospy.Subscriber("cmd_vel", Twist, node.twist_callback)
    rospy.loginfo("Subscribing to %s"%
                  (s1.name))
    rospy.loginfo("Subscribing to %s"%
                  (s2.name))

    # Dynamic configure
    srv = Server(TwistDynamicConfig, node.dynamic_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
