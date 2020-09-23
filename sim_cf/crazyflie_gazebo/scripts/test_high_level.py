#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import math
import numpy as np



def omega_dx(t):
	r=0
	Phi_g1=180
	Phi_g3=180
	omega_max=1400
	gamma1=(1/omega_max)*Phi_g1
	gamma3=(1/omega_max)*Phi_g3
	beta1=-(3/4)*(1/gamma1**3)*omega_max
	beta3=-(3/4)*(1/gamma3**3)*omega_max
	delta=0.26
	if(t<delta):
		r=(beta1/3)*(t-gamma1)**3-beta1*gamma1**2*t+beta1*gamma1**3/3
	if(t>delta):		
		r=(beta3/3)*(gamma3+delta-t)**3-beta3*gamma3**2*(2*gamma3+delta-t)+beta3*gamma3**3/3
	return r

def circle_trajectory(freq, duration,r):
    x = list()
    y = list()
    t = list()
    for i in range(freq):
        t0 = 2 * math.pi * (1.0/(freq-1)) * duration * i
        x.append(r * math.cos(t0/duration))
        y.append(r * math.sin(t0/duration))
        t.append((1.0/(freq-1)) * duration * i)
    return (t,x,y)
    
hovering_z=1.5
dt=0.01

def stablize(cf,t,dt,x,y,z):
    t0= rospy.Time.now()
    while True:
        cf.send_position_setpoint(x,y,z,0)
        rospy.sleep(dt)
        if (rospy.Time.now()-t0).to_sec()>t:
            break
    

def take_off(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        cf.send_position_setpoint(0,0,hovering_z*i/n,0)
        rospy.sleep(dt)
    for i in range(0,2*n):
        cf.send_position_setpoint(0,0,hovering_z,0)
        rospy.sleep(dt)
        
def take_off2(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        r=[0.,0.,hovering_z*i/n]
        v=[0.,0.,0.]
        a=[0.,0.,0.]
        rpy=[0.,0.,0.]
        rate=[0.,0.,0.]
        cf.send_full_state_setpoint(r,v,a,rpy,rate)
        rospy.sleep(dt)
    for i in range(0,2*n):
        r=[0,0,hovering_z]
        v=[0,0,0]
        a=[0,0,0]
        rpy=[0,0,0]
        rate=[0,0,0]
        cf.send_full_state_setpoint(r,v,a,rpy,rate)
        rospy.sleep(dt)
        
def land(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        cf.send_position_setpoint(0,0,hovering_z*i/n,0)
        rospy.sleep(dt)


def flip360(cf,t,dt):
    n=int(t/dt)
    points=[]
    z0=hovering_z
    w=2*np.pi/t
    for i in range(0,n):
        ti=i*t/n
        r=[0,0,z0]#x=0; y=0;z=z0
        v=[0,0,0]
        a=[0,0,0]
        rpy=[0,w*ti,0]
        rate=[0,w,0]
        if ti>0.300*t and ti<0.6*t:        #enter to restablizing phase in advance
            rate=[0,0,0]
            cf.send_full_state_setpoint(r,v,a,rpy,rate)
        elif ti>0.6*t:
            rate=[0,0,0]
            cf.send_full_state_setpoint(r,v,a,rpy,rate)
            break
        cf.send_full_state_setpoint(r,v,a,rpy,rate)
        rospy.sleep(dt)
    
def flip360_rate(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        ti=i*dt
        r=[0,0,hovering_z]#x=0; y=0;z=z0
        v=[0,0,0]
        a=[0,0,0]
        rpy=[0,0,0]
        rate=[0,omega_dx(ti)/180.*3.1415926,0]
        cf.send_full_state_setpoint(r,v,a,rpy,rate)
        rospy.sleep(dt) 
   
        
    
def climb(cf,t,vz,dt):
    n=int(t/dt)
    for i in range(0,n):
        cf.send_position_setpoint(0,0,hovering_z+vz*i*dt,0)
        rospy.sleep(dt)
        
        


def omega_d_dotx(t):
	r=0
	Phi_g1=180
	Phi_g3=180
	omega_max=1400
	gamma1=(1/omega_max)*Phi_g1
	gamma3=(1/omega_max)*Phi_g3
	beta1=-(3/4)*(1/gamma1**3)*omega_max
	beta3=-(3/4)*(1/gamma3**3)*omega_max
	delta=0.26
	if(t<delta):
		r=beta1*(gamma1-t)**2-beta1*gamma1**2
	if(t>delta):
		r=beta3*gamma3**2-beta3*(delta+gamma3-t)**2
	return r
    

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("cf1", "/cf1")

    cf.setParam("commander/enHighLevel", 1)
    rospy.sleep(5)
    
        #send cf2 to the hovering point
    tt=1
    print("take off")
    take_off(cf,tt,dt)
        
    #stablize
    print("stablize")
    ts=3.0
    stablize(cf,ts,dt,0,0,hovering_z)
      
      
      #limb   
    print("climb")
    tc=.2;vz=1
    climb(cf,tc,vz,dt)
    #flip
    
    print("flip")
    tf=0.52
    flip360_rate(cf,tf,dt)
        
    #stablize
    ts=2
    stablize(cf,ts,dt,0,0,hovering_z)
        
        #land
    tl=2
    land(cf,tl,dt)

    #cf.takeoff(targetHeight = 1.5, duration = 2.0)
    #time.sleep(5.0)

    # (t , x , y) = circle_trajectory(50 , 10 , 0.8)

    # cf.goTo(goal = [0.8, 0, 1.5], yaw=0.0, duration = 4, relative = False)
    # time.sleep(4)

    # for i in range(len(x)):
    #     cf.goTo(goal = [x[i], y[i], 1.5], yaw=0.0, duration = 10.0/50.0, relative = False)
    #     time.sleep(1*10.0/50.0)

    # # # cf.land(targetHeight = 0.0 , duration = 4.0)
    # # # time.sleep(20.0)
    # # # rospy.spin()
    # # cf.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    # # time.sleep(15.0)

    #cf.land(targetHeight = 0.0, duration = 2.0)
    #time.sleep(3.0)
    
    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("takeoff.csv")

    # traj2 = uav_trajectory.Trajectory()
    # traj2.loadcsv("losange.csv")

    # print(traj2.duration)

    # cf.uploadTrajectory(0, 0, traj2)
    # # cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    # cf.startTrajectory(0, timescale=1.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(1, timescale=2.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(0, timescale=1.0, reverse=True)
    # time.sleep(traj1.duration * 1.0)

    cf.stop()
