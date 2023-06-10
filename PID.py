#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from itertools import count
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

##### setting time
index = count(step=0.004)

#### making list of data for making graph
x_vals=[]
y_vals=[]

w=[]
act=[]



#### PID parameters
p=5
i=2.5
d=0



def callback(msg):
	global x
	global y
	global w
	q=msg.range*0.6923 ### each rotation has 520 tricks
	q=round(q,2)
	act.append(q)
	w=act[-400:]	
	a=next(index)
	x_vals.append(a)
	#y=20*np.sin(a*1+180)
	y=0 ##### input of controller (Desired)
	if y>0:
		y=y
	else:
		y=y
	y_vals.append(y) #### add data to the lists
	x = x_vals[-400:] #### save recent 400 digits
	y = y_vals[-400:]
	
	###### PID calculations
	if len(x)>2 and len(y)>2 and len(w)>2:
		dt=x[-1]-x[-2]
		de0=y[-2]
		ac0=w[-2]
	else:
		dt=0
		de0=0
		ac0=0
	de=y[-1]
	ac=w[-1]	
	e1=de-ac
	e0=de0-ac0
	delE=(e0+e1)/2
	inte=dt*delE
	if dt==0:
		dere=0
	else:
		dere=delE/dt
	kp=p
	ki=i
	#kd=d+0.0001*dere*q
	kd=d	
	v=kp*e1+ki*inte+kd*dere ### PID controller goes to motors as voltage
	#v=5
	pub=rospy.Publisher('/Motor_shoulder_pitch', Float64, queue_size=10) ### publish in motor topic
	aa=float(-v)
	pub.publish(aa) ##### publish voltage into motor rostopic
	#print(abs(e1))
	print(aa)
def animate(i): #### graph animation function
	plt.cla()
	plt.plot(x,y)
	plt.plot(x,w)
#def listener():
	#rospy.init_node('listener', anonymous=True) 
	#rospy.spin()
#if __name__ == '__main__':
rospy.init_node('callback', anonymous=True)  ###### make a ros node
rospy.Subscriber("/Shoulder_pitch", Range, callback, queue_size=10)  #### collecting data from motor encoder
ani=  FuncAnimation(plt.gcf(), animate, interval=200) 
plt.show()
if __name__ == '__main__':
	rospy.spin()
