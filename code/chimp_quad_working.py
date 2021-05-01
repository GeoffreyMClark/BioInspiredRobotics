import time
import numpy as np
from scipy import signal as scp
import pybullet as p
import pybullet_data as pd
import time
import math
import tinyik
import os
from IKmodel import loadRearIKModel,loadFrontIKModel

# actuator design for individual leg
rear_ik = loadRearIKModel()
front_ik = loadFrontIKModel()

def current_milli_time():
        return round(time.time(),6)

class chimp_class:
	position = np.array([0.0,0.0,0.0])
	orientation = np.array([0.0,0.0,0.0])
	joint_angles = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_vels = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_accs = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

class control_class:
	t_step = 0.0
	action = 0
	position = np.array([0.0,0.0,0.25])
	orientation = np.array([0.0,0.0,0.0])
	target_foot_position = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
	target_angle = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	torque = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	# m = np.array([1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5])
	# b = np.array([.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017])
	# k = np.array([1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2])
	m = np.ones((32))*1.5
	b = np.array([0.5,0.5,0.0,0.5,0.0,0.0,0.0,0.0,0.0,0.0,      0.5,0.5,0.0,0.5,0.0,0.0,0.0,0.0,0.0,0.0,       0.3,0.3,0.0,0.3,0.01,0.0,        0.3,0.3,0.0,0.3,0.01,0.0])
	k = np.array([60.0,60.0,0.0,60.0,0.0,0.0,0.0,0.0,0.0,0.0,   60.0,60.0,0.0,60.0,0.0,0.0,0.0,0.0,0.0,0.0,    50.0,50.0,0.0,50.0,1.0,0.0,    50.0,50.0,0.0,50.0,1.0,0.0])
	# s1 s2 st elbow w1 wt f11 f12 f21 f22
	# h1 h2 ht knee  ankle toe

def add_robot():
	# Set gravity
	p.setGravity(0,0,-9.8)
	# Generate floor
	p.setAdditionalSearchPath(pd.getDataPath())
	floor = p.loadURDF("plane.urdf")
	# Generate robot
	startPos = [0,0,.5]
    #below was used to make sure urdf files could be called. if needed then change 
    #inside '' to what your working directory is
	#os.chdir('')
	robot = p.loadURDF("v1.urdf",startPos) #,flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
	numJoints = p.getNumJoints(robot)
	for j in range (-1,numJoints):
		p.changeVisualShape(robot,j,rgbaColor=[1,1,1,1])
	pos0=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	robot_position_ctrl(robot,numJoints,pos0,200)
	# zero robot on ground for one second
	for i in range(1*240):
		# pos=np.array([-10.0,0.0,0.0,10.0,180.0,180.0,0.0,0.0,0.0,0.0,  -10.0,0.0,0.0,-10.0,180.0,180.0,0.0,0.0,0.0,0.0,  0.0,0.0,0.0,40.0,-10.0,0.0,  0.0,0.0,0.0,40.0,10.0,0.0])
		pos=np.array([-10.0,0.0,0.0,10.0,180.0,180.0,0.0,0.0,0.0,0.0,  -10.0,0.0,0.0,-10.0,180.0,180.0,0.0,0.0,0.0,0.0,  0.0,0.0,0.0,40.0,-10.0,0.0,  0.0,0.0,0.0,40.0,10.0,0.0])
		robot_position_ctrl(robot,numJoints,pos,5)
		p.stepSimulation()
		time.sleep(dt)
	return robot,numJoints

def robot_data(robot, robot_state, dt):
	[location, quaternions] = p.getBasePositionAndOrientation(robot)
	robot_state.position = np.asarray(location)
	robot_state.orientation = np.asarray(p.getEulerFromQuaternion(quaternions))
	joints = p.getJointStates(robot,range(numJoints))
	robot_state.joint_angles = np.array([state[0] for state in joints])*180/np.pi
	joint_vels = np.array([state[1] for state in joints])*180/np.pi
	joint_torques = np.array([state[3] for state in joints])
	robot_state.joint_accs = (robot_state.joint_vels-joint_vels)*dt
	robot_state.joint_vels = joint_vels

def robot_position_ctrl(robot,numJoints,pos,force):
	pos_adj = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
	pos_ctrl = pos*pos_adj*np.pi/180
	for j in range(numJoints):
		p.setJointMotorControl2(robot,j,p.POSITION_CONTROL,pos_ctrl[j],0,force=force)

#inputs robot id, torque applied to joint, joint number     
def robot_torque_control(robot,numJoints,torque_ctrl):
	for j in [0,1,3,10,11,13,20,21,23,24,26,27,29,30]:
		# mod = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
		p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.VELOCITY_CONTROL,force=0)
		p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.TORQUE_CONTROL,force=torque_ctrl[j])

#inputs robot id, target angle, joint number, parameters m b k
def impedance_control(robot, robot_state, control_state, numJoints):
	tar = control_state.target_angle*np.pi/180
	#get angle, velocity and acceleration
	jpos = robot_state.joint_angles*np.pi/180
	jvel = robot_state.joint_vels*np.pi/180
	jacc = robot_state.joint_accs*np.pi/180
	#calculate torque based on previous values then return torque
	a = control_state.m*jacc
	b = control_state.b*jvel
	c = control_state.k*(jpos-tar)
	torque = -(a + b + c)
	# robot_torque_control(robot, numJoints, torque)
	robot_torque_control(robot, 3, torque)

def movement_controller(robot, robot_state, control_state, numJoints):
	# Standing 
	if control_state.action==0:
		pos_fy = np.clip(control_state.t_step/5,.1,.3)
		pos_ry = np.clip(control_state.t_step/5,.1,.3)

		# print(pos_y)
		control_state.target_foot_position = np.array([[0.0,pos_ry,0.0],[0.0,pos_ry,0.0],[0.0,pos_fy,0.0],[0.0,pos_fy,0.0]])
		control_state.target_angle = np.array([-10.0,0.0,0.0,10.0,180.0,180.0,0.0,0.0,0.0,0.0,  -10.0,0.0,0.0,-10.0,180.0,180.0,0.0,0.0,0.0,0.0,  0.0,0.0,0.0,40.0,-10.0,0.0,  0.0,0.0,0.0,40.0,10.0,0.0])

		# s1 s2 st elbow w1 wt f11 f12 f21 f22
		# h1 h2 ht knee  ankle toe

		impedance_control(robot, robot_state, control_state, numJoints)

	

	# Walking
	elif control_state.action==2:
		# seconds/stride | step duty cycle | Stride length | step height
		sps=2; DC=0.85; SL=5; SH=2
		# phase offsets
		leg_off=np.array([0.25,0.75,0.5,0.0])
		stance=np.array([False,False,False,False])
		leg_phase=np.array([0.0,0.0,0.0,0.0])
		leg_z=np.array([0.0,0.0,0.0,0.0])
		leg_x=np.array([0.0,0.0,0.0,0.0])
		phase = (scp.sawtooth(control_state.t_step*(2*np.pi)/sps)+1)/2
		# Front foot position
		for i in [0,1]:
			leg_phase[i]=(scp.sawtooth((control_state.t_step*(2*np.pi/sps)+leg_off[i]*(2*np.pi)))+1)/2
			# print("phase - ",leg_phase[0])
			if leg_phase[i]<DC: 
				stance[i] = True
				leg_x[i] = SL/2 - SL*(leg_phase[i]/DC)
				leg_z[i] = -11
			else: 
				stance[i] = False
				leg_x[i] = -SL/2 + SL*((leg_phase[i]-DC)/(1-DC))
				leg_z[i] = SH*np.sin(((leg_phase[i]-DC)/(1-DC))*np.pi)-11
			
		# Rear foot position
		for i in [2,3]:
			leg_phase[i]=(scp.sawtooth((control_state.t_step*(2*np.pi/sps)+leg_off[i]*(2*np.pi)))+1)/2
			if leg_phase[i]<DC: 
				stance[i] = True
				leg_x[i] = SL/2 - SL*(leg_phase[i]/DC)
				leg_z[i] = -11
			else: 
				stance[i] = False
				leg_x[i] = -SL/2 + SL*((leg_phase[i]-DC)/(1-DC))
				leg_z[i] = SH*np.sin(((leg_phase[i]-DC)/(1-DC))*np.pi)-11


		control_state.target_foot_position = np.array([[-leg_x[0],leg_z[0],0.0],[-leg_x[1],leg_z[1],0.0],[leg_x[2]-1,leg_z[2],0.0],[leg_x[3]-1,leg_z[3],0.0]])
		r = rear_ik(control_state.target_foot_position[2:4,:],training=False).numpy()
		f = front_ik(control_state.target_foot_position[0:2,:],training=False).numpy()

		control_state.target_angle = np.array([-f[0,0],f[0,1],0.0,-f[0,2],180.0,180.0,0.0,0.0,0.0,0.0,  -f[1,0],f[1,1],0.0,f[1,2],180.0,180.0,0.0,0.0,0.0,0.0,  r[0,0],r[0,1],0.0,-r[0,2],-10.0,0.0,  r[1,0],r[1,1],0.0,-r[1,2],10.0,0.0])
		# control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)




def control_selector(control_state):
	if control_state.t_step>2:
		control_state.action=2















if __name__ =="__main__":
	# initialize world
	p.connect(p.GUI)
	dt = 1./200.

	# Add robot
	robot,numJoints = add_robot()
	robot_state = chimp_class()
	control_state = control_class()
	robot_data(robot, robot_state, dt)

	while(1):
		control_state.t_step = control_state.t_step +dt
        # Robot observation
		robot_data(robot, robot_state, dt)

		# controller
		control_selector(control_state)
		movement_controller(robot, robot_state, control_state, numJoints)

		# Take simulation steps
		p.stepSimulation()
		time.sleep(dt)