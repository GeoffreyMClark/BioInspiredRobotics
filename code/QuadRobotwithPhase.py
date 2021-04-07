import time
import numpy as np
from scipy import signal as scp
import pybullet as p
import pybullet_data as pd
import time
import math
import tinyik
import os
from IKmodel import buildIKModel

# actuator design for individual leg
model_ik = buildIKModel()

def current_milli_time():
        return round(time.time(),6)

class cheetah_class:
	position = np.array([0.0,0.0,0.0])
	orientation = np.array([0.0,0.0,0.0])
	joint_angles = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_vels = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_accs = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

class control_class:
	t_step = 0.0
	action = 0
	position = np.array([0.0,0.0,0.25])
	orientation = np.array([0.0,0.0,0.0])
	target_foot_position = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
	target_angle = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	torque = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	m = np.array([1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5])
	b = np.array([.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017,.017])
	k = np.array([1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2])


def add_robot():
	# Set gravity
	p.setGravity(0,0,-9.8)
	# Generate floor
	p.setAdditionalSearchPath(pd.getDataPath())
	floor = p.loadURDF("plane.urdf")
	# Generate robot
	startPos = [0,0,0.5]
    
    #below was used to make sure urdf files could be called. if needed then change 
    #inside '' to what your working directory is
	#os.chdir('')
    
	robot = p.loadURDF("mini_cheetah.urdf",startPos) #,flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
	numJoints = p.getNumJoints(robot)
	for j in range (-1,numJoints):
		p.changeVisualShape(robot,j,rgbaColor=[1,1,1,1])
	pos0=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	robot_position_ctrl(robot,numJoints,pos0,200)
	# zero robot on ground for two seconds
	for i in range(1*240):
		pos=np.array([15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0])
		robot_position_ctrl(robot,numJoints,pos,200)
		p.stepSimulation()
		time.sleep(dt)
	# pos=np.array([15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0,15.0,-90.0,160.0,0.0])
	# robot_position_ctrl(robot,numJoints,pos,0)
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
	pos_adj = np.array([-1,1,1,1,1,1,1,1,-1,1,1,1,1,1,1,1])
	pos_ctrl = pos*pos_adj*np.pi/180
	for j in range(numJoints):
		p.setJointMotorControl2(robot,j,p.POSITION_CONTROL,pos_ctrl[j],0,force=force)

#inputs robot id, torque applied to joint, joint number
def robot_torque_control(robot,numJoints,torque_ctrl):
	for j in range(numJoints):
		p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.VELOCITY_CONTROL,force=0)
		p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.TORQUE_CONTROL,force=torque_ctrl[j])

#inputs robot id, target angle, joint number, parameters m b k
def impedance_control(robot, robot_state, control_state, numJoints):
	tar = control_state.target_angle
	#get angle, velocity and acceleration
	jpos = robot_state.joint_angles
	jvel = robot_state.joint_vels
	jacc = robot_state.joint_accs
	#calculate torque based on previous values then return torque
	torque = -(control_state.m*jacc + control_state.b*jvel + control_state.k*(jpos-tar))
	robot_torque_control(robot, numJoints, torque)

def movement_controller(robot, robot_state, control_state, numJoints):
	# Standing 
	if control_state.action==0:
		pos_y = np.clip(control_state.t_step/5,.1,.3)
		print(pos_y)
		control_state.target_foot_position = np.array([[0.0,pos_y,0.0],[0.0,pos_y,0.0],[0.0,pos_y,0.0],[0.0,pos_y,0.0]])
		new_angles = model_ik(control_state.target_foot_position,training=False).numpy()
		control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)

	# Hobble (sinusoids) - doesnt work
	elif control_state.action==1:
		# phase offsets
		leg1_off=0.0; leg2_off=1.0; leg3_off=0.5; leg4_off=1.5
		speed = np.pi
		leg1x = .1*   np.cos(speed*control_state.t_step + speed*leg1_off)
		leg1z = np.clip(.05* np.sin(speed*control_state.t_step + speed*leg1_off) + .25, 0, .25)
		leg2x = .1*   np.cos(speed*control_state.t_step + speed*leg2_off)
		leg2z = np.clip(.05* np.sin(speed*control_state.t_step + speed*leg2_off) + .25, 0, .25)
		leg3x = .1*   np.cos(speed*control_state.t_step + speed*leg3_off)+.025
		leg3z = np.clip(.05* np.sin(speed*control_state.t_step + speed*leg3_off) + .25, 0, .25)
		leg4x = .1*   np.cos(speed*control_state.t_step + speed*leg4_off)+.025
		leg4z = np.clip(.05* np.sin(speed*control_state.t_step + speed*leg4_off) + .25, 0, .25)
		control_state.target_foot_position = np.array([[0.0,leg1z,leg1x],[0.0,leg2z,leg2x],[0.0,leg3z,leg3x],[0.0,leg4z,leg4x]])
		new_angles = model_ik(control_state.target_foot_position,training=False).numpy()
		control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)
	
	# Walking
	elif control_state.action==2:
		# seconds/stride | step duty cycle | Stride length | step height
		sps=1; DC=0.85; SL=.2; SH=.1
		# phase offsets
		leg_off=np.array([0.0,0.5,0.25,0.75])
		stance=np.array([False,False,False,False])
		leg_phase=np.array([0.0,0.0,0.0,0.0])
		leg_z=np.array([0.0,0.0,0.0,0.0])
		leg_x=np.array([0.0,0.0,0.0,0.0])
		phase = (scp.sawtooth(control_state.t_step*(2*np.pi)/sps)+1)/2
		for i in range(4):
			leg_phase[i]=(scp.sawtooth((control_state.t_step*(2*np.pi/sps)+leg_off[i]*(2*np.pi)))+1)/2
			if leg_phase[i]<DC: 
				stance[i] = True
				leg_x[i] = SL/2 - SL*(leg_phase[i]/DC)
				leg_z[i] = 0.3
			else: 
				stance[i] = False
				leg_x[i] = -SL/2 + SL*((leg_phase[i]-DC)/(1-DC))
				leg_z[i] = -SH*np.sin(((leg_phase[i]-DC)/(1-DC))*np.pi)+.3

		control_state.target_foot_position = np.array([[0.02,leg_z[0],leg_x[0]+.02],[-0.02,leg_z[1],leg_x[1]+.02],[0.02,leg_z[2],leg_x[2]+.08],[-0.02,leg_z[3],leg_x[3]+.08]])
		new_angles = model_ik(control_state.target_foot_position,training=False).numpy()
		control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)




def control_selector(control_state):
	if control_state.t_step>4:
		control_state.action=2















if __name__ =="__main__":
	# initialize world
	p.connect(p.GUI)
	dt = 1./200.

	# Add robot
	robot,numJoints = add_robot()
	robot_state = cheetah_class()
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