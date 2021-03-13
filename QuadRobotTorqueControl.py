import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
import tinyik
import os

# actuator design for individual leg
leg = tinyik.Actuator([[-.49, .0, 1.9], 'z', [-.62, .0, .0], 'x', [.0, -2.09, .0], 'x', [.0, -1.8, .0]])

def current_milli_time():
        return round(time.time(),6)

class cheetah_class:
	position = np.array([0.0,0.0,0.0])
	orientation = np.array([0.0,0.0,0.0])
	joint_angles = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_vels = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	joint_accs = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

class goal_class:
	position = np.array([0.0,0.0,0.25])
	orientation = np.array([0.0,0.0,0.0])


def add_robot():
	# Set gravity
	p.setGravity(0,0,-9.8)
	# Generate floor
	p.setAdditionalSearchPath(pd.getDataPath())
	floor = p.loadURDF("plane.urdf")
	# Generate robot
	startPos = [0,0,0.5]
    
    #below was used to make sure urdf files could be called. if needed then change 
    #inside '' to what your working directory
	#os.chdir('working directory')
    
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

def robot_position_ctrl(robot,numJoints,pos,force):
	pos_adj = np.array([-1,1,1,1,1,1,1,1,-1,1,1,1,1,1,1,1])
	pos_ctrl = pos*pos_adj*np.pi/180
	for j in range(numJoints):
		p.setJointMotorControl2(robot,j,p.POSITION_CONTROL,pos_ctrl[j],0,force=force)

def robot_torque_control(robot,torque_ctrl):
	for j in [1,2,5,6,9,10,13,14]:
		p.setJointMotorControl2(robot,j,p.TORQUE_CONTROL,force=torque_ctrl[j])

#inputs robot id, torque applied to joint, joint number
def robot_torque_control2(robot,torque_ctrl,j):
    p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.VELOCITY_CONTROL,force=0)
    p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=j,controlMode=p.TORQUE_CONTROL,force=torque_ctrl)

def robot_data(robot, robot_state):
	[location, quaternions] = p.getBasePositionAndOrientation(robot)
	robot_state.position = np.asarray(location)
	robot_state.orientation = np.asarray(p.getEulerFromQuaternion(quaternions))
	joints = p.getJointStates(robot,range(numJoints))
	robot_state.joint_angles = np.array([state[0] for state in joints])*180/np.pi
	joint_vels = np.array([state[1] for state in joints])*180/np.pi
	joint_torques = np.array([state[3] for state in joints])
	robot_state.joint_accs = (robot_state.joint_vels-joint_vels)/500
	robot_state.joint_vels = joint_vels

#inputs robot id, target angle, joint number, parameters m b k
def impedance_control(robot, tar, j,m,b,k):
    #get angle, velocity and acceleration
	jpos = p.getJointState(robot,j)[0]
	jvel = p.getJointState(robot,j)[1]
	jacc = p.getJointState(robot,j)[3]
    #calculate torque based on previous values then return torque
	tor = -m*jacc-b*jvel-k*(jpos-tar);
	return tor

    #3 2 5 1 3 3

if __name__ =="__main__":
    #parameters used for joint 1 and 2, values used were from random testing
	m1 = 1
	b1 = 1
	k1 = 50
	m2 = 1
	b2 = .1
	k2 = 20
	# initialize world
	#p.disconnect()
	p.connect(p.GUI)
	p.resetDebugVisualizerCamera(0,20,-20,[0,0,0])
	dt = 1./200.

	# Add robot
	robot,numJoints = add_robot()
	robot_state = cheetah_class()
	goal_state = goal_class()
	robot_data(robot, robot_state)
    
	#index for going through tar and inc for incrementing index
	index = 0
	inc = 1
    #create target list of angles ranging from 0 to pi for joint 2
	tar = np.linspace(0,np.pi,101)
    
    #1 front right
    #5 front left
    #9 back right
    #13 back left
    
    
	while(1):
		# Robot observation
        #get current desired value for joint 2 from target list
		jposd = tar[index]
        #call impedance function for joints
		tr1 = impedance_control(robot,-np.pi/4,1,m1,b1,k1)
		tr2 = impedance_control(robot,-np.pi/4,5,m1,b1,k1)
		tr3 = impedance_control(robot,-np.pi/4,9,m1,b1,k1)
		tr4 = impedance_control(robot,-np.pi/4,13,m1,b1,k1)

		tr5 = impedance_control(robot,np.pi/2,2,m2,b2,k2)
		tr6 = impedance_control(robot,np.pi/2,6,m2,b2,k2)
		tr7 = impedance_control(robot,np.pi/2,10,m2,b2,k2)
		tr8 = impedance_control(robot,np.pi/2,14,m2,b2,k2)

        #incrementing index from 0 to 100 then decrementing from 100 to 0
        #increasing/decreasing by 1 each loop
		index = index + inc
		if(index == 100):
			inc = -1
		elif(index == 0):
			inc = 1
		robot_data(robot, robot_state)
		leg.angles = np.deg2rad([robot_state.joint_angles[0], -robot_state.joint_angles[1], -robot_state.joint_angles[2]])

        #call function applying torque tr# to joint given for robot		
		robot_torque_control2(robot, tr1,1)
		robot_torque_control2(robot, tr2,5)
		robot_torque_control2(robot, tr3,9)
		robot_torque_control2(robot, tr4,13)

		robot_torque_control2(robot, tr5,2)
		robot_torque_control2(robot, tr6,6)
		robot_torque_control2(robot, tr7,10)
		robot_torque_control2(robot, tr8,14)

		# Impediance controller (convert R to P joints)
        
		# Take simulation steps
		p.stepSimulation()
		time.sleep(dt)






