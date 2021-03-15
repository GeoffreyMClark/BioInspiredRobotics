import time
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

#function that gives ang1 and ang2 for the desired angles to make the robot stand up
def stand_up(robot):
	m1 = 1
	b1 = 1
	k1 = 15
	m2 = 1
	b2 = 1
	k2 = 15

	tr1 = impedance_control(robot,-ang1,1,m1,b1,k1)
	tr2 = impedance_control(robot,-ang1,5,m1,b1,k1)
	tr3 = impedance_control(robot,-ang1,9,m1,b1,k1)
	tr4 = impedance_control(robot,-ang1,13,m1,b1,k1)

	tr5 = impedance_control(robot,ang2,2,m2,b2,k2)
	tr6 = impedance_control(robot,ang2,6,m2,b2,k2)
	tr7 = impedance_control(robot,ang2,10,m2,b2,k2)
	tr8 = impedance_control(robot,ang2,14,m2,b2,k2)
    
	robot_torque_control2(robot, tr1,1)
	robot_torque_control2(robot, tr2,5)
	robot_torque_control2(robot, tr3,9)
	robot_torque_control2(robot, tr4,13)

	robot_torque_control2(robot, tr5,2)
	robot_torque_control2(robot, tr6,6)
	robot_torque_control2(robot, tr7,10)
	robot_torque_control2(robot, tr8,14)
    
    #3 2 5 1 3 3

if __name__ =="__main__":
    #angles used for standing
	ang1 = np.pi/4
	ang2 = np.pi/2
    
    #creating desire angle arrays for each joint to be moved
	rang1 = 6
	rang2 = 4
	updes1 = np.ones(100)*(-ang1)
	updes1 = np.ones(100)*(-ang1-ang1/rang1)
    
	updes2 = np.ones(100)*(-ang1)
	updes2[20:100] = np.ones(80)*(-ang1-ang1/rang1)

	updes3 = np.ones(100)*(-ang1)
	updes3[40:100] = np.ones(60)*(-ang1-ang1/rang1)

	updes4 = np.ones(100)*(-ang1)
	updes4[60:100] = np.ones(40)*(-ang1-ang1/rang1)



	lowdes1 = np.ones(100)*(ang2+ang2/rang2)
	lowdes1[10:100] = np.ones(90)*(ang2-ang2/rang2)

	lowdes2 = np.ones(100)*(ang2)
	lowdes2[20:100] = np.ones(80)*(ang2+ang2/rang2)
	lowdes2[30:100] = np.ones(70)*(ang2-ang2/rang2)

	lowdes3 = np.ones(100)*(ang2)
	lowdes3[40:100] = np.ones(60)*(ang2+ang2/rang2)
	lowdes3[50:100] = np.ones(50)*(ang2-ang2/rang2)

	lowdes4 = np.ones(100)*(ang2)
	lowdes4[60:100] = np.ones(40)*(ang2+ang2/rang2)
	lowdes4[70:100] = np.ones(30)*(ang2-ang2/rang2)

    #parameters used for joint 1 and 2, values used were from random testing
	m1 = 1
	b1 = 1
	k1 = 15
	m2 = 1
	b2 = 1
	k2 = 15
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
    
	#index for going through desire angles and inc for incrementing index
	index = 100
	inc = -1
    
    #1 front right shoulder, #2 front right elbow
    #5 front left shoulder, #6 front left elbow
    #9 back right hip, #10 back right knee
    #13 back left hip, #13 back left knee
    
    
	while(1):
		# Robot observation

        #while decrementing robot will go to stand up position
		if(inc == -1):
			stand_up(robot)
        #robot begins to hobble one foot at a time
		else:
        
        #call impedance function for joints getting the torques
			tr1 = impedance_control(robot,updes1[index],1,m1,b1,k1)
			tr2 = impedance_control(robot,updes2[index],5,m1,b1,k1)
			tr3 = impedance_control(robot,updes3[index],9,m1,b1,k1)
			tr4 = impedance_control(robot,updes4[index],13,m1,b1,k1)

			tr5 = impedance_control(robot,lowdes1[index],2,m2,b2,k2)
			tr6 = impedance_control(robot,lowdes2[index],6,m2,b2,k2)
			tr7 = impedance_control(robot,lowdes3[index],10,m2,b2,k2)
			tr8 = impedance_control(robot,lowdes4[index],14,m2,b2,k2)
    
        #call function applying torque tr# to joint given for robot		    
			robot_torque_control2(robot, tr1,1)
			robot_torque_control2(robot, tr2,5)
			robot_torque_control2(robot, tr3,9)
			robot_torque_control2(robot, tr4,13)

			robot_torque_control2(robot, tr5,2)
			robot_torque_control2(robot, tr6,6)
			robot_torque_control2(robot, tr7,10)
			robot_torque_control2(robot, tr8,14)

        #incrementing index from 0 to 100 then decrementing from 100 to 0
        #increasing/decreasing by 1 each loop
		index = index + inc
		if(index == 100):
			inc = -1
		elif(index == 0):
			inc = 1
		robot_data(robot, robot_state)
		leg.angles = np.deg2rad([robot_state.joint_angles[0], -robot_state.joint_angles[1], -robot_state.joint_angles[2]])

		# Impediance controller (convert R to P joints)
        
		# Take simulation steps
		p.stepSimulation()
		time.sleep(dt)






