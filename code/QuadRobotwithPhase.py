import time
import numpy as np
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
	m = np.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0])
	b = np.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0])
	k = np.array([15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0])


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
	torque = -control_state.m*jacc - control_state.b*jvel - control_state.k*(jpos-tar)
	robot_torque_control(robot, numJoints, torque)

def movement_controller(robot, robot_state, control_state, numJoints):
	# Standing 
	if control_state.action==0
		control_state.target_foot_position = np.array([[0.0,0.25,0.0],[0.0,0.25,0.0],[0.0,0.25,0.0],[0.0,0.25,0.0]])
		new_angles = model_ik(control_state.target_foot_position,training=False).numpy()
		control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)

	# hobble
	elif control_state.action==1
		leg1x = np.cos(control_state.dt)
		leg1z = np.sin(control_state.dt)+.25
		leg2x = np.cos(control_state.dt+.25)
		leg2z = np.sin(control_state.dt+.25)+.25
		leg3x = np.cos(control_state.dt+.50)
		leg3z = np.sin(control_state.dt+.50)+.25
		leg4x = np.cos(control_state.dt+.75)
		leg4z = np.sin(control_state.dt+.75)+.25

		control_state.target_foot_position = np.array([[0.0,leg1z,leg1x],[0.0,leg2z,leg2x],[0.0,leg3z,leg3x],[0.0,leg4z,leg4x]])
		new_angles = model_ik(control_state.target_foot_position,training=False).numpy()
		control_state.target_angle = np.concatenate((new_angles,np.zeros((4,1))),axis=1).flatten()
		impedance_control(robot, robot_state, control_state, numJoints)
	pass


















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
		movement_controller(robot, robot_state, control_state, numJoints)

		# Take simulation steps
		p.stepSimulation()
		time.sleep(dt)