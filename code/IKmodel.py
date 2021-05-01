import tinyik
import numpy as np
import csv
import time
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
# import tensorboard
import os

def current_milli_time():
        return round(time.time(),6)

def save_data_in_csv(positions, angles, num_samples,file_name):
    csv1 = open(file_name, "a")
    for i in range(num_samples):
        csv1.write(str(positions[i,0]) + "," + str(positions[i,1]) + "," + str(positions[i,2]) + "," + str(angles[i,0]) + "," + str(angles[i,1]) + "," + str(angles[i,2]) + "\n")
    csv1.close()

def read_data_in_csv(file_name):
    with open(file_name) as csv_file:
        data = np.asarray(list(csv.reader(csv_file, delimiter=',')), dtype=np.float32)
        positions = data[:,0:3]
        angles = data[:,3:6]
    return positions,angles

def buildRearLeg():
    leg = tinyik.Actuator([[.0000000001, .0, .0], 'z', [.0, -1.76, .0], 'x', [.0, -4.83, .0], 'z', [.0, -5.41, .0]])
    # leg.angles = np.deg2rad([0,0,0])
    # tinyik.visualize(leg)
    return leg

def buildFrontLeg():
    leg = tinyik.Actuator([[.0000000001, .0, .0], 'z', [.0, -1.76, .0], 'x', [.0, -5.38, .0], 'z', [.0, -8.11, .0]])
    # leg.angles = np.deg2rad([0,0,0])
    # tinyik.visualize(leg)
    return leg

def generateRearIKData(num_samples,file_name):
    for j in range(50):
        print(j)
        angles = np.array([])
        positions = np.array([])
        leg = buildRearLeg()
        r1 = np.random.uniform(80,-80,num_samples)
        r2 = np.random.uniform(25,-25,num_samples)
        r3 = np.random.uniform(0,-135,num_samples)
        angles = np.concatenate((r1.reshape(-1,1), r2.reshape(-1,1), r3.reshape(-1,1)),axis=1)
        positions = np.empty([0,3])
        for i in range(num_samples):
            leg.angles = np.deg2rad(angles[i])
            # print(angles[i])
            # print(leg.ee)
            # tinyik.visualize(leg)
            positions = np.concatenate((positions,leg.ee.reshape(1,3)),axis=0)
            # print(i)
        save_data_in_csv(positions, angles,num_samples,file_name)
    return positions, angles

def generateFrontIKData(num_samples,file_name):
    for j in range(80):
        print(j)
        angles = np.array([])
        positions = np.array([])
        leg = buildFrontLeg()
        r1 = np.random.uniform(40,-40,num_samples)
        r2 = np.random.uniform(25,-25,num_samples)
        r3 = np.random.uniform(0,-60,num_samples)
        angles = np.concatenate((r1.reshape(-1,1), r2.reshape(-1,1), r3.reshape(-1,1)),axis=1)
        positions = np.empty([0,3])
        for i in range(num_samples):
            leg.angles = np.deg2rad(angles[i])
            # print(angles[i])
            # print(leg.ee)
            # tinyik.visualize(leg)
            positions = np.concatenate((positions,leg.ee.reshape(1,3)),axis=0)
            # print(i)
        save_data_in_csv(positions, angles,num_samples,file_name)
    return positions, angles

def buildIKModel(path_name):
    checkpoint_path = path_name; checkpoint_dir = os.path.dirname(checkpoint_path)
    # Create a callback that saves the model's weights
    cp_callback = tf.keras.callbacks.ModelCheckpoint(filepath=checkpoint_path, save_weights_only=True, verbose=0)
    model_ik = tf.keras.Sequential([
        layers.Dense(50, activation=tf.nn.relu, input_shape=[3]),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(3)
    ])
    model_ik.compile(optimizer='adam', loss='mae')
    model_ik.summary()
    return model_ik, cp_callback

def loadRearIKModel():
    checkpoint_path = "training/cp_chimp_rear.ckpt"
    checkpoint_dir = os.path.dirname(checkpoint_path)
    model_ik = tf.keras.Sequential([
        layers.Dense(50, activation=tf.nn.relu, input_shape=[3]),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(3)
    ])
    model_ik.compile(optimizer='adam', loss='mae')
    model_ik.summary()
    # Loads the weights
    model_ik.load_weights(checkpoint_path)
    return model_ik

def loadFrontIKModel():
    checkpoint_path = "training/cp_chimp_front.ckpt"
    checkpoint_dir = os.path.dirname(checkpoint_path)
    model_ik = tf.keras.Sequential([
        layers.Dense(50, activation=tf.nn.relu, input_shape=[3]),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(3)
    ])
    model_ik.compile(optimizer='adam', loss='mae')
    model_ik.summary()
    # Loads the weights
    model_ik.load_weights(checkpoint_path)
    return model_ik





if __name__ =="__main__":
    # Rear Leg generation
    # leg = buildRearLeg()
    # leg.angles = np.deg2rad([10,0,-20])
    # calc_pos = np.array(leg.ee).reshape(1,3)
    # # tinyik.visualize(leg)
    # generateRearIKData(100000,"chimp_rear_IK.csv")
    # positions, angles = read_data_in_csv("chimp_rear_IK.csv")
    # model, cp_callback = buildIKModel("training/cp_chimp_rear.ckpt")
    # EPOCHS = 60
    # model.fit(positions, angles,epochs=EPOCHS, validation_split = 0.2, verbose=1, callbacks=[cp_callback])
    # control_angles = model(calc_pos,training=False).numpy() 

    # Front Leg generation
    leg = buildRearLeg()
    leg.angles = np.deg2rad([-10,0,10])
    # calc_pos = np.array(leg.ee).reshape(1,3)
    leg.ee = [0,-11,0]
    ang = leg.angles
    print("desired angles     - ",np.rad2deg(ang))
    print("desired position   - ",leg.ee)

    tinyik.visualize(leg)
    # generateFrontIKData(100000,"chimp_front_IK.csv")
    # positions, angles = read_data_in_csv("chimp_front_IK.csv")
    # model, cp_callback = buildIKModel("training/cp_chimp_front.ckpt")
    # EPOCHS = 50
    # model.fit(positions, angles,epochs=EPOCHS, validation_split = 0.2, verbose=1, callbacks=[cp_callback])
    # control_angles = model(calc_pos,training=False).numpy() 

    print("desired angles     - ",np.rad2deg(leg.angles))
    print("desired position   - ",leg.ee)
    print("------------------------------------------")
    print("calculated angles  - ",control_angles)
    leg.angles = np.deg2rad(control_angles[0])
    print("calculated position- ",leg.ee)
    # tinyik.visualize(leg)







    



