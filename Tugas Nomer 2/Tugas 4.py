############################################################
# Pioneer P3DX - Robot Localization and Pose Control
# (c)2024 Ditya Garda Nugraha, ditya.205022@mhs.its.ac.id
############################################################

import sim   # Import the remote API module
import math
import sys
import time
import keyboard
import numpy as np

# Connect to the remote API server
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

# Get the handle of the p3dx & disc we want to retrieve the position and orientation of
res, p3dx_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)

def getSensorsHandle(clientID):
    isNewCoppeliaSim=True
    sensorsHandle=np.array([])
    for i in range(16):#16=jlh sensor
        if (isNewCoppeliaSim):
            sensorHandle=sim.simxGetObjectHandle(
                clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']',sim.simx_opmode_blocking)[1]
        else:
            sensorHandle=sim.simxGetObjectHandle(
                clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
        #First call proximity sensor must use opmode_streaming
        _, _, _, _, _=sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle=np.append(sensorsHandle,sensorHandle)
        sensorsHandle=np.int32(sensorsHandle)
    return sensorsHandle

def getDistance(clientID, sensorsHandle):
    distances=np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, sensorsHandle[i], sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        distances=np.append(distances,distance)
    return distances

def getDistanceOrientation(sensorData):
    dist_act=min(obj_distances[3],obj_distances[4])
    ori_act=obj_distances[5]-obj_distances[2] #if >0 -> kiri, <0 -> kanan
    obj_dist_ori_act=(dist_act,ori_act)
    return obj_dist_ori_act
# Get the handle of motor of p3dx
motorLeftHandle=sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)[1]
motorRightHandle=sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)[1]
motors_handle=(motorLeftHandle,motorRightHandle)

#time now
samp_time, n = 0.1, 1.0
time_start=time.time()
# --- Distance & Orientation Reference
obj_dist_ori_ref = [0.5,0.0]
# --- Object Handle
sensors_handle=getSensorsHandle(clientID)

i=0
# ===  Main Program  ===
# ----------------------
print('Program Started')
while(True):
    t_now=time.time()-time_start
    if t_now >= samp_time*n:
        res, disc_handle = sim.simxGetObjectHandle(clientID, '/Disc', sim.simx_opmode_blocking)
        # Get the position of the p3dx & disc
        res, p3dx_position = sim.simxGetObjectPosition(clientID, p3dx_handle, -1, sim.simx_opmode_blocking)
        res, disc_position = sim.simxGetObjectPosition(clientID, disc_handle, -1, sim.simx_opmode_blocking)
        res, p3dx_orientation = sim.simxGetObjectOrientation(clientID, p3dx_handle, -1, sim.simx_opmode_blocking)
        res, disc_orientation = sim.simxGetObjectOrientation(clientID, disc_handle, -1, sim.simx_opmode_blocking)
        #Get sensors data
        obj_distances = getDistance(clientID,sensors_handle)
        obj_dist_ori_act = getDistanceOrientation(obj_distances)
        # Get the orientation of the p3dx & disc
        p3dx_x = p3dx_position[0]
        p3dx_y = p3dx_position[1]
        p3dx_orientation_g = p3dx_orientation[2]*180/math.pi

        disc_x = disc_position[0]
        disc_y = disc_position[1]
        disc_orientation_g = disc_orientation[2]*180/math.pi

        # Do control or navigation calculation
        ex = disc_x - p3dx_x
        ey = disc_y - p3dx_y
        theta = math.atan2(ey,ex)*180/math.pi
        etheta = theta - p3dx_orientation_g

        edist = 0.5
        ethetatol = 0.2
        Kp = 0.6
        Kd = 7

        wcom = Kp*etheta
        vcom = 65.0
        vl = vcom-wcom
        vr = vcom+wcom
        psil = vl/48.75
        psir = vr/48.75

        _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[0], psil, sim.simx_opmode_oneshot)
        _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[1], psir, sim.simx_opmode_oneshot)

        if math.sqrt((ex*ex)+(ey*ey))<edist:
            i=i+1

        if i>2:
            eg = disc_orientation_g - p3dx_orientation_g
            wcom = Kp*eg
            vl = -wcom
            vr = wcom
            psil = vl/48.75
            psir = vr/48.75
            _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[0], psil, sim.simx_opmode_oneshot)
            _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[1], psir, sim.simx_opmode_oneshot)
            _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[0], 0, sim.simx_opmode_oneshot)
            _ = sim.simxSetJointTargetVelocity(
            clientID, motors_handle[1], 0, sim.simx_opmode_oneshot)

        # Print the position and orientation of the p3dx & disc
        #print('t = {:.2f} > pose of P3DX = {:.2f},{:.2f} ; orientation of P3DX = {:.2f}'.format(t_now,p3dx_x,p3dx_y,p3dx_orientation_g))
        #print('t = {:.2f} > pose of disc = {:.2f},{:.2f} ; orientation of disc = {:.2f}'.format(t_now,disc_x,disc_y,disc_orientation_g))
        if keyboard.is_pressed('esc'): break

# Disconnect from the remote API server
# --- Simulation Finished
sim.simxFinish(clientID)
print('program ended\n')
