#!/bin/python3
# -*- coding: utf-8 -*-
'''
eye in hand
'''
import rospy
import os
import json
import time
import cv2
import numpy as np
# take FAIRINO ROBOT SDK as an example
from fairino import Robot
# ROS message type 
from std_msgs.msg import Float32MultiArray

camera2base = [ 
 [ 0.99885901,  0.02414404  ,0.04120379,99.50256158],
 [ 0.02214817, -0.99859083 , 0.04822672,-542.01673407],
 [ 0.04231012 ,-0.0472591 , -0.99798619,846.15889269],
 [ 0.0,0.0,0.0,1.0]]

tag_trans_mat = []
robot_arm = []

def init():
    '''
    init the robot arm 
    '''
    global robot_arm
    robot_arm = Robot.RPC('192.168.58.2')

def save_to_file(matrix):
    log_dir = "./log"
    file_name = "1.txt"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    file_path = os.path.join(log_dir, file_name)

    index = 1
    while os.path.exists(file_path):
        index += 1
        file_path = os.path.join(log_dir, f"{index}.txt")

    matrix_list = matrix.tolist()

    with open(file_path, 'w') as file:
        matrix_str = json.dumps(matrix_list)
        file.write(matrix_str)
        print('File has been saved!')


def camera_callback(rece_tag_trans_mat):
    '''
    ROS callback function: get the transform matrix of tag to camera from camera process
    raw data is a 2D array, but it must be compressed to 1D array when entering the ROS pipeline
    so we need to decode it to 2D array
    @input:
        rece_tag_trans_mat: tag2camera matrix from ROS
    @output:
        None
    '''
    # tag2camera matrix
    global tag_trans_mat
    if rece_tag_trans_mat.data == []:
        pass
    else :
        tag_trans_mat = rece_tag_trans_mat.data
        tag_trans_mat = list(tag_trans_mat)
        tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]

def get_transform_mat(X, Y, Z, RX, RY, RZ):
    '''
    get transform matrix from 6D data of robot arm's end
    @input:
        X, Y, Z, RX, RY, RZ: 6D data of robot arm's end
    @output:
        end_to_base: transform matrix of robot arm's end to base
    '''
    rx = np.deg2rad(RX)
    ry = np.deg2rad(RY)
    rz = np.deg2rad(RZ)

    Rx = np.array([[1, 0, 0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                [0, 1, 0],
                [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz), np.cos(rz), 0],
                [0, 0, 1]])

    R = np.dot(np.dot(Rz, Ry),Rx)

    # 平移向量
    tx = X
    ty = Y
    tz = Z

    # 变换矩阵
    end_to_base = np.array([[R[0, 0], R[0, 1], R[0, 2], tx],
                [R[1, 0], R[1, 1], R[1, 2], ty],
                [R[2, 0], R[2, 1], R[2, 2], tz],
                [0, 0, 0, 1]])
    return end_to_base
      
def tf_get_obj_to_base2(end2base,camera2end,obj2camera):
    '''
    give obj2camera and camera2base, return obj2base
    @input:
        end2base: transform matrix of end to base
        camera2end: transform matrix of camera to end
        obj2camera: transform matrix of object to camera
    @output:
        obj2base: transform matrix of object to base
    '''
    obj2base = np.dot(np.dot(end2base,camera2end),obj2camera)
    return obj2base
    
def get_RT_from_transform_mat(transform_mat):
    '''
    give M, return R and T
    @input:
        M: transform matrix
    @output:
        R: rotation matrix
        T: translation vector
    '''
    rot_mat = transform_mat[:3,:3]
    translate_mat = transform_mat[:3,3]
    return rot_mat,translate_mat


def get_transform_mat_from_RT(R,T):
    '''
    give R and T, return M
    @input:
        R: rotation matrix
        T: translation vector
    @output:
        M: transform matrix
    '''
    Z = [0.0, 0.0, 0.0, 1.0]
    M = np.vstack((np.hstack((R, T)), Z))
    return M

def main():

    R_tag2camera_list = [] 
    T_tag2camera_list = []
    R_camera2end = []
    T_camera2end = []
    R_end2base = []
    T_end2base = []
    R_end2base_list = []
    T_end2base_list = []
    R_camera2end = np.array(R_camera2end)
    T_camera2end = np.array(T_camera2end)
    global tag_trans_mat
    rospy.Subscriber('/tag_trans_mat',Float32MultiArray,camera_callback)
    sample_times = input('------Please input sampling times: (15~30 is recommanded)------')
    input('------Press ENTER to start calibration: ------')
    for i in range(int(sample_times)):
        # FAIRINO ROBOT SDK
        # robot_arm_end: 6DoF data of robot arm's end, a list of 6 float numbers, [X, Y, Z, RX, RY, RZ]
        
        robot_arm_end = robot_arm.GetActualToolFlangePose(0)
        robot_arm_end = robot_arm_end[-6:]
        
        # get end2base matrix
        end2base = get_transform_mat(robot_arm_end[0],robot_arm_end[1],robot_arm_end[2],robot_arm_end[3],robot_arm_end[4],robot_arm_end[5])
        
        # get rotation matrix and translation vector from end2base matrix
        R_end2base, T_end2base = get_RT_from_transform_mat(end2base)

        # get rotation matrix and translation vector from tag2camera matrix
        # tag2camera matrix is from ROS communication pipeline
        print('tag2camera: ',tag_trans_mat)
        tag_trans_mat = np.array(tag_trans_mat)
        R_tag2camera , T_tag2camera = get_RT_from_transform_mat(tag_trans_mat)

        # append the data to the list
        R_end2base_list.append(R_end2base)
        T_end2base_list.append(T_end2base)
        R_tag2camera_list.append(R_tag2camera)
        T_tag2camera_list.append(T_tag2camera)

        input('--------Press ENTER to add data after the robot arm moves: --------')

    R_camera2end,T_camera2end = cv2.calibrateHandEye(R_end2base_list,T_end2base_list,R_tag2camera_list,T_tag2camera_list,method=cv2.CALIB_HAND_EYE_TSAI)

    print('R_camera2end',R_camera2end)
    print('T_camera2end',T_camera2end)
    
    # save
    save_to_file(get_transform_mat_from_RT(R_camera2end,T_camera2end))

if __name__ == "__main__":
    try:
        # init your robot arm
        init()
        # run main function to calibrate
        main()
    except rospy.ROSInterruptException as e:
        print(e,"\n")