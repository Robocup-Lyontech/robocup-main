#!/usr/bin/env python
__author__ ='Florian Dupuis'

import rospy
import random
import subprocess
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import time
import json
import datetime
import os, sys

def create_model_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    if modelname[-1] == "2":
        model = modelname[:-2]
    else :
        model = modelname
    f = open('/home/student/robocup_ws/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/'+str(model)+'/'+str(model)+'.sdf')
    obj = f.read()
    # Replace modelname
    #can = can.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = obj
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req

def random_flat_3():

    poses = [[1.75, 0, 0.3, 0, 0, 0],
                [1.1, -0.2, 0.3, 0, 0, 0],
                [-4.49, 1.677, 0.98, 0, 0, 0],
                [-2.78, -0.5, 0.66, 0, -0, 0],
                [-2.5, -1.0, 0.66, 0, -0, 0],
                [-2.8, -1.67, 0.66, 0, -0, 0],
                [-6.3, -1.35, 00.66, 0, -0, -0.32],
                [-4.5, 1.47, 0.518, 0, -0, -0.32],
                [-6.63, -1.2, 0.66, 0, -0, 0],
                [-6.6, -0.88, 0.66, 0, -0, 0],
                [-4.46, 2.73, 0.952, 0, -0, -0.73],
                [-4.49, 2.29, 0.952, 0, -0, -1.8],
                [-4.48, 2.67, 0.5, 0, -0, -2.8],
                [-4.48, 2.34, 0.5, 0, -0, 2.89]]

    objects = ["ycb-sugar",
                "ycb-bleach",
                "ycb-tuna",
                "ycb-coffee",
                "ycb-mustard",
                "ycb-tomatosoup",
                "ycb-jello",
                "ycb-sugar-2",
                "sprite",
                "cocacola", 
                "ycb-tuna-2",
                "ycb-coffee-2",
                "ycb-bleach-2",
                "ycb-tomatosoup-2"]

    random_spawn(poses, objects, "flat_3")
    return

def random_flat_5():
    poses = [[3.4, -6.11, 0.953, 0, -0, 0],
            [5.04, -7.45, 0.73, 3.1415, 0, 0],
            [3.42, -7.03, 0.952, 0, -0, 2.2],
            [3.41, -6.89, 0.508, 0, 0, 0],
            [3.42, -5.75, 0.953, 0, -0, -1.41],
            [5.39, -7.7, 0.66, 0, -0, -0],
            [5.79, -7.55, 0.78, 0, 0, 0.54],
            [1.24, -5.49, 0.5, 0, -0, -1.85],
            [0.96, -5.47, 0.95, 0, -0, -1.6]]

    objects = ["ycb-sugar",
                "ycb-bleach",
                "ycb-tuna",
                "ycb-coffee",
                "ycb-mustard",
                "ycb-tomatosoup",
                "ycb-jello",
                "ycb-sugar-2",
                "sprite"]

    random_spawn(poses, objects, "flat_5")
    return


def random_ycb():

    poses = [[-0.323, 2.39, 0.3, 0, -0, 0],
            [-1.00422, 5.50495, 0.30154, 0, -0, 0],
            [1.0, -1.28, 0.523, 0, -0, 0],
            [-0.555923, 1.73, 0.3, 0, -0, 0],
            [3.22638, 4.86343, 0.66, 0, -0, 0],
            [3.89705, 6.61011, 0, 0, -0, 0],
            [3.49, 3.2, 0.66, 0, -0, 0],
            [0.374345, 5.87205, 0, 0, -0, 0],
            [-1.18, 5.31, 0.3, 0, -0, 0],
            [1.0, -1.28, 1.321, 0, -0, 0]]

    objects = ["ycb-sugar",
            "ycb-bleach",
            "ycb-tuna",
            "ycb-coffee",
            "ycb-mustard",
            "ycb-tomatosoup",
            "ycb-jello",
            "ycb-sugar-2",
            "ycb-bleach-2",
            "ycb-tomatosoup-2"]

    random_spawn(poses, objects, "ycb")
    return


def random_spawn(poses, objects, env):
    pairs = [('Environment',env)]
    for i in range(len(poses)):
        pose = random.choice(poses)
        obj = random.choice(objects)
        poses.remove(pose)
        objects.remove(obj)
        rospy.logwarn("OBJECT: %s POSE: %s",str(obj),str(pose))
        pairs.append((obj, pose))
        req = create_model_request(obj,
                            pose[0], pose[1], pose[2],  # position
                            pose[3], pose[4], pose[5],  # rotation
                            1, 1, 1)  # size
        spawn_srv.call(req)
    fichier = dict(pairs)
    file_name = "/home/student/robocup_ws/src/robocup-main/environment_manager/scripts/Simulations/"+str(env)+"_"+datetime.datetime.today().strftime("%d-%m-%Y_%H:%M:%S")+".json"
    with open(file_name, 'w') as simu :
        json.dump(fichier,simu)
    rospy.logwarn("JSON FILE DONE")
    return

def file_spawn(fichier):
    with open(fichier, 'r') as read_file:
        data = json.load(read_file)
    env = data.get("Environment")
    del data["Environment"]
    rospy.logwarn("LAUNCHING SIMULATION WITH ENVIRONMENT: %s",env)
    cmd = "roslaunch pmb2_2dnav_gazebo pmb2_mapping.launch public_sim:=true world:="+env+"_no_objects"
    subprocess.Popen(['/bin/bash','-c',cmd])

    time.sleep(20)
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.logerr("Waiting for /gazebo/spawn_sdf_model service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.logerr("Connected to service!")

    rospy.logerr("RANDOM OBJECT SPAWN BEGINNING...")

    for obj, pose in data.items():
        rospy.logwarn("OBJECT: %s POSE: %s",str(obj),str(pose))
        req = create_model_request(obj,
                            pose[0], pose[1], pose[2],  # position
                            pose[3], pose[4], pose[5],  # rotation
                            1, 1, 1)  # size
        spawn_srv.call(req)
    return


if __name__ == '__main__':
    rospy.init_node('simu_env')
    argv = rospy.myargv(argv=sys.argv)
    rospy.logwarn(argv)
    if len(argv) == 2:
        file_spawn(argv[1])
    else:
        env_names = ["ycb_no_objects","flat_3_no_objects","flat_5_no_objects"]
        env = random.choice(env_names)
        rospy.logwarn("LAUNCHING SIMULATION WITH ENVIRONMENT: %s",env)
        cmd = "roslaunch pmb2_2dnav_gazebo pmb2_mapping.launch public_sim:=true world:="+env
        subprocess.Popen(['/bin/bash','-c',cmd])

        time.sleep(20)
        spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.logerr("Waiting for /gazebo/spawn_sdf_model service...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.logerr("Connected to service!")
        
        rospy.logerr("RANDOM OBJECT SPAWN BEGINNING...")

        if env == "flat_3_no_objects" :
            random_flat_3()

        elif env == "flat_5_no_objects" :
            random_flat_5()
        
        elif env == "ycb_no_objects" :
            random_ycb()







