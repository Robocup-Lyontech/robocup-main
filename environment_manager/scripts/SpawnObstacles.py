#!/usr/bin/env python
__author__ ='Florian Dupuis, GuillaumeBnd'

import rospy
import random
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse, DeleteModel, DeleteModelRequest, DeleteModelResponse
from tf.transformations import quaternion_from_euler
from copy import deepcopy

sdf_can = """<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.000000</ixx>
          <ixy>0.000000</ixy>
          <ixz>0.000000</ixz>
          <iyy>0.000000</iyy>
          <iyz>0.000000</iyz>
          <izz>0.000000</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 0.075 3.1416 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.15</length>
          </cylinder>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode />
          </friction>
          <contact>
            <ode />
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <pose>0 0 0.075 3.1416 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.15</length>
          </cylinder>
        </geometry>
        <material>
          <script>
	      <uri>model://sprite/material/scripts</uri>
	      <uri>model://sprite/material/textures</uri>
              <name>Custom/sprite</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>"""

def create_can_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    can = deepcopy(sdf_can)
    # Replace modelname
    can = can.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = can
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


if __name__ == '__main__':
    rospy.init_node('spawn_models')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    delete_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    # Spawn objects
    rospy.loginfo("Spawning Cans")
    for i in range(1,10):
      model_name = "can"+str(i)
      rospy.logwarn(model_name)
      del_req = DeleteModelRequest()
      del_req.model_name = model_name
      try:
        rospy.logwarn(delete_srv.call(del_req))
      except:
        pass
      req = create_can_request("can"+str(i),
                              (random.random()*random.randrange(-1.0,1,1)), (random.random()*random.randrange(-4.0,-1.0,1)), 0.75,  # position
                              3.1416, 0.0, 0.0,  # rotation
                              0.1, 0.1, 0.1)  # size
      spawn_srv.call(req)
    rospy.sleep(1.0)
    
