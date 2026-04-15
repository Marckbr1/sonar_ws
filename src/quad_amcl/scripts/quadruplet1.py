#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image
from quad_amcl.msg import StampedInteger, StampedArrayFloat
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped  # Nuevo import
import cv2
import os
import numpy as np
import tensorflow as tf

class Node(object):
    def __init__(self):
        rospy.loginfo("Starting Quadruplet with TF support...")

        # Configuración de TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.base_frame_id = "base_link"
        self.odom_frame_id = "odom"
        self.map_frame_id = "map"
        self.sonar_frame_id = "sonar_frame"

        # Variables para almacenar la última pose recibida
        self.current_pose = None
        self.last_pose_time = rospy.Time.now()

        # [Mantén todo el resto de tu inicialización existente...]
        
        # Params
        self.satellite = [] # Current satellite imgs
        self.particleCount = 0 # Last particle count
        self.lastHeader = None
        self.batchSize = 120
        self.son_ready = False
        self.sat_ready = False
        ## Neural network loading procedures

        # Get model directory
        models_dir = rospy.get_param('~model_path')

        # Load the sonar and satellite models
        self.son_model = tf.saved_model.load(os.path.join(models_dir, 'sonar'))
        self.sat_model = tf.saved_model.load(os.path.join(models_dir, 'satellite'))

        tmp_img = np.random.randn(128,256, 3).astype(np.float32)
        tmp_img = np.expand_dims(tmp_img, axis=0)

        self.son_model(tmp_img)
        ## ROS procedures
        self.br = CvBridge()

        # Publishers
        self.pubImg = rospy.Publisher('/rank', StampedArrayFloat,queue_size=3) # Particles rank

        # Subscribers
        rospy.Subscriber("/son_small",Image,self.son_callback,queue_size=3)
        rospy.Subscriber("/sat_crop",Image,self.sat_callback,queue_size=100)
        rospy.Subscriber("/n_particles",StampedInteger,self.nparticles_callback,queue_size=3)

        # Nuevo suscriptor para la pose estimada
        rospy.Subscriber("/amcl_pose", PoseStamped, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        """Callback para actualizar la pose del robot"""
        self.current_pose = msg.pose
        self.last_pose_time = rospy.Time.now()
        self.publish_tf()

    def publish_tf(self):
        """Publica todas las transformaciones necesarias"""
        if self.current_pose is None:
            return

        # 1. Publica transformación base_link -> odom
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.last_pose_time
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = self.current_pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # 2. Publica transformación base_link -> sonar_frame (posición física del sonar)
        t_sonar = geometry_msgs.msg.TransformStamped()
        t_sonar.header.stamp = self.last_pose_time
        t_sonar.header.frame_id = self.base_frame_id
        t_sonar.child_frame_id = self.sonar_frame_id
        
        # Ajusta estos valores según tu instalación física real
        t_sonar.transform.translation.x = 0.5  # 0.5m frente al robot
        t_sonar.transform.translation.y = 0.0
        t_sonar.transform.translation.z = 0.2  # 0.2m de altura
        
        # Orientación del sonar (ajustar según necesidad)
        q = quaternion_from_euler(0, 0, 0)
        t_sonar.transform.rotation.x = q[0]
        t_sonar.transform.rotation.y = q[1]
        t_sonar.transform.rotation.z = q[2]
        t_sonar.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t_sonar)

    # [Mantén todos tus otros callbacks existentes...]




    def nparticles_callback(self, msg):
        self.sat_ready = False
        self.satellite = []
        self.particleCount = msg.data
        rospy.loginfo("Received particle count = %d",self.particleCount)

    def sat_callback(self, msg):

        # If last msg timestamp is different, clear son img (Unsync)
        if self.lastHeader != None and \
            msg.header.stamp != self.lastHeader.stamp:
            rospy.loginfo("Son img unsync!!")
            self.son_ready = False

        # Update last header
        self.lastHeader = msg.header

        # Store satellite image temporary.
        sat_img = self.br.imgmsg_to_cv2(msg)

        # Preprocess satellite image.
        sat_img = cv2.resize(sat_img, (256, 128))
        sat_img = cv2.cvtColor(sat_img, cv2.COLOR_BGR2RGB)
        sat_img = np.array(sat_img, dtype=np.float32) / 255.0
        
        # Store sat img on sat_img vector
        self.satellite.append(sat_img)
        rospy.loginfo("Received sat img %d!", len(self.satellite))

        # Call process data if ready
        if len(self.satellite) >= self.particleCount:
            self.sat_ready = True

        if self.sat_ready and self.son_ready:
            self.process()

    def son_callback(self, msg):
        # If last msg timestamp is different, clear sat imgs (Unsync)
        if self.lastHeader != None and \
            msg.header.stamp != self.lastHeader.stamp:
            rospy.loginfo("Sat imgs unsync!!")
            self.sat_ready = False
            self.satellite = []
                
        # Update last header
        self.lastHeader = msg.header

        # Store sonar image temporarily.
        son_img = self.br.imgmsg_to_cv2(msg)

        # Preprocess sonar image.
        # son is already 256x128
        #son_img = cv2.resize(son_img, (256, 128))
        son_img = cv2.cvtColor(son_img, cv2.COLOR_BGR2RGB)
        son_img = np.array(son_img, dtype=np.float32) / 255.0
        son_img = np.expand_dims(son_img, axis=0)

        # Propagate through encoding network.
        self.son_out = self.son_model(son_img)[0]
        self.son_ready = True

        # Call process data if ready
        if self.son_ready and self.sat_ready:
            self.process()

    def process(self):
        rospy.loginfo("Process!")
        time_begin = rospy.Time.now()
        # Compute rank
        rank_msg = StampedArrayFloat()

        # Propagate satellite images through encoding network and process.
        self.sat_out = self.sat_model(np.stack(self.satellite, axis=0)[:120])
        # It does not work when particle count is greater than 120

        for j, sat_vec in enumerate(self.sat_out):
            distance = np.linalg.norm(self.son_out - sat_vec)
            rank_msg.data.append(distance)

        # Publish rank
        rank_msg.header = self.lastHeader
        self.pubImg.publish(rank_msg)

        # Clear previous imgs
        self.satellite = []
        self.son_ready = False
        self.sat_ready = False


        time_end = rospy.Time.now()
        duration = time_end - time_begin

        rospy.loginfo("Finished Processing in " + str(duration.to_sec()) + " secs")

    def start(self):
        rospy.loginfo("Quadruplet Ready!")
        rospy.spin()


def main():
    rospy.init_node("quadruplet_py", anonymous=False)
    node = Node()
    node.start()

if __name__ == '__main__':
    main()
