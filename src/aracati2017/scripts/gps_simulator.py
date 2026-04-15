#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

class GPSSimulator:
    def __init__(self):
        self.pub = rospy.Publisher('/dgps', NavSatFix, queue_size=10)
        
        # Coordenadas de referencia (puede ser cualquier punto)
        # El nodo aerial_image solo necesita que exista el mensaje,
        # luego usará las coordenadas relativas del vehículo
        self.ref_lat = 0.0
        self.ref_long = 0.0
        
        rospy.loginfo("✅ GPS Simulator iniciado (modo cartesiano)")
        rospy.loginfo("   Publicando referencia en (0,0)")
        
        # Publicar un solo mensaje (no necesita actualizarse)
        self.publish_gps()
    
    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps"
        msg.latitude = self.ref_lat
        msg.longitude = self.ref_long
        msg.altitude = 0.0
        
        self.pub.publish(msg)
        rospy.loginfo("   GPS referencia publicada")

if __name__ == '__main__':
    rospy.init_node('gps_simulator')
    GPSSimulator()
    rospy.spin()