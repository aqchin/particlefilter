#!/usr/bin/env python
# license removed for brevity
import math
import rospy
import random
import map_utils
import helper_functions
from read_config import read_config
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class Robot():
    def __init__(self):

        self.config = read_config()
        rospy.init_node('Robot')
        
        self.map_subscriber = rospy.Subscriber(
                "/map", 
                OccupancyGrid,
                self.map_data)
        
        self.pose_pub = rospy.Publisher(
                "/particlecloud",
                PoseArray,
                queue_size =10
        )
        random.seed(100)
        


    def map_data(self, map_d):

        self.map_meta = MapMetaData()
        map_meta = map_d.info
        self.width = map_meta.width
        self.height = map_meta.height

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []


        self.particles = []
        for i in range(0, self.config["num_particles"]):

            x = random.randrange[0, self.width]
            y = random.randrange[0, self.height]
            angle = random.randrange[0, 2* math.pi]
            weight = (1/self.config["num_particles"])
            pose = get_pose(x, y, angle)
            
            pose_array.poses.append(pose)
            point = (x,y,angle,weight, pose)
            self.particles.append(point)
        
        self.pose_pub.publish(pose_array)
        self.rate.sleep()

        self.lmap = Map(map_d) #or map_d.data

if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
