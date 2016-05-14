#!/usr/bin/env python
# license removed for brevity
import math
import numpy
import rospy
import random
from map_utils import Map
import helper_functions
from read_config import read_config
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray
from sklearn.neighbors import KDTree
from sensor_msgs.msg import LaserScan

class Robot():
    def __init__(self):

        self.config = read_config()
        print "STARTUSSSSSSSSS"
        rospy.init_node('Robot')
        print "HAHAHAHAHAHAHAHAHA"
        self.laserStatus = False
        self.map_data_bool = True
        self.rate = rospy.Rate(1)
        self.rate.sleep()
        self.move_list = self.config["move_list"]
        self.map_subscriber = rospy.Subscriber(
                "/map",
                OccupancyGrid,
                self.map_data
        )

        self.pose_pub = rospy.Publisher(
                "/particlecloud",
                PoseArray,
                queue_size =10
        )

        self.lmap_pub = rospy.Publisher(
                "/likelihood_field",
                OccupancyGrid,
                queue_size = 10,
                latch = True
        )
        self.laser_sub = rospy.Subscriber(
                "/base_scan",
                LaserScan,
                self.updateLaserData
        )
        print "WTFFFFFFFFFFFFFFF"
        random.seed(100)
        rospy.spin()

    def updateLaserData(self, data):
        if self.laserStatus:
            self.current_scan = data
            self.laserStatus = False

    def getLaserData(self):
        self.laserStatus = True
        while self.laserStatus:
            continue


    def map_data(self, map_d):
        print "WTF PLZ" 
        if self.map_data_bool:
            self.map_data_bool = False
            self.map_meta = MapMetaData()
            map_meta = map_d.info
            self.width = map_meta.width
            self.height = map_meta.height

            self.data = map_d.data
            print "MAP DATA"
            self.pose_array = PoseArray()
            self.pose_array.header.stamp = rospy.Time.now()
            self.pose_array.header.frame_id = 'map'
            self.pose_array.poses = []


            self.particles = []
            for i in range(0, self.config["num_particles"]):

                point = []      
                point.append(random.randrange(0, self.width))
                point.append(random.randrange(0, self.height))
                d_angle = random.randrange(0, 360)
                point.append(float(d_angle*math.pi/180.0))
                point.append((1/self.config["num_particles"]))

                self.particles.append(point)
            print "POSE PRINT PUBLISH"

            self.publish_particles(self.particles)

            self.lmap = Map(map_d) #or map_d.data
            self.buildLMap()
            self.initial_move()

            self.make_move()

    def publish_particles(self, particless):

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
    
        for p in particless:
            pose = helper_functions.get_pose(p[0], p[1], p[2])
            pose_array.poses.append(pose)
        
        self.pose_pub.publish(pose_array)
        self.rate.sleep()

    def buildLMap(self):

        self.occupied = []
        self.all_points = []

        for r in range(self.height):
            for c in range(self.width):

                (x, y) = self.lmap.cell_position(r, c);
                if self.data[r*self.height + c] == 100:
                    self.occupied.append([x, y])

                self.all_points.append([x, y])


        kdt = KDTree(self.occupied)
        (dist, ind) = kdt.query(self.all_points, k=1)
        self.laser_sd = self.config["laser_sigma_hit"]
        result_pdf = [self.PDF(temp, self.laser_sd) for temp in dist]

        self.lmap.grid = numpy.reshape(result_pdf, self.lmap.grid.shape)
        self.lmap_pub.publish(self.lmap.to_message())

    def PDF(self, n, sd):
        const = 1.0 / (sd * math.sqrt(2.0*math.pi))
        e = math.exp(-0.5 * (n)**2 / (sd)**2)
        return const * e

    def initial_move(self):

        in_move = self.move_list.pop()
        move_angle = in_move[0]
        helper_functions.move_function(move_angle, 0)
        self.resample()
        for i in self.particles:
            i[2]+=random.gauss(0, self.config["first_move_sigma_angle"])
           
        self.publish_particles(self.particles)
        

        for i in range(1):
            helper_functions.move_function(0, in_move[1])
            self.resample()
            for p in self.particles:
                a= p[0]
                b = p[1]
                p[0]+=p[0]*math.cos(p[2])
                p[0]+=random.gauss(0, self.config["first_move_sigma_x"])
                
                p[1]+=p[1]*math.sin(p[2])
                p[1]+=random.gauss(0, self.config["first_move_sigma_y"])
                print "Probbbbbbbbbbbbbbbbbb"
                print a, " also ", b
                print p[0], " anda ", p[1]
                print math.fabs(p[0]+a), " AND ", math.fabs(p[1]+b)
            self.publish_particles(self.particles)

    def make_move(self):
        print "HA" 

    def resample(self):

        self.getLaserData()
        total_weight = 0
        for p in self.particles:
            x = p[0]
            y = p[1]
            (row, col) = self.lmap._cell_index(x,y)


            if self.lmap.get_cell(x, y) == 1:
                p[3] = 0
            p_total = 0
            for i in range(len(self.current_scan.ranges)):
                cur_angle = p[2] + self.current_scan.angle_min
                cur_angle += i * self.current_scan.angle_increment
                new_x = x * math.cos(cur_angle)
                new_y = y + math.sin(cur_angle)

                #Add from likelihood field
                pz = self.config["laser_z_hit"] * self.lmap.get_cell(new_x,new_y)
                pz += self.config["laser_z_rand"]
                p_total += (pz)

            p[3] = p[3] * p_total

            total_weight += p[3]

        for p in self.particles:
            p[3] /= total_weight













if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
