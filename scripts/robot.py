#!/usr/bin/env python
# license removed for brevity
import math
import rospy
import random
import map_utils
import helper_functions
from read_config import read_config
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray
from sklearn.neighbors import KDTree
from sensor_msgs.msg import LaserScan

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

        random.seed(100)

    def updateLaserData(self, data):
        if self.laserStatus:
            self.current_scan = data
            self.laserStatus = False

    def getLaserData(self):
        self.laserStatus = True
        while self.laserStatus:
            continue


    def map_data(self, map_d):

        self.map_meta = MapMetaData()
        map_meta = map_d.info
        self.width = map_meta.width
        self.height = map_meta.height
        self.data = map_d.data

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

    def buildLMap(self):

        self.occupied = []
        self.all_points = []

        for r in range(len(self.height)):
            for c in range(len(self.width)):

                (x, y) = self.lmap.cell_position(r, c);
                if(self.data[r][c] == 100)
                    self.occupied.append([x, y])

                self.all_points.append([x, y])


        kdt = KDTree(self.occupied)
        (dist, ind) = kdt.query(self.all_points, k=1)
        self.laser_sd = self.config["laser_sigma_hit"]
        result_pdf = [self.PDF(temp, self.laser_sd) for temp in kdt]

        self.lmap.grid = numpy.reshape(result_pdf, self.lmap.grid.shape)
        self.lmap_pub.publish(self.lmap.to_message())

    def PDF(self, n, sd):
        const = 1.0 / (sd * math.sqrt(2.0*math.pi))
        e = math.exp(-0.5 * (n)**2 / (sd)**2)
        return const * e

    def initial_move(self, WTFFF):

        in_move = self.move_list.pop()
        move_angle = in_move[0]
        move_function(move_angle, 0)

        for i in self.particles:
            particles[2]+=random.gauss(0, self.config["first_move_sigma_angle"])

        #TO DO PUBLISH PARticles

        for i in range(in_move[2]):
            move_function(0, in_move[1])




    def resample(self, ????):

        getLaserData()
        total_weight = 0
        for p in self.particles:
            x = p.x
            y = p.y
            if map.data[x][y] == 100:
                p.weight = 0
            p_total = 0
            for i in range(len(current_scan.ranges)):
                cur_angle = p.angle + current_scan.angle_min
                cur_angle += i * current_scan.angle_increment
                new_x = x * math.cos(cur_angle)
                new_y = y + math.sin(cur_angle)

                #Add from likelihood field
                pz = self.config["laser_z_hit"] * likelihoodfield[new_x][new_y]
                pz += self.config["laser_z_rand"]
                p_total += (pz**2)

            p.weight = p.weight * p_total

            total_weight += p.weight

        for p in self.particles:
            p.weight /= total_weight













if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
