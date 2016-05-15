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

        rospy.init_node('Robot')

        self.laserStatus = False
        self.map_data_bool = True
        self.particle_weights = []
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

        random.seed(self.config["seed"])
        rospy.spin()

    def updateLaserData(self, data):
        if self.laserStatus:
            self.current_scan = data
            self.laserStatus = False

    def getLaserData(self):
        self.laserStatus = True
        while self.laserStatus:
            continue
    def create_points(self):
        count = 0
        print "creattee"
        ar = numpy.random.choice(range(len(self.not_occupied)), self.config["num_particles"])
        for a in ar:
            point = []
            (x,y) = self.not_occupied[a]
            count +=1
            point.append(x)
            point.append(y)
            d_angle = random.randrange(0, 360)
            point.append(float(d_angle*math.pi/180.0))
            point.append((1.0/self.config["num_particles"]))
            self.particles.append(point)
        print "CRAAAA"
            #    print point
        self.publish_particles()


    def map_data(self, map_d):

        if self.map_data_bool:
            self.map_data_bool = False
            self.map_meta = MapMetaData()
            map_meta = map_d.info
            self.width = map_meta.width
            self.height = map_meta.height
            self.not_occupied = list()
            self.data = map_d.data

            self.pose_array = PoseArray()
            self.pose_array.header.stamp = rospy.Time.now()
            self.pose_array.header.frame_id = 'map'
            self.pose_array.poses = []

            print "Create points "
            self.particles = []






            self.lmap = Map(map_d) #or map_d.data
            self.original_map = Map(map_d)
            self.buildLMap()
            self.create_points()
            self.initial_move()

     #       self.make_move()

    def publish_particles(self):
        print len(self.particles), " particle length"
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
    
        for p in self.particles:
            pose = helper_functions.get_pose(p[0], p[1], p[2])
            pose_array.poses.append(pose)
        
        self.pose_pub.publish(pose_array)
        self.rate.sleep()

    def buildLMap(self):

        self.occupied = []
        self.all_points = []
        
    #    for p in self.particles:
    #        (x,y) = self.lmap.cell_position(p[0], p[1])
    #        p[0] = x
    #        p[1] = y

        for r in range(self.height):
            for c in range(self.width):

                (x, y) = self.lmap.cell_position(r, c);
                if self.data[r*self.width + c] == 100:
                    self.occupied.append([x, y])
                else:
                    self.not_occupied.append((x,y))
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
        print "initial Move"
        in_move = self.move_list.pop(0)
        move_angle = in_move[0]
        print "Move angle ", move_angle
        helper_functions.move_function(move_angle, 0)

        for i in self.particles:
            i[2]+=move_angle #+ random.gauss(0, self.config["first_move_sigma_angle"])

        self.publish_particles()
        

        for i in range(in_move[2]):
            print "move step"
            helper_functions.move_function(0, in_move[1])
           
            for p in self.particles:
                a= p[0]
                b = p[1]
                p[0]+=in_move[1]*math.cos(p[2])
            #    p[0]+=random.gauss(0, self.config["first_move_sigma_x"])
                
                p[1]+=in_move[1]*math.sin(p[2])
            #    p[1]+=random.gauss(0, self.config["first_move_sigma_y"])
                print p
            self.reweight()
            self.resample()

            self.publish_particles()
        print "END OF FIRST MOVEEEEEEEEEEEEEEEEEEE"


    def make_move(self):
        while len(self.move_list) > 0:
            in_move = self.move_list.pop(0)
            move_angle = in_move[0]

            helper_functions.move_function(move_angle, 0)

            for i in self.particles:
                i[2]+=move_angle

            self.publish_particles()


            for i in range(int(in_move[2])):
                helper_functions.move_function(0, in_move[1])
                self.reweight()
                self.resample()
                for p in self.particles:
                    p[0]+=in_move[1]*math.cos(p[2])
                    p[1]+=in_move[1]*math.sin(p[2])

                self.publish_particles()

    def reweight(self):

        self.getLaserData()
        total_weight = 0.0
        count = 0
        ccnt = 0
        print "Reweight"
        for p in self.particles:
            x = p[0]
            y = p[1]
            p_total = 0.0
            #if self.original_map.get_cell(x, y) == 100:
            if self.original_map.get_cell(x, y) == 1.0 or math.isnan(self.original_map.get_cell(x,y)):
                p[3] = 0.0
                print "ZEROOOED", x, y, self.original_map.get_cell(x, y)
       #         print self.width, self.height
                ccnt += 1

            else:

                for i in range(len(self.current_scan.ranges)):
                    cur_angle = p[2] + self.current_scan.angle_min
                    cur_angle += i * self.current_scan.angle_increment
                    new_x = x + self.current_scan.ranges[i] * math.cos(cur_angle)
                    new_y = y + self.current_scan.ranges[i] * math.sin(cur_angle)

                #Add from likelihood field
                    if (i*self.current_scan.angle_increment) > self.current_scan.range_min and (i*self.current_scan.angle_increment) < self.current_scan.range_max:
                        if not math.isnan(self.lmap.get_cell(new_x, new_y)):
                            pz = self.config["laser_z_hit"] * self.lmap.get_cell(new_x,new_y) 
                        else:
                            pz = 0

                        pz += self.config["laser_z_rand"]

                        if not math.isnan(pz):
                            p_total += (pz)
        #    print p



            if not math.isnan(p_total):
                count += 1


            p[3] = p[3] * p_total
     #       if not math.isnan(p[3]):
#                total_weight += p[3]
        self.normalizeParticles()

    def normalizeParticles(self):
        print "normalize"
        particleWeights = [w * 100 for __, __, __, w in self.particles]
        print "PArticle weights" 

        print particleWeights
        totalWeights = 0.0
        for w in particleWeights:
            if math.isnan(w):
                print "naning is not good"
                totalWeights += 0.0
            else:
                totalWeights += w
        print totalWeights
        particleWeights = [w / totalWeights for w in particleWeights]
        print "Normalized weights"
        print particleWeights
        #if sum(particleWeights) != 1.0:
        #    print "Houston we've got a problem", sum(particleWeights)

        for i in range(len(self.particles)):
            self.particles[i][3] = particleWeights[i]


        #for p in self.particles:
        #    if not math.isnan(p[3]):
        #        total_weight += p[3]

        #print "total weight ", total_weight
        #for p in self.particles:
        #    if not math.isnan(p[3]):
        #        p[3] = p[3]/total_weight*1.0
        #    print p

        #countt = 0    
        #a = 0.0


        #for p in self.particles:
        #    if p[3] == 0:
        #        countt += 1
        #    
        #    a += p[3]
        #    print p[3], " AND " , a
        #print "Normalized sum " , a
        #print "COunt ", countt, " Ccnt ", ccnt



    def resample(self):
        print "Resample"  
        current_pos = list() 
        self.particle_weights = []
        fake_choices = []
        count = 0
        cnt = 0.0
        for p in self.particles:
            fake_choices.append(count)
            count += 1
            self.particle_weights.append(p[3])
            cnt += p[3]

        aa = numpy.random.choice(range(self.config["num_particles"]), self.config["num_particles"], p=self.particle_weights)
        
  #      print aa 
        temp = []
        for i in aa:
     
            temp.append(self.particles[i])
            
            if math.isnan(self.original_map.get_cell(self.particles[i][0], self.particles[i][1])):
                print "IS NAN"
        print "RESAMPLE DATA"

       # print temp
        self.particles = temp

        for p in self.particles:
            p[0]+=random.gauss(0, self.config["resample_sigma_x"])
            p[1]+=random.gauss(0, self.config["resample_sigma_y"])
            p[2]+=random.gauss(0, self.config["resample_sigma_angle"])
        print "end resample"
        print self.particles




if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
