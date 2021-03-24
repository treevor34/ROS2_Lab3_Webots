import rclpy
import time
import math
from math import cos, sin, tan, pi, atan2
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

class FindPoles(Node):
    def __init__(self):
        super().__init__('findpoles_cmdvel')
        #leave a like and subscribe
        self.subs_front_ds = self.create_subscription(Range, 'front_DS', self.frontds_cb, 1)
        self.subs_right_ds = self.create_subscription(Range, 'right_DS', self.rightds_cb, 1)
        self.subs_left_ds = self.create_subscription(Range, 'left_DS', self.leftds_cb, 1)
        self.subs_left_ps = self.create_subscription(Float64, 'left_PS', self.leftps_cb, 1)
        self.subs_right_ps = self.create_subscription(Float64, 'right_PS', self.rightps_cb, 1)
        #self.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        self.create_subscription(Float64, 'odom', self.odom_cb, 1)
        self.create_subscription(Range, 'camera', self.cam_cb, 1)

        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        #vehicle parameters
        self.finder = False #hardcode used for hitting the middle squares
        self.done = False #
        self.rot = True #lock for if it should rotate when it hits a distance
        self.speed = 3.0
        self.curZone = 16
        #self.nsew = "n" #cardinal direction
        self.cmd = Twist() #forgot
        self.prev = 180 #previous angle, used to find XY
        self.direct = 180 #current direction aiming
        self.switcher = 0
        
        self.f = 0.0 #distance sensors
        self.r = 0.0
        self.l = 0.0

        self.pastD = 0.0 #uh
        self.deg = 0.0 #degrees
        self.yaw = 0.0 #yaw

        self.lps = 0.0 #position sensors
        self.rps = 0.0

        #???self.rotCount = 0
        self.x = 15.0 #X and Y
        self.y = -15.0

        self.r1 = 0.0 #blue
        self.r2 = 0.0 #green
        self.r3 = 0.0 #yellow

        self.complete = False
        
        self.zone = [".", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", "."]
    
    def findLoc(self):
        z = 0
        if(self.x > 10):
            if(self.y > 10):
                z = 4
            elif(self.y > 0):
                z = 8
            elif(self.y < -10):
                z = 16
            elif(self.y < 0):
                z = 12
        elif(self.x > 0):
            if(self.y > 10):
                z = 3
            elif(self.y > 0):
                z = 7
            elif(self.y < -10):
                z = 15
            elif(self.y < 0):
                z = 11
        elif(self.x < -10):
            if(self.y > 10):
                z = 1
            elif(self.y > 0):
                z = 5
            elif(self.y < -10):
                z = 13
            elif(self.y < 0):
                z = 9
        elif(self.x < 0):
            if(self.y > 10):
                z = 2
            elif(self.y > 0):
                z = 6
            elif(self.y < -10):
                z = 14
            elif(self.y < 0):
                z = 10            
            
        if(z != 0):
            self.zone[z - 1] = "X"
        self.curZone = z
        #self.get_logger().info(str(self.zone[15]))
    
    def changeDir(self):
        left = self.l*39.37
        right = self.r*39.37
        front = self.f*39.37
        if(front < 10):
            if(left < right): #turn right
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 270
                else:
                    self.direct -= 90
            elif(right < left): #turn left
                if(self.direct == 270):
                    self.direct = 0
                else:
                    self.direct += 90
            self.rot = True
            self.done = False
        elif(self.direct == 180 and self.zone[self.curZone -5] == "X"):
            if(left < right): #turn right
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 270
                else:
                    self.direct -= 90
            elif(right < left): #turn left
                if(self.direct == 270):
                    self.direct = 0
                else:
                    self.direct += 90
            self.rot = True
            self.done = False
        elif(self.direct == 270 and self.zone[self.curZone - 2] == "X"):
            if(left < right): #turn right
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 270
                else:
                    self.direct -= 90
            elif(right < left): #turn left
                if(self.direct == 270):
                    self.direct = 0
                else:
                    self.direct += 90
            self.rot = True
            self.done = False
        elif(self.direct == 0 and self.zone[self.curZone + 3] == "X"):
            if(left < right): #turn right
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 270
                else:
                    self.direct -= 90
            elif(right < left): #turn left
                if(self.direct == 270):
                    self.direct = 0
                else:
                    self.direct += 90
            self.rot = True
            self.done = False
        elif(self.direct == 90 and self.zone[self.curZone] == "X"):
            if(left < right): #turn right
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 270
                else:
                    self.direct -= 90
            elif(right < left): #turn left
                if(self.direct == 270):
                    self.direct = 0
                else:
                    self.direct += 90
            self.rot = True
            self.done = False

        #self.get_logger().info("New Direction:------------------")
        #self.get_logger().info(str(self.direct))
        
    def findXY(self):
        distance = ((self.lps+self.rps)/2) * .80686
        distStep = distance - self.pastD
        if(self.yaw >= 270):
            pyX = distStep*(math.sin((2*math.pi) - self.yaw))
            pyY = distStep*(math.cos((2*math.pi) - self.yaw))
            self.x = self.x - pyX
            self.y = self.y - pyY
        elif(self.yaw >= 180 and self.yaw <= 270):
            pyX = distStep*(math.sin(self.yaw - math.pi))
            pyY = distStep*(math.cos(self.yaw - math.pi))
            self.x = self.x - pyX
            self.y = self.y + pyY
        elif(self.yaw >= 90 and self.yaw <= 180):
            pyX = distStep*(math.sin(math.pi - self.yaw))
            pyY = distStep*(math.cos(math.pi - self.yaw))
            self.x = self.x + pyX
            self.y = self.y + pyY
        elif(self.yaw <= 90):
            pyX = distStep*(math.sin(self.yaw))
            pyY = distStep*(math.cos(self.yaw))
            self.x = self.x + pyX
            self.y = self.y - pyY
        self.pastD = distance        
    
    def cameraCalc(self):
        x1 = -20
        y1 = -20
        x2 = -20
        y2 = 20
        x3 = 20
        y3 = 20
        a = (-2 * x1) + (2 * x2)
        b = (-2 * y1) + (2 * y2)
        c = ((self.r1 * self.r1) - (self.r2 * self.r2) - (x1 * x1) + (x2 * x2) - (y1 * y1) + (y2 * y2))
        d = (-2 * x2) + (2 * x3)
        e = (-2 * y2) + (2 * y3)
        f = ((self.r2 * self.r2) - (self.r3 * self.r3) - (x2 * x2) + (x3 * x3) - (y2 * y2) + (y3 * y3))
        self.x = (((c*e) - (f*b)) / ((e*a) - (b*d)))
        self.y = (((c*d) - (a*f)) / ((b*d) - (a*e)))

    def printer(self):
        self.get_logger().info("X:")
        self.get_logger().info(str(self.x))
        self.get_logger().info("Y:")
        self.get_logger().info(str(self.y))
        self.get_logger().info("Current Zone:")
        self.findLoc()
        self.get_logger().info(str(self.curZone))
        self.get_logger().info("Horrizontal map:")
        self.get_logger().info(str(self.zone))
        count = 0
        cc = 0
        for i in self.zone:
            if(self.zone[cc] == "X"):
                count += 1
            cc += 1
        if count == 16:
            self.complete = True

    def actualProcess(self):
        front = self.f*39.37
        left = self.l*39.37
        right = self.r*39.37
        if((self.switcher % 2) == 0):
            #self.get_logger().info("Swap")
            v_rot = -.3
            v_rot_slow = -.05
        elif((self.switcher % 2) != 0):
            #self.get_logger().info("Swap")
            v_rot = .3
            v_rot_slow = .05
        
        distance = (((self.lps+self.rps)/2) * .80686)# - .5

        #if(self.nsew == "w" or self.nsew == "e"):
        if(self.direct == 90 or self.direct == 270):
            stopper = self.x
        else:
            stopper = self.y
        self.findXY()

#        if(self.finder == True):
#            self.findLoc()
            #self.get_logger().info("Finder: Zone = ")
            #self.get_logger().info(str(self.curZone))
#            self.finder = False
#            if(self.curZone == 3):
#                self.direct = 0
                #self.nsew = "s"
#                self.done = False
#            elif(self.curZone == 15):
#                self.direct = 270
                #self.nsew = "w"
#                self.done = False
#            elif(self.curZone == 14):
#                self.direct = 180
                #self.nsew = "n"
#                self.done = False
#            elif(self.curZone == 2):
#                self.direct = 270
                #self.nsew = "w"
#                self.done = False



        #self.get_logger().info(str(self.x))
        #self.get_logger().info(str(self.y))
        #self.get_logger().info(str(self.done))
        #self.get_logger().info(str(self.switcher))
        #self.get_logger().info(str(self.tmpDist))
        self.cmd.linear.x = self.speed
        self.cmd.angular.z = 0.0
        #if(abs((stopper % 10) - 5) < .27 and self.done == False): 
        if(abs(distance % 10) < .27 and self.done == False): 
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = v_rot
            #self.get_logger().info(str(self.deg))
            if((self.deg < abs(self.direct-5) and self.deg > abs(self.direct-14))):
                self.rot = False
                #self.get_logger().info("Reset Rot false")
            #resets boolean about when to rotate
            elif(self.deg > abs(self.direct+5) and self.deg < abs(self.direct+14)):
                #self.get_logger().info("Reset Rot false")
                self.rot = False
            #stops spinning after 1 desired rotation
            elif(self.deg > self.direct-.34 and self.deg < self.direct+.34 and self.rot == False):
                #self.get_logger().info("STOPPP IF")
                self.done = True
                self.finder = True
                self.cmd.linear.x = self.speed
                self.cmd.angular.z = 0.0
                self.switcher += 1
                self.cameraCalc()
                self.printer()
            #spins slower when close to desired direction
            elif(self.direct == 0 and self.rot == False):
                if(self.deg > 356.5 or self.deg < 3.4):
                    #self.get_logger().info("Spins slow 0")
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = v_rot_slow
            #same as previous
            elif(self.deg > self.direct-1.75 and self.deg < self.direct+1.75 and self.rot == False):
                #self.get_logger().info("Spins slower")
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = v_rot_slow
            #This is to change direction
            if(self.rot == False and self.done == True): #and front < 10
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = v_rot
                #self.rot = True
                #self.done = False
                self.changeDir()
                #self.get_logger().info("Change Direction IF")
        #resets "done" boolean after driving a bit
        elif((stopper % 10) < .27): #need to change this soon
            self.rot = True
            self.done = False
            #self.get_logger().info("Reset done")
            

        
        #self.get_logger().info(str(self.cmd.linear.x))
        #self.get_logger().info(str(self.cmd.angular.z))
        self.prev = self.deg
        self.pubs_cmdvel.publish(self.cmd)

    def odom_cb(self, msg):
        self.yaw = msg.data
        self.deg = self.yaw * (180/pi)
        #self.get_logger().info(str(self.yaw))
    def cam_cb(self, msg):
        #self.get_logger().info("blue, green, yellow")
        self.r1 = msg.range
        self.r2 = msg.min_range
        self.r3 = msg.max_range
        #self.get_logger().info(str(self.r1))
        #self.get_logger().info(str(self.r2))
        #self.get_logger().info(str(self.r3))
    def frontds_cb(self, msg):
        self.f = msg.range
    def rightds_cb(self, msg):
        self.r = msg.range
    def leftds_cb(self, msg):
        self.l = msg.range
    def leftps_cb(self, msg):
        self.lps = msg.data
    def rightps_cb(self, msg):
        self.rps = msg.data
        #self.get_logger().info("New Loop---------")
        if(self.complete == False):
            self.actualProcess()

def main(args=None):
    rclpy.init(args=args)

    fp = FindPoles()
    rclpy.spin(fp)

    fp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()