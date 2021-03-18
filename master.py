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

        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        #vehicle parameters
        self.finder = False #hardcode used for hitting the middle squares
        self.done = False #
        self.rot = True #lock for if it should rotate when it hits a distance
        self.speed = 3.0
        self.curZone = 16
        self.nsew = "n" #cardinal direction
        self.cmd = Twist() #forgot
        self.prev = 180 #previous angle, used to find XY
        self.direct = 180 #current direction aiming
        
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
        self.y = -15.5
        
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
                    self.nsew = "w"
                elif(self.direct == 90):
                    self.direct = 0
                    self.nsew = "s"
                elif(self.direct == 180):
                    self.direct = 90
                    self.nsew = "e"
                elif(self.direct == 270):
                    self.direct = 180
                    self.nsew = "n"
            elif(right < left): #turn left
                if(self.direct == 0 or self.direct == 360):
                    self.direct = 90
                    self.nsew = "e"
                elif(self.direct == 90):
                    self.direct = 180
                    self.nsew = "n"
                elif(self.direct == 180):
                    self.direct = 270
                    self.nsew = "w"
                elif(self.direct == 270):
                    self.direct = 0
                    self.nsew = "s"
        self.get_logger().info("New Direction:------------------")
        #self.get_logger().info("New Direction:------------------")
        self.get_logger().info(str(self.direct))
        
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
    
    #def cameraCalc(self):

    def actualProcess(self):
        front = self.f*39.37
        left = self.l*39.37
        right = self.r*39.37
        v = self.speed
        distance = (((self.lps+self.rps)/2) * .80686)# - .5
        #self.x += v*cos(self.prev)
        if(self.nsew == "w" or self.nsew == "e"):
            stopper = self.x
        else:
            stopper = self.y
        self.findXY()

        if(self.finder == True):
            self.findLoc()
            self.finder = False
            if(self.curZone == 3):
                self.direct = 0
                self.nsew = "s"
            elif(self.curZone == 15):
                self.direct = 270
                self.nsew = "w"
            elif(self.curZone == 14):
                self.direct = 180
                self.nsew = "n"
            elif(self.curZone == 2):
                self.direct = 270
                self.nsew = "w"

        #self.get_logger().info(str(self.x))
        #self.get_logger().info(str(self.y))
        self.cmd.linear.x = self.speed
        self.cmd.angular.z = 0.0
        if(abs((stopper % 10) - 5) < .27): 
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3
            self.get_logger().info(str(self.deg))
            if((self.deg > abs(self.direct-420) and self.deg < abs(self.direct-451))):
                if(self.rot == True):
                    #self.get_logger().info("setting rot false")
                    self.rot = False
            #resets boolean about when to rotate
            elif(self.deg > abs(self.direct+20) and self.deg < abs(self.direct+43)):
                if(self.rot == True):
                    #self.get_logger().info("setting rot false")
                    self.rot = False
            #stops spinning at desired rotation
            elif(self.deg > self.direct-.34 and self.deg < self.direct+.34 and self.rot == False):
                #self.get_logger().info("Stop spinning")
                self.done = True
                self.finder = True
                self.cmd.linear.x = self.speed
                self.cmd.angular.z = 0.0
            #spins slower when close to desired direction
            elif(self.direct == 0 and self.rot == False):
                if(self.deg > 356.5 or self.deg < 3.4):
                    #self.get_logger().info("Spin Slower")
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.05
            elif(self.deg > self.direct-1.75 and self.deg < self.direct+1.75 and self.rot == False):
                #self.get_logger().info("Spin Slower")
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.05
            if(self.rot == False and front < 10 and self.done == True):
                #self.get_logger().info("Setting up to change direction: Rot now True")
                #self.get_logger().info("Current front: ")
                #self.get_logger().info(str(front))
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.3
                self.rot = True
                self.done = False
                self.changeDir()
        elif(distance % 10 > 2):
            self.rot = True
            self.done = False
            #self.get_logger().info("Reset Rotation: to True")

        
        #self.get_logger().info(str(self.cmd.linear.x))
        #self.get_logger().info(str(self.cmd.angular.z))
        self.prev = self.deg
        #self.get_logger().info(str(front)) #------------------------------------------------------------------------------------------
        self.pubs_cmdvel.publish(self.cmd)

    
    #def odom_cb(self, msg: Odometry):
    #    q = msg.pose.pose.orientation
    def odom_cb(self, msg):
        self.yaw = msg.data
        self.deg = self.yaw * (180/pi)
        #self.get_logger().info(str(self.yaw))
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
        self.actualProcess()

def main(args=None):
    rclpy.init(args=args)

    fp = FindPoles()
    rclpy.spin(fp)

    fp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()