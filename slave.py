import time
import rclpy
import math
from webots_ros2_core.webots_node import WebotsNode
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import cos, sin, tan, pi, atan2
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster 

class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__("slave_node", args)
        #enable sensors
        self.service_node_vel_timestep = 16
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)
        
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.right_motor.setPosition(float('inf'))
        self.right_motor.setVelocity(0)

        self.left_motor = self.robot.getDevice('left wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)

        self.cam = self.robot.getDevice('camera1')
        self.cam.enable(self.timestep)
        self.cam_pub = self.create_publisher(Range, 'camera', 1)

        self.right_pos_sensor = self.robot.getDevice('right wheel sensor')
        self.left_pos_sensor = self.robot.getDevice('left wheel sensor')
        self.right_pos_sensor.enable(self.service_node_vel_timestep)
        self.left_pos_sensor.enable(self.service_node_vel_timestep)
        self.sensor_pub_right_pos = self.create_publisher(Float64, 'right_PS', 1)
        self.sensor_pub_left_pos = self.create_publisher(Float64, 'left_PS', 1)

        self.front_dist_sensor = self.robot.getDevice('front_ds')
        self.right_dist_sensor = self.robot.getDevice('right_ds')
        self.left_dist_sensor = self.robot.getDevice('left_ds')
        self.front_dist_sensor.enable(self.service_node_vel_timestep)
        self.right_dist_sensor.enable(self.service_node_vel_timestep)
        self.left_dist_sensor.enable(self.service_node_vel_timestep)
        self.sensor_pub_front_dist = self.create_publisher(Range, 'front_DS', 1)
        self.sensor_pub_right_dist = self.create_publisher(Range, 'right_DS', 1)
        self.sensor_pub_left_dist = self.create_publisher(Range, 'left_DS', 1)

        #self.odom_pub = self.create_publisher(Odometry,"odom",1)
        self.odom_pub = self.create_publisher(Float64, 'odom' ,1)

        self.get_logger().info("Sensor Enabled")

        self.motor_max_speed = self.left_motor.getMaxVelocity()
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)
        
        #self.start_device_manager(DEVICE_CONFIG)#Enabling this ruins my distance sensors
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step = 0.032
        self.odom_timer = self.create_timer(self.time_step, self.odom_callback)
        self.cam_timer = self.create_timer(self.time_step, self.cam_callback)
        self.wheel_gap = 2.28  # in meter
        self.wheel_radius = 0.8  # in meter
        self.r1 = 0.0
        self.r2 = 0.0
        self.r3 = 0.0
    
    def cam_callback(self):
        msg_cam = Range()
        self.cam.recognitionEnable(self.timestep)
        recog = self.cam.getRecognitionObjects()
        f = self.front_dist_sensor.getValue()*39.37
        if(len(recog) != 0):
            if abs(recog[0].get_position()[0]) < .02:
                #green
                if(recog[0].get_colors()[0] == 0 and recog[0].get_colors()[1] == 1 and recog[0].get_colors()[2] == 0):
                    self.r2 = f + 3.14
                    #c2 +=1
                #blue
                elif(recog[0].get_colors()[0] == 0 and recog[0].get_colors()[1] == 0 and recog[0].get_colors()[2] == 1):
                    self.r1 = f + 3.14
                    #c1 +=1
                #yelow
                elif(recog[0].get_colors()[0] == 1 and recog[0].get_colors()[1] == 1 and recog[0].get_colors()[2] == 0):
                    self.r3 = f + 3.14
                    #c3 +=1

        msg_cam.range = self.r1
        msg_cam.min_range = self.r2
        msg_cam.max_range = self.r3
        
        self.cam_pub.publish(msg_cam)
    
    def odom_callback(self):
        self.publish_odom()
    
    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_odom(self):
        time.sleep(0.03)
        dt = self.time_step
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = self.euler_to_quaternion(self.th, 0.0, 0.0)

        qx = odom_quat[0]
        qy = odom_quat[1]
        qz = odom_quat[2]
        qw = odom_quat[3]
        yaw = Float64()
        yaw.data = (atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) + pi) #* (180/pi)
        self.odom_pub.publish(yaw)

    def cmdVel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       self.wheel_gap) / (2.0 * self.wheel_radius))

        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        self.wheel_gap) / (2.0 * self.wheel_radius))
        
        left_speed = min(self.motor_max_speed,
                    max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,
                    max(-self.motor_max_speed, right_speed))

        self.right_motor.setVelocity(right_speed)
        self.left_motor.setVelocity(left_speed)

        #self.get_logger().info(str(right_speed))
        #self.get_logger().info(str(right_speed))

    def sensor_callback(self):
        msg_left_PS = Float64()
        msg_right_PS = Float64()
        self.left_pos_sensor.enable(self.timestep)
        self.right_pos_sensor.enable(self.timestep)
        msg_left_PS.data = self.left_pos_sensor.getValue()
        msg_right_PS.data = self.right_pos_sensor.getValue()
        self.sensor_pub_left_pos.publish(msg_left_PS)
        self.sensor_pub_right_pos.publish(msg_right_PS)
        
        msg_front_DS = Range()
        self.front_dist_sensor.enable(self.timestep)
        msg_front_DS.range = self.front_dist_sensor.getValue()
        self.sensor_pub_front_dist.publish(msg_front_DS)

        msg_right_DS = Range()
        self.right_dist_sensor.enable(self.timestep)
        msg_right_DS.range = self.right_dist_sensor.getValue()
        self.sensor_pub_right_dist.publish(msg_right_DS)

        msg_left_DS = Range()
        self.left_dist_sensor.enable(self.timestep)
        msg_left_DS.range = self.left_dist_sensor.getValue()
        self.sensor_pub_left_dist.publish(msg_left_DS)


def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
