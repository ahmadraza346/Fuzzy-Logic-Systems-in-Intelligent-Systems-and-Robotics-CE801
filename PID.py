#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

class MinimalPublisher(Node):
	def __init__(self):
		super().__init__('forward_demo')
		
		self.scan_sub=self.create_subscription(LaserScan, 'scan' , self.laser_callback, qos_profile_sensor_data) 
		#....................................addding wait to robot.....................................................
		time.sleep(15)
		#........................vaiables and p,i,d values......................................................
		self.laser_ranges=[]
		self.errors=[]
		self.desired_distance=0.5
		self.ei=0
		self.p=0.695
		self.i=0.056
		self.d=0.345
		for i in range(1440):
			self.laser_ranges.append(0.0)
		self.publisher_=self.create_publisher(Twist,'cmd_vel',10)
		self.cmd=Twist()
		timer_period=1.0
		self.timer=self.create_timer(timer_period,self.timer_callback)
		self.i=0
	
	def laser_callback(self,msg):
		self.get_logger().info(str(msg.ranges[90]))
		self.laser_ranges=msg.ranges
	
	def PID(self):
		
	
#........................................calculating errors..........................................
		current_distance=min(self.laser_ranges[1040:1120])

		e=self.desired_distance-current_distance
		self.errors.append(e)
		self.ei=self.ei+e
		if(self.ei>0.4):
			self.ei=0.4
		if(self.ei<-0.4):
			self.ei=-0.4
		
		ed=e-self.errors[len(self.errors)-2]
#.....................assigning the angular as well as linear speed values....................................		
		output=self.p*e+self.i*self.ei+self.d*ed
		self.cmd.angular.z=output
		self.cmd.linear.x=0.2

			

			
	
	def timer_callback(self):
		self.PID()
		self.publisher_.publish(self.cmd)
		string= 'Publishing:' +str(self.cmd.linear.x)
		self.get_logger().info(string)
		
def main(args=None):
	rclpy.init(args=args)
	minimal_publisher= MinimalPublisher()
	rclpy.spin(minimal_publisher)
		
	minimal_publisher.destroy_node()
	rclpy.shutdown()
if __name__=='__main__':
	main()
	
		
	
