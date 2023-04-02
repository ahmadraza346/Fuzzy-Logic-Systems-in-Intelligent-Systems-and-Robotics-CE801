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
		time.sleep(10)
#...........................................values of input membership functions....................................................
		self.low=[0,0,0.25,0.5]
		self.med=[0.25,0.5,0.5,0.75]
		self.high=[0.5,0.75,1.5,1.5]
#.............................................rules inputs.............................................................
		self.front_labels=['low','low','low','med','med','med','high','high','high']
		self.back_labels=['low','med','high','low','med','high','low','med','high']
		self.laser_ranges=[]
		
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
	
	
		

#.............................................................returning the membership values.................................................................................
	def fuzzify(self,x,A):
		if(x>=A[0] and x<A[1]):
			return ((x-A[0])/(A[1]-A[0]))
		elif(x>=A[1] and x<=A[2]):
			return 1
		elif(x>A[2] and x<A[3]):
			return ((A[3]-x)/(A[3]-A[2]))
		else:
			return 0
#..........................................................Take sensor values as inputs and pass them to fuzzify function...................................................	
	def partition(self,x):
		value=[]
		label=[]
		if(x>=0 and x<=0.5):
			value.append(self.fuzzify(x,self.low))
			label.append('low')
		if(x>=0.5 and x<=0.75):
			value.append(self.fuzzify(x,self.med))
			label.append('med')
		if(x>=0.5):
			if x>1.5:
				x=1.5
			value.append(self.fuzzify(x,self.high))
			label.append('high')
		return value,label

	
#.........................................................defining the rule base............................................................
	
	def rules(self,label_front,label_back):
		if(label_front=='low' and label_back=='low'):
			turning='left'
			speed='low'
		elif(label_front=='low' and label_back=='med'):
			turning='left'
			speed='low'
		elif(label_front=='low' and label_back=='high'):
			turning='left'
			speed='med'
		elif(label_front=='med' and label_back=='low'):
			turning='right'
			speed='med'
		elif(label_front=='med' and label_back=='med'):
			turning='med'
			speed='med'
		elif(label_front=='med' and label_back=='high'):
			turning='left'
			speed='med'
		elif(label_front=='high' and label_back=='low'):
			turning='right'
			speed='low'
		elif(label_front=='high' and label_back=='med'):
			turning='right'
			speed='low'
		elif(label_front=='high' and label_back=='high'):
			turning='right'
			speed='med'
		return turning,speed
	

#....................................taking the membership values and appending them to a column...................................		
	def firing_rules(self, value,returned_labels,labels):
		return_values=[0,0,0,0,0,0,0,0,0]
		for i in range(len(returned_labels)):
			for j in range (len(labels)):
				if(returned_labels[i]==labels[j]):
					return_values[j]=value[i]
		return return_values
#..................................................................getting centroid values from membership functions of outputs..................................................
	def get_centroid_speed(self,label):
		if(label=='low'):
			centroid=0.05
		elif(label=='med'):
			centroid=0.15
		elif(label=='high'):
			centroid=0.25
		return centroid
	
	def get_centroid_turning(self,label):
		if(label=='left'):
			centroid=0.3
		elif(label=='med'):
			centroid=0.0
		elif(label=='right'):
			centroid=-0.3
		return centroid
			
	

#......................................................................calling ref following..........................................		
	def r_e_f(self):
#...............................................laser ranges....................................................
		self.front=min(self.laser_ranges[1140:1220])
		self.back=min(self.laser_ranges[960:1040])
#..........................................declaring neceassray lists................................................		
		self.turning_labels=[]
		self.speed_labels=[]
		self.turning_values=[]
		self.speed_values=[]
		self.front_values_returned=[]
		self.back_values_returned=[]

#................................................getting the membership values and labels................................................		
		self.front_value,self.front_label=self.partition(self.front)
		self.back_value,self.back_label=self.partition(self.back)
		
#.................................calling rules to save them in a list...........................................................		
		for i in range(len(self.front_labels)):
			t,s=self.rules(self.front_labels[i],self.back_labels[i])
			self.turning_labels.append(t)
			self.speed_labels.append(s)
			
#.............................................getting the firing values................................................			
		self.front_values_returned=self.firing_rules(self.front_value,self.front_label,self.front_labels)
		self.back_values_returned=self.firing_rules(self.back_value,self.back_label,self.back_labels)
		
		for i in range(len(self.front_values_returned)):
			self.turning_values.append(min(self.front_values_returned[i],self.back_values_returned[i]))
			self.speed_values.append(min(self.front_values_returned[i],self.back_values_returned[i]))
		print(self.turning_values)
		
#..................................................defuzzifying the output turning...................................................		
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.turning_values)):
			if self.turning_values[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.turning_values[i]*self.get_centroid_turning(self.turning_labels[i]))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.turning_values[i]
		self.cmd.angular.z=self.defuzzifier_numerator/self.defuzzifier_denominator
		
		
#..............................................defuzzifying the output speed.............................................................................		
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.speed_values)):
			if self.speed_values[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.speed_values[i]*self.get_centroid_speed(self.speed_labels[i]))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.speed_values[i]
		self.cmd.linear.x=self.defuzzifier_numerator/self.defuzzifier_denominator
			

			
	
	def timer_callback(self):
		self.r_e_f()
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
	
		
	
