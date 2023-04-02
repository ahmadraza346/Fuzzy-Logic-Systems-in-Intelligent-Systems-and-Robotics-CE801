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
		time.sleep(15)
#...........................................values of input membership functions....................................................
		self.low=[0.1,0.1,0.25,0.5]
		self.med=[0.25,0.5,0.5,0.75]
		self.high=[0.5,0.75,1,1]
#.............................................rules inputs.............................................................		
		self.left_labels=['low','low','low','low','low','low','low','low','low','med','med','med','med','med','med','med','med','med','high','high','high','high','high','high',
		'high','high','high']
		self.center_labels=['low','low','low','med','med','med','high','high','high','low','low','low','med','med','med','high','high','high',
		'low','low','low','med','med','med','high','high','high']
		self.right_labels=['low','med','high','low','med','high','low','med','high','low','med','high','low','med','high','low','med','high',
		'low','med','high','low','med','high','low','med','high']
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
		if(x>=0.1 and x<=0.5):
			value.append(self.fuzzify(x,self.low))
			label.append('low')
		if(x>=0.25 and x<=0.75):
			value.append(self.fuzzify(x,self.med))
			label.append('med')
		if(x>=0.5):
			if x>1.0:
				x=1.0
			value.append(self.fuzzify(x,self.high))
			label.append('high')
		return value,label

#.........................................................defining the rule base............................................................	
	
	def rules(self,label_left,label_center,label_right):
		if(label_left=='low' and label_center=='low' and label_right=='low'): #1
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='low' and label_right=='med'): #2
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='low' and label_right=='high'): #3
			turning='right'
			speed='med'
		if(label_left=='low' and label_center=='med' and label_right=='low'): #4
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='med' and label_right=='med'): #5
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='med' and label_right=='high'): #6
			turning='right'
			speed='med'
		if(label_left=='low' and label_center=='high' and label_right=='low'): #7
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='high' and label_right=='med'): #8
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='high' and label_right=='high'): #9
			turning='right'
			speed='low'
		if(label_left=='med' and label_center=='low' and label_right=='low'): #10
			turning='left'
			speed='low'
		if(label_left=='med' and label_center=='low' and label_right=='med'): #11
			turning='left'
			speed='low'
		if(label_left=='med' and label_center=='low' and label_right=='high'): #12
			turning='right'
			speed='low'
		if(label_left=='med' and label_center=='med' and label_right=='low'): #13
			turning='left'
			speed='med'
		if(label_left=='med' and label_center=='med' and label_right=='med'): #14
			turning='med'
			speed='low'
		if(label_left=='med' and label_center=='med' and label_right=='high'): #15
			turning='right'
			speed='med'
		if(label_left=='med' and label_center=='high' and label_right=='low'): #16
			turning='left'
			speed='low'
		if(label_left=='med' and label_center=='high' and label_right=='med'): #17
			turning='left'
			speed='low'
		if(label_left=='med' and label_center=='high' and label_right=='high'): #18
			turning='right'
			speed='low'
		if(label_left=='high' and label_center=='low' and label_right=='low'): #19
			turning='left'
			speed='med'
		if(label_left=='high' and label_center=='low' and label_right=='med'): #20
			turning='left'
			speed='low'
		if(label_left=='high' and label_center=='low' and label_right=='high'): #21
			turning='left'
			speed='low'
		if(label_left=='high' and label_center=='med' and label_right=='low'): #22
			turning='left'
			speed='med'
		if(label_left=='high' and label_center=='med' and label_right=='med'): #23
			turning='left'
			speed='med'
		if(label_left=='high' and label_center=='med' and label_right=='high'): #24
			turning='left'
			speed='low'
		if(label_left=='high' and label_center=='high' and label_right=='low'): #25
			turning='left'
			speed='med'
		if(label_left=='high' and label_center=='high' and label_right=='med'): #26
			turning='left'
			speed='med'
		if(label_left=='high' and label_center=='high' and label_right=='high'): #27
			turning='med'
			speed='med'	

		return turning,speed
	
#....................................taking the membership values and appending them to a column...................................
		
	def firing_rules(self, value,returned_labels,labels):
		return_values=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		for i in range(len(returned_labels)):
			for j in range (len(labels)):
				if(returned_labels[i]==labels[j]):
					return_values[j]=value[i]
		return return_values


#..................................................................getting centroid values from membership functions of outputs..................................................	
	def get_centroid_speed(self,label):
		if(label=='low'):
			centroid=0.025
		elif(label=='med'):
			centroid=0.125
		elif(label=='high'):
			centroid=0.225
		return centroid
	
	def get_centroid_turning(self,label):
		if(label=='left'):
			centroid=0.7
		elif(label=='med'):
			centroid=0.0
		elif(label=='right'):
			centroid=-0.7
		return centroid
			
	

		
	def r_e_f(self):
#...............................................laser ranges....................................................
		self.left=min(self.laser_ranges[60:160])
		self.center=self.laser_ranges[0]
		self.right=min(self.laser_ranges[1280:1380])
#..........................................declaring neceassray lists................................................		
		self.turning_labels=[]
		self.speed_labels=[]
		self.turning_values=[]
		self.speed_values=[]
		self.left_values_returned=[]
		self.center_values_returned=[]
		self.right_values_returned=[]

#................................................getting the membership values and labels...............................................
		
		self.left_value,self.left_label=self.partition(self.left)
		self.center_value,self.center_label=self.partition(self.center)
		self.right_value,self.right_label=self.partition(self.right)
		
#.................................calling rules to save them in a list...........................................................			
		for i in range(len(self.left_labels)):
			t,s=self.rules(self.left_labels[i],self.center_labels[i],self.right_labels[i])
			self.turning_labels.append(t)
			self.speed_labels.append(s)

#.............................................getting the firing values................................................			
		self.left_values_returned=self.firing_rules(self.left_value,self.left_label,self.left_labels)
		self.center_values_returned=self.firing_rules(self.center_value,self.center_label,self.center_labels)
		self.right_values_returned=self.firing_rules(self.right_value,self.right_label,self.right_labels)
		
		for i in range(len(self.left_values_returned)):
			self.turning_values.append(min(self.left_values_returned[i],self.center_values_returned[i],self.right_values_returned[i]))
			self.speed_values.append(min(self.left_values_returned[i],self.center_values_returned[i],self.right_values_returned[i]))


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
	
		
	
