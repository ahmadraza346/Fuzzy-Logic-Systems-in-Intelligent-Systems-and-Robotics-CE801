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
#...........................................values of input membership functions for obs avoidance....................................................
		self.low=[0.1,0.1,0.25,0.5]  
		self.med=[0.25,0.5,0.5,0.75]
		self.high=[0.5,0.75,1,1]
#........................centroid values in output membership functions for the obs avoidance...................................
		self.centroids_speed=[0.025,0.125,0.225]
		self.centroids_turning=[0.75,0,-0.75]
#.............................................rules inputs for obs avoidance.............................................................
		self.left_labels=['low','low','low','low','low','low','low','low','low','med','med','med','med','med','med','med','med','med','high','high','high','high','high','high',
		'high','high','high']
		self.center_labels=['low','low','low','med','med','med','high','high','high','low','low','low','med','med','med','high','high','high',
		'low','low','low','med','med','med','high','high','high']
		self.right_labels=['low','med','high','low','med','high','low','med','high','low','med','high','low','med','high','low','med','high',
		'low','med','high','low','med','high','low','med','high']
#...........................................values of input membership functions for ref....................................................
		self.low1=[0,0,0.25,0.5]
		self.med1=[0.25,0.5,0.5,0.75]
		self.high1=[0.5,0.75,1.5,1.5]
#........................centroid values in output membership functions for the ref...................................		
		self.centroids1_speed=[0.05,0.15,0.25]
		self.centroids1_turning=[0.75,0,-0.75]
#.............................................rules inputs for ref.............................................................		
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
	def partition(self,x,low,med,high):
		value=[]
		label=[]
		if(x>=low[0] and x<=low[3]):
			value.append(self.fuzzify(x,low))
			label.append('low')
		if(x>=med[0] and x<=med[3]):
			value.append(self.fuzzify(x,med))
			label.append('med')
		if(x>=high[0]):
			if x>high[3]:
				x=high[3]
			value.append(self.fuzzify(x,high))
			label.append('high')
		return value,label
#..........................................................Take sensor values as inputs and pass them to fuzzify function...................................................		
	def partition_obs_ref(self,x,obs,ref):
		value=[]
		label=[]
		if(x>=obs[0] and x<=obs[3]):
			if x>obs[3]:
				x=obs[3]
			value.append(self.fuzzify(x,obs))
			label.append('obs')
		if(x>=ref[0] ):
			if(x>ref[3]):
				x=ref[3]
			value.append(self.fuzzify(x,ref))
			label.append('ref')
		return value,label
	
	
#.........................................................defining the rule base for obs avoidance............................................................
	def rules(self,label_left,label_center,label_right):
		if(label_left=='low' and label_center=='low' and label_right=='low'): #1
			turning='left'
			speed='low'
		if(label_left=='low' and label_center=='low' and label_right=='med'): #2
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='low' and label_right=='high'): #3
			turning='right'
			speed='med'
		if(label_left=='low' and label_center=='med' and label_right=='low'): #4
			turning='left'
			speed='low'
		if(label_left=='low' and label_center=='med' and label_right=='med'): #5
			turning='right'
			speed='low'
		if(label_left=='low' and label_center=='med' and label_right=='high'): #6
			turning='right'
			speed='med'
		if(label_left=='low' and label_center=='high' and label_right=='low'): #7
			turning='left'
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

	
#.........................................................defining the rule base for ref............................................................
	def rules1(self,label_front,label_back):
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
	def firing_rules(self, value,returned_labels,labels,return_values):
		
		for i in range(len(returned_labels)):
			for j in range (len(labels)):
				if(returned_labels[i]==labels[j]):
					return_values[j]=value[i]
		return return_values
#..................................................................getting centroid values from membership functions of outputs..................................................
	def get_centroid_speed(self,label,center):
		if(label=='low'):
			centroid=center[0]
		elif(label=='med'):
			centroid=center[1]
		elif(label=='high'):
			centroid=center[2]
		return centroid
	
	def get_centroid_turning(self,label,center):
		if(label=='left'):
			centroid=center[0]
		elif(label=='med'):
			centroid=center[1]
		elif(label=='right'):
			centroid=center[2]
		return centroid
			


		
	def merged(self):
#...............................................laser ranges for obs avoidance....................................................
		self.left=min(self.laser_ranges[60:160])
		self.center=self.laser_ranges[0]
		self.right=min(self.laser_ranges[1280:1380])
		self.turning_labels=[]
		self.speed_labels=[]
		self.turning_values=[]
		self.speed_values=[]
		self.left_values_returned=[]
		self.center_values_returned=[]
		self.right_values_returned=[]

#................................................getting the membership values and labels for obs avoidance................................................			
		self.left_value,self.left_label=self.partition(self.left,self.low,self.med,self.high)
		self.center_value,self.center_label=self.partition(self.center,self.low,self.med,self.high)
		self.right_value,self.right_label=self.partition(self.right,self.low,self.med,self.high)
		
#.................................calling rules to save them in a list for obs...........................................................		
		for i in range(len(self.left_labels)):
			t,s=self.rules(self.left_labels[i],self.center_labels[i],self.right_labels[i])
			self.turning_labels.append(t)
			self.speed_labels.append(s)
#.............................................getting the firing values for obs................................................				
		return_val=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]	
		self.left_values_returned=self.firing_rules(self.left_value,self.left_label,self.left_labels,return_val)
		self.center_values_returned=self.firing_rules(self.center_value,self.center_label,self.center_labels,return_val)
		self.right_values_returned=self.firing_rules(self.right_value,self.right_label,self.right_labels,return_val)
		
		for i in range(len(self.left_values_returned)):
			self.turning_values.append(min(self.left_values_returned[i],self.center_values_returned[i],self.right_values_returned[i]))
			self.speed_values.append(min(self.left_values_returned[i],self.center_values_returned[i],self.right_values_returned[i]))

#..................................................defuzzifying the outputs for obs avoidance...................................................		
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.turning_values)):
			if self.turning_values[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.turning_values[i]*self.get_centroid_turning(self.turning_labels[i],self.centroids_turning))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.turning_values[i]
		obs_avoidance_turning=self.defuzzifier_numerator/self.defuzzifier_denominator
		
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.speed_values)):
			if self.speed_values[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.speed_values[i]*self.get_centroid_speed(self.speed_labels[i],self.centroids_speed))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.speed_values[i]
		obs_avoidance_speed=self.defuzzifier_numerator/self.defuzzifier_denominator
			
#...............................................laser ranges for obs avoidance....................................................
		self.front=min(self.laser_ranges[1140:1220])
		self.back=min(self.laser_ranges[960:1040])
		self.turning_labels1=[]
		self.speed_labels1=[]
		self.turning_values1=[]
		self.speed_values1=[]
		self.front_values_returned=[]
		self.back_values_returned=[]
#................................................getting the membership values and labels for ref................................................			
		self.front_value,self.front_label=self.partition(self.front,self.low1,self.med1,self.high1)
		self.back_value,self.back_label=self.partition(self.back,self.low1,self.med1,self.high1)

#.................................calling rules to save them in a list for ref...........................................................			
		for i in range(len(self.front_labels)):
			t,s=self.rules1(self.front_labels[i],self.back_labels[i])
			self.turning_labels1.append(t)
			self.speed_labels1.append(s)
			
#.............................................getting the firing values for ref................................................	
		return_val=[0,0,0,0,0,0,0,0,0]	
		self.front_values_returned=self.firing_rules(self.front_value,self.front_label,self.front_labels,return_val)
		self.back_values_returned=self.firing_rules(self.back_value,self.back_label,self.back_labels,return_val)
		
		for i in range(len(self.front_values_returned)):
			self.turning_values1.append(min(self.front_values_returned[i],self.back_values_returned[i]))
			self.speed_values1.append(min(self.front_values_returned[i],self.back_values_returned[i]))

#..................................................defuzzifying the outputs for obs avoidance...................................................			
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.turning_values1)):
			if self.turning_values1[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.turning_values1[i]*self.get_centroid_turning(self.turning_labels1[i],self.centroids1_turning))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.turning_values1[i]
		ref_turning=self.defuzzifier_numerator/self.defuzzifier_denominator
		
		self.defuzzifier_numerator=0
		self.defuzzifier_denominator=0
		
		for i in range(len(self.speed_values1)):
			if self.speed_values1[i]!=0:
				self.defuzzifier_numerator=self.defuzzifier_numerator+(self.speed_values1[i]*self.get_centroid_speed(self.speed_labels1[i],self.centroids1_speed))
				self.defuzzifier_denominator=self.defuzzifier_denominator+self.speed_values1[i]
		ref_speed=self.defuzzifier_numerator/self.defuzzifier_denominator	
	
	
#.............................membership function ranges for obs and ref......................................................
		self.obs=[0.1,0.1,0.25,0.4]
		self.ref=[0.3,0.45,1.0,1.0]
		self.obs_firing_str=0
		self.ref_firing_str=0
		self.labels_list=[]
		self.values_list=[]
		
#................................................getting the membership values and labels for obs avoidance................................................
		self.laser_val=min(self.front,self.back,self.left,self.center,self.right)
		self.values_list,self.labels_list=self.partition_obs_ref(self.laser_val,self.obs,self.ref)

#...................................................defuzzyfying the output...............................................................................
		turning_sum=0
		speed_sum=0
		firing_sum=0
		for i in range (len(self.labels_list)):
			if(self.labels_list[i]=='obs'):
				turning_sum=turning_sum+(self.values_list[i]*obs_avoidance_turning)
				speed_sum=speed_sum+(self.values_list[i]*obs_avoidance_speed)
			if(self.labels_list[i]=='ref'):
				turning_sum=turning_sum+(self.values_list[i]*ref_turning)
				speed_sum=speed_sum+(self.values_list[i]*ref_speed)
		for i in range (len(self.labels_list)):
			firing_sum=firing_sum+self.values_list[i]
		
		self.cmd.linear.x=speed_sum/firing_sum
		self.cmd.angular.z=turning_sum/firing_sum
		
		
	def timer_callback(self):
		self.merged()
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
	
		
	
