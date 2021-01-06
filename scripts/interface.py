#!/usr/bin/env python

import rospy
import math
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import *
from causa_final_assignment.srv import *

##It contains the current robot coordinates
current_position=Point()

##To publish new goals 
pub_goal= None

##To read the current position
sub_odom= None

##To cancel the target
pub_target_reached=None

##Read the current position of the robot from odom
def current_position(odom_data):
	global current_position
	current_position = odom_data.pose.pose.position

##Takes command from the user and execute the task requested	
#
#1) Reach random target
#
#2) Choose target
#
#3) Follow the wall
#
#4) Stop the robot
#
#Possible targets:
#
#(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)

def main():
	global pub_goal,sub_odom,pub_target_reached
	global current_position
	pub_target_reached = rospy.Publisher('move_base/cancel',GoalID,queue_size=1)
	pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	pub_goal = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
	sub_odom = rospy.Subscriber('/odom',Odometry,current_position)
	client_random = rospy.ServiceProxy('/random', Random)
	client_wall_follow = rospy.ServiceProxy('/wall_follower_switch', SetBool)
	rospy.init_node('interface', anonymous=True)
	rate = rospy.Rate(50) # 50hz
	
	while not rospy.is_shutdown():
		print("Insert a command:")
		print("1) Reach random target")
		print("2) Choose target")
		print("3) Follow the wall")
		print("4) Stop the robot")
		
		command= input()
		
		#Reach a random position 
		if command ==1 : 
			resp = client_wall_follow(False)
			random_pos=client_random()
			print("X target: ")
			print(random_pos.x)
			print("Y target: ")
			print(random_pos.y)
			
			move_to(random_pos.x,random_pos.y)
			reach_target(random_pos.x,random_pos.y)		
		
		#Choose next position to reach		 
		elif command == 2 :	
			resp = client_wall_follow(False)		
			x_target,y_target = take_position_from_user()
			move_to(x_target,y_target)
			reach_target(x_target,y_target)
		
		#Follow the wall
		elif command == 3 :			
			resp = client_wall_follow(True)
			print("start following the wall")
		
		#Stop the robot	
		elif command == 4 :
			msg = Twist()
			msg.linear.x = 0
			msg.angular.z = 0		
    	
			resp = client_wall_follow(False)	
			pub_cmd_vel.publish(msg)
			print("Robot stopped")
			
		else:	
			print("wrong command!")	
			
		rate.sleep()

##Publish the target in order to use move_base service to reach it
def move_to(x_target,y_target):
	global pub_goal

	target_goal= MoveBaseActionGoal()
	target_goal.goal.target_pose.header.frame_id = "map"
	target_goal.goal.target_pose.pose.orientation.w = 1
	target_goal.goal.target_pose.pose.position.x = x_target
	target_goal.goal.target_pose.pose.position.y = y_target
	pub_goal.publish(target_goal)
	return 
	
##Keeps the program in sleep until the target isn't reached 
# it also compute and print the distance 	
def reach_target(x_target,y_target):
	global sub_odom
	global current_position
	global pub_target_reached
	
	target_distance=math.sqrt(pow(current_position.x-x_target,2) + pow(current_position.y-y_target,2))
	while target_distance>1 :
		print("reaching the target, distance:")
		print(target_distance)
		#print ("x coordinate:")
		#print(x_position)
		#print ("y coordinate:")
		#print(y_position)
		target_distance=math.sqrt(pow(current_position.x-x_target,2) + pow(current_position.y-y_target,2))
		rospy.sleep(1)
	
	target_reached=GoalID()
	pub_target_reached.publish(target_reached)	#delete the target in order to be sure that is considered reached
	print("target reached")	
	return

##Takes one of the possible target from the user
#
#Possible targets:
#
#(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)
def take_position_from_user():
	x_target=None
	y_target=None
	
	while not((x_target==-4 and y_target==-3)or
	          (x_target==-4 and y_target== 2)or
	          (x_target==-4 and y_target== 7)or
	          (x_target== 5 and y_target==-7)or
	          (x_target== 5 and y_target==-3)or
	          (x_target== 5 and y_target== 1)):
				print("choose next target, the possible one are the following:")
				print("(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)")
				print("X target: ")
				x_target = input()
				print("Y target: ")
				y_target = input()

	return x_target,y_target

		   
	
	

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
