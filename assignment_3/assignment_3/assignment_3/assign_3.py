import rclpy
from array import *
from rclpy.node import Node
from  std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import JointState
from group_assignment_interfaces.srv import Vel ## need to create the actual interface
from  math import cos, sin, pi, acos,sqrt,atan2
import numpy as np
import time


class Pos_Controller_Node(Node):
    def __init__(self):
        super().__init__('vel_controller') #creating a node with the name vel_kinematics
        self.get_logger().info("Node Started")
        #creating server object
        self.server=self.create_service(MoveToPos,'vel_control',self.callback_move_to_pos)
        self.subscriber_=self.create_subscription(JointState,"joint_states",self.callback_get_curr_pos,10)
        self.publisher_=self.create_publisher(Float64MultiArray,"forward_velocity_controller/commands",10)
        
        #link lengths
        self.l1=2.05
        self.l2=0.9
        self.l3=1
        self.l4=0.5
    
        self.kp_1=11
        self.kd_1=15

        self.kp_2=7
        self.kd_2=8.5
        
        self.kp_3=200
        self.kd_3=30
		
        self.q1_m=0
        self.q2_m=0
        self.q3_m=0
	
        self.q1_r=0
        self.q2_r=0
        self.q3_r=0
	
        self.q1_e_p=0
        self.q2_e_p=0
        self.q3_e_p=0

	##for calculating rotation matrices
	self.q1_pos=0
        self.q2_pos=0
        self.q3_pos=0
        
        self.start_time=0
        self.t_p=time.time()
        self.f = open("velocities.txt", "w")
    
    def dh_to_tran(self,d,theta,a,alpha):
        ca=cos(alpha)
        sa=sin(alpha)
        ct=cos(theta)
        st=sin(theta)
        T=np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
        return T


    def controller(self,Jacob):
        t=time.time()
        dt=t-self.t_p
        
        #multiply request by inverse jacobian to get the joint angles 
        q_ref =np.array([self.q1_r],[self.q2_r],[self.q3_r]) 
        j_ref = np.matmul(Jacob,q_ref)
        q1_ref = j_ref[0][0]
        q2_ref = j_ref[0][1]
        q3_ref = j_ref[0][2]      	
        q_meas = np.array([self.q1_m],[self.q2_m],[self.q3_m])
        j_meas = np.matmul(Jacob,q_meas)
	q1_meas = j_meas[0][0]
        q2_meas = j_meas[0][1]
        q3_meas = j_meas[0][2]      	                  
        
        q1_e=q1_ref-q1_meas
        q2_e=q2_ref-q2_meas
        q3_e=q3_ref-q3_meas
        
        if t-self.start_time<=10:
            self.f.write(str(t-self.start_time)+','+str(q1_meas)+','+str(q1_ref)+','+str(q2_meas)+','+str(q2_ref)+','+str(q3_meas)+','+str(q3_ref)+'\n')


        q1_d=(q1_e-self.q1_e_p)/dt
        q2_d=(q2_e-self.q2_e_p)/dt
        q3_d=(q3_e-self.q3_e_p)/dt

        out=Float64MultiArray()
        out.data=[0.0,0.0,0.0]
        out.data[0]=self.kp_1*q1_e+self.kd_1*q1_d
        out.data[1]=self.kp_2*q2_e+self.kd_2*q2_d
        out.data[2]=self.kp_3*q3_e+self.kd_3*q3_d +9.8 ##would gravity term look like this

        self.t_p=t
        self.q1_e_p=q1_e
        self.q2_e_p=q2_e
        self.q3_e_p=q3_e
        self.publisher_.publish(out)


    #function for processing velocity request 
    def callback_move_to_pos(self,request,response):
        
        self.q1_r=request.q[0]
        self.q2_r=request.q[1]
        self.q3_r=request.q[2]

        self.q1_e_p=self.q1_r-self.q1_m
        self.q2_e_p=self.q2_r-self.q2_m
        self.q3_e_p=self.q3_r-self.q3_m

        self.q3_e_sum=0
        
        self.t_p=time.time()
        self.start_time=self.t_p
        response.confirm="Moving..."

        return response

    def callback_get_curr_pos(self,msg):
        
        #field name for velocity is velocity
        self.q1_m=msg.velocity[0]
        self.q2_m=msg.velocity[1]
        self.q3_m=msg.velocity[2]
        
        
        #retrieving position to calculate jacobian
        self.q1_pos=msg.position[0]
        self.q2_pos=msg.position[1]
        self.q3_pos=msg.position[2]
        
        dh=np.array([[self.l1,0,0,0],[0.1,q1,self.l2,0],[0,q2,self.l3,0],[q3-self.l4,0,0,0]])
        T=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        	
       	#H matrix 0_1      	
       	for i in range(0,2):
           T_1 =np.matmul(T,self.dh_to_tran(i[0],i[1],i[2],i[3]))       	
       	#H matrix 0_2      
       	for i in range(0,3):
           T_2 =np.matmul(T,self.dh_to_tran(i[0],i[1],i[2],i[3]))
       	
        #H matrix 0_3	
        for i in dh:
           T=np.matmul(T,self.dh_to_tran(i[0],i[1],i[2],i[3]))

	#joint 1 Jacobian      
        d_0_3 = np.array([T[3][0]],[T[3][1]],[T[3][2]]]
        R_0 = np.array([0],[0],[1])        
                
        j1_lin = np.cross(R_0,d_0_3)   #calculate linear jacobian for joint 1
        j1_ang = np.array([0],[0],[1]) #angular jacobian for joint 1
        
        #Joint 2 Jacobian
        z = np.array([0],[0],[1])
        j2_lin = np.matmul(T_2,z)   #calculate linear jacobian for joint 2
        j2_ang = np.array([0],[0],[0]) #angular jacobian for joint 2
        
        #Joint3 Jacobian
        j3_lin = np.matmul(T_2,z)   #calculate linear jacobian for joint 3
        j3_ang = np.array([0],[0],[0]) #angular jacobian for joint 3
        
        
        #constructing the jacobian
        Jacob = np.array([j1_lin,j2_lin,j3_lin],[j1_ang,j2_ang,j3_ang])        
                             
        #inverse jacobian
        Jacob = inv(np.matrix(Jacob))
        
        #inverse Jacobian matrix will be passed to controller
        self.controller(Jacob)
        

def main(args=None):
    rclpy.init(args=args) #Initializes the ROS communication for a given context
    node=Pos_Controller_Node() #Creates an instance of the SubscriberNode
    rclpy.spin(node) #Causes the node to continue to work till it is shut down manually (Ctrl+C)
    rclpy.shutdown() #shutdown the previously initialized context

if __name__=="__main__":
    main()
