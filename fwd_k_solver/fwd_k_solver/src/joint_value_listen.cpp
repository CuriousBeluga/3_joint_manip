
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;




//double result2[4][4] = {{0, 0, 0, 0},{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

class MinimalSubscriber : public rclcpp::Node
{
  public:


    MinimalSubscriber()
    : Node("joint_listener")
    {

 publisher= this->create_publisher<std_msgs::msg::String>("Pose", 10);
 subscription_= this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
	   
 
   
    }

  
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {

        // store data into a local made array

	    double thetas[3];
	    int i = 0;
		for(std::vector<double>::iterator it = msg->position.begin(); it!=msg->position.end(); it++)
		{
		thetas[i] = *it;
			i++;
	
		}


	     	//receive q values in a 1x3 matrix
	//each variable joint angle will be called thetas

	    //define joint DH parameters
	    
	    //A1
	    
	    double  a1 = 1; 
	    double d1 = 2;
	    double theta1 = thetas[0]; 
	    double alpha1 = 0;

	    //A2
	    double a2 = 0;
	    double d2 = 0;
	    double theta2 = thetas[1];
	    double alpha2 = 3.1415;


 	    //A3

	    double a3 = 0;
	    double d3 = thetas[2];
	    double theta3 = 0;
	    double alpha3 = 0;
	    

	    //define the joint transform matrices
	double f_A1[4][4] = {{cos(theta1), -sin(theta1)*cos(alpha1), sin(theta1)*sin(alpha1), a1*cos(theta1)}, {sin(theta1), cos(theta1)*cos(alpha1),-cos(theta1)*sin(alpha1),a1*sin(theta1)}, {0,sin(alpha1), cos(alpha1), d1},{0,0,0,1}};

	double f_A2[4][4] = {{cos(theta2), -sin(theta2)*cos(alpha2), sin(theta2)*sin(alpha2), a2*cos(theta2)}, {sin(theta2), cos(theta2)*cos(alpha2),-cos(theta2)*sin(alpha2),a2*sin(theta2)}, {0,sin(alpha2), cos(alpha2), d2},{0,0,0,1}};

	double f_A3[4][4] = {{cos(theta3), -sin(theta3)*cos(alpha3), sin(theta3)*sin(alpha3), a3*cos(theta3)}, {sin(theta3), cos(theta3)*cos(alpha3),-cos(theta3)*sin(alpha3),a3*sin(theta3)}, {0,sin(alpha3), cos(alpha3), d3},{0,0,0,1}};


	double result1[4][4] = {{0, 0, 0, 0},{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

	double result2[4][4] = {{0, 0, 0, 0},{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};



//first multiplication
for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
        for (int u = 0; u < 4; u++)
	{  result1[i][j] += f_A1[i][u] * f_A2[u][j];
 	   }

	}

      }

//second multiplication

for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
        for (int u = 0; u < 4; u++)
        {  result2[i][j] += result1[i][u] * f_A3[u][j];
           }

        }

     }


// for testing 
    //std::cout <<"End Effector Pose"<<std::endl;
    //std::cout <<result2[0][3]<<std::endl;
    //std::cout <<result2[1][3]<<std::endl;
    //std::cout <<result2[2][3]<<std::endl;


auto message = std_msgs::msg::String();
        message.data = "H-matrix, Row 1: " + std::to_string(result2[0][0]) + " " + std::to_string(result2[0][1]) +" "+std::to_string(result2[0][2]) +" "+ std::to_string(result2[0][3]) + " Row 2: " + std::to_string(result2[1][0])+ " " + std::to_string(result2[1][1]) +" "+std::to_string(result2[1][2]) +" "+ std::to_string(result2[1][3]) + " Row 3: " + std::to_string(result2[2][0])+ " " + std::to_string(result2[2][1]) +" "+std::to_string(result2[2][2]) +" "+ std::to_string(result2[2][3]); 

    publisher->publish(message);
    
    } //end function

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
 
