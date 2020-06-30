#include <ros/ros.h>
#include <std_msgs/Int8.h>

const int course = 0;
const int triangle = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_change");
    ros::NodeHandle n;

    ros::Publisher formation_pub = n.advertise<std_msgs::Int8>("/formation/change", 10);

    int formation_flag;
    std_msgs::Int8 formation_name;

    while(ros::ok())
    {
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
        std::cout << "Input the flag:  0 for course formation, 1 for triangle formation" << std::endl;
        std::cin >> formation_flag;

        switch(formation_flag)
        {
            case 0:
                formation_name.data = course;
                formation_pub.publish(formation_name);
				std::cout << "Drone formation state: Course" << std::endl;
            	break;

            case 1:
                formation_name.data = triangle;
                formation_pub.publish(formation_name);
				std::cout << "Drone formation state: Triangle" << std::endl;
				break;
        }

        
        std::cout << std::endl;
    }

    return 0;
}



