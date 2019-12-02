#include "Rx90.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "stdio.h"
#include <thread>
#include "std_msgs/Duration.h"
#include "std_msgs/Time.h"

#define pi 3.14159265359
#include <fstream>

bool play = true;
void my_handler(int s){
	printf("Caught signal %d\n",s);
	play = false;
}

void tokenize(std::string const &str, const char delim, std::vector<float> &out) {
	size_t start;
	size_t end = 0;

	while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
		end = str.find(delim, start);
		out.push_back(atoi(str.substr(start, end - start).c_str()));
	}
}

int main(int argc, char **argv) {
	printf("\n");
	printf("//////////////////////////////////////////////////\n");
	printf("\n");
	printf("            RX 90 SIMULATION AND CONTROL          \n");
	printf("\n");
	printf("\n");
	printf("                       GRVC                       \n");
	printf("//////////////////////////////////////////////////\n\n");

	//Ros publisher
	ros::init(argc, argv, "publisher");
	
	// Rx90
	Rx90 rx90(argv[1], "90,-90,0,0,0,45");
	char key;

	std::cout << "Insert the file with target trajectory: "<<std::endl;
	std::string filePath;
	std::cin >> filePath;
	std::cout << "Is it working in joints or position (insert [joints|position|tool]): " << std::endl;
	std::string mode;
	std::cin >> mode;

	std::ifstream trajFile(filePath);


	rx90.readToolPosition();

	std::string line;
	while (std::getline(trajFile, line) && play) {
			// std::vector<float> tokens; /// NOT REALLY NEEDED
			// tokenize(line, ",", tokens);
		if(mode == "joints"){
			rx90.moveJoints(line);
		}else if(mode == "position"){
			rx90.movePosition(line);
		}else if(mode == "tool"){
			rx90.movePositionTool(line);
		}else{
			std::cout << "Bad mode " << mode<< "! Exiting."<< std::endl;
			play = false;
		}
	}
}
