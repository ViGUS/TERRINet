#include "Rx90.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "stdio.h"
#include <thread>
#include "std_msgs/Duration.h"
#include "std_msgs/Time.h"


#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    
	// Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map(0);	// 0: cool depth map 3: usual depth map
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
	// while(true){
	// 	rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    //     rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
	// 	//rs2::frame depth = data.get_color_frame();
    //     // Query frame size (width and height)
    //     const int w = depth.as<rs2::video_frame>().get_width();
    //     const int h = depth.as<rs2::video_frame>().get_height();
    //     // Create OpenCV matrix of size (w,h) from the colorized depth data
    //     cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    //     // Update the window with new data
    //     cv::imshow("Display window", image);
	// 	cv::waitKey(1);
	// }
	//Ros publisher
	ros::init(argc, argv, "publisher");
	
	// Rx90
	Rx90 rx90(argv[1], "90,-90,0,0,0,45");
	char key;

	
	int counter = 0;
//	while (std::getline(trajFile, line) && play) {
	while (play){
			// std::vector<float> tokens; /// NOT REALLY NEEDED
			// tokenize(line, ",", tokens);

				
		std::cout << "Will it work in joints or position? (insert [joint|position|tool|custom]): " << std::endl;
		std::string mode = "";
		char cchar = 0;
		while(true){
			cchar = std::cin.get();
			if(cchar != '\n'){
				mode += cchar;
			}else{
				break;
			}
		}

		std::cout << mode <<std::endl;

		std::string line;
		std::vector<float> tokens; /// NOT REALLY NEEDED
		if(mode != "custom"){
			std::cout << "Introduce 6 values separated by comas (example: 0,-90,90,0,0,0): " << std::endl;
			line = "";
			char cchar = 0;
			while(true){
				cchar = std::cin.get();
				if(cchar != '\n'){
					line += cchar;
				}else{
					break;
				}
			}

			tokenize(line, ',', tokens);
			if(tokens.size() != 6){
				std::cout << "You have introduced bad target values. \"" << line<< "\". Which has " << tokens.size() << " elements instead of 6" << std::endl;
			}
		}else{
			std::cout << "INTRODUCE CUSTOM COMMAND: " << std::endl;
			line = "";
			char cchar = 0;
			while(true){
				cchar = std::cin.get();
				if(cchar != '\n'){
					line += cchar;
				}else{
					break;
				}
			}
		}
		std::cout << "Mode: " << mode << ". cmd: " << line  <<std::endl;

		if(mode == "joint"){
			rx90.moveJoints(line);
		}else if(mode == "position"){
			rx90.movePosition(line);
		}else if(mode == "tool"){
			rx90.movePositionTool(line);
		}else if(mode == "custom"){
			std::cout << "You are going to enter a custom command, watch out" << std::endl;
			rx90.sendCommand(line);
		}else{
			std::cout << "Bad mode " << mode<< "! Exiting."<< std::endl;
			break;
			play = false;
		}


		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
		//rs2::frame depth = data.get_color_frame();
		// Query frame size (width and height)
		const int w = depth.as<rs2::video_frame>().get_width();
		const int h = depth.as<rs2::video_frame>().get_height();
		// Create OpenCV matrix of size (w,h) from the colorized depth data
		cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
		imwrite( "image_"+std::to_string(counter)+".jpg", image );
		counter++;
		// Update the window with new data
		cv::imshow("Display window", image);
		cv::waitKey(1);

		
	}
}
