#include "Rx90.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "stdio.h"
#include <thread>
#include "std_msgs/Duration.h"
#include "std_msgs/Time.h"


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

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

		std::cout << "Insert the file with target trajectory: "<<std::endl;
	std::string filePath;
	std::cin >> filePath;
	std::cout << "Is it working in joints or position (insert [joints|position|tool]): " << std::endl;
	std::string mode;
	std::cin >> mode;

	if(mode == "tool"){
		rx90.readToolPosition();
	}

	std::ifstream trajFile(filePath);

	std::string line;
	int counter = 0;
	while (std::getline(trajFile, line) && play) {
		std::vector<float> tokens; /// NOT REALLY NEEDED
		tokenize(line, ',', tokens);
		if(tokens.size() != 6){
			std::cout << "You have introduced bad target values. \"" << line<< "\". Which has " << tokens.size() << " elements instead of 6" << std::endl;
		}
		if(mode == "joint"){
			rx90.moveJoints(line);
		}else if(mode == "position"){
			rx90.movePosition(line);
		}else if(mode == "tool"){
			rx90.movePositionTool(line);
		}else{
			std::cout << "Bad mode " << mode<< "! Exiting."<< std::endl;
			break;
			play = false;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
		rs2::frame color = data.get_color_frame();

		// Query frame size (width and height)
		const int w = depth.as<rs2::video_frame>().get_width();
		const int h = depth.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		cv::Mat depthcv(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat image_color(cv::Size(640,480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

		cv::cvtColor(image_color, image_color, CV_BGR2RGB);

		imwrite( "image_"+std::to_string(counter)+".jpg", image_color );
		imwrite( "depth_"+std::to_string(counter)+".jpg", depthcv );

		// Update the window with new data
		cv::imshow("Display window", image_color);
		cv::waitKey(10);
		counter++;

		// std::cout << "press enter to move to next" << std::endl;
		// getchar();

		
	}
}
