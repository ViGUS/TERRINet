//
// C++ Implementation: Rx90
//
// Description:
//
//
// <franroyal@yahoo.es>
//
//

#include "Rx90.h"
#include <sstream>
#include <cmath>
#include <stdexcept>
#include "gazebo_msgs/ApplyJointEffort.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"


#define pi 3.14159265359

#define DELTA_VH 25
#define DELTA_DIAG (0.707 * DELTA_VH)
#define END "\r\n"


Rx90::Rx90(const std::string &serialPort, const std::string &originPoint){
	init(serialPort, originPoint);
	x = 0.0;
	y = 0.0;
	readToolPosition();
}

Rx90::~Rx90() { close(); }

void Rx90::init(const std::string &serialPort, const std::string &originPoint) {
	
	serial_ = new serial::Serial(serialPort, 9600, serial::Timeout::simpleTimeout(500));
	
	// Set origin precision point
	std::stringstream command_pose,command_origin;
	command_origin << "DO SET #POSE=#PPOINT(" << originPoint.c_str() << ")";
	std::cout<<originPoint.c_str()<<std::endl;
	sendCommand(command_origin.str());
	//sendCommand(command_pose.str());
	sendCommand("SPEED 30");
	sendCommand("DO ABOVE");
	sendCommand("DO MOVE #POSE");
	sendCommand("HERE ORIGIN", true);
	sendCommand("DO OPENI");
	sendCommand("DO ENABLE CP");
	
}

void Rx90::close() { serial_->close(); }

void Rx90::sendCommand(const std::string &command, bool waitQuestionMark) {

	if (serial_->isOpen()) {
		int len= 0;
		serial_->write(command+END);
		if (waitQuestionMark) {
			char qm;
			do {
				len = serial_->read((uint8_t*)&qm, 1);
			} while (len != 0 && qm != '?');
		serial_->write(END);
		}

		char r;
		do {
			len = serial_->read((uint8_t*)&r, 1);
			std::cout << r;
		} while (len != 0);
	}
	else {
		std::cout << "Serial port not opened" << std::endl;
	}
}

void Rx90::panic() { sendCommand("PANIC"); }

void Rx90::readToolPosition(){ sendCommand("DO HERE REFERENCE"); }


void Rx90::moveJoints(std::string _joints){
	printf("\nMoving joints\n");
	std::stringstream command_pose;
	command_pose << "DO SET #POSE=#PPOINT(" << _joints << ")";
	sendCommand(command_pose.str());
	sendCommand("DO MOVE #POSE");
	printf("\nEnding...\n");
}

void Rx90::movePosition(std::string _position){
	printf("\nMoving position\n");
	std::stringstream command_pose;
	command_pose << "DO SET POSE=TRANS(" << _position << ")";
	sendCommand(command_pose.str());
	sendCommand("DO MOVE POSE");
	printf("\nEnding...\n");
}

void Rx90::movePositionTool(std::string _position){
	printf("\nMoving position\n");
	std::stringstream command_pose;
	command_pose << "DO SET POSE=TRANS(" << _position << ")";
	sendCommand(command_pose.str());
	sendCommand("DO MOVE REFERENCE:POSE");
	printf("\nEnding...\n");
}


void Rx90::move(const Action &action) {
	std::string Point;

	switch (action) {
	case NONE:
		break;
	case UP:
		y += DELTA_VH;
		break;
	case DOWN:
		y -= DELTA_VH;
		break;
	case RIGHT:
		x += DELTA_VH;
		break;
	case LEFT:
		x -= DELTA_VH;
		break;
	case UP_RIGHT:
		x += DELTA_DIAG;
		y += DELTA_DIAG;
		break;
	case UP_LEFT:
		x -= DELTA_DIAG;
		y += DELTA_DIAG;
		break;
	case DOWN_LEFT:
		x -= DELTA_DIAG;
		y -= DELTA_DIAG;
		break;
	case DOWN_RIGHT:
		x += DELTA_DIAG;
		y -= DELTA_DIAG;
		break;
	case CATCH:
		catchIt();
		break;
	default:
		std::cout << "unexpected!";
	}

	// send the command
	if (action != POSITION)
	{
		std::stringstream sstr;
		sstr << "DO SET P" << "=SHIFT(POSE BY " << (int)x << "," << (int)y << ",0)";
		std::string command = sstr.str();
		sendCommand(command);
		sendCommand("SPEED 10");
		sendCommand("DO MOVES P");
	}
	else
	{
	}
}


void Rx90::catchIt() { sendCommand("DO CLOSEI"); }

void Rx90::printAction(const Action &action) {
	std::cout << "Rx90::printAction: ";
	switch (action) {
	case NONE:
		std::cout << "none!";
		break;
	case UP:
		std::cout << "up!";
		break;
	case DOWN:
		std::cout << "down!";
		break;
	case RIGHT:
		std::cout << "right!";
		break;
	case LEFT:
		std::cout << "left!";
		break;
	case UP_RIGHT:
		std::cout << "up-right!";
		break;
	case UP_LEFT:
		std::cout << "up-left!";
		break;
	case DOWN_LEFT:
		std::cout << "down-left!";
		break;
	case DOWN_RIGHT:
		std::cout << "down-right!";
		break;
	case CATCH:
		std::cout << "catch!";
		break;
	case POSITION:
		std::cout << "moving to a position!";
		break;
	default:
		std::cout << "unexpected!";
	}
	std::cout << std::endl;
}
