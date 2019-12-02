//
// C++ Interface: Rx90
//
// Description: 
//
//
// <franroyal@yahoo.es>
//
//

#ifndef RX90_H
#define RX90_H

#include <serial/serial.h>

#include <iostream>

class Rx90 {
public:
	Rx90(const std::string& serialPort, const std::string& originPoint);
	~Rx90();

  enum Action { NONE, UP, DOWN, LEFT, RIGHT, UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT, CATCH ,POSITION, JOINTS};
	static void printAction(const Action& action);
	void move(const Action& action);
	void panic();
	void readToolPosition();
	void moveJoints(std::string _joints);
	void movePosition(std::string _position);
	void movePositionTool(std::string _position);

	void sendCommand(const std::string& command, bool waitQuestionMark = false);
private:
	void init(const std::string& serialPort, const std::string& originPoint);
	void close();
	void catchIt();
	serial::Serial *serial_; 
	double x, y;
};

#endif  // RX90_H

