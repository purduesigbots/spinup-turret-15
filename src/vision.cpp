#include "vision.h"
#include "subsystems.h"
#include "comms/comms.hpp"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "LatPullDown/Oak_1_latency_compensator.hpp"


#define START_CHAR 0b11001100
#define END_CHAR 0b00110011

#define GOAL_COLOR 0b00000001
#define LEFT_RIGHT 0b00000010
#define HEIGHT 0b00000011

std::tuple<double,double,double> get_turret_pose() {
	// get odom x,y, and heading
	// get turret heading
	arms::Point p =  arms::odom::getPosition();
	//-7.5
	//7.2
	return std::make_tuple(p.x, p.y, arms::odom::getHeading() - turret::get_position());
}
std::tuple<double,double> get_goal_vector(std::tuple<double,double,double> pose) {
	// get goal distance
	// calculate vector to the goal
	return std::make_tuple(0,0);
}
double get_latency() {
	// get latency from the ReceiveComms class
	return 35;
}

Oak_1_latency_compensator latency_compensator(
	5, // max buffer of 7
	10, // get odom position every 10ms
	get_turret_pose, // function to get pose of the turret
	get_goal_vector, // function to calulate the vector to the goal based on the distance
	get_latency // function to get the latency of the current frame
	);


namespace vision {
    void task() {
        comms::ReceiveComms communication(8, 115200, START_CHAR, END_CHAR);
        communication.start();

        uint64_t color = communication.get_data(GOAL_COLOR);
		uint64_t lr = communication.get_data(LEFT_RIGHT);
		uint64_t height = communication.get_data(HEIGHT);

        float speed = (140 - (float)lr) / 140 * 100;
		
		// // save speeds in an array and set speed to the previous speed if color is 3
		// float previous_speed;
		// if (color == 3 || color == 2) {
		// 	printf("Previous Speed Used \n");
		// 	speed = previous_speed;
		// } else {
		// 	previous_speed = speed;
		// }

		// float min_speed = 0.20;

		// if(speed < 0.0){
		// 	if(speed > -min_speed) {
		// 		speed = -min_speed;
		// 	}
		// 	// if(turret.rotation(deg) <= tar_angle - deadzone) {speed = 0.0;}
		// } else if(speed > 0.0) {
		// 	if(speed < min_speed) {
		// 		speed = min_speed;
		// 	}
		// 	// if(turret.rotation(deg) > tar_angle + deadzone) {speed = 0.0;}
		// } else {
		// 	speed = 0.0;
		// }
		
		// 0 is red
		// 1 is blue
		// 3 is nothing detected

		if (color == 0) {
			speed = 0;
		} 

		printf("Color:      %llu\n", color);
		printf("Left/Right: %llu\n", lr);
		printf("Height:     %llu\n", height);
		printf("Speed:      %f\n", speed);
		printf("----------------\n");

		turret::move(speed * -1);
    }
}