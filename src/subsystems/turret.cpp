#include "ARMS/chassis.h"
#include "ARMS/odom.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/config.h"
#include "turret.hpp"
#include "vision.hpp"
#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"
#include "Eigen/Core"

using namespace pros;

namespace turret {

    /**
    *
    * PUBLIC DATA (see header file for documentation)
    *
    */

    namespace{ //Anonymous namespace for private methods and data

        /*
        *
        * PRIVATE DATA
        *
        */

        //Turret's motor, uses robot specific config for port
        Motor motor(TURRET_MOTOR, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);

        //Target angle for turret to face
        double target_angle = 0.0; 
        //Turret speed limit in RPM
        double max_velocity = 0.0; 
        //Whether or not the vision system is working
        bool vision_working = true;
        int endgame_power = 0;

        //previous voltage applied to turret motor
        double prev_voltage = 0;
        //Controller i sum term
        double integral = 0;
        //Previous error term
        double last_error = 0;
        //previous heading term
        double last_heading = 0;

        //Conversion factor from motor rotations to degrees
        const double ROT_TO_DEG = 37.5;
        //Left limit of turret in degrees
        const double LEFT_LIMIT = 80.0;
        //Right limit of turret in degrees
        const double RIGHT_LIMIT = -80.0; 
        //How close the turret needs to be to the target angle to be settled
        const double SETTLE_THRESHHOLD = 0; 

        /**
        * Enumerated class containing the state of the turret
        * 
        * DISABLED: Turret control system disabled
        * 
        * MANUAL: Manual control (vision control disabled)
        * 
        * VISION: Vision system control
        */
        enum class State {
            DISABLED,
            MANUAL,
            VISION,
            ENDGAME
        };

        //Current state of the turret
        State state = State::MANUAL; //Set state to manual by default
        //Current target color

        #define TURRET_DEBUG true
        int printCounter = 0;

        #if JOSH_LAT_COMP
            /**
            *
            * PRIVATE DATA + METHODS FOR LATENCY COMPENSATION
            *
            */

            
            using namespace Eigen;

            Matrix<double, 6, 6> a_matrix{
                {1.0,0.005615894974418574,0.0,0.0008916274770076209,0.0,0.0},
                {0.0,0.12317899488371485,0.0,0.17832549540152418,0.0,0.0},
                {0.0,0.0008916274770076208,1.0,0.005615894974418574,0.0,0.0},
                {0.0,0.17832549540152415,0.0,0.12317899488371484,0.0,0.0},
                {0.0,-0.009117373106523002,0.0,0.009117373106523002,1.0,0.00020997663087602824},
                {0.0,0.0,0.0,0.0,0.0,-0.9580046738247944},
            };
            
            Matrix<double,6,3> b_matrix{
                {0.00024294217503691306,-4.940892549858086e-05,0.0,},
                {0.04858843500738261,-0.009881785099716172,0.0,},
                {-4.940892549858084e-05,0.000242942175036913,0.0,},
                {-0.009881785099716168,0.0485884350073826,0.0,},
                {-0.0005642089621609017,0.0005642089621609016,0.0010850148694335885,},
                {0.0,0.0,0.2170029738867177}
            }; 
            // {1.0,0.007959240322613458,0.0,-0.0014517178711872625,0.0,0.0},
            //     {0.0,0.5918480645226915,0.0,-0.2903435742374525,0.0,0.0},
            //     {0.0,-0.0014517178711872625,1.0,0.007959240322613458,0.0,0.0},
            //     {0.0,-0.2903435742374525,0.0,0.5918480645226915,0.0,0.0},
            //     {0.0,-0.018162226671075213,0.0,0.018162226671075213,1.0,0.0013053771931128538},
            //     {0.0,0.0,0.0,0.0,0.0,-0.7389245613774292},
            // {0.000113087298743753,8.044595079457918e-05,0.0},
            //     {0.0226174597487506,0.016089190158915835,0.0},
            //     {8.044595079457918e-05,0.000113087298743753,0.0},
            //     {0.016089190158915835,0.0226174597487506,0.0},
            //     {-6.299460140975364e-05,6.299460140975354e-05,0.0009636131267409948},
            //     {0.0,0.0,0.19272262534819895}
            

            //must discretize matricies before entering them in
            float sign(float x){
                if (x > 0) return 1;
                if (x < 0) return -1;
                return 0;
            }

            class state_space{
                private: 
                //A matrix
                // x by x matrix
                MatrixXd A;
                //B matrix
                //x by u matrix
                MatrixXd B;
                VectorXd u;
                VectorXd x;
                MatrixXd Binv;
                double X;
                double Y;
                double theta;
                double dt;
                int dec_counter;
                bool isInitialized;

                public:

                state_space(MatrixXd a, MatrixXd b, int u_dim, int x_dim, double dT): A(a), B(b), dt(dT){
                    this->u = VectorXd(u_dim);
                    this->x = VectorXd(x_dim);
                    if(a.rows() != x_dim){
                        throw "RowsError: A matrix Requires same number of Rows as States";
                    }
                    if(a.cols() != x_dim){
                        throw "ColumnsError: A matrix requires same number of Columns as States";
                    }
                    if(b.cols() != u_dim){
                        throw "ColumnsError: B matrix requires same number of Columns as Inputs";
                    }
                    if(b.rows() != x_dim){
                        throw "ColumnsError: B matrix requires same number of Rows as States";
                    }
                    this->Binv = this->B.completeOrthogonalDecomposition().pseudoInverse();
                    
                    this->dec_counter = 0;

                    this->isInitialized = false;
                }
                VectorXd update(VectorXd u){
                    this->x += this->A * this->x + this-> B * u;
                    return this->x;
                }
                VectorXd update(VectorXd u, bool predict){
                    VectorXd x = this->x;
                    x += this->A * this->x + this-> B * u;
                    return x;
                }
                void set_x(VectorXd x){
                    this->x = x;
                    this->isInitialized = true;
                }
                float calc_desired_w(float target_theta, float dec_theta){
                    float max_vel = 1.32; //rad/sec
                    float dec_dt = dec_theta/max_vel;
                    float max_acc = max_vel/(dec_theta/max_vel); //rad/sec^2;
                    float target_w = 0;
                    float error = x[4] - target_theta;

                    if (std::abs(error) >= dec_theta){
                        target_w = max_vel;
                        dec_counter = 0;
                    }
                    else{
                        target_w = std::max(0.0,target_w - max_acc * dt * dec_counter);
                        dec_counter += 1;
                    }
                    return sign(-error)* target_w;
                }

                
                float calc_feedforward(float target_x, float target_y, VectorXd u){
                    X = arms::odom::getPosition().x;
                    Y = arms::odom::getPosition().y;
                    theta = arms::odom::getHeading(true);
                    float target_theta = atan2( target_y - Y, target_x - X);
                    float target_w = calc_desired_w(target_theta, 0.1); //change dec theta
                    VectorXd reference(x.size());
                    reference << x[0], x[1], x[2], x[3], target_theta, target_w;
                    VectorXd x_f = update(u,true);

                    float vl = std::min(std::abs(x_f[1]), 200*4/39.37/60*M_PI)*sign(x_f[1]);
                    float vr = std::min(std::abs(x_f[3]), 200*4/39.37/60*M_PI)*sign(x_f[3]);
                    float fut_x = X + (vl+vr)/2*cos(theta)*dt;
                    float fut_y = Y + (vl+vr)/2*sin(theta)*dt;
                    x_f[4] = atan2(target_y - fut_y , target_x - fut_x);
                    x_f[5] = calc_desired_w(x_f[4], 0.08);

                    VectorXd output = Binv*(x_f - (A * reference));
                    return output[2];
                }
                //not needed
                void discretize_A(MatrixXd A,float dt){
                    A = (MatrixXd::Identity(A.rows(),A.cols())+0.5*A*dt)*(MatrixXd::Identity(A.rows(),A.cols())-0.5*A*dt).inverse();
                    std::cout << A;
                }
                bool isInit(){
                    return isInitialized;
                }
                
                
            };
            
            state_space robot_model = state_space(a_matrix, b_matrix, 3, 6, 0.01);
        #endif

        /*
        *
        * PRIVATE METHODS
        *
        */

        /**
        * Inline function to convert from rotations to degrees
        */
        inline double rot_to_deg(double rot) { return rot * ROT_TO_DEG; }
        
        /**
        * Inline function to convert from degrees to rotations
        */
        inline double deg_to_rot(double deg) { return deg / ROT_TO_DEG; }

        /**
         * Checks if the vision system sees a valid target (based on the target color)
         * CONTAINS CHECK FOR VISION BROKEN
         * 
         * @return True if the last seen goal is a valid target, false otherwise
         */
        bool is_valid_target(){
            
            return false; //If vision is not working, return false
        }

        /**
         * PID and feedforward calculation for vision control
         * 
         * @param angle_error The angle error between the turret and the goal
         * @return The voltage to apply to the turret motor (mV)
         */
        double get_vision_voltage(double angle_error){
            //Convert to sqrt curve
            angle_error = sqrt(fabs(angle_error)) * (angle_error < 0 ? -1 : 1);
            #if JOSH_LAT_COMP
                if(!robot_model.isInit()){
                    VectorXd x(6);
                    x << 0,0,0,0,arms::odom::getHeading(true),0;
                    robot_model.set_x(x);
                }
            #endif
            //Calculate PID output:
            //Anti-windup for integral term:
            if(!TURRET_AW || (TURRET_AW && fabs(TURRET_KP * angle_error) < 12000)){
                integral += angle_error;
                //printf("\nIntegral: %f, Angle Error: %f", integral, angle_error);
            }
            double pOut = TURRET_KP * angle_error; //proportional term
            double iOut = TURRET_KI * integral; //integral term
            double dOut = TURRET_KD * (angle_error - last_error); //derivative term
            last_error = angle_error; //update last error
            double output = pOut + iOut + dOut; //output = sum of PID terms

            //Calculate feedforward output:
            if(TURRET_FF){
                //Add feedforward term to output (feedforward voltage times change in heading since last calculation)
                output += -TURRET_FF_V * (arms::odom::getHeading() - last_heading); //Reversed due to motor directions
                last_heading = arms::odom::getHeading(); //update last heading
                arms::Point goal_pos = vision::get_goal_pos()/39.37; //in meters
                #if JOSH_LAT_COMP
                    VectorXd u = VectorXd(3);
                    double left_volt_drive = arms::chassis::leftMotors->get_voltages()[0]/1000.0;
                    double right_volt_drive = arms::chassis::rightMotors->get_voltages()[0]/1000.0;
                    u <<  left_volt_drive, right_volt_drive, prev_voltage/1000.0;
                    double model_feedforward = robot_model.calc_feedforward(goal_pos.x, goal_pos.y, u);
                    output += model_feedforward*1000.0;
                    if(int(left_volt_drive) % 4 == 0){
                        std::cout <<  "Model_volt" <<model_feedforward <<"LEft drive" << left_volt_drive <<  '\n';
                    }
                #endif
            }

            if(TURRET_MIN_V != 0.0){
                if(output < 0 && output > -TURRET_MIN_V){
                    //If the output is negative and less than the minimum voltage, set it to the minimum voltage
                    output = -TURRET_MIN_V;
                } else if(output > 0 && output < TURRET_MIN_V){
                    //If the output is positive and less than the minimum voltage, set it to the minimum voltage
                    output = TURRET_MIN_V;
                }
            }
            
            if((get_angle() < RIGHT_LIMIT && output < 0) || (get_angle() > LEFT_LIMIT && output > 0)){
                //If the turret is at a limit, set the output to 0 to prevent turret damage
                output = 0;
            }

            if(motor.get_actual_velocity() > TURRET_MAX_V){
                //Speeding, slow down
                output = TURRET_MAX_V; 
            } else if(motor.get_actual_velocity() < -TURRET_MAX_V){
                //Speeding negative dir, slow down
                output = TURRET_MAX_V; 
            }

            if(fabs(angle_error) <= SETTLE_THRESHHOLD){
                //If the turret is settled, set the integral term to 0
                integral = 0;
                output = 0;
            }

            //If the turret is settled, return 0mV, otherwise return the calculated output:
            return output; 
        }

        /**
         * Asynchronous task loop for turret control
         */
        void task_func() { 
            //Set motor brake mode to hold
            motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
            while(true) {
                //Switch for desired control mode
                switch(state) {
                    case State::DISABLED: //Emergency stop basically
                        motor.move_voltage(0);
                        target_angle = 0;
                        integral = 0;
                        break;
                    case State::MANUAL: //Manual control
                        motor.move_absolute(deg_to_rot(target_angle), max_velocity);
                        integral = 0;
                        break;
                    case State::VISION: {//Vision control
                        // If the vision system is working, enable vision control
                        double error = vision::get_error();
                        target_angle = get_angle(false) - error;
                        double target = get_vision_voltage(error);
                        
                        motor.move_voltage(target);

                        if(TURRET_DEBUG && printCounter++ % 5 == 0){
                            printf("\nTurret Error: %3.2f, Target: %5.2f", error, target);
                        }
                        break;
                    }
                    case State::ENDGAME:
                        motor.move_voltage(120 * endgame_power);
                        break;
                }
                if(TURRET_FF && printCounter > 10){
                    #if JOSH_LAT_COMP
                        VectorXd u = VectorXd(3);
                        double left_volt_drive = arms::chassis::leftMotors->get_voltages()[0]/1000.0;
                        double right_volt_drive = arms::chassis::rightMotors->get_voltages()[0]/1000.0;
                        u <<  left_volt_drive, right_volt_drive, prev_voltage;
                        robot_model.update(u);
                        prev_voltage = motor.get_voltage();
                    #endif
                }
                //Loop delay
                pros::delay(10);
            }
        }
    }

    /*
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void initialize() {
        calibrate();
        pros::Task task(task_func, "Turret Task");
    }

    void calibrate() {
        // Set the motor to move to the left
        printf("Moving turret to the left\n");
        motor.move(85);

        // Wait until the limit switch is hit. This ensures the turret stops at a 
        // consistent location
        printf("Waiting for limit switch\n");
        pros::delay(500);
        while(fabs(motor.get_actual_velocity()) > 1) {
            pros::delay(20);
        }

        // Stop the motor so it doesn't break the ring gear
        printf("Stopping motor\n");
        motor.move(0);
        pros::delay(100);
        // Now tell the motor to move back to face forward.
        printf("Moving to face forward\n");
        double offset = -2.15;
        motor.move_relative(offset, 250); //Tune left value, more negative is more right offset from limit switch
        pros::delay(1000);
        motor.move(0);

        // Tare the position so that forward is 0.0
        printf("Taring position\n");
        motor.tare_position();
        pros::delay(100);
        printf("done\n");
    }

    double get_angle(bool radians) {
        return rot_to_deg(motor.get_position()) * (radians? M_PI/180 : 1);
    }

    void goto_angle(double angle, double velocity, bool async) {
        // Clamp the angle so that we don't try to move to a position that will 
        // harm the ring gear or burn out the motor
        target_angle = std::clamp(angle, RIGHT_LIMIT, LEFT_LIMIT);
        max_velocity = velocity;

        if(!async){
            wait_until_settled();
        }
    }

    void goto_rel_move(double angle, double velocity, bool async) {
        goto_angle(get_angle() + angle, velocity, async);
    }

    double get_angle_error() {
        return get_angle() - target_angle;
    }

    bool settled() {
        return fabs(get_angle_error()) <= SETTLE_THRESHHOLD;
    }

    void move_endgame(int power) {
        endgame_power = power;
    }

    void wait_until_settled() {
        // While the target_angle is outside the range we want, we sleep.
        // Once it is within SETTLE_THRESHHOLD degrees of the target angle, we quit
        // the loop.
        while(!settled()) {
            pros::delay(10);
        }
    }

    void debug_screen() {
        pros::lcd::print(0, "TURRET");
        if(state == State::DISABLED) {
            pros::lcd::print(1, " State: Disabled");
            return;
        } else if(state == State::MANUAL) {
            pros::lcd::print(1, " State: Manual");
        } else if(state == State::VISION) {
            pros::lcd::print(1, " State: Vision");
        }
        if(vision::get_targ_goal() == vision::Goal::BOTH){
            pros::lcd::print(2, " Target Color: Both");
        } else if(vision::get_targ_goal() == vision::Goal::RED){
            pros::lcd::print(2, " Target Color: Red");
        } else if(vision::get_targ_goal() == vision::Goal::BLUE){
            pros::lcd::print(2, " Target Color: Blue");
        }
        lcd2::pages::print_line(2, 3, " Vision Status: %s", vision::is_working() ? "OPERATIONAL" : "SUSPENDED, NO IRIS DATA!");
        lcd2::pages::print_line(2, 4, " Current Angle: %f", get_angle());
        lcd2::pages::print_line(2, 5, " Target Angle: %f", target_angle);
        lcd2::pages::print_line(2, 6, " Angle Error: %f", get_angle_error());
        lcd2::pages::print_line(2, 7, " Settled: %s, Temp: %2.0f", settled() ? "True" : "False", motor.get_temperature());
    }

    void toggle_vision_aim() {
        if(state == State::VISION) {
            disable_vision_aim();
        } else {
            enable_vision_aim();
        }
    }

    void enable_vision_aim() {
        state = State::VISION;
        integral = 0;
    }

    void disable_vision_aim() {
        state = State::MANUAL;
        target_angle = 0.0;
    }

    void disable_turret() {
        state = State::DISABLED;
    }

    void enable_endgame() {
        state = State::ENDGAME;
    }
} //End namespace turret
