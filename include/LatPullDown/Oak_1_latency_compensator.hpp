#ifndef OAK_1_COMPENSATOR_HPP
#define OAK_1_COMPENSATOR_HPP
#include "main.h"

class Oak_1_latency_compensator
{
    public:
        Oak_1_latency_compensator(
            int buffer_size, 
            int loop_time, 
            std::tuple<double, double, double> (*get_robot_turret_pose)(), 
            double (*get_old_goal_distance)(), 
            double (*get_latency)(),
            bool debug
            );
        virtual ~Oak_1_latency_compensator();
        double get_new_goal_distance(double gamme);
        std::array<double, 2> update_goal_pose(double distance, double gamma);
        
    private:
        void latency_comp_task_fn();
        void begin_task();
        bool use_debug;
        std::vector<std::tuple<double, double, double>> odom_queue;
        std::tuple<double, double> to_goal_vector;
        std::tuple<double, double, double> (*get_robot_turret_pose)();
        double (*get_old_goal_distance)();
        double (*get_latency)();
        int buffer_size;
        int loop_time;
        pros::Task *latency_comp_task;
};


#endif