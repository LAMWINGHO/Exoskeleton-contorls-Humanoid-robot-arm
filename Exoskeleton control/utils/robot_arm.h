#ifndef ROBOT_ARM_H_
#define ROBOT_ARM_H_

#include <iostream>
#include <fstream>
#include <vector>   //for using vectors instead of arrays
#include <map>
#include <thread>
#include <queue>
#include <libserial/SerialPort.h>
#include <bldc_ctrl.h>
#include <me3_ctrl.h>

//#include <eigen3/unsupported/Eigen/EulerAngles>
#include <nlohmann/json.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "kalman.hpp"

using namespace LibSerial;
using namespace std;
using json = nlohmann::json;


// Define Exo arm
#define EXO_ARM_L 0
#define EXO_ARM_R 1

#define EXO_ARM_SAMPLING_MS     50


constexpr const char* const TORQUE_SIM_JSON = "./configs/torque_sim_blank.json";

enum OPMODE
{
    CONTROL = 0,       // read encoder angle to send to robot
    FEEDBACK = 1,      // read robot's tourqe to feedback exo-skeleton
    FEEDFORWORD = 2,    // increase tourqe to exo-skeleton with same-direction    
    MANUEL = 3,
    STOP = 4,           // no websocket packet send to robot
    ALIGN_ANGLE = 5,    // align as a relative angle for control
    TORQUE_SIM = 6
};

// robot side status
struct robot_status {
    float   read_torque;        // recent torque
    float   read_angle;         // recent angle
    float   filtered_torque;
    std::vector<float> filtered_torques;
    
    float   reference_torque;     // toruqe turning point to apply torque mode

    float   delta_torque;
    float   delta_torque_threshold;

    KalmanFilter torque_kf;
    float       kalman_r;
};


typedef struct my_serials {
    SerialPort      *l_arm_port;
    SerialPort      *r_arm_port;
    // pthread_mutex_t usb_lockWrite;
    // pthread_mutex_t usb_lockRead;
} my_serials_t;


class arm_ctrl
{
private:

    OPMODE ctrl_mode;
    std::map<std::string,int> joint_angles;        // joint_angles based on encoder
    std::map<std::string,int> joint_torques;
    
    
    vector<thread*> arm_thread;

    bool control_enabled;
    bool angle_aligned;

    std::map<string,int> L_arm_index;
    std::map<string,int> R_arm_index;

    json torque_buffer;
    json::iterator torque_buffer_it; 
    
    // queue<int> torques;

    // // Euler information
    // Eigen::Matrix3f m;
    void initial_exo_arm(std::map<string,int> ids, SerialPort *port , std::vector<bldc_ctrl>* joints);
  

public:
    //arm_ctrl(std::vector<int> &ids, OPMODE mode, SerialPort *port);
    //arm_ctrl(json* status, std::map<std::string,int> &ids, OPMODE mode, SerialPort *port);

    arm_ctrl(json* status, json* setting, OPMODE mode, my_serials* ports);
    ~arm_ctrl();
    
    void arm_operation(string& ctrl_cmd);
    void setOPmode(OPMODE mode){ this->ctrl_mode = mode;};
    OPMODE getOPmode(){ return this->ctrl_mode; };
    // return 
    //int read_joint_angles();

    
    // main process which will be done by the thread(s) to controll exoskeleton
    int send_joint_torques();
    int send_angle_to_ME3();
    



    std::string robot_type;
    int sampling_rate = EXO_ARM_SAMPLING_MS;
    ME3_ctrl me3_robot;
    
 
    json* motor_status;
    json* motor_setting;

    std::vector<bldc_ctrl> L_bldc_joints;
    std::vector<bldc_ctrl> R_bldc_joints;
    std::map<std::string,robot_status> joint_robots;
    
};

#endif
