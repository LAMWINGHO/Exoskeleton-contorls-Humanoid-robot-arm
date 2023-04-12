#include <stdio.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <errno.h>
#include <vector>   //for using vectors instead of arrays
#include <map>
#include <array>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <libserial/SerialPortConstants.h>
#include <nlohmann/json.hpp>

#include "crow.h"
#include <unordered_set>
#include <mutex>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <robot_arm.h>
//#include <bldc_ctrl.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "kalman.hpp"


using namespace Eigen;
using namespace LibSerial;
using namespace std;
using json = nlohmann::json;

constexpr const char* const SERIAL_PORT_STR = "/dev/ttyUSB0";

constexpr const char* const MOTOR_SETTING_JSON =  "./configs/motor_settings.json";
constexpr const char* const EXO_exo_motor_status_JSON = "./configs/exo_status.json";


//constexpr std::array<int, 5> L_ARM_ID = {1,2,3,4,5};         //joint ID for RS485 control
// constexpr int L_ARM_ID[] = {1,2,3,4,5};         //joint ID for RS485 control
// constexpr int R_ARM_ID[] = {6,7,8,9,10};        //joint ID for RS485 control

//using LibSerial::SerialPort ;
// Set the baud rates.

static my_serials_t my_serials;


#define DEBUG_MODE

class http_server
{
private:
    crow::SimpleApp app;
    std::mutex mtx;;
    std::unordered_set<crow::websocket::connection*> users;
    thread *t;

public:
    http_server(json* setting);
    http_server();
    ~http_server(){};
    void start_server();
    void send_message(string text);

    // variable
    bool isSettingChanged;
    bool isConnected;
    json* motor_setting;
    string robot_cmd;
    
};



http_server::http_server(json* setting)
{
    this->isConnected = false;
    this->isSettingChanged = true;
    this->motor_setting = setting;
    this->robot_cmd = "";
    // std::stringstream ss;
    // std::ifstream input("/home/pi/project/ex                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     o_control_cpp/configs/motor_settings.json");
    // input >> this->motor_setting;
   
    t = new thread(&http_server::start_server,this);
}

http_server::http_server()
{
    // this->isConnected = false;
    // this->isSettingChanged = false;
    // std::stringstream ss;
    // std::ifstream input("/home/pi/project/exo_control_cpp/configs/motor_settings.json");
    // input >> this->motor_setting;
   
    // t = new thread(&http_server::start_server,this);
}

void http_server::start_server()
{
    // CROW_ROUTE(this->app, "/data")
    // ([]{
    //     //std::string stats = this->exo_motor_status.dump();
    //     std::string stats = "status";
    //     return stats;
    // });

    //.methods(crow::HTTPMethod::GET
    CROW_ROUTE(this->app, "/ws")
        .websocket()
        .onopen([&](crow::websocket::connection& conn){
                CROW_LOG_INFO << "new websocket connection";
                std::lock_guard<std::mutex> _(mtx);
                users.insert(&conn);
                //send motor setting when connect
                this->isConnected = true;
                std::string s = this->motor_setting->dump();
                conn.send_text(s);


                })
        .onclose([&](crow::websocket::connection& conn, const std::string& reason){
                CROW_LOG_INFO << "websocket connection closed: " << reason;
                std::lock_guard<std::mutex> _(mtx);
                users.erase(&conn);
                })

        .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary){
                std::lock_guard<std::mutex> _(mtx);
                if (data.find("Ctrl_Command")!=string::npos){
                    // Start arm operation and send exo_motor_status to robot side
                    this->robot_cmd = data;
                }
                else if (data.find("Read_Setting")!=string::npos)
                {
                    std::string s = this->motor_setting->dump();
                    conn.send_text(s);
                }
                else if (data.find("Motor_Setting")!=string::npos)
                {
                    // update motor_setting
                    *(this->motor_setting) = json::parse(data);
                    this->isSettingChanged = true;
                }
               
                });

    CROW_ROUTE(app, "/")
    ([]{
        char name[256];
        gethostname(name, 256);
        crow::mustache::context x;
        x["servername"] = name;
	
        auto page = crow::mustache::load("setting.html");
        
        
        return page.render(x);
     });

     

     

    app.port(45680)
        .multithreaded()
        .run();
}

void http_server::send_message(string text)
{
    if (!users.empty())
    {
        for(auto u:users)
            u->send_text(text);
    }
}

void UpdateMotorSetting(arm_ctrl* arm, json* setting)
{
    // robot_type
    arm->robot_type = setting->at("Robot_Type").get<std::string>();
    arm->sampling_rate = setting->at("Sampling_Rate").get<int>();

    // assume left arm
    for (auto &joint:arm->L_bldc_joints)
    {
        // set encodeOffset
        // int value = (*setting).at("Motor_Setting").at(joint.status.name).at("encodeOffset").get<int>();
        // if (joint.status.encoder_offset != value)
        // {
        //     // update encoder offset
        //     joint.status.encoder_offset = value;
        //     joint.write_joint_encoder_offset();
        // }
        joint.status.encoder_offset = (*setting).at("Motor_Setting").at(joint.status.name).at("encodeOffset").get<int>();
        joint.write_joint_encoder_offset();
        // set torqueLow
        joint.status.torqueLow = setting->at("Motor_Setting").at(joint.status.name).at("torqueLow").get<int>();
        // set torqueHigh
        joint.status.torqueHigh = setting->at("Motor_Setting").at(joint.status.name).at("torqueHigh").get<int>();

        joint.status.gearRatio = setting->at("Motor_Setting").at(joint.status.name).at("gearRatio").get<int>();
        
        // set driving_torque
        joint.status.driving_torque = setting->at("Motor_Setting").at(joint.status.name).at("drivingTorque").get<int>();
        // set encoder limit
        joint.status.encode_limit = setting->at("Motor_Setting").at(joint.status.name).at("encodeLimit").get<int>();
        // set torque offset
        joint.status.torqueOffset = setting->at("Motor_Setting").at(joint.status.name).at("torqueOffset").get<float>();
        // set torque Ratio
        joint.status.torqueRatio = setting->at("Motor_Setting").at(joint.status.name).at("torqueRatio").get<float>();

        joint.status.ME3Motor = setting->at("Motor_Setting").at(joint.status.name).at("ME3Motor").get<std::string>();
        joint.status.ME3MotorInx = setting->at("Motor_Setting").at(joint.status.name).at("ME3MotorInx").get<int>();
        joint.status.ME3Dir = setting->at("Motor_Setting").at(joint.status.name).at("ME3Dir").get<int>();


    }
    for (auto &joint:arm->R_bldc_joints)
    {
        // set encodeOffset
        joint.status.encoder_offset = (*setting).at("Motor_Setting").at(joint.status.name).at("encodeOffset").get<int>();
        joint.write_joint_encoder_offset();
        // set torqueLow
        joint.status.torqueLow = setting->at("Motor_Setting").at(joint.status.name).at("torqueLow").get<int>();
        // set torqueHigh
        joint.status.torqueHigh = setting->at("Motor_Setting").at(joint.status.name).at("torqueHigh").get<int>();
        joint.status.gearRatio = setting->at("Motor_Setting").at(joint.status.name).at("gearRatio").get<int>();
        // set driving_torque
        joint.status.driving_torque = setting->at("Motor_Setting").at(joint.status.name).at("drivingTorque").get<int>();
        // set encoder limit
        joint.status.encode_limit = setting->at("Motor_Setting").at(joint.status.name).at("encodeLimit").get<int>();
        // set torque Ratio
        joint.status.torqueOffset = setting->at("Motor_Setting").at(joint.status.name).at("torqueOffset").get<float>();
        // set torque Ratio
        joint.status.torqueRatio = setting->at("Motor_Setting").at(joint.status.name).at("torqueRatio").get<float>();

        joint.status.ME3Motor = setting->at("Motor_Setting").at(joint.status.name).at("ME3Motor").get<std::string>();
        joint.status.ME3MotorInx = setting->at("Motor_Setting").at(joint.status.name).at("ME3MotorInx").get<int>();
        joint.status.ME3Dir = setting->at("Motor_Setting").at(joint.status.name).at("ME3Dir").get<int>();

    }

    if((*setting).find("Save") != (*setting).end())
    {
        (*setting).erase("Save");
        std::ofstream file(MOTOR_SETTING_JSON);
        file << std::setw(4) << (*setting) << std::endl;        // use this instead of dump()
        file.close();

    }
    
    
}

// global 
static json motor_setting;
static json exo_motor_status;

int main(int argc, char* argv[])
{
    float r_para = 5;
    if (argc > 1)
    {
        r_para = atof(argv[1]);        
        // set R parameter to balance the delay and smoothness
    }

    my_serials.l_arm_port = new SerialPort();
    // Open the hardware serial ports.
    try
    {
        // Open the Serial Ports at the desired hardware devic es.
        my_serials.l_arm_port->Open(SERIAL_PORT_STR);
        my_serials.l_arm_port->SetBaudRate(BaudRate::BAUD_115200);
        my_serials.l_arm_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        my_serials.l_arm_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        my_serials.l_arm_port->SetParity(Parity::PARITY_NONE);
        my_serials.l_arm_port->SetStopBits(StopBits::STOP_BITS_1);

        my_serials.r_arm_port = my_serials.l_arm_port;

    }
    catch (const LibSerial::OpenFailed)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }


    std::ostringstream ss;
    std::ifstream input(MOTOR_SETTING_JSON);
    input >> motor_setting;
    input.close();
    std::ifstream input2(EXO_exo_motor_status_JSON);
    input2 >> exo_motor_status;
    input2.close();

    // json torque_ss;

    // std::ifstream input3(TORQUE_SIM_JSON);
    // input3 >> torque_ss;
    // input3.close();
    




    // //test json
    // for (auto& el : torque_ss.items())
    // {
    //     auto el_key = el.key();
    //     json el_value = el.value();
    //     float timestamp =  el.value().at("timestamp").get<float>();
    //     int r1_torque = el.value().at("simtorque").at("R1").get<int>();
    //     int r2_torque = el.value().at("simtorque").at("R2").get<int>();
    //     int r3_torque = el.value().at("simtorque").at("R3").get<int>();

    //     //el_value.at("timestamp")

    // }
    

    json setting = motor_setting.at("Motor_Setting");
    arm_ctrl arm_joints = arm_ctrl(&exo_motor_status, &motor_setting, OPMODE::STOP, &my_serials);

    for (auto &joint_robot:arm_joints.joint_robots)
    {
        Eigen::MatrixXd R(1, 1); // Measurement noise covariance
        joint_robot.second.kalman_r = r_para;
        R << r_para;
        joint_robot.second.torque_kf.Set_MatrixXD_R(R);
    } 

    // #if EXO_ARM_R == 1
    // arm_ctrl r_arm_joints = arm_ctrl(&exo_motor_status, R_arm_index, OPMODE::STOP, my_serials.r_arm_port);
    // for (auto &joint_robot:r_arm_joints.joint_robots)
    // {
    //     Eigen::MatrixXd R(1, 1); // Measurement noise covariance
    //     joint_robot.second.kalman_r = r_para;
    //     R << r_para;
    //     joint_robot.second.torque_kf.Set_MatrixXD_R(R);
    // } 
    // #endif
    // #if EXO_ARM_L == 1
    // arm_ctrl l_arm_joints = arm_ctrl(&exo_motor_status, L_arm_index, OPMODE::STOP, my_serials.l_arm_port);
    // for (auto &joint_robot:l_arm_joints.joint_robots)
    // {
    //     Eigen::MatrixXd R(1, 1); // Measurement noise covariance
    //     joint_robot.second.kalman_r = r_para;
    //     R << r_para;
    //     joint_robot.second.torque_kf.Set_MatrixXD_R(R);
    // } 
    // #endif

    // initialize Left arm joints    
    
    http_server server(&motor_setting);

    while(true)
    {
        auto start_time = std::chrono::steady_clock::now();
        if (server.isSettingChanged){
            server.isSettingChanged = false;
            UpdateMotorSetting(&arm_joints, &motor_setting);

        }
        arm_joints.arm_operation(server.robot_cmd);
         
        // start to send angles to robot        
        if ((arm_joints.getOPmode() == CONTROL) || (arm_joints.getOPmode() == ALIGN_ANGLE)){
            if (arm_joints.robot_type.compare("ME3") != 0)
            {
                std::string stats = exo_motor_status.dump();
                server.send_message(stats);
                std::cout << stats<< std::endl;
            }
            else
            {
                //arm_joints.me3_robot. . .send_angle_to_ME3(&arm_joints);
                arm_joints.send_angle_to_ME3();
                #ifdef DEBUG_MODE
                std::string stats = exo_motor_status.dump();
                server.send_message(stats);   
                #endif
            }

            
        }
        
        
        
        
        
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::milliseconds(arm_joints.sampling_rate) - std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::this_thread::sleep_for(std::chrono::milliseconds(elapsed));
        // std::ostringstream stats_ss;
        // stats_ss << "L1," << angle << ";" ;
        // auto stats = stats_ss.str();
        //server.send_message(stats);
        // cout << l_arm_joints.joints[0].status.angle << "," << l_arm_joints.joint_robots.at("L1").filtered_torque << ","\
        // << l_arm_joints.joints[1].status.angle << "," << l_arm_joints.joint_robots.at("L2").filtered_torque << endl;
        
        
        //angle += 1;
    }
}
 
