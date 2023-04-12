#ifndef BLDC_CTRL_H
#define BLDC_CTRL_H_

#include <libserial/SerialPort.h>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <nlohmann/json.hpp>

using namespace LibSerial;

constexpr const int READ_BUFFER_SIZE = 256 ;
constexpr const int BLDC_SUCCESS = 0 ;
constexpr const int BLDC_FAIL = 1 ;




// exo-skeleten BLDC status
struct bldc_status
{
    std::string name;   // joint name
    int id;             // BLDC id
    int torque;         // read torque

    uint16_t encoder;   // encoder value
    uint16_t encoder_offset;    //set encoder zero let encoder won't cross zero during movement

    float angle;          // read angle
    float last_angle;     // last angle
    

    float relative_angle; // relative angle = angle - angle_offset
    float angle_offset;   // angle offset (update in each alignment)

    bool exist;         // no response
    


    bool moveClockwise; // motor movement
    int movingSpeed;    // no use in operation

    int torqueLow;
    int torqueHigh;

    int gearRatio;      // to angle

    float torqueRatio;   // torque radio for robot torquee feedback to exo-skeleton
    float torqueOffset;   // torque offset to balance friction of joint

    int driving_torque; // set touque if feedback mode
    
    int temperature;    // bldc temperature

    int encode_limit = 16383;

    //me3 mapping
    std::string ME3Motor;
    int ME3MotorInx;
    int ME3Dir;
    float ME3Angle_ref;
    float ME3Angle;
};



using json = nlohmann::json;

class bldc_ctrl
{
private:
    
    SerialPort  *serial_port;
    DataBuffer read_buffer;

    int serial_send_commd (DataBuffer wbytes);
    int serial_read(int read_size);
    
public:
    bldc_ctrl(std::string name, int id, SerialPort *port, json* setting_json);
    ~bldc_ctrl();

    bool isExist() { return status.exist; }
    std::string getName() { return status.name; }
    void getRelAngle();

    bool isClockWise();    
    int read_joint_encoder();
    int read_joint_angle();
    int read_joint_model();

    int send_joint_torque(int torque);
    int write_joint_encoder_offset();

    bldc_status status;     //bldc status
    
};

#endif
