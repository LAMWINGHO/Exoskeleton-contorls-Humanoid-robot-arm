#include <robot_arm.h>
#include <vector>

using namespace std;

void arm_ctrl::initial_exo_arm(std::map<string,int> ids, SerialPort *port , std::vector<bldc_ctrl>* joints)
{
    for (auto id:ids)
    {
        bldc_ctrl motor = bldc_ctrl(id.first,id.second,port,this->motor_setting);
        if (motor.isExist())
        {
            joints->emplace_back(motor);
            
            id.second = 0;      
            // this->joint_angles.insert(id);      // angle = 0;
            // this->joint_torques.insert(id);     // torque = 0;

            int n = 3; // Number of states
            int m = 1; // Number of measurements

            double dt = 1.0/30; // Time step

            Eigen::MatrixXd A(n, n); // System dynamics matrix
            Eigen::MatrixXd C(m, n); // Output matrix
            Eigen::MatrixXd Q(n, n); // Process noise covariance
            Eigen::MatrixXd R(m, m); // Measurement noise covariance
            Eigen::MatrixXd P(n, n); // Estimate error covariance


            robot_status robot;
            // Discrete LTI projectile motion, measuring position only
            A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
            C << 1, 0, 0;

            // Reasonable covariance matrices
            Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
            //R << 5;
            R << 0.1;
            P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
            // Construct the filter
            KalmanFilter kfc(dt,A, C, Q, R, P);

            robot.torque_kf = kfc;
            robot.read_angle = 0;
            robot.filtered_torque = 0;
            robot.read_torque = 0;

            this->joint_robots.emplace(id.first,robot);


        }
        else
        {
            if (id.first.find("L") != string::npos)
            {
                // set value to "none" if the motor does not exist
                //this->motor_status->at("ExoLeft").at(id.first).at("angle") = "None";
                //this->motor_status->at("ExoLeft").at(id.first).at("torque") = "None";
                // set value to "none" if the motor does not exist
                this->motor_status->at("ExoLeft").erase(id.first);
                //this->motor_status->at("ExoLeft").at(id.first).at("torque") = "None";

            }
            if (id.first.find("R") != string::npos)
            {
                this->motor_status->at("ExoRight").erase(id.first);
                
                // this->motor_status->at("ExoRight").at(id.first).at("angle") = "None";
                // this->motor_status->at("ExoRight").at(id.first).at("torque") = "None";
            }
            
        }
    }
}

arm_ctrl::arm_ctrl(json* status, json* msetting, OPMODE mode, my_serials* ports)
{
    // initial ME3 robot setting
    // const char* ip = "192.168.1.105";
    // const char* test_ip = "192.168.1.122";
    //ME3_ctrl robot = ME3_ctrl(test_ip);
    std::string robot_ip = msetting->at("Robot_IP").get<std::string>();
    this->me3_robot.setIP(&robot_ip[0]);

    this->sampling_rate = msetting->at("Sampling_Rate").get<int>();
    // robot type
    this->robot_type = msetting->at("Robot_Type").get<std::string>();

    for (auto& el : msetting->at("Motor_Setting").items())
    {
        pair<string,int> arm_index;
        string id_str =  el.value().at("ID").get<std::string>();
        // string name = el.key();
        arm_index.first = el.key();

        arm_index.second = atoi(id_str.c_str());
        


        if (arm_index.first.find("L") != string::npos)
        {
            this->L_arm_index.insert(arm_index);
        }
        if (arm_index.first.find("R") != string::npos)
        {
            this->R_arm_index.insert(arm_index);
        }
        // std::cout << el.key() << "and " << el.value() << '\n';
    }

    this->motor_status = status;
    //json setting = msetting->at("Motor_Setting");
    this->motor_setting = &(msetting->at("Motor_Setting"));


    initial_exo_arm(this->L_arm_index, ports->l_arm_port, &this->L_bldc_joints);
    initial_exo_arm(this->R_arm_index, ports->r_arm_port, &this->R_bldc_joints);

    this->ctrl_mode = mode;
}

// arm_ctrl::arm_ctrl(json* status, std::map<std::string,int> &ids, OPMODE mode, SerialPort *port)
// {
//     this->motor_status = status;
//     for (auto id:ids)
//     {
//         bldc_ctrl motor = bldc_ctrl(id.first,id.second,port);
//         if (motor.isExist())
//         //if (true)
//         {
//             this->joints.emplace_back(motor);
//             id.second = 0;      
//             this->joint_angles.insert(id);      // angle = 0;
//             this->joint_torques.insert(id);     // torque = 0;

//             int n = 3; // Number of states
//             int m = 1; // Number of measurements

//             double dt = 1.0/30; // Time step

//             Eigen::MatrixXd A(n, n); // System dynamics matrix
//             Eigen::MatrixXd C(m, n); // Output matrix
//             Eigen::MatrixXd Q(n, n); // Process noise covariance
//             Eigen::MatrixXd R(m, m); // Measurement noise covariance
//             Eigen::MatrixXd P(n, n); // Estimate error covariance


//             robot_status robot;
//             // Discrete LTI projectile motion, measuring position only
//             A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
//             C << 1, 0, 0;

//             // Reasonable covariance matrices
//             Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
//             //R << 5;
//             R << 0.1;
//             P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
//             // Construct the filter
//             KalmanFilter kfc(dt,A, C, Q, R, P);

//             robot.torque_kf = kfc;
//             robot.read_angle = 0;
//             robot.filtered_torque = 0;
//             robot.read_torque = 0;

//             this->joint_robots.emplace(id.first,robot);


//         }
//     }
    
//     this->ctrl_mode = mode;
    
// }
// arm_ctrl::arm_ctrl(std::vector<int> &ids, OPMODE mode, SerialPort *port)
// {    
//     for (auto id:ids)
//     {
//         bldc_ctrl motor = bldc_ctrl(id,port);
//         if (motor.isExist())
//         {
//             this->joints.emplace_back(motor);

//         }
//     }
    
//     this->ctrl_mode = mode;
    
// }

arm_ctrl::~arm_ctrl()
{
    //this->serial_port->Close();
}


//"+IPD : L_UP_ARM  15 20 00\n L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)\n R_UP_ARM  25 20 00\n R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)";
int arm_ctrl::send_angle_to_ME3()
{

    char buffer[1024];
    uint L_Upper[3] = {15,20,0};
    uint L_Forearm[3]= {0,0,0};
    uint L_Finers[5] = {0x700,0x700,0x700,0x700,0x700};

    uint R_Upper[3] = {15,20,0};
    uint R_Forearm[3] = {0,0,0};
    uint R_Finers[5] = {0x700,0x700,0x700,0x700,0x700};
    

    // mapping angles to output arrays
    for (auto &joint:this->L_bldc_joints)
    {
        joint.getRelAngle();
        if (joint.status.ME3Dir == 0){
            joint.status.ME3Angle = (joint.status.angle - joint.status.ME3Angle_ref);
        }
        else{
            joint.status.ME3Angle = (joint.status.ME3Angle_ref - joint.status.angle);
        }
        
        // map joint angle status to corresponidng ME3 motor
        if (joint.status.ME3Motor.compare("L_UP_ARM")==0)
        {
            L_Upper[joint.status.ME3MotorInx] += joint.status.ME3Angle;
        }
        if (joint.status.ME3Motor.compare("L_FOREARM")==0)
        {
            L_Forearm[joint.status.ME3MotorInx] += joint.status.ME3Angle;
        }
    }
    for (auto &joint:this->R_bldc_joints)
    {
        joint.getRelAngle();
        if (joint.status.ME3Dir == 0){
            joint.status.ME3Angle = (joint.status.angle - joint.status.ME3Angle_ref);
        }
        else{
            joint.status.ME3Angle = (joint.status.ME3Angle_ref - joint.status.angle);
        }
        
        // map joint angle status to corresponidng ME3 motor
        if (joint.status.ME3Motor.compare("R_UP_ARM")==0)
        {
            R_Upper[joint.status.ME3MotorInx] += joint.status.ME3Angle;
        }
        if (joint.status.ME3Motor.compare("R_FOREARM")==0)
        {
            R_Forearm[joint.status.ME3MotorInx] += joint.status.ME3Angle;
        }
    }

    // int j = snprintf(buffer, sizeof(buffer),
    // "+IPD : L_UP_ARM  %d %d %d\nL_FOREARM %d %d %d (thumb: 0x%03x fore: 0x%03x middle: 0x%03x ring: 0x%03x pinky: 0x%03x)\n"
    // "R_UP_ARM  %d %d %d\nR_FOREARM %d %d %d (thumb: 0x%03x fore: 0x%03x middle: 0x%03x ring: 0x%03x pinky: 0x%03x)",
    //    L_Upper[0], L_Upper[1], L_Upper[2],L_Forearm[0],L_Forearm[1],L_Forearm[2],L_Finers[0],L_Finers[1],L_Finers[2],L_Finers[3],L_Finers[4],R_Upper[0], R_Upper[1], R_Upper[2],R_Forearm[0],R_Forearm[1],R_Forearm[2],R_Finers[0],R_Finers[1],R_Finers[2],R_Finers[3],R_Finers[4]);

int j = snprintf(buffer, sizeof(buffer),
    "+IPD : L_UP_ARM  %d %d %d\nL_FOREARM %d %d %d \n"
    "R_UP_ARM  %d %d %d\nR_FOREARM %d %d %d \n",
       L_Upper[0], L_Upper[1], L_Upper[2],L_Forearm[0],L_Forearm[1],L_Forearm[2],R_Upper[0], R_Upper[1], R_Upper[2],R_Forearm[0],R_Forearm[1],R_Forearm[2]);

    
    this->me3_robot.send_bytes_to_ME3(&buffer[0]);
    return 0;
}

int arm_ctrl::send_joint_torques()
{
    for (auto &joint:this->L_bldc_joints)
    {
        if (this->ctrl_mode == FEEDBACK)
        {
            joint.send_joint_torque(joint.status.driving_torque);
            //joint.read_joint_angle();
        }
        // read joint angle
        joint.read_joint_angle();
        // get relative anlge
        joint.getRelAngle();

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << joint.status.relative_angle;
        
                
        this->motor_status->at("ExoLeft").at(joint.status.name).at("angle") = ss.str();

        // this->joint_angles.insert(j);
        
    }
    for (auto &joint:this->R_bldc_joints)
    {
        if (this->ctrl_mode == FEEDBACK)
        {
            joint.send_joint_torque(joint.status.driving_torque);
            //joint.read_joint_angle();
        }
        // read joint angle
        joint.read_joint_angle();
        // get relative anlge
        joint.getRelAngle();

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << joint.status.relative_angle;
        
                
        this->motor_status->at("ExoRight").at(joint.status.name).at("angle") = ss.str();

        // this->joint_angles.insert(j);
        
    }
    //this->upper_euler[0] = 
    return 0;
}


void arm_ctrl::arm_operation(string& ctrl_cmd)
{
    
    if(!ctrl_cmd.empty()){
        if (ctrl_cmd.find("Start_Operation")!=string::npos){
            this->ctrl_mode = CONTROL;
        }
        else if (ctrl_cmd.find("Stop_Operation")!=string::npos){
            this->ctrl_mode = STOP;
        }
        else if (ctrl_cmd.find("Alignment")!=string::npos){
            this->ctrl_mode = ALIGN_ANGLE;
        }
        else if (ctrl_cmd.find("Torque_Sim")!=string::npos){
            std::ostringstream ss;
            std::ifstream input(TORQUE_SIM_JSON);
            input >> torque_buffer;
            input.close();
            this->ctrl_mode = TORQUE_SIM;
            json ctrl_info = json::parse(ctrl_cmd);
            string armHand =  ctrl_info.at("TorqueSimHand").get<std::string>();
            string motorName =  ctrl_info.at("TorqueSimNum").get<std::string>();
            string moveFormat =  ctrl_info.at("TorqueSimFormat").get<std::string>();

            int raw_torque = 0;
            motorName.insert(motorName.begin(),armHand[0]);
            
            if (moveFormat.compare("PositionMove") == 0)
            {
                for (auto& el : torque_buffer.items())
                {   
                    float timestamp =  el.value().at("timestamp").get<float>();
                    if ((timestamp > 2) && (timestamp < 10))
                    {
                        el.value().at("simtorque").at(motorName) = raw_torque++;
                    }
                    else if (timestamp >= 10)
                    {
                        el.value().at("simtorque").at(motorName) = raw_torque--;
                        /* code */
                    }
                    else{
                        el.value().at("simtorque").at(motorName) = 0;
                    }
                    if (raw_torque<0)
                    {
                        el.value().at("simtorque").at(motorName) = 0;

                    }
                }
            } else if (moveFormat.compare("NegativeMove") == 0)
            {
                for (auto& el : torque_buffer.items())
                {   
                    float timestamp =  el.value().at("timestamp").get<float>();
                    if ((timestamp > 2) && (timestamp < 10))
                    {
                        el.value().at("simtorque").at(motorName) = raw_torque--;
                    }
                    else if (timestamp >= 10)
                    {
                        el.value().at("simtorque").at(motorName) = raw_torque++;
                        /* code */
                    }
                    else{
                        el.value().at("simtorque").at(motorName) = 0;
                    }
                    if (raw_torque>0)
                    {
                        el.value().at("simtorque").at(motorName) = 0;

                    }
                }
                
            }
            else if (moveFormat.compare("PositionPulse") == 0)
            {
                for (auto& el : torque_buffer.items())
                {   
                    float timestamp =  el.value().at("timestamp").get<float>();
                    if ((timestamp > 5) && (timestamp < 15) )
                    {
                        el.value().at("simtorque").at(motorName) = 50;
                    }
                    else{
                        el.value().at("simtorque").at(motorName) = 0;
                    }
                }
            }
            else
            {
                for (auto& el : torque_buffer.items())
                {   
                    float timestamp =  el.value().at("timestamp").get<float>();
                    if ((timestamp > 5) && (timestamp < 15) )
                    {
                        el.value().at("simtorque").at(motorName) = -50;
                    }
                    else{
                        el.value().at("simtorque").at(motorName) = 0;
                    }
                }

            }

            // for (auto& el : torque_buffer.items())
            // {
                
            //     el.value().at("simtorque").at(motorNum)
            //     std::cout << "key: " << el.key() << ", value:" << el.value() << '\n';
                
            // }
            


            // float timestamp =  torque_buffer_it.value().at("timestamp").get<float>();
            // int raw_torque = torque_buffer_it.value().at("simtorque").at(joint.getName()).get<int>();
            

            this->torque_buffer_it = torque_buffer.begin();
        }
        ctrl_cmd.clear();
    }
    
    switch(this->ctrl_mode)
    {
        case CONTROL:
            send_joint_torques();
            if (this->robot_type.compare("ME3")==0)
            {
                send_angle_to_ME3();
            }
            

            ///////////////////////////
            // test kalman filter
            ///////////////////////////
            // for (auto &joint:this->L_bldc_joints)
            // {
            //     float angle =joint.getAngle();
            //     // use exo skeleton angle pretend to be a robot-torque
            //     joint_robots.find(joint.getName())->second.read_torque = angle;
            // }
            
            // for (auto &joint_robot:joint_robots)
            // {
                
            //     if (!joint_robot.second.torque_kf.getinitialized()){
            //         Eigen::VectorXd x0(3);
            //         double t = 0;
            //         x0 << joint_robot.second.read_torque, 0, -9.81;
            //         joint_robot.second.torque_kf.init(t,x0);
                    
            //     }
            //     else{
            //         Eigen::VectorXd y(1);
            //         y << joint_robot.second.read_torque;
            //         joint_robot.second.torque_kf.update(y);
            //         joint_robot.second.filtered_torque = joint_robot.second.torque_kf.state()[0];

            // //     }              
            // }
            break;
        case FEEDBACK:
            break;
        case FEEDFORWORD:
            break;
        case MANUEL:
            break;
        case STOP:
            break;
        case TORQUE_SIM:
        {
            float timestamp =  torque_buffer_it.value().at("timestamp").get<float>();
            // assign right arm torque
            for (auto &joint:this->R_bldc_joints)
            {
                int raw_torque = torque_buffer_it.value().at("simtorque").at(joint.getName()).get<int>();
                joint.status.driving_torque = (int)((float)raw_torque * (float)joint.status.torqueRatio + (float)joint.status.torqueOffset);
            }
            // assign left arm torque
            for (auto &joint:this->L_bldc_joints)
            {
                int raw_torque = torque_buffer_it.value().at("simtorque").at(joint.getName()).get<int>();
                joint.status.driving_torque = (int)((float)raw_torque * (float)joint.status.torqueRatio + (float)joint.status.torqueOffset);
            }

            if (++torque_buffer_it >= torque_buffer.end())
            {
                this->ctrl_mode = STOP;
            }
            send_joint_torques();
        }break;
        case ALIGN_ANGLE:
            send_joint_torques();
            // Align skeleton and robot angles
            if (!this->motor_status->at("InitAngle").get<bool>()){
                this->motor_status->at("InitAngle") = true;
                for (auto &joint:this->L_bldc_joints)
                {
                    //joint.read_joint_angle();
                    joint.status.angle_offset = joint.status.angle;
                    joint.status.ME3Angle_ref = joint.status.angle;
                    joint.status.ME3Angle = joint.status.angle;
                }
                for (auto &joint:this->R_bldc_joints)
                {
                    //joint.read_joint_angle();
                    joint.status.angle_offset = joint.status.angle;
                    joint.status.ME3Angle_ref = joint.status.angle;
                    joint.status.ME3Angle = joint.status.angle;
                }

            }
            else{
                this->motor_status->at("InitAngle") = false;
                this->ctrl_mode = CONTROL;
            }

            break;
    }
    
}
