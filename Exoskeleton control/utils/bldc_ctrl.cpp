#include <bldc_ctrl.h>

//#define MODULE_TEST

int bldc_ctrl::serial_send_commd (DataBuffer wbytes)
{
    // uint8_t checksum = 0;
    // for (std::vector<uint8_t>::iterator it = wbytes.begin(); it != wbytes.end()-1; ++it)
	// {
	// 	checksum += *it;
	// }
    // wbytes[wbytes.size() - 1] = checksum;
    
    
    this->serial_port->Write(wbytes);
    return 0;
}

int bldc_ctrl::serial_read(int read_size)
{
    this->read_buffer.clear();
    size_t ms_timeout = 250 ;
    char header;


    try
    {
        do
        {
            this->serial_port->ReadByte(header,ms_timeout);
        }while(header != 0x3e);

        this->serial_port->Read(this->read_buffer,read_size-1,ms_timeout);

        
    }
    catch (const ReadTimeout&)
    {
        
        std::cerr << "Joint: " << this->getName() << ", BLDC ID: " << this->status.id << " response has timed out." << std::endl ;
        return 1;
    }

    this->read_buffer.insert(this->read_buffer.begin(),header);
    

    return 0;
}

bldc_ctrl::bldc_ctrl(std::string name, int id, SerialPort *port, json* setting_json)
{

    this->status.name = name;
    this->serial_port = port;
    this->status.id = id;
    this->status.torque = 0;
    this->status.angle = 0;
    this->status.last_angle = this->status.angle;

    this->status.moveClockwise = false;
    this->status.movingSpeed = 100;
    this->status.torqueOffset = 10;


    this->status.driving_torque = 0;
    this->status.temperature = 25;
    
    int ret = this->read_joint_model();
    
}

bldc_ctrl::~bldc_ctrl()
{
}

/**
 * @brief read the driver model, motor model, hardware version number, 
 *        and firmware version number
 * @param None
 */

int bldc_ctrl::read_joint_model()
{
    DataBuffer writebuf = {0x3e ,0x12 ,0x00 ,0x00 ,0x00};
    int length = writebuf.size();
    writebuf[2] = this->status.id;
    for (int i=0; i<length-1 ; i++)
        writebuf[length-1] += writebuf[i];
    this->serial_send_commd(writebuf);
    // read driver response
    if (this->serial_read(48) == BLDC_FAIL )
    {
        this->status.exist = false;
        return BLDC_FAIL;
    }
    this->status.exist = true;

    // delay a 50s
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return BLDC_SUCCESS;
}


/**
 * @brief read the current angle
 *        
 * @param None
 */
int bldc_ctrl::read_joint_angle()
{
    uint8_t cmd = 0x92;
    DataBuffer writebuf = {0x3e ,cmd ,0x00 ,0x00 ,0x00};
    int length = writebuf.size();
    writebuf[2] = this->status.id;
    for (int i=0; i<length-1 ; i++)
        writebuf[length-1] += writebuf[i];

    this->serial_send_commd(writebuf);
    // read driver response
    if (this->serial_read(14) == BLDC_FAIL )
    {
        return BLDC_FAIL;
    }
    if ((this->read_buffer[0] != 0x3E) && (this->read_buffer[1] != cmd) && (this->read_buffer[2] != this->status.id) )
        return BLDC_FAIL;

    int64_t angle=0;
    float motor_angle;
    angle |= (int64_t)read_buffer[5];
    angle |= (((int64_t)read_buffer[6])<<8);
    angle |= (((int64_t)read_buffer[7])<<16);
    angle |= (((int64_t)read_buffer[8])<<24);
    angle |= (((int64_t)read_buffer[9])<<32);
    angle |= (((int64_t)read_buffer[10])<<40);
    angle |= (((int64_t)read_buffer[11])<<48);
    angle |= (((int64_t)read_buffer[12])<<56);

    this->status.angle = (float)angle/this->status.gearRatio/100;
    // if (this->status.id == 9){
    //     printf("ID %d, Angle: %f \n",this->status.id, this->status.angle);
    // }
    
    

    return BLDC_SUCCESS;
}


/**
 * @brief read the current position of the encoder,
 *        original position of encode and encoder offset
 * @param None
 */
int bldc_ctrl::read_joint_encoder()
{
    uint8_t cmd = 0x90;
    DataBuffer writebuf = {0x3e ,cmd ,0x00 ,0x00 ,0x00};

    int length = writebuf.size();
    writebuf[2] = this->status.id;
    for (int i=0; i<length-1 ; i++)
        writebuf[length-1] += writebuf[i];

    this->serial_send_commd(writebuf);
    // read driver response
    if (this->serial_read(12) == BLDC_FAIL )
    {
        return BLDC_FAIL;
    }
    if ((this->read_buffer[0] != 0x3E) && (this->read_buffer[1] != cmd) && (this->read_buffer[2] != this->status.id) )
        return BLDC_FAIL;

    this->status.encoder = read_buffer[5] + read_buffer[6]*256;
    this->status.encoder_offset = read_buffer[9] + read_buffer[10]*256;

    return BLDC_SUCCESS;
}

/**
 * @brief Writes the encoder value to 
 *        ROM as the motor zero command
 * @param uint16_t encoder_offset
 */
int bldc_ctrl::write_joint_encoder_offset()
{
    uint8_t cmd = 0x91;

    int sum = 0;
    DataBuffer writebuf = {0x3e ,cmd ,0x00 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00};
    writebuf[2] = this->status.id;
    for (int i=0; i<4 ; i++)
        writebuf[4] += writebuf[i];
        
    writebuf[5] = this->status.encoder_offset%256;
    writebuf[6] = this->status.encoder_offset/256;
    sum = 0;
    for (int i=5; i<7 ; i++)
        writebuf[7] += writebuf[i];

    this->serial_send_commd(writebuf);
    // read driver response
    const int length = 8;
    if (this->serial_read(length) == BLDC_FAIL )
    {
        return BLDC_FAIL;
    }

    // delay a 50s
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // // get data checksum
    // for (int i=5; i<length ; i++)
    //     sum += this->read_buffer[i];
    
    // if ((this->read_buffer[0] != 0x3E) && (this->read_buffer[1] != cmd) \
    //     && (this->read_buffer[2] != this->status.id) && (this->read_buffer[length-1] != sum))
    //     return BLDC_FAIL;
    // // get temperature, Tourqe current , motor speed and encoder 
    // this->status.temperature = this->read_buffer[5];

    // this->status.torque = (int)(((this->read_buffer[7] & 0xFF) << 8) | (this->read_buffer[6] & 0xFF));
    // this->status.encoder = (uint16_t)(this->read_buffer[10] + this->read_buffer[11]*256);
    
    return BLDC_SUCCESS;
}

/**
 * @brief read the current position of the encoder,
 *        original position of encode and encoder offset
 * @param int torque 
 */
int bldc_ctrl::send_joint_torque(int torque)
{
    // tourue from -2000 to 2000 corresponding to -32A to 32A    
    uint8_t cmd = 0xa1;
    int sum = 0;
    DataBuffer writebuf = {0x3e ,cmd ,0x00 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00};
    writebuf[2] = this->status.id;
    for (int i=0; i<4 ; i++)
        writebuf[4] += writebuf[i];
        
    // writebuf[5] = torque%256;       // low byte
    // writebuf[6] = torque/256;       // high byte

    writebuf[5] = (uint8_t)(torque & 0x00ff);   // low byte
    writebuf[6] = (uint8_t)((torque & 0xff00)>>8);   // high byte

    
    sum = 0;
    for (int i=5; i<7 ; i++)
        writebuf[7] += writebuf[i];

    this->serial_send_commd(writebuf);
    // read driver response
    const int length = 13;
    if (this->serial_read(length) == BLDC_FAIL )
    {
        for (auto& byte:writebuf)
        {
            std::cout << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (((int)byte) & 0xFF) << " ";
            
        }
        std::cout << std::endl;
        return BLDC_FAIL;
    }
    // get data checksum
    for (int i=5; i<length ; i++)
        sum += this->read_buffer[i];
    
    if ((this->read_buffer[0] != 0x3E) && (this->read_buffer[1] != cmd) \
        && (this->read_buffer[2] != this->status.id) && (this->read_buffer[length-1] != sum))
        return BLDC_FAIL;
    // get temperature, Tourqe current , motor speed and encoder 
    this->status.temperature = this->read_buffer[5];

    this->status.torque = (int)(((this->read_buffer[7] & 0xFF) << 8) | (this->read_buffer[6] & 0xFF));
    this->status.encoder = (uint16_t)(this->read_buffer[10] + this->read_buffer[11]*256);
    
    return BLDC_SUCCESS;
}

void bldc_ctrl::getRelAngle()
{
    //this->status.angle = ((float)(this->status.encoder*360))/this->status.encode_limit;
    
    this->status.relative_angle = this->status.angle - this->status.angle_offset;
    
}

#ifdef MODULE_TEST
int main()
{
    

    return 0;
}
#endif
