
#include <pg70_action_server.h>


pg70ActionServer::pg70ActionServer(std::string name) :
   pg70_as_(nh_, name, boost::bind(&pg70ActionServer::execute_cb, this, _1), false),
   action_name(name)
{

   ROS_INFO("gripper Action client initialization");


   nh_.getParam("/schunk_pg70/Grasp/portname", port_name_);
   nh_.getParam("/schunk_pg70/Grasp/baudrate", baudrate_);
   nh_.getParam("/schunk_pg70/Grasp/gripper_id", gripper_id_);

/*
  com_port_ = new serial::Serial (port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));

   if (com_port_->isOpen())
   {
      ROS_INFO_STREAM("PG70: Serial port: " << port_name_ << " openned");
         
      //Get initial state and discard input buffer
      while(pg70_error_ == 0xff)
      {
         pg70_error_ = getError(com_port_);
         ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
      }
      
      //Start periodic gripper state reading 
      getPeriodicPositionUpdate(com_port_, TF_UPDATE_PERIOD);
   }
   else
      ROS_ERROR_STREAM("ActionServer: Serial port " << port_name_ << " not opened");

   joint_sub_ = nh_.subscribe("schunk_pg70/joint_states", 1, &pg70ActionServer::joint_callback, this);
   */

   ROS_INFO("Set pvac service Connection: Waiting");  
   set_pvac = nh_.serviceClient<schunk_pg70::set_pvac>("/schunk_pg70/set_pvac");
   set_pvac.waitForExistence();
   ROS_INFO("Set pvac service Connection: OK");


   pg70_as_.start();
   ROS_INFO("Gripper Action Client: OK");

}

pg70ActionServer::~pg70ActionServer()
{
  com_port_->close();      //Close port
  delete com_port_;        //delete object
}




void pg70ActionServer::joint_callback(const sensor_msgs::JointState &msg)
{
	
   joint_state_ = msg;

   act_position_  = msg.position[0]*URDF_SCALE_FACTOR;
   act_velocity_  = msg.velocity[0]*URDF_SCALE_FACTOR;
   act_current_   = msg.effort[0]*URDF_SCALE_FACTOR;



	return;
}


void pg70ActionServer::getPeriodicPositionUpdate(serial::Serial *port, float update_period)
{
  std::vector<uint8_t> output;
  output.push_back(0x05);                 //message from master to the module
  output.push_back(gripper_id_);          //gripper id
  output.push_back(0x06);                 //Data Length
  output.push_back(0x95);                 //Command get state
    
  unsigned int IEEE754_bytes[4];
  float_to_IEEE_754(update_period, IEEE754_bytes);
  
  output.push_back(IEEE754_bytes[0]);    //period first byte
  output.push_back(IEEE754_bytes[1]);    //period second byte
  output.push_back(IEEE754_bytes[2]);    //period third byte
  output.push_back(IEEE754_bytes[3]);    //period fourth byte
  output.push_back(0x07);

  //Checksum calculation
  uint16_t crc = 0;                      

  for(size_t i = 0; i < output.size(); i++)
    crc = CRC16(crc,output[i]);

  //Add checksum to the output buffer
  output.push_back(crc & 0x00ff);
  output.push_back((crc & 0xff00) >> 8);
  
  //Send get_state message to the module 
  port->write(output);   
}



float pg70ActionServer::IEEE_754_to_float(uint8_t *raw)
{
  int sign = (raw[0] >> 7) ? -1 : 1;
  int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;

  uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

  float fraction = 0.5f;
  for (uint8_t ii = 0; ii < 24; ++ii)
    fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

  float significand = sign * fraction;

  return ldexpf(significand, exponent);
}

void pg70ActionServer::float_to_IEEE_754(float position, unsigned int *output_array)
{
  unsigned char *p_byte = (unsigned char*)(&position);

  for(size_t i = 0; i < sizeof(float); i++)
    output_array[i] = (static_cast<unsigned int>(p_byte[i]));
}

uint16_t pg70ActionServer::CRC16(uint16_t crc, uint16_t data)
{
  const uint16_t tbl[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
  };

  return(((crc & 0xFF00) >> 8)  ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)]);
}



uint8_t pg70ActionServer::getError(serial::Serial *port)
{
  ROS_INFO("Reading current module state...");
  std::vector<uint8_t> output;
  output.push_back(0x05);           //message from master to module
  output.push_back(gripper_id_);    //module id
  output.push_back(0x06);           //Data Length
  output.push_back(0x95);           //Command get state
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x01);           //get state only once
 
  //Checksum calculation
  uint16_t crc = 0;

  for(size_t i = 0; i < output.size(); i++)
    crc = CRC16(crc,output[i]);

  //Add checksum to the output buffer
  output.push_back(crc & 0x00ff);
  output.push_back((crc & 0xff00) >> 8);

  //Send message to the module and wait for response
  port->write(output);
  ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();

  //Read response
  std::vector<uint8_t> input;
  port->read(input, INPUT_BUFFER_SIZE);

  //Process current state message
  int get_state_index = 0;
  if(input.size() > 0)
  {
    if(input.size() > 7)
    {
      for(size_t i = 0; i < input.size(); i++)
        if ((input[i] == 0x07) && (input[i+1] == 0x95)) get_state_index = i;

      int pg70_error_ = input[get_state_index + 7];

      switch (pg70_error_)
      {
        case 0x00: ROS_INFO("PG70: No error detected"); break;
        case 0xC8: ROS_ERROR("PG70: Error: 0xC8 detected: Wrong ramp type"); break;
        case 0xD2: ROS_ERROR("PG70: Error: 0xD2 detected: Config memory"); break;
        case 0xD3: ROS_ERROR("PG70: Error: 0xD3 detected: Program memory"); break;
        case 0xD4: ROS_ERROR("PG70: Error: 0xD4 detected: Invalid phrase"); break;
        case 0xD5: ROS_ERROR("PG70: Error: 0xD5 detected: Soft low"); break;
        case 0xD6: ROS_ERROR("PG70: Error: 0xD6 detected: Soft high"); break;
        case 0xD7: ROS_ERROR("PG70: Error: 0xD7 detected: Pressure"); break;
        case 0xD8: ROS_ERROR("PG70: Error: 0xD8 detected: Service required"); break;
        case 0xD9: ROS_ERROR("PG70: Error: 0xD9 detected: Emergency stop"); break;
        case 0xDA: ROS_ERROR("PG70: Error: 0xDA detected: Tow"); break;
        case 0xE4: ROS_ERROR("PG70: Error: 0xE4 detected: Too fast"); break;
        case 0xEC: ROS_ERROR("PG70: Error: 0xEC detected: Math error"); break;
        case 0xDB: ROS_ERROR("PG70: Error: 0xDB detected: VPC3"); break;
        case 0xDC: ROS_ERROR("PG70: Error: 0xDC detected: Fragmentation"); break;
        case 0xDE: ROS_ERROR("PG70: Error: 0xDE detected: Current"); break;
        case 0xDF: ROS_ERROR("PG70: Error: 0xDF detected: I2T"); break;
        case 0xE0: ROS_ERROR("PG70: Error: 0xE0 detected: Initialize"); break;
        case 0xE1: ROS_ERROR("PG70: Error: 0xE1 detected: Internal"); break;
        case 0xE2: ROS_ERROR("PG70: Error: 0xE2 detected: Hard low"); break;
        case 0xE3: ROS_ERROR("PG70: Error: 0xE3 detected: Hard high"); break;
        case 0x70: ROS_ERROR("PG70: Error: 0x70 detected: Temp low"); break;
        case 0x71: ROS_ERROR("PG70: Error: 0x71 detected: Temp high"); break;
        case 0x72: ROS_ERROR("PG70: Error: 0x72 detected: Logic low"); break;
        case 0x73: ROS_ERROR("PG70: Error: 0x73 detected: Logic high"); break;
        case 0x74: ROS_ERROR("PG70: Error: 0x74 detected: Motor voltage low"); break;
        case 0x75: ROS_ERROR("PG70: Error: 0x75 detected: Motor voltage high"); break;
        case 0x76: ROS_ERROR("PG70: Error: 0x76 detected: Cable break"); break;
        case 0x78: ROS_ERROR("PG70: Error: 0x78 detected: Motor temp"); break;
      }
      //Re-start periodic position reading 
      getPeriodicPositionUpdate(com_port_, TF_UPDATE_PERIOD);
      return((uint8_t)pg70_error_);
    }
    else
    {
      //Re-start periodic position reading 
      getPeriodicPositionUpdate(com_port_, TF_UPDATE_PERIOD);
      return(0xff) ;
    }
  }  
}




void pg70ActionServer::execute_cb(const schunk_pg70::GraspGoalConstPtr& goal)
{
   bool success;
   float position;

   new_command_.request.goal_position = goal->position;
   new_command_.request.goal_velocity = goal->velocity;
   new_command_.request.goal_acceleration = goal->acceleration;
   new_command_.request.goal_current = goal->current;

   set_pvac.call(new_command_);

   if(new_command_.response.goal_accepted = false)
   {   
      ROS_ERROR("Goal not accepted");
      pg70_as_result_.success = false;
      pg70_as_.setSucceeded(pg70_as_result_);

      ROS_INFO("Grasp completed with output: %d ", success);

      return;
   }

   std::cout << goal->max_time << "\n";
   ros::Time start_time = ros::Time::now();
   // Controllo che sia stato compeltato
   std::cout << start_time << "\n";

  // ros::Duration counter = start_time;

   while(ros::ok())
   {
      std::cout << (ros::Time::now() - start_time).toSec() << "\n";

      if (goal->max_time < ((ros::Time::now() - start_time).toSec()))
      {
         ROS_WARN("Timer expired");
         pg70_as_result_.success = false;
         break;
      }   
         
      
      if( (act_position_ - goal->position) < 1) 
      {
         ROS_INFO("Goal reached succesfully");
         pg70_as_result_.success = success;
         break;
      }
   }


   pg70_as_.setSucceeded(pg70_as_result_);

   ROS_INFO("Grasp completed with output: %d ", success);
   return;
   //position = (joint_state_.position[0] + joint_state_.position[1])*1000;

/*
   ROS_INFO_STREAM("PG70: Moving from: " << act_position_ << " [mm] to " << goal->position << " [mm]");
   ROS_INFO("PG70: Data used: %f [mm/s], %f [mm/s2], %f [A]", goal->velocity, goal->acceleration, goal->current);
   

   std::vector<uint8_t> output;
   output.push_back(0x05);                //message from master to module
   output.push_back(gripper_id_);          //module id
   output.push_back(0x11);                //11-Len
   output.push_back(0xB0);                //Command "mov pos"

   //Position <0-69>mm
   unsigned int IEEE754_bytes[4];
   float_to_IEEE_754(goal->position,IEEE754_bytes);

   output.push_back(IEEE754_bytes[0]);    //Position first byte
   output.push_back(IEEE754_bytes[1]);    //Position second byte
   output.push_back(IEEE754_bytes[2]);    //Position third byte
   output.push_back(IEEE754_bytes[3]);    //Position fourth byte

   //Velocity<0-82>mm/s
   float_to_IEEE_754(goal->velocity, IEEE754_bytes);
   output.push_back(IEEE754_bytes[0]);    //Velocity first byte
   output.push_back(IEEE754_bytes[1]);    //Velocity second byte
   output.push_back(IEEE754_bytes[2]);    //Velocity third byte
   output.push_back(IEEE754_bytes[3]);    //Velocity fourth byte

   //Acceleration<0-320>mm/s2
   float_to_IEEE_754(goal->acceleration, IEEE754_bytes);
   output.push_back(IEEE754_bytes[0]);    //Acceleration first byte
   output.push_back(IEEE754_bytes[1]);    //Acceleration second byte
   output.push_back(IEEE754_bytes[2]);    //Acceleration third byte
   output.push_back(IEEE754_bytes[3]);    //Acceleration fourth byte

   //Current<0-2999>mA
   
   float current_f = (float)goal->current / 1000.0;
   std::cout << "Current" << goal->current << "\n";

   float_to_IEEE_754(current_f, IEEE754_bytes);

   output.push_back(IEEE754_bytes[0]);    //Current first byte
   output.push_back(IEEE754_bytes[1]);    //Current second byte
   output.push_back(IEEE754_bytes[2]);    //Current third byte
   output.push_back(IEEE754_bytes[3]);    //Current fourth byte

   //Checksum calculation
   uint16_t crc = 0;

   for(size_t i = 0; i < output.size(); i++)
      crc = CRC16(crc,output[i]);

   //Add checksum to the output buffer
   output.push_back(crc & 0x00ff);
   output.push_back((crc & 0xff00) >> 8);

   //Send message to the module
   port->write(output);




   ros::Time start_time = ros::Time::now();
   // Controllo che sia stato compeltato

  // ros::Duration counter = start_time;

   while(ros::ok())
   {
      if (goal->max_time < ((ros::Time::now() - start_time).toSec()))
      {
         ROS_WARN("Timer expired");
         pg70_as_result_.success = false;
         break;
      }   
         
      
      if( (act_position_ - goal->position) < 1) 
      {
         ROS_INFO("Goal reached succesfully");
         pg70_as_result_.success = success;
         break;
      }
   }
   */

   pg70_as_.setSucceeded(pg70_as_result_);

   ROS_INFO("Grasp completed with output: %d ", success);
   return;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pg70_action_server");
   
   pg70ActionServer server(ros::this_node::getName());
   
   ros::spin();

   return 0;
}



