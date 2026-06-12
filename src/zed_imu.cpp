///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// ----> Includes
#include <zed_lib/sensorcapture.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <unistd.h> // for usleep
#include <iostream>
#include <iomanip>
// <---- Includes

void PublishRawIMU(sl_oc::sensors::SensorCapture* sens_, ros::Publisher& imu_pub_)
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(5000);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields (NED frame)
    imu_msg.linear_acceleration.x = -imu_data.aZ;
    imu_msg.linear_acceleration.y =  imu_data.aY;
    imu_msg.linear_acceleration.z =  imu_data.aX;

    imu_msg.angular_velocity.x = -imu_data.gZ * M_PI / 180;
    imu_msg.angular_velocity.y =  imu_data.gY * M_PI / 180;
    imu_msg.angular_velocity.z =  imu_data.gX * M_PI / 180;

    // Publish the sensor_msgs/Imu message
    imu_pub_.publish(imu_msg);
  }
}


// The main function
int main(int argc, char *argv[]){
    // ----> Silence unused warning
    nice(19);
    
    (void)argc;
    (void)argv;
    // <---- Silence unused warning
    //ROS publisher
    ros::init(argc, argv, "imu_publisher");

    ros::NodeHandle n;
    ros::Publisher imuData_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
    // Create a SensorCapture object
    sl_oc::sensors::SensorCapture sens;

    // ----> Get a list of available camera with sensor
    std::vector<int> devs = sens.getDeviceList();

    if( devs.size()==0 )
    {
        ROS_INFO("No available ZED Mini or ZED2 cameras");
        return EXIT_FAILURE;
    }
    // <---- Get a list of available camera with sensor

    // ----> Inizialize the sensors
    if( !sens.initializeSensors( devs[0] ) )
    {
        ROS_INFO("Connection failed");
        return EXIT_FAILURE;
    }

    std::string frameId = std::to_string(sens.getSerialNumber());

    // std::cout << "Sensor Capture connected to camera sn: " << sens.getSerialNumber() << std::endl;
    // <---- Inizialize the sensors

    // ----> Get FW version information
    uint16_t fw_maior;
    uint16_t fw_minor;

    sens.getFirmwareVersion( fw_maior, fw_minor );

    while(ros::ok()) {   
        PublishRawIMU(&sens, imuData_pub);
        ros::spinOnce(); 
    }

    return EXIT_SUCCESS;
}
