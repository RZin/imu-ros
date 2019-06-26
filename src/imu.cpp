// This program publishes imu data from usb port
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>        // sensor_msgs::Imu
#include <string>                   // std::string std::stod
#include <iostream>                 // std::stringstream std::cout
#include <serial/serial.h>          // serial::Serial serial::Timeout::simpleTimeout
#include <vector>                   // std::vector
#include <sstream>                  // std::stringstream
#include <exception>                // std::exception

// constants
std::string TOPIC = "imu";
std::string PORT = "/dev/ttyACM0";  // port name
const int BAUD = 115200;            // for incoming data
int LINE_LENGHT = 65;               // lengh of string line coming from imu
int RATE = 10000;                   // frequency to publish at
int TIMOUT = 10;                    // delay in ms

std::vector<std::string> split(std::string string_line, char delimeter) {
    // splits line into a vector of str values
    std::stringstream ss(string_line);
    std::string item;
    std::vector<std::string> splitted;
    while (std::getline(ss, item, delimeter))
        splitted.push_back(item);
    return splitted;
}

std::vector<double> strings_to_doubles_vector(std::vector<std::string> strings_vector) {
    // converts  vector of strings to vector of doubles
    std::vector<double> doubles_vector;
    double imu_val;
    std::string::size_type sz; // size_type

    for (int i = 0; i < strings_vector.size(); i++) {
        try {
            imu_val = std::stod(strings_vector[i], &sz); }
        catch(std::exception& ia) {
            imu_val = 0.0;} // todo what value should be used if "-" or "@" or " " is recieved from imu ?
        doubles_vector.push_back(imu_val);
    }
    return doubles_vector;
}

void publish_imu(ros::Publisher imu_pub, std::vector<double> imu_vector) {
    // publish_imu data [a in m/s^2] and [w in rad/s]
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = TOPIC;
    imu_msg.header.stamp = ros::Time::now();
    // linear_acceleration
    imu_msg.linear_acceleration.x = imu_vector[1];
    imu_msg.linear_acceleration.y = imu_vector[2];
    imu_msg.linear_acceleration.z = imu_vector[3];
    // angular_velocity
    imu_msg.angular_velocity.x = imu_vector[4];
    imu_msg.angular_velocity.y = imu_vector[5];
    imu_msg.angular_velocity.z = imu_vector[6];
    // Publish the message.
    imu_pub.publish(imu_msg);
}


int main(int argc, char **argv) {

    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "IMU");
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(TOPIC, 10000);

    // open port, baudrate, timeout in milliseconds
    serial::Serial my_serial(PORT, BAUD, serial::Timeout::simpleTimeout(TIMOUT));

    // check if serial port open
    std::cout << "Is the serial port open?";
    if(my_serial.isOpen())
        std::cout << " Yes." << "\n";
    else
        std::cout << " No." << "\n";

    // define variables
    const char delimeter = ',';
    std::string imu_data_line;
    std::vector<std::string> imu_strings_vector;
    std::vector<double> imu_doubles_vector;

    // Loop at RATE Hz until the node is shut down.
    ros::Rate rate(RATE);
    while(ros::ok()) {
        // read
        std::string imu_data_line = my_serial.read(LINE_LENGHT);
        // split
        imu_strings_vector = split(imu_data_line, delimeter);
        // convert
        imu_doubles_vector = strings_to_doubles_vector(imu_strings_vector);
        // publish
        publish_imu(imu_pub, imu_doubles_vector);
        // Wait until it's time for another iteration.
        rate.sleep();
    }
}
