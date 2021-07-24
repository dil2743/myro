#include "ros/ros.h"
#include "myro/sensorData.h"
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <iostream>


std::string FILENAME = "/home/meditab/catkin_ws/src/myro/src/sensorData.csv"; // run the file from /myro/src folder otherwise there will be path error

std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<float>> pairs where
    // each pair represents <column name, column values>
    // Create a vector of <string, float vector> pairs to store the result
    // ROS_INFO("%s",filename.c_str());
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, float vector> pairs to result
            result.push_back({colname, std::vector<float> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        float colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "readNode");

  ros::NodeHandle n;

  ros::Publisher senorData_pub = n.advertise<myro::sensorData>("sensorMsgs", 1000,false);

  ros::Rate loop_rate(10);

  std::vector<std::pair<std::string, std::vector<float>>> dummyData = read_csv(FILENAME);
  int row_counter = 0 ;
  while (0 == senorData_pub.getNumSubscribers()) {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(1).sleep();
  }
  while (ros::ok())
  {
    myro::sensorData msg;

    std::stringstream ss;

    msg.distance_sensor1 = dummyData.at(0).second.at(row_counter);
    msg.distance_sensor2 = dummyData.at(1).second.at(row_counter);
    row_counter++;
    if( row_counter > (dummyData.at(0).second.size())) {
     //end of file exit
     ROS_WARN("END OF FILE");
     ros::shutdown();
     exit(0);
    }
    ROS_INFO("Sensor1 : %f and  Sensor 2: %f", msg.distance_sensor1 ,msg.distance_sensor2);
    
    senorData_pub.publish(msg);
     
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}