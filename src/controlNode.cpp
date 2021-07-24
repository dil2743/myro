#include "ros/ros.h"
#include "myro/sensorData.h"
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair

#define CLOCKWISE_ROTATION          1
#define COUNTERCLOCKWISE_ROTATION   0
#define SEPRATION_BETWEEN_SENSOR    30
#define MOTOR_PPR                   2000
#define ROBOT_WHEEL_DIAMETER        5
#define DISTANCE_FROM_CG_TO_WHEEL   15
#define PAEALLEL_DISTANCE_FROM_WALL 30

std::string OUTPUT_FILE="/home/meditab/catkin_ws/src/myro/data/output.csv";

const float distance_coverd_in_360_degree = 2 * M_PI * (DISTANCE_FROM_CG_TO_WHEEL) ;
const float distance_covered_by_wheel_in_1_rotation = 2 * M_PI * ROBOT_WHEEL_DIAMETER/2 ;
const float pulse_required_for_1_cm_rotion =  MOTOR_PPR / distance_covered_by_wheel_in_1_rotation ;
const float distance_covered_in_one_degree_rotation = distance_coverd_in_360_degree / 360 ;
const float distance_covered_with_one_pulse = distance_covered_by_wheel_in_1_rotation / MOTOR_PPR ;
const float pulse_required_per_degree = distance_covered_in_one_degree_rotation / distance_covered_with_one_pulse ;


void write_csv(std::string filename,  std::vector<std::string> dataset){
    // Create an output filestream object
    std::ofstream myFile(filename,std::ofstream::out | std::ofstream::app);
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j);
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    myFile.close();
}

void senorDataCallback(const myro::sensorData::ConstPtr& msg)
{
    static int itteration = 0 ;
    float sensorOneDistance = msg->distance_sensor1;
    float sensorTwoDistance = msg->distance_sensor2;

    /* ****************************Question 1: Angle calculation *******************************
    /*
    *   let us assme that we have positive and negative angles 
    *   if angle is positive then we have to rotate in clockwise direction for reorenting the robot and 
    *   vice versa
    */

   float difference = sensorOneDistance - sensorTwoDistance ;
   bool rotation_direction = difference > 0 ? CLOCKWISE_ROTATION : COUNTERCLOCKWISE_ROTATION ;

   float angleInDegree = atan((difference)/SEPRATION_BETWEEN_SENSOR)  *(180/3.1415);

   std::string angle = std::to_string(abs(angleInDegree));


  ROS_INFO("S1:%f S2:%f Angle:%f", msg->distance_sensor1 ,
                                msg->distance_sensor2, angleInDegree);
    
  /* ******************************** Question 2: calculate the number of pulses to be given for M1, M2, M3 and M4
                        rtorotate the robot to maintain parallelity with the wall *************************************************************************/

  /*
  * Now as the robot is in square shape of 30*30cm
  * We can say that to make 360 degree rotation about its CG it will need to move 2*PI*(side/2)
  * so 2*PI*(30/2) = 3.14*30 =   94.2 cm
  * Now the robot wheel diameter is 5 cm 
  * distance covered by wheel in each complet turn = 2 * 3.14 * 5/2 = 15.7cm
  * Also we can say that 94.2cm = 360 degreee -> 1 degree = 0.261666667 cm
  * From Robot model it is clear that to rotate the robot, all wheel must move in same direction with
  * same speed.
  * It is given that PPR is 2000 pulse i,e 15.7cm = 2000 pulse
  * In 1 pulse we can move by 15.7/2000 = 0.00785 cm 
  * So to rotate 1degree, pulse required is = distance in 1 degree / distance in one pulse 
  * = 0.261666667/0.00785 = 33.333333376 pulse.
  * 
  */
    float pulse_required = abs(angleInDegree) * pulse_required_per_degree ;

    std::string M1 = std::to_string(pulse_required);
    std::string M2 = std::to_string(pulse_required);
    std::string M3 = std::to_string(pulse_required);
    std::string M4 = std::to_string(pulse_required);
    std::string direction = angleInDegree > 0 ? "clockwise" :"counterclockwise" ;
        
    /* *************************** Question 3 : After making the robot parallel to the wall, find the number of pulses to be given for M1, M2, M3
    and M4 so that it will arrive at 30cms from the wall *********************************************/

    /*
    *   To find number to pulse required by robot to move 30cm away from wall (robot already alligned parallel with wall)
    *   According to given diagram (in question ) for robot to make a 30 cm sepration from wall 
    *   robot need to move only M3 and M4 (due to its omnidirectional configuration) in opposite direction
    *   Also the current distance of robot is same as the smallest among two sensors (as we are applying correction w.r.t it)
    *   let current distance = min(S1,S2);
    *   required_distance_to_move = abs(30 - current_distance)
    *    and direction = current_distance > 30 ? forward :backward
    *
    */

    float current_distance = std::min(msg->distance_sensor1,msg->distance_sensor2);
    float required_distance = (PAEALLEL_DISTANCE_FROM_WALL - current_distance) ;
    float require_pulse_on_motor = abs(required_distance) * pulse_required_for_1_cm_rotion ;

    std::string requierd_direction = required_distance < 0 ? "clockwise" : "counterclockwise" ;
    // here counterclockwise means Forward direction and clockwise means Backward direction (w.r.t whole robot)
    std::string dis_M1 = "0" ;
    std::string dis_M2 = "0" ;
    std::string dis_M3 = std::to_string(require_pulse_on_motor);
    std::string dis_M4 = std::to_string(require_pulse_on_motor);

    /* we can have represented it more clearly with sign on pulse as shown below but question doesn't need that formate 
        
        M3 = std::to_string(require_pulse_on_motor) ; // in clockwise direction
        M4 = std::to_string(-require_pulse_on_motor) ; // in counterclockwise direction

        M4 = std::to_string(require_pulse_on_motor) ; // in clockwise direction
        M3 = std::to_string(-require_pulse_on_motor) ; // in counterclockwise direction

    */
    std::vector <std::string> dataToStore = {angle , M1,M2,M3,M4,direction,dis_M1,dis_M2,dis_M3,dis_M4,requierd_direction };
    write_csv(OUTPUT_FILE, dataToStore );
}

int main(int argc, char **argv)
{

  ROS_WARN("Positive Angle = CLockWise drift");
  ROS_WARN("Negetive Angle = AntiCLockWise drift");

  ros::init(argc, argv, "controlNode");

  ros::NodeHandle n; 
  
  bool status = remove(OUTPUT_FILE.c_str()); // remove existing file for fresh run
  std::vector<std::string> output_header =  { "Angle","M1 Ticks","M2 Ticks","M3 Ticks","M4 Ticks","Direction","M1 Ticks","M2 Ticks","M3 Ticks","M4 Ticks","Direction"};
  write_csv(OUTPUT_FILE, output_header ); // store output header 

  ros::Subscriber sub = n.subscribe("sensorMsgs", 1000, senorDataCallback);

  ros::spin();

  return 0;
}