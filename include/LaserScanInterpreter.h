#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "bank.h"
#include "moving_objects/msg/moving_object.hpp"
#include "moving_objects/msg/moving_object_array.hpp"
#include "moving_objects/msg/gpsx.hpp"
#include "moving_objects/msg/minimu9_ahrs.hpp"

//#include "laserscan_interpreter_default_parameter_values.h"

#ifndef LASERSCAN_INTERPRETER_H
#define LASERSCAN_INTERPRETER_H

class Bank;
class BankArgument;


class LaserScanInterpreterNode: public rclcpp:: Node
{   
  private:
    friend class Bank;
      
    /* SUBSCRIBE INFO */
    std::string subscribe_topic;
    int subscribe_buffer_size;    
    
      /* HZ CALCULATION */
    double optimize_nr_scans_in_bank;
    double max_confidence_for_dt_match;
    int received_messages;
    const int max_messages = 100;
    double start_time;
    const double max_time = 1.5;
    double width_factor;



      /* BANK AND ARGUMENT */
    std::vector<Bank *> banks;
    std::vector<BankArgument> bank_arguments;

    // Publisher for found moving objects
    // placed here because it needs node object
    rclcpp::Publisher<moving_objects::msg::MovingObjectArray>::SharedPtr pub_objects;

    

    /* STATES OF MESSAGE RECEIVING */
    typedef enum
    {
        WAIT_FOR_FIRST_MESSAGE_HZ,
        CALCULATE_HZ,
        INIT_BANKS,
        FIND_MOVING_OBJECTS
    } state_t;
    state_t state;


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscription_;
    /* CALLBACK */                                            
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Added
    // Subscriber for the topic gps publishes to
    //TODO: change the msg to the package/type being used on boat
    rclcpp::Subscription<moving_objects::msg::Gpsx>::SharedPtr gps_data_subscription_;
    void gps_data_callback(const moving_objects::msg::Gpsx::SharedPtr msg);
    double latitude;
    double longitude;
    
    // Added
    // Subscriber for the topic imu publishes to
    //TODO: change the msg to the package/type being used on boat
    rclcpp::Subscription<moving_objects::msg::Minimu9AHRS>::SharedPtr imu_data_subscription_;
    void imu_data_callback(const moving_objects::msg::Minimu9AHRS::SharedPtr msg);
    double compass_heading; //TODO: if not in true north -> convert to true from magnetic


  public:
    LaserScanInterpreterNode();
    ~LaserScanInterpreterNode();


    void onInit();
};
        
#endif