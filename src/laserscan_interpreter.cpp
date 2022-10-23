#include "LaserScanInterpreter.h"

using std::placeholders::_1;

//using namespace find_moving_objects;

/* CONFIDENCE CALCULATION FOR BANK */
double a_factor = -20 / 3;
double root_1=0.35, root_2=0.65; // optimized for bank coverage of 0.5s, adapted in hz calculation
double width_factor = 0.0;

//TODO: better put it in bank.cpp
double Bank::calculateConfidence(const moving_objects::msg::MovingObject & mo,
                                const BankArgument & ba,
                                const double dt,
                                const double mo_old_width)
{
return /*ba.ema_alpha **/ // Using weighting decay decreases the confidence while,
        (ba.base_confidence // how much we trust the sensor itself,
        + a_factor * (dt-root_1) * (dt-root_2) // a well-adapted bank size in relation to the sensor rate and environmental context
        - width_factor * fabs(mo.seen_width - mo_old_width)); // and low difference in width between old and new object,
        // make us more confident
}

//CONSTRUCTOR
LaserScanInterpreterNode::LaserScanInterpreterNode():Node("laser_scan_interpreter_node"){
    this->received_messages =0;
    this->optimize_nr_scans_in_bank = 0;
    this->subscribe_topic = "/scanner/scan";
    this->subscribe_buffer_size = 1;
    this->optimize_nr_scans_in_bank = 0.3;
    this->max_confidence_for_dt_match = 0.5;

    // Delta width confidence factor
    this->width_factor = 0.5;

    // Publisher for found moving objects
    // placed here because it needs Node object
    pub_objects = this->create_publisher<moving_objects::msg::MovingObjectArray>("moving_objects", 10);


    // Added
    gps_data_subscription_ = this->create_subscription<moving_objects::msg::Gpsx>("/gpsx", 10, std::bind(&LaserScanInterpreterNode::gps_data_callback, this, _1));
    imu_data_subscription_ = this->create_subscription<moving_objects::msg::Minimu9AHRS>("/minimu", 10, std::bind(&LaserScanInterpreterNode::imu_data_callback, this, _1));
    laserscan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scanner/scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanInterpreterNode::laserScanCallback, this, _1));
    
    onInit();

}


LaserScanInterpreterNode::~LaserScanInterpreterNode(){
    int nr_banks = banks.size();
    for (int i=0; i<nr_banks; ++i)
    {
        delete banks[i];
    }
    banks.clear();
}

    

void LaserScanInterpreterNode::onInit()
{    
    BankArgument bank_argument;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "onInit");


    // If optimize_nr_scans_in_bank != 0, then yes
    if (this->optimize_nr_scans_in_bank != 0.0)
    {
        state = WAIT_FOR_FIRST_MESSAGE_HZ;
    }
    else
    {
        state = INIT_BANKS;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), " onInit state = INIT_BANKS");
    }
    
    // Add this as the first bank_argument
    bank_arguments.push_back(bank_argument);
    
    

}   


void LaserScanInterpreterNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "LaserScanInterpreterNode::laserScanCallback");

    switch(state)
    {   
        /* 
        * MAIN STATE - WHEN ALL IS INITIALIZED
        */
        case FIND_MOVING_OBJECTS:
        {
            // Can message be added to bank?
            if (banks[0]->addMessage(&(*msg)) != 0) // De-reference ConstPtr object and take reference of result to get a 
                                                    // pointer to a PointCloud2 object
            {
                // Adding message failed (should never happen for LaserScan)
                break;
            }

            // If so, then find and report objects
            banks[0]->findAndReportMovingObjects();

            break;
        }

        /* 
        * BEFORE MAIN STATE CAN BE SET, THIS CASE MUST HAVE BEEN EXECUTED
        */
        case INIT_BANKS:
        {
            // Create banks
            if (banks.size() == 0)
            {
                banks.resize(1);
                banks[0] = new Bank(*this);
            }
            
            // Init bank
            if (banks[0]->init(bank_arguments[0], &(*msg)) != 0)
            {
                // If init fails (should never happen for LaserScan) we do not change state, but use this one again
                break;
            }

            // Change state
            state = FIND_MOVING_OBJECTS;
            break;

        }
        /* 
            * CALCULATE HZ OF TOPIC AND UPDATE SIZE OF BANK
        */
        case CALCULATE_HZ:
        {
            // spin until target is reached
            received_messages++;
            
            //TODO: !!!!!!!!!!!!!!!!!!!!!!!!!!
            double now = this->now().seconds();

            

            //const double elapsed_time = now - start_time;
            const double elapsed_time = now - start_time;
            //const double elapsed_time = rclcpp::Clock(RCL_ROS_TIME).now().toSec() - start_time;
            
            if (max_time <= elapsed_time ||
                    max_messages <= received_messages)
                {
                    // Calculate HZ
                    const double hz = received_messages / elapsed_time;
                    
                    // Set nr of messages in bank
                    const double nr_scans = optimize_nr_scans_in_bank * hz;
                    bank_arguments[0].nr_scans_in_bank = nr_scans - ((long) nr_scans) == 0.0 ? nr_scans + 1 : ceil(nr_scans);
                
                    // Sanity check
                    if (bank_arguments[0].nr_scans_in_bank < 2)
                    {
                    bank_arguments[0].nr_scans_in_bank = 2;
                    }

                    // Update confidence roots and amplitude factor
                    root_1 = optimize_nr_scans_in_bank * 0.6;
                    root_2 = optimize_nr_scans_in_bank * 1.4;
                    a_factor = 4 * max_confidence_for_dt_match / (2*root_1*root_2 - root_1*root_1 - root_2*root_2);
                
            
                
                    // Change state since we are done
                    state = INIT_BANKS;
                }
                break;
        }

        /* 
        * WHEN CALCULATING HZ OF TOPIC, WAIT FOR A MESSAGE TO ARRIVE AND SAVE THE TIME
        */
        case WAIT_FOR_FIRST_MESSAGE_HZ:
        {
        double start_time = this->now().seconds();
        
        // Change state
        state = CALCULATE_HZ;
        break;
        } 

        /* 
        * THERE ARE NO MORE STATES - TERMINATE, THIS IS AN ERROR
        */
        default:
        {
        }
    }
}

// Added
void LaserScanInterpreterNode::gps_data_callback(const moving_objects::msg::Gpsx::SharedPtr msg){
    latitude = msg->latitude;
    longitude = msg->longitude;
}

// Added
void LaserScanInterpreterNode::imu_data_callback(const moving_objects::msg::Minimu9AHRS::SharedPtr msg){
    compass_heading = msg->compassheading;
}




int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanInterpreterNode>());
    rclcpp::shutdown();

    return 0;
}