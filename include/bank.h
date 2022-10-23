#ifndef BANK_H
#define BANK_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include "moving_objects/msg/moving_object.hpp"
#include "moving_objects/msg/moving_object_array.hpp"
#include "LaserScanInterpreter.h"



class LaserScanInterpreterNode;

class BankArgument
{
    public:
        /** General (all apply to LaserScan) */
        double ema_alpha; 
        /**< The EMA weighting decrease coefficient---a value in <code>[0,1]</code>.
         * Initialized to 1.0 (i.e. no EMA). */
        
        int nr_scans_in_bank; 
        /**< The number of scan messages stored in the bank.
         * Initialized to 11. */
        
        int points_per_scan;  
        /**< The number of points per scan message. 
         *   For <code>sensor_msgs::LaserScan</code>, <code>ranges.size()</code> is used and cannot be changed; for 
         *   <code>sensor_msgs::PointCloud2</code>, a custom number, defining the resolution of the bank, should be specified.
         *   Initialized to 360. */
        
        double angle_min; 
        /**< The smallest angle (in radians) defined by the scan points in the bank. 
         * For <code>sensor_msgs::LaserScan</code>, <code>angle_min</code> is used. 
         * For <code>sensor_msgs::PointCloud2</code>, this should be specified 
         * (if <code>angle_max-angle_min</code> is smaller than the view angle of the sensor, 
         *  then the end points in the bank will be the smallest range of all points lying outside the view; 
         *  if larger, then there will be "empty" ranges in the beginning and end of the bank). 
         *  Initialized to -PI degrees*/
        
        double angle_max; 
        /**< The largest angle (in radians) defined by the scan points in the bank. 
         * For <code>sensor_msgs::LaserScan</code>, <code>angle_max</code> is used. 
         * For <code>sensor_msgs::PointCloud2</code>, this should be specified 
         * (if <code>angle_max-angle_min</code> is smaller than the view angle of the sensor, 
         *  then the end points in the bank will be the smallest range of all points lying outside the view; 
         *  if larger, then there will be "empty" ranges in the beginning and end of the bank).
         * Initialized to PI. */
        
        //TODO: den hreiazetai nomizw
        bool sensor_frame_has_z_axis_forward;
        /**< Set this to <code>true</code> in case the delivered data is given in a camera/optical frame with the Z-axis 
         * pointing forward, instead of the X-axis (and the X-axis pointing right and the Y-axis pointing down).
         * Note that setting this option to <code>true</code> causes the <code>ema</code> and 
         * <code>objects_closest_point_markers</code> to be shown incorrectly since they are in fact 
         * <code>sensor_msgs::LaserScan</code> messages.
         * The <code>velocity_arrows</code> and <code>delta_position_lines</code> are still shown correctly, though.
         * Initialized to <code>false</code>. */ 
        
        double object_threshold_edge_max_delta_range; 
        /**< The maximum difference in range (meters) between two consecutive scan points belonging to the same object. 
         * Initialized to 0.15. */
        
        int object_threshold_min_nr_points; 
        /**< The minimum number of consecutive scan points defining an object.
         * Initialized to 5. */
        
        double object_threshold_max_distance; 
        /**< The maximum distance in meters from the sensor to the object, in order to report it.
         * Initialized to 6.5. */
        
        double object_threshold_min_speed; 
        /**< The minimum speed in meters per second an object must have, in order to report it.
         * Initialized to 0.03. */
        
        int object_threshold_max_delta_width_in_points; 
        /**< The maximum difference in width (in scan points) an object is allowed to have, 
         * between the oldest and newest scans in the bank, in order to report it.
         * Initialized to 5. */
        
        double object_threshold_min_confidence;
        /**< The minimum confidence an object must have, in order to report it.
         * Initialized to 0.67. */
        
        double object_threshold_bank_tracking_max_delta_distance; 
        /**< Maximum distance an object is allowed to move in meters between two consecutive scans
         * to continue tracking it through the bank.
         * Initialized to 0.2. */
        
        double base_confidence; 
        /**< The base confidence of the system/sensor. 
         * Initialized to 0.3. */
        
        bool publish_objects; 
        /**< Whether to publish <code>find_moving_objects::MovingObjectArray</code> messages, 
         * containing the found objects.
         * Initialized to <code>true</code>. */
        
        bool publish_ema; 
        /**< Whether to publish <code>sensor_msgs::LaserScan</code> messages showing (using intensities) which scan points 
         * define objects. 
         * Initialized to <code>false</code>. */
        
        bool publish_objects_closest_point_markers; 
        /**< Whether to publish the point on each found object closest to the sensor, 
         * using <code>sensor_msgs::LaserScan</code> messages. 
         * Initialized to <code>false</code>. */
        
        bool publish_objects_velocity_arrows; 
        /**< Whether to publish arrows, using <code>visualization_msgs::MarkerArray</code> messages, 
         * showing the position and velocity of each found object.
         * Initialized to <code>false</code>. */
        
        bool publish_objects_delta_position_lines; 
        /**< Whether to publish lines, using <code>visualization_msgs::MarkerArray</code> messages, 
         * showing the change in position between the oldest and newest scans in the bank for each found object.
         * Initialized to <code>false</code>.  */
        
        bool publish_objects_width_lines; 
        /**< Whether to publish lines, using <code>visualization_msgs::MarkerArray</code> messages, 
         * showing the width for each found object.
         * Initialized to <code>false</code>.  */
        
        bool velocity_arrows_use_full_gray_scale; 
        /**< Whether to color the arrows using the full gray scale 
         * ([0,1];  0=low,  1=high confidence), 
         * or to use the grayness [<code>object_threshold_min_confidence</code>,1]. 
         * Initialized to <code>false</code>. */
        
        bool velocity_arrows_use_sensor_frame; 
        /**< Show arrows in sensor frame 
         * (if several frame options are true, then sensor, base, fixed, map (default) is the precedence order). 
         * Initialized to <code>false</code>. */
        
        bool velocity_arrows_use_base_frame; 
        /**< Show arrows in base frame 
         * (if several frame options are true, then sensor, base, fixed, map (default) is the precedence order). 
         * Initialized to <code>false</code>. */
        
        bool velocity_arrows_use_fixed_frame; 
        /**< Show arrows in fixed frame 
         * (if several frame options are true, then sensor, base, fixed, map (default) is the precedence order). 
         * Initialized to <code>false</code>. */
        
        std::string velocity_arrow_ns; 
        /**< Namespace of the velocity arrows. 
         * Initialized to <code>"velocity_arrow_ns"</code>. */
        
        std::string delta_position_line_ns; 
        /**< Namespace of the delta position lines. 
         * Initialized to <code>"delta_position_line_ns"</code>. */
        
        std::string width_line_ns; 
        /**< Namespace of the width lines. 
         * Initialized to <code>"width_line_ns"</code>. */
        
        std::string topic_objects; 
        /**< The topic on which to publish <code>find_moving_objects::MovingObjectArray</code> messages. 
         * Initialized to <code>"/moving_objects_arrays"</code>. */
        
        std::string topic_ema;
        /**< The topic on which to publish the messages showing which scan points define objects.
         * Initialized to <code>"/ema;"</code>. */
        
        std::string topic_objects_closest_point_markers; 
        /**< The topic on which to publish the messages showing the point on each found object closest to the sensor.
         * Initialized to <code>"/objects_closest_point_markers"</code>. */
        
        std::string topic_objects_velocity_arrows; 
        /**< The topic on which to publish the messages showing the position and velocity of each found object using arrows.
         * Initialized to <code>"/objects_velocity_arrows"</code>. */
        
        std::string topic_objects_delta_position_lines;
        /**< The topic on which to publish the messages showing the delta position of each found object using lines.
         * Initialized to <code>"/objects_delta_position_lines"</code>. */
        
        std::string topic_objects_width_lines;
        /**< The topic on which to publish the messages showing the width of each found object using lines.
         * Initialized to <code>"/objects_width_lines"</code>. */
        
        int publish_buffer_size; 
        /**< The size of each publish buffer. 
         * Initialized to 10. */
        
        std::string map_frame; 
        /**< The name of the map (i.e. the globally fixed) frame.
         * Initialized to <code>"map"</code>. */
        
        std::string fixed_frame; 
        /**< The name of the frame fixed for the robot but movable in the map frame.
         * Initialized to <code>"odom"</code>. */
        
        std::string base_frame;
        /**< The name of the frame fixed on the robot.
         * Initialized to <code>"base_link"</code>. */
        
        
        //   // TODO: dox
        //   float merge_threshold_max_angle_gap;
        //   float merge_threshold_max_end_points_distance_delta;
        //   float merge_threshold_max_velocity_direction_delta;
        //   float merge_threshold_max_speed_delta;
        
        
        std::string node_name_suffix;
        /**< Add a suffix to the reported node name in the <code>origin_node_name</code> field of the  
         * <code>MovingObjectArray</code> messages.
         * Initialized to the empty string <code>""</code>.
         */
        
        /**
         * Initializes the member variables to the stated values.
         */
        BankArgument();
        
        /**
         * @brief Allow Bank to access private members of this class.
         * @relates Bank
         */
        friend class Bank;
        
    private:
        friend std::ostream& operator<<(std::ostream& os, const BankArgument& ba);
        std::string sensor_frame; 
        /**< The name of the sensor frame. Set to the frame of the sensor. */
        
        float angle_increment; 
        /**< The angular difference between two consecutive scan points. 
         * For <code>sensor_msgs::LaserScan</code>, the corresponding value of the first message. 
         * For <code>sensor_msgs::PointCloud2</code>, this is calculated based on
         * <code>angle_max</code>, <code>angle_min</code> and <code>points_per_scan</code>. */
        
        float time_increment; 
        /**< The time difference between two consecutive scan points. 
        * For <code>sensor_msgs::LaserScan</code>, 
        * the corresponding value of the first message. 
        * For <code>sensor_msgs::PointCloud2</code>, this is set to 0. */
        
        float scan_time; 
        /**< The time needed for each complete scan. 
        * For <code>sensor_msgs::LaserScan</code>, the corresponding value of the first message. 
        * For <code>sensor_msgs::PointCloud2</code>, this is set to 0. */
        
        float range_min;
        /**< The minimum range the sensor can measure. 
        * For <code>sensor_msgs::LaserScan</code>, the corresponding value of the first message. 
        * For <code>sensor_msgs::PointCloud2</code>, this is set to 0.01. */
        
        float range_max; 
        /**< The maximum range the sensor can measure. 
        * For <code>sensor_msgs::LaserScan</code>, the corresponding value of the first message. 
        * For <code>sensor_msgs::PointCloud2</code>, this is set to <code>object_threshold_max_distance</code>. */
        
        bool sensor_is_360_degrees;
        /**< Whether the sensor is scanning 360 degrees.
        * This is determined based on the angle limits of the bank */
        
        void check(); 
        /**< Validate the specified values. For numeric values, this could include a range check. */
    
    
};


/**
 * The bank contains a number of scan messages 
 * and is able to find the position and velocity of moving objects based on these scans. 
 * 
 * Note that only ONE single source is supposed to feed the bank with messages 
 * and that the "static" parameters 
 * (such as the number of scan points and the view angle (cf. FoV) of the sensor etc) 
 * must be equal between messages. 
 * If they are not, then the behavior of the bank is undefined and the bank might even cause a segmentation fault.
 * 
 * Also note that you might need to sleep for a couple of seconds after creating the <code>Bank</code> object 
 * so that the started <code>tf::tfListener</code> can collect transformation data.
 */
class Bank
{
private:
    /* BANK ARGUMENTS */
    BankArgument bank_argument;
    
    /* BANK */
    float ** bank_ranges_ema;
    double * bank_stamp;
    unsigned int bank_ranges_bytes;
    bool bank_is_initialized;
    bool bank_is_filled;
    double resolution;
    
    /* INDICES FOR THE BANK */
    int bank_index_newest;
    int bank_index_put; // nr_scans_in_bank is/should be greater than 1!
    
    

    /* Basic functionality used by the functions below*/
    void initBank(BankArgument bank_argument);
    //   virtual long addFirstMessage(const sensor_msgs::LaserScan::ConstPtr &);
    //   virtual long addFirstMessage(const sensor_msgs::PointCloud2::ConstPtr &, 
    //                                const bool discard_message_if_no_points_added);
    virtual long addFirstMessage(const sensor_msgs::msg::LaserScan *);
    inline void initIndex();
    inline void advanceIndex();
    //   void mergeFoundObjects(MovingObjectArray * moa);
    
    /* 
    * Recursive tracking of an object through history to get the indices of its middle, 
    * left and right points in the oldest scans, along with the sum of all ranges etc.
    */
    void getOldIndices(const float range_min,
                        const float range_max,
                        const unsigned int object_width_in_points,
                        const int          current_level,
                        const unsigned int levels_searched,
                        const unsigned int index_mean,
                        const unsigned int consecutive_failures_to_find_object,
                        const unsigned int threshold_consecutive_failures_to_find_object,
                        int * index_min_old,
                        int * index_mean_old,
                        int * index_max_old,
                        float * range_sum_old,
                        float * range_at_min_index_old,
                        float * range_at_max_index_old);
        

    std::string getStringPutPoints();

    // Publisher for moving objects
    //rclcpp::Publisher<moving_objects::msg::MovingObjectArray>::SharedPtr pub_objects;

    // Added
    LaserScanInterpreterNode * node;
    struct global_position{
        double latitude, longitude;
    };
    // Added 
    global_position findLatitudeLongitudeOfObject(double bearing, double distance);
    double computeBearing(double angle_to_lidar_frame, double north_heading);
    
    
    

    
public:
    //   /**
    //    * Creates an instance of Bank and starts a <code>tf::TransformListener</code>.
    //    */
    //Bank(rclcpp::Publisher<moving_objects::msg::MovingObjectArray>::SharedPtr pub_objects);
    
    Bank(LaserScanInterpreterNode &node);
    /**
     * De-allocates reserved memory for the bank.
     */
    ~Bank();

    /**
     * Initiate bank with received data, if possible.
     * 
     * This function should be called repeatedly until it succeeds (i.e. returns 0), after this, 
     * it should not be called again (doing so would compromise the stored data)!
     * @param bank_argument An instance of <code>BankArgument</code>, specifying the behavior of the bank.
     * @param msg Pointer to the first received message to be added to the bank.
     * @return 0 on success, -1 if this function must be called again.
     */
    virtual long init(BankArgument bank_argument, const sensor_msgs::msg::LaserScan * msg);
    
    
    /**
     * Add a <code>sensor_msgs::LaserScan</code> message to the bank (replace the oldest scan message).
     * 
     * @param msg Pointer to the received message to be added to the bank.
     * @return 0.
     */
    virtual long addMessage(const sensor_msgs::msg::LaserScan * msg);
    
    
    
    
    /**
     * Find moving objects based on the contents of the bank, and possibly report them.
     */
    void findAndReportMovingObjects();
    
    /**
     * Confidence calculation.
     * 
     * <strong>This function must be implemented by the application using the bank.</strong>
     * @param mo The object for which to calculate confidence.
     * @param ba An instance of BankArgument with specified details.
     * @param delta_time The difference in time between the oldest and newest scans in the bank.
     * @param mo_old_width The old width of the object.
     * @return The user-defined confidence value.
     */
    virtual double calculateConfidence(const moving_objects::msg::MovingObject & mo,
                                        const BankArgument & ba,
                                        const double delta_time,
                                        const double mo_old_width);
};




#endif // BANK_H