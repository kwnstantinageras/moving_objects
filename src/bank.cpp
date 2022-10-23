
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


/* C/C++ */
#include <iostream> 
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>
// #include <pthread.h>

/* Local includes */
#include "bank.h"



const float TWO_PI = 2*M_PI;

/*
 * Constructors
 */
BankArgument::BankArgument()
{
  ema_alpha = 1.0;
  nr_scans_in_bank = 11;
  points_per_scan = 360;
  angle_min = -M_PI;
  angle_max = M_PI;
  sensor_frame_has_z_axis_forward = false;
  object_threshold_edge_max_delta_range = 0.15;
  object_threshold_min_nr_points = 5;
  object_threshold_max_distance = 6.5;
  object_threshold_min_speed = 0.03;
  object_threshold_max_delta_width_in_points = 5;
  object_threshold_min_confidence = 0.67;
  object_threshold_bank_tracking_max_delta_distance = 0.2;
  base_confidence = 0.3;
  publish_objects = true;
  publish_ema = true;
  publish_objects_closest_point_markers = false;
  publish_objects_velocity_arrows = false;
  publish_objects_delta_position_lines = false;
  publish_objects_width_lines = false;
  velocity_arrows_use_full_gray_scale = false;
  velocity_arrows_use_sensor_frame = false;
  velocity_arrows_use_base_frame = false;
  velocity_arrows_use_fixed_frame = false;
  velocity_arrow_ns = "velocity_arrow_ns";
  delta_position_line_ns = "delta_position_line_ns";
  width_line_ns = "width_line_ns";
  topic_objects = "moving_objects_arrays";
  topic_ema = "ema";
  topic_objects_closest_point_markers = "objects_closest_point_markers";
  topic_objects_velocity_arrows = "objects_velocity_arrows";
  topic_objects_delta_position_lines = "objects_delta_position_lines";
  topic_objects_width_lines = "objects_width_lines";
  publish_buffer_size = 10;
  map_frame = "map";
  fixed_frame = "odom";
  base_frame = "base_link";
  
  node_name_suffix = "";
}



//Bank::Bank(rclcpp::Publisher<moving_objects::msg::MovingObjectArray>::SharedPtr pub_objects)

Bank::Bank(LaserScanInterpreterNode &node)
{
  bank_is_initialized = false;
  bank_is_filled = false;
  
  this->node = &node;

  
}

/*
 * Destructor
 */
Bank::~Bank()
{
  for (int i=0; i<bank_argument.nr_scans_in_bank; ++i)
  {
    free(bank_ranges_ema[i]);
  }
  free(bank_ranges_ema);
  free(bank_stamp);
}


/*
 * Check values of bank arguments.
 */
void BankArgument::check()
{
  /* TODO: write the correct ros2 commands
  ROS_ASSERT_MSG(0.0 <= ema_alpha && ema_alpha <= 1.0,
                 "The EMA weighting decrease coefficient must be a value in [0,1]."); 
  
  ROS_ASSERT_MSG(2 <= nr_scans_in_bank, 
                 "There must be at least 2 messages in the bank. Otherwise, velocities cannot be calculated."); 
  
  ROS_ASSERT_MSG(0 < points_per_scan, 
                 "There must be at least 1 point per scan.");
  
  ROS_ASSERT_MSG(angle_max - angle_min <= TWO_PI, 
                 "Angle interval cannot be larger than 2*PI (360 degrees)."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_edge_max_delta_range, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(1 <= object_threshold_min_nr_points, 
                 "An object must consist of at least 1 point.");
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_max_distance, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_min_speed,
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_max_delta_width_in_points, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_min_confidence && object_threshold_min_confidence <= 1.0, 
                 "Cannot be negative or larger than 1.0."); 

//   ROS_ASSERT_MSG(sensor_frame_has_z_axis_forward);
//   ROS_ASSERT_MSG(base_confidence); 
//   ROS_ASSERT_MSG(publish_objects); 
//   ROS_ASSERT_MSG(publish_ema); 
//   ROS_ASSERT_MSG(publish_objects_closest_point_markers); 
//   ROS_ASSERT_MSG(publish_objects_velocity_arrows); 
//   ROS_ASSERT_MSG(velocity_arrows_use_full_gray_scale); 
//   ROS_ASSERT_MSG(velocity_arrows_use_sensor_frame);  
//   ROS_ASSERT_MSG(velocity_arrows_use_base_frame); 
//   ROS_ASSERT_MSG(velocity_arrows_use_fixed_frame); 
  
  ROS_ASSERT_MSG(!publish_objects_velocity_arrows || velocity_arrow_ns != "", 
                 "If publishing velocity arrows, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_delta_position_lines || delta_position_line_ns != "", 
                 "If publishing delta position lines, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_width_lines || width_line_ns != "", 
                 "If publishing width lines, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects || topic_objects != "", 
                 "If publishing MovingObjectArray messages, then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_ema || topic_ema != "", 
                 "If publishing object points via LaserScan visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_closest_point_markers || topic_objects_closest_point_markers != "", 
                 "If publishing the closest point of each object via LaserScan visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_velocity_arrows || topic_objects_velocity_arrows != "", 
                 "If publishing the velocity of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_delta_position_lines || topic_objects_delta_position_lines != "", 
                 "If publishing the delta position of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_width_lines || topic_objects_width_lines != "", 
                 "If publishing the width of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(1 <= publish_buffer_size, 
                 "Publish buffer size must be at least 1."); 
  
  ROS_ASSERT_MSG(map_frame != "", 
                 "Please specify map frame."); 
  
  ROS_ASSERT_MSG(fixed_frame != "", 
                 "Please specify fixed frame."); 
  
  ROS_ASSERT_MSG(base_frame != "", 
                 "Please specify base frame."); 
  
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_angle_gap && merge_threshold_max_angle_gap <= angle_max - angle_min,
//                  "Invalid gap angle.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_end_points_distance_delta,
//                  "Distance delta cannot be negative.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_velocity_direction_delta && merge_threshold_max_velocity_direction_delta <= M_PI,
//                  "The maximum angle between two vectors is always greater than 0 and smaller than PI.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_speed_delta,
//                  "Speed delta cannot be negative.");
*/
}



/*
 * Initialize bank based on information received from the user and sensor
 */
void Bank::initBank(BankArgument bank_argument)
{
  if (bank_is_initialized)
  {
    return;
  }
  
  bank_argument.check();
  
  bank_index_put = -1;
  bank_index_newest = -1;
  
  
  

  /* Init bank */
  this->bank_argument = bank_argument;
  this->bank_argument.sensor_is_360_degrees = fabsf(bank_argument.angle_max - bank_argument.angle_min - TWO_PI) <= 
                                              2.0 * bank_argument.angle_increment; // Safety margin
  
  bank_stamp = (double *) malloc(bank_argument.nr_scans_in_bank * sizeof(double));
  bank_ranges_ema = (float **) malloc(bank_argument.nr_scans_in_bank * sizeof(float*));
  //ROS_ASSERT_MSG(bank_ranges_ema != NULL, "Could not allocate buffer space for messages.");
  for (unsigned int i=0; i<bank_argument.nr_scans_in_bank; ++i)
  {
    bank_ranges_ema[i] = (float *) malloc(bank_argument.points_per_scan * sizeof(float));
    //ROS_ASSERT_MSG(bank_ranges_ema[i] != NULL, "Could not allocate buffer space message %d.", i);
  }
  
  // Nr scan points
  bank_ranges_bytes = sizeof(float) * bank_argument.points_per_scan;
  
  // Init sequence nr
  //moa_seq = 0;
  
  bank_is_initialized = true;
}


/* 
 * Recursive tracking of an object through history to get the indices of its middle, 
 * left and right points in the oldest scans, along with the sum of all ranges etc.
 */
void Bank::getOldIndices(const float range_min,
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
                         float * range_at_max_index_old)
{
  // Base case reached?
  if (levels_searched == bank_argument.nr_scans_in_bank)
  {
    return;
  }
  
  // To find the end indices of the object,
  int left = index_mean;
  float prev_range = bank_ranges_ema[current_level][index_mean];
  float range_sum = prev_range;
  int right_upper_limit_out_of_bounds = bank_argument.points_per_scan;
  unsigned int width_in_points = 1; // prev_rang = range at index_mean
  
  // Check range
  if (prev_range < range_min ||
      range_max < prev_range)
  {
    *index_min_old = -1;
    *index_mean_old = -1;
    *index_max_old = -1;
    *range_sum_old = 0;
    *range_at_min_index_old = 0;
    *range_at_max_index_old = 0;
    return;
  }
  
  // Search lower index side, with possible wrap around
  bool stopped = false;
  for (int i=index_mean-1; 0<=i; --i)
  {
    // Same type of range check as in the main code
    const float range = bank_ranges_ema[current_level][i];
    if (range_min <= range &&
        range <= range_max &&
        fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
    {
      left = i;
      prev_range = range;
      range_sum += range;
      width_in_points++;
    }
    else
    {
      stopped = true;
      break;
    }
  }
  if (!stopped)
  {
    // Continue from highest index
    for (int i=bank_argument.points_per_scan-1; index_mean<i; --i)
    {
      // Same type of range check as in the main code
      const float range = bank_ranges_ema[current_level][i];
      if (range_min <= range &&
          range <= range_max &&
          fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        left = i;
        prev_range = range;
        range_sum += range;
        right_upper_limit_out_of_bounds--;
        width_in_points++;
      }
      else
      {
        break;
      }
    }
  }
  // prev_range holds the range at left
  *range_at_min_index_old = prev_range;
  
  // Search higher index side
  stopped = false;
  int right = index_mean;
  prev_range = bank_ranges_ema[current_level][index_mean];
  for (int i=index_mean+1; i<right_upper_limit_out_of_bounds; ++i)
  {
    // Same type of range check as in the main code
    const float range = bank_ranges_ema[current_level][i];
    if (range_min <= range &&
        range <= range_max &&
        fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
    {
      right = i;
      prev_range = range;
      range_sum += range;
      width_in_points++;
    }
    else
    {
      stopped = true;
      break;
    }
  }
  // Here we must make sure that we have not already wrapped around while going left
  if (!stopped && right_upper_limit_out_of_bounds == bank_argument.points_per_scan)
  {
    // Continue from lowest index (0) - we did not wrap around while going left
    for (int i=0; i<left; ++i)
    {
      // Same type of range check as in the main code
      const float range = bank_ranges_ema[current_level][i];
      if (range_min <= range &&
          range <= range_max &&
          fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        right = i;
        prev_range = range;
        range_sum += range;
        width_in_points++;
      }
      else
      {
        break;
      }
    }
  }
  // prev_range holds the range at right
  *range_at_max_index_old = prev_range;
  
  // Did we find a valid object?
  unsigned int misses = consecutive_failures_to_find_object;
  if (width_in_points < bank_argument.object_threshold_min_nr_points  ||
      bank_argument.object_threshold_max_delta_width_in_points < abs(int(width_in_points - object_width_in_points))  ||
      bank_argument.object_threshold_bank_tracking_max_delta_distance  < 
        fabs(range_sum / width_in_points - *range_sum_old / object_width_in_points))
    // range_sum_old holds the range sum of the previous (newer) scanned object
  {
    // No
    misses++;
    if (threshold_consecutive_failures_to_find_object < misses)
    {
      // Return -1 to signal that no index_mean was found
      *index_min_old = -1;
      *index_mean_old = -1;
      *index_max_old = -1;
      *range_sum_old = 0;
      *range_at_min_index_old = 0;
      *range_at_max_index_old = 0;
      return;
    }
  }
  else
  {
    // Yes
    misses = 0;
  }
  
  // If reaching this point, a valid object was found
  // Update end points
  *index_min_old = left;
  *index_mean_old = (left + (width_in_points-1) / 2) % bank_argument.points_per_scan;
  *index_max_old = right;
  *range_sum_old = range_sum;
  
  // Continue searching based on the new index_mean
  getOldIndices(range_min,
                range_max,
                width_in_points,
                (current_level - 1) < 0 ? bank_argument.nr_scans_in_bank - 1 : current_level - 1, // wrap around
                levels_searched + 1,
                *index_mean_old,
                misses,
                threshold_consecutive_failures_to_find_object,
                index_min_old,
                index_mean_old,
                index_max_old,
                range_sum_old, // *range_sum_old was set to range_sum above
                range_at_min_index_old,
                range_at_max_index_old);
}


/*
 * Find and report moving objects based on the current content of the bank
 */
void Bank::findAndReportMovingObjects()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "findAndReportMovingObjects");

  // Is the bank filled with scans?
  if (!bank_is_filled)
  {
    //ROS_WARN("Bank is not filled yet-cannot report objects!");
    return;
  }
  
  // Moving object array message
  moving_objects::msg::MovingObjectArray moa;
  
  // Old positions of the objects in moa
  moving_objects::msg::MovingObjectArray moa_old_positions;
  
  /* Find objects in the new scans */
  unsigned int nr_objects_found = 0;
  unsigned int nr_object_points = 0;
  const float range_max = (bank_argument.range_max < bank_argument.object_threshold_max_distance  ?
                           bank_argument.range_max : bank_argument.object_threshold_max_distance);
  const float range_min = bank_argument.range_min;
  unsigned int i=0;
  
  // Stamps
  rclcpp::Time old_time = rclcpp::Time(bank_stamp[bank_index_put]);
  rclcpp::Time new_time = rclcpp::Time(bank_stamp[bank_index_newest]);
  
  // Handle 360 degrees sensors!
  unsigned int upper_limit_out_of_bounds_scan_point = bank_argument.points_per_scan;
  while(i<upper_limit_out_of_bounds_scan_point)
  {
    /* Find first valid scan from where we currently are */
    const float range_i = bank_ranges_ema[bank_index_newest][i];
    float object_range_sum = range_i;
    
    // Is i out-of-range?
    if (range_i < bank_argument.range_min ||
        bank_argument.range_max < range_i)
    {
      i++;
      continue;
    }
    
    // i is a valid scan
    nr_object_points = 1;
    float object_range_min = range_i;
    float object_range_max = range_i;
    unsigned int object_range_min_index = i;
    unsigned int object_range_max_index = i;
    
    // Count valid scans that are within the object threshold
    
    float range_at_angle_begin = range_i; // Might be updated later
    float range_at_angle_end;             // Updated later
    unsigned int index_at_angle_begin = i; // Might be updated later
    unsigned int index_at_angle_end;       // Updated later
    float prev_range = range_i;
    unsigned int j=i+1;
    for (; j<bank_argument.points_per_scan; ++j)
    {
      const float range_j = bank_ranges_ema[bank_index_newest][j];
      
      // Range check
      if (bank_argument.range_min <= range_j  &&
          range_j <= bank_argument.range_max  &&
          fabsf(prev_range - range_j) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        // j is part of the current object
        nr_object_points++;
        object_range_sum += range_j;
        
        // Update min and max ranges
        if (range_j < object_range_min) 
        {
          object_range_min = range_j;
          object_range_min_index = j;
        }
        else if (object_range_max < range_j) 
        {
          object_range_max = range_j;
          object_range_max_index = j;
        }
        prev_range = range_j;
      }
      else
      {
        // j is not part of this object
        break;
      }
    }
    
    // Update range at end
    range_at_angle_end = prev_range;
    index_at_angle_end = j-1; // j is not part of the object
    
    // If i is 0 and sensor is 360 deg, then we must also search higher end of scan points and account for these
    if (i == 0 && bank_argument.sensor_is_360_degrees)
    {
      // Start from i again
      prev_range = range_i;
      //Save prev_range so we can restore it?
      
      // Do not step all the way to j again - it has already been considered
      for (unsigned int k=bank_argument.points_per_scan-1; j<k; k--)
      {
        const float range_k = bank_ranges_ema[bank_index_newest][k];
        
        // Range check
        if (bank_argument.range_min <= range_k  &&
            range_k <= bank_argument.range_max  &&
            fabsf(prev_range - range_k) <= bank_argument.object_threshold_edge_max_delta_range)
        {
          // k is part of the current object
          nr_object_points++;
          object_range_sum += range_k;
          
          // Adapt the loop upper limit
          upper_limit_out_of_bounds_scan_point--;
          
          // Update min and max ranges
          if (range_k < object_range_min) 
          {
            object_range_min = range_k;
            object_range_min_index = k;
          }
          else if (object_range_max < range_k) 
          {
            object_range_max = range_k;
            object_range_max_index = k;
          }
          prev_range = range_k;
        }
        else
        {
          // k is not part of this object
          break;
        }
      }
      
      // Update range at begin; it might not be range_i anymore
      range_at_angle_begin = prev_range;
      index_at_angle_begin = upper_limit_out_of_bounds_scan_point;
    }
    
    /* Evaluate the found object (it consists of at least the ith scan) */
    const float distance = object_range_sum / nr_object_points; // Average distance
    const float object_seen_width = sqrt( range_at_angle_begin * 
                                          range_at_angle_begin + 
                                          range_at_angle_end * 
                                          range_at_angle_end - 
                                          2 * range_at_angle_begin * 
                                              range_at_angle_end * 
                                              cosf (bank_argument.angle_increment * nr_object_points)
                                          ); // This is the seen object width using the law of cosine
    
    // Threshold check
    if (bank_argument.object_threshold_min_nr_points <= nr_object_points)
    {
      // Valid object
      nr_objects_found++;
      
      // Recursively derive the min, mean and max indices and the sum of all ranges of the object (if found) 
      // in the oldest scans in the bank
      const unsigned int index_min = index_at_angle_begin;
      const unsigned int index_max = index_at_angle_end;
      const unsigned int index_mean = (index_min + (nr_object_points-1) / 2) % bank_argument.points_per_scan;
                                      // Accounts for 360 deg sensor => i==0 could mean that index_max < index_min
      int index_min_old = -1;
      int index_mean_old = -1;
      int index_max_old = -1;
      float range_sum_old = object_range_sum;
      float range_at_min_index_old = 0;
      float range_at_max_index_old = 0;
      getOldIndices(range_min,
                    range_max,
                    nr_object_points,
                    (bank_index_newest - 1) < 0 ? bank_argument.nr_scans_in_bank - 1 : bank_index_newest - 1,
                    1, // levels searched
                    index_mean,
                    0, // consecutive misses
                    0, // threshold for consecutive misses; 0 -> allow no misses
                    &index_min_old,
                    &index_mean_old,
                    &index_max_old,
                    &range_sum_old,
                    &range_at_min_index_old,
                    &range_at_max_index_old);
      
      // Could we track object?
      if (0 <= index_mean_old)
      {
        // YES!
        // Create a Moving Object
        moving_objects::msg::MovingObject mo;
        moving_objects::msg::MovingObject mo_old_positions;
        
        // Set the expected information
        mo.map_frame = bank_argument.map_frame;
        mo.fixed_frame = bank_argument.fixed_frame;
        mo.base_frame = bank_argument.base_frame;
        mo.header.frame_id = bank_argument.sensor_frame;
        //mo.header.seq = nr_objects_found;
        mo.header.stamp = new_time; // ros::Time(bank_stamp[bank_index_newest]);
        mo.seen_width = object_seen_width;
        mo.angle_begin = index_min * bank_argument.angle_increment + bank_argument.angle_min;
        mo.angle_end   = index_max * bank_argument.angle_increment + bank_argument.angle_min;
        const double angle_mean = index_mean * bank_argument.angle_increment + bank_argument.angle_min;
        mo.distance_at_angle_begin = range_at_angle_begin;
        mo.distance_at_angle_end   = range_at_angle_end;
        // Position is dependent on the distance and angle_mean
        // Reference coordinate system (relation to the Lidar):
        //   x: forward
        //   y: left
        //   z: up        
        mo.distance = distance;
        

        // Added
        double bearing;
        bearing = computeBearing((mo.angle_begin + mo.angle_end)/2, node->compass_heading);
        global_position position = findLatitudeLongitudeOfObject(bearing, mo.distance);
        mo.latitude = position.latitude;
        mo.longitude = position.longitude;


        // Optical frame?
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          mo.position.x = (double) - distance * sinf(angle_mean);
          mo.position.y = 0.0;
          mo.position.z = (double) distance * cosf(angle_mean);
        }
        else
        {
          // No, X-axis forward, Y-axis left, Z-axis up
          mo.position.x = (double) distance * cosf(angle_mean);
          mo.position.y = (double) distance * sinf(angle_mean);
          mo.position.z = 0.0;
        }
        
        // This will be negated rotation around the Y-axis in the case of an optical frame!
        mo.angle_for_closest_distance = object_range_min_index * bank_argument.angle_increment + 
                                        bank_argument.angle_min;
        mo.closest_distance = object_range_min;
        
        // Optical frame?
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          mo.closest_point.x = - object_range_min * sinf(mo.angle_for_closest_distance);
          mo.closest_point.y = 0.0;
          mo.closest_point.z = object_range_min * cosf(mo.angle_for_closest_distance);
        }
        else
        {
          // No, X-axis forward, Y-axis left, Z-axis up
          mo.closest_point.x = object_range_min * cosf(mo.angle_for_closest_distance);
          mo.closest_point.y = object_range_min * sinf(mo.angle_for_closest_distance);
          mo.closest_point.z = 0.0;
        }
        
        // Distance from sensor to object at old time
        const unsigned int nr_object_points_old = (index_min_old <= index_max_old) ? 
                                                  (index_max_old - index_min_old + 1) :
                                                  bank_argument.points_per_scan - (index_min_old - index_max_old) + 1;
        const float distance_old = range_sum_old / nr_object_points_old;
        // distance is found at index_mean_old, this is the angle at which distance is found
        const double distance_angle_old = index_mean_old * bank_argument.angle_increment + bank_argument.angle_min;
        // Covered angle
        const double covered_angle_old = nr_object_points_old * bank_argument.angle_increment;
        // Width of old object
        const double object_seen_width_old = sqrt( range_at_min_index_old * 
                                                   range_at_min_index_old + 
                                                   range_at_max_index_old * 
                                                   range_at_max_index_old - 
                                                   2 * range_at_min_index_old * 
                                                       range_at_max_index_old * 
                                                       cosf (covered_angle_old)
                                                 ); // This is the seen object width using the law of cosine
        // Coordinates at old time
        double x_old;
        double y_old;
        double z_old;
        
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          x_old = - distance_old * sinf(distance_angle_old);
          y_old = 0.0;
          z_old = distance_old * cosf(distance_angle_old);
        }
        else
        {
          x_old = distance_old * cosf(distance_angle_old);
          y_old = distance_old * sinf(distance_angle_old);
          z_old = 0.0;
        }
        
        mo_old_positions.position.x = x_old;
        mo_old_positions.position.y = y_old;
        mo_old_positions.position.z = z_old;
        
     
        geometry_msgs::msg::PointStamped in;
        geometry_msgs::msg::PointStamped out;
        
        // Transform old point into map, fixed and base frames at old_time
        in.header.frame_id = bank_argument.sensor_frame;
        in.header.stamp = old_time;
        in.point = mo_old_positions.position;
        
        const double dx_sensor = mo.position.x - x_old;
        const double dy_sensor = mo.position.y - y_old;
        const double dz_sensor = mo.position.z - z_old;
        
        
        // And with what velocity
//         const double dt = bank_stamp[bank_index_newest] - bank_stamp[bank_index_put];
        const double dt = mo.header.stamp.sec - bank_stamp[bank_index_put];
//         ROS_ERROR_STREAM("newest stamp = " << mo.header.stamp.toSec() << std::endl << "oldest stamp = " << bank_stamp[bank_index_put] << std::endl << "dt = " << dt << std::endl);
        mo.velocity.x = dx_sensor / dt;
        mo.velocity.y = dy_sensor / dt;
        mo.velocity.z = dz_sensor / dt;
        
        // Calculate speed and normalized velocity
        mo.speed = sqrt(mo.velocity.x * mo.velocity.x  +  
                        mo.velocity.y * mo.velocity.y  +  
                        mo.velocity.z * mo.velocity.z);

        // Avoid division by 0
        if (0 < mo.speed)
        {
          mo.velocity_normalized.x = mo.velocity.x / mo.speed;
          mo.velocity_normalized.y = mo.velocity.y / mo.speed;
          mo.velocity_normalized.z = mo.velocity.z / mo.speed;
        }
        else
        {
          mo.velocity_normalized.x = 0.0;
          mo.velocity_normalized.y = 0.0;
          mo.velocity_normalized.z = 0.0;
        }
        
        // Threshold check
        if (bank_argument.object_threshold_min_speed <= mo.speed || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_map_frame || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_fixed_frame || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_base_frame)
        {
          // Calculate confidence value using the user-defined function
          mo.confidence = calculateConfidence(mo, 
                                              bank_argument, 
                                              dt, 
                                              object_seen_width_old);
          
          // Bound the value to [0,1]
          mo.confidence = (mo.confidence < 0.0  ?  0.0  :  mo.confidence);
          mo.confidence = (mo.confidence < 1.0  ?  mo.confidence  :  1.0);
          
          // Are we confident enough to report this object?
          if (bank_argument.object_threshold_min_confidence <= mo.confidence)
          {
            // Adapt EMA message intensities
            if (bank_argument.publish_ema)
            {
              // Are we avoiding wrapping around the bank edges?
              if (index_min <= index_max)
              {
                // YES
                for (unsigned int k=index_min; k<=index_max; ++k)
                {
                  //msg_ema.intensities[k] = 300.0f;
                }
              }
              else
              {
                // NO - we are wrapping around
                // index_max < index_min
                for (unsigned int k=index_min; k<bank_argument.points_per_scan; ++k)
                {
                  //msg_ema.intensities[k] = 300.0f;
                }
                for (unsigned int k=0; k<index_max; ++k)
                {
                  //msg_ema.intensities[k] = 300.0f;
                }
              }
            }
            
            // Push back the moving object info to the msg
            moa.objects.push_back(mo);
            moa_old_positions.objects.push_back(mo_old_positions);
          }
        }
      }
    }
    
    i = index_at_angle_end + 1;
    nr_object_points = 0;
  }
  
  // Filter found objects
//   mergeFoundObjects(&moa);
  
  // Moving object array message
  //++moa_seq;
  if (bank_argument.publish_objects && 0 < moa.objects.size())
  {
    //moa.origin_node_name = ros::this_node::getName() + bank_argument.node_name_suffix;
    
    // Publish MOA message
    // Added
    node->pub_objects->publish(moa);
  }
}
// Debug/print bank column/msg
std::string Bank::getStringPutPoints()
{
  float * bank_put = bank_ranges_ema[bank_index_put];  
  std::ostringstream stream;
  stream << "Bank points (at put index):";
  for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
  {
    stream << " " << bank_put[i];
  }
  stream << std::endl;
  std::string string = stream.str();
  return string;
}


// Init indices
inline void Bank::initIndex()
{
  bank_index_put = 1;
  bank_index_newest = 0;
}


// Advance indices
inline void Bank::advanceIndex()
{
  bank_index_put = (bank_index_put + 1) % bank_argument.nr_scans_in_bank; // points to the oldest message
  bank_index_newest = (bank_index_newest + 1) % bank_argument.nr_scans_in_bank; // points to the newest/this message
}


// Init bank based on LaserScan msg
long Bank::init(BankArgument bank_argument, const sensor_msgs::msg::LaserScan * msg)
{

  /*camera/optical frames axis
  if (!bank_is_initialized)
  {
    if (!bank_argument.sensor_frame_has_z_axis_forward && strstr(msg->header.frame_id.c_str(), "_optical") != NULL)
    {
      ROS_WARN_STREAM("The sensor frame (" << msg->header.frame_id.c_str() << ") seems to be a camera/optical frame. "
                      "Perhaps sensor_frame_has_z_axis_forward should be set?");
    }
    else if (bank_argument.sensor_frame_has_z_axis_forward)
    {
      ROS_WARN_STREAM("Please note that setting sensor_frame_has_z_axis_forward will cause the ema and "
                      "objects_closest_points messages to be shown incorrectly.");
    }
  }*/
  
  bank_argument.sensor_frame    = msg->header.frame_id;
  bank_argument.points_per_scan = msg->ranges.size();
  bank_argument.angle_min       = msg->angle_min;
  bank_argument.angle_max       = msg->angle_max;
  bank_argument.angle_increment = msg->angle_increment;
  bank_argument.time_increment  = msg->time_increment;
  bank_argument.scan_time       = msg->scan_time;
  bank_argument.range_min       = msg->range_min;
  bank_argument.range_max       = msg->range_max;
  resolution                    = bank_argument.angle_increment;
  
  initBank(bank_argument);
  
  //ROS_DEBUG_STREAM("Bank arguments:" << std::endl << bank_argument);
  
  return addFirstMessage(msg);
}


// Add FIRST LaserScan message to bank - no ema
long Bank::addFirstMessage(const sensor_msgs::msg::LaserScan * msg)
{
  bank_stamp[0] = msg->header.stamp.sec;
  
//   if (bank_argument.ema_alpha != 1.0)
//   {
    float * bank_put = bank_ranges_ema[0];
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] == std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else if (msg->ranges[i] == -std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_min - 0.01;
      }
//       else if (msg->ranges[i] != msg->ranges[i])
      else if (std::isnan(msg->ranges[i]))
      {
        // The range is NaN
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else
      {
        // Turn infinity into large value
        bank_put[i] = msg->ranges[i];
      }
    }
//   }
//   else
//   {
//     memcpy(&bank_ranges_ema[0][0], msg->ranges.data(), bank_ranges_bytes);
//   }
  
  initIndex(); // set put to 1 and newest to 0
  bank_is_filled = false;
  
  //ROS_DEBUG_STREAM("First message (LaserScan):" << std::endl << *msg);
  
  return 0;
}

// Add LaserScan message and perform EMA
long Bank::addMessage(const sensor_msgs::msg::LaserScan * msg)
{
  // Save timestamp
  bank_stamp[bank_index_put] = msg->header.stamp.sec;
  
  // Save EMA of ranges, if applicable, otherwise, perform memcpy
  const double alpha = bank_argument.ema_alpha;
//   if (alpha != 1.0)
//   {
    const double alpha_prev = 1.0 - bank_argument.ema_alpha;
    float * bank_put = bank_ranges_ema[bank_index_put];
    float * bank_newest = bank_ranges_ema[bank_index_newest];
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] == std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else if (msg->ranges[i] == -std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_min - 0.01;
      }
      else if (msg->ranges[i] != msg->ranges[i])
      {
        // The range is NaN
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else
      {
        // Turn infinity into large value
        bank_put[i] = alpha * msg->ranges[i]  +  alpha_prev * bank_newest[i];
      }
    }
//   }
//   else
//   {
//     memcpy(&bank_ranges_ema[bank_index_put][0], msg->ranges.data(), bank_ranges_bytes);
//   }
  
  advanceIndex();
  if (!bank_is_filled && bank_index_put < bank_index_newest)
  {
    bank_is_filled = true;
  }
  
  return 0;
}


// Added                        (angle_begin+angle_end)/2)
double Bank::computeBearing(double angle_to_lidar_frame, double north_heading){

  // angle_to_lidar_frame specified by angle_min and angle_max of laserScan, it needs to be converted to modulo 360
  double angle_to_lidar_frame_mod360 = ((int)angle_to_lidar_frame)%360;
  
  // bearing is always defined as an angle clockwise from north
  double bearing;
  bearing = (int)(angle_to_lidar_frame_mod360 - north_heading)%360;

  return bearing;
}

// Added
Bank::global_position Bank::findLatitudeLongitudeOfObject(double bearing, double distance){
  
  double boat_latitude = node->latitude;
  double boat_longitude = node->longitude;

  global_position object_position;;
  int R = 6371000; // mean radius of Earth in meters
  
  //convert boat_latitude and boat_longitude from decimal degrees(the gpsx topic gives them that way) -> radians
  double boat_latitude_radians = (boat_latitude*M_PI)/180;
  double boat_longitude_radians = (boat_longitude*M_PI)/180;


  /*Formula:
  φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
	λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
  where 	φ is latitude, λ is longitude, θ is the bearing (clockwise from north), δ is the angular distance d/R; d being the distance travelled, R the earth’s radius
  */
  object_position.latitude = asin(sin(boat_latitude_radians)*cos(distance/R) + cos(boat_latitude_radians)*sin(distance/R)*cos(bearing));
  object_position.longitude = boat_longitude_radians + atan2(sin(bearing)*sin(distance/R)*cos(boat_latitude_radians), cos(distance/R) - sin(boat_latitude_radians)*sin(object_position.latitude));
  
  // Convert to DD from radians
  object_position.latitude = (object_position.latitude*180)/M_PI;
  object_position.longitude = (object_position.longitude*180)/M_PI;

  return object_position;

}

