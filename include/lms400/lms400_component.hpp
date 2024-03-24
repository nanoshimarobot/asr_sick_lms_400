#pragma once

#include <cerrno>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sick_lms400.h"

class LMS400 : public rclcpp::Node
{
private:
  // TCP/IP connection parameters
  std::string hostname_;
  std::string password_;
  int port_;

  // Filter settings
  int filter_;
  int mean_filter_params_;
  double range_filter_params_min_, range_filter_params_max_;

  // Turn intensity data on/off
  bool intensity_;

  // Turn laser on/off
  bool laser_enabled_;

  // Basic measurement parameters
  double angular_resolution_, scanning_frequency_;
  double min_angle_, max_angle_;
  int eRIS_;

  // Password for changing to userlevel 3 (service)
  bool loggedin_;
  int debug_ = 1;

  asr_sick_lms_400::asr_sick_lms_400 lms_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  rclcpp::TimerBase::SharedPtr main_timer_;

public:
  LMS400(const rclcpp::NodeOptions & options) : LMS400("", options) {}
  LMS400(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lms400_node", name_space)
  {
    using namespace std::chrono_literals;
    // connection settings
    hostname_ = this->declare_parameter<std::string>("hostname", "192.168.0.100");
    password_ = this->declare_parameter<std::string>("password", "client");
    port_ = this->declare_parameter<int>("port", 2111);

    // filter settings
    filter_ = this->declare_parameter<int>("filter", 11);
    eRIS_ = this->declare_parameter<int>("enable_eRIS", 1);
    mean_filter_params_ = this->declare_parameter<int>("mean_filter_parameter", 3);
    range_filter_params_min_ = this->declare_parameter<double>("range_filter_parameter_min", 700.0);
    range_filter_params_max_ =
      this->declare_parameter<double>("range_filter_parameter_max", 3000.0);

    angular_resolution_ = this->declare_parameter<double>("angular_resolution", 0.1);
    scanning_frequency_ = this->declare_parameter<double>("scanning_frequency", 360.0);
    laser_enabled_ = this->declare_parameter<bool>("enable_laser", true);
    intensity_ = this->declare_parameter<bool>("enable_intensity", true);
    min_angle_ = this->declare_parameter<double>("min_angle", 55.0);
    max_angle_ = this->declare_parameter<double>("max_angle", 125.0);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("output_scan", rclcpp::QoS(1));

    loggedin_ = false;

    if (start() != 0) {
      return;
    }

    lms_.StartMeasurement(intensity_);

    main_timer_ = this->create_wall_timer(1ms, [this]() {
      if (this->laser_enabled_) {
        sensor_msgs::msg::LaserScan msg;
        msg = lms_.ReadMeasurement();
        msg.header.frame_id = "laser";  // ?
        msg.header.stamp = this->get_clock()->now();
        if (msg.ranges.size() != 0) {
          scan_pub_->publish(msg);
        }
      }
    });
  }

  ~LMS400() {}

  /**
   * @brief Start LMS
   *
   * @return int
   */
  int start()
  {
    lms_ = asr_sick_lms_400::asr_sick_lms_400(hostname_.c_str(), port_, debug_);

    // check lms connection
    if (lms_.Connect() != 0) {
      RCLCPP_ERROR(
        this->get_logger(), "> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]...[failed!]",
        hostname_.c_str(), port_);
      return -1;
    }
    RCLCPP_INFO(
      this->get_logger(), "> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]... [done]",
      hostname_.c_str(), port_);

    lms_.StopMeasurement();

    if (strncmp(password_.c_str(), "NULL", 4) != 0) {
      if (lms_.SetUserLevel(4, password_.c_str()) != 0)
        RCLCPP_WARN(
          this->get_logger(), "> [SickLMS400] Unable to change userlevel to 'Service' using %s",
          password_.c_str());
      else {
        loggedin_ = true;
        // Enable or disable filters
        if ((mean_filter_params_ >= 2) && (mean_filter_params_ <= 200))
          lms_.SetMeanFilterParameters(mean_filter_params_);

        if (
          (range_filter_params_min_ >= 700.0) &&
          (range_filter_params_max_ > range_filter_params_min_) &&
          (range_filter_params_max_ <= 3600.0))
          lms_.SetRangeFilterParameters(
            (float)range_filter_params_min_, (float)range_filter_params_max_);

        lms_.EnableFilters(filter_);

        RCLCPP_INFO(
          this->get_logger(),
          "> [SickLMS400] Enabling selected filters (%d, %f, %f, %d)... "
          "[done]",
          mean_filter_params_, (float)range_filter_params_min_, (float)range_filter_params_max_,
          filter_);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(), "> [SickLMS400] Userlevel 3 password not given. Filter(s) disabled!");
    }

    // Enable extended RIS detectivity
    if (eRIS_) {
      lms_.EnableRIS(1);
      RCLCPP_INFO(this->get_logger(), "> [SickLMS400] Enabling extended RIS detectivity... [done]");
    }
    // Set scanning parameters
    if (
      lms_.SetResolutionAndFrequency(
        scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "> [SickLMS400] Couldn't set values for resolution, frequency, "
        "and min/max angle. Using previously set values.");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "> [SickLMS400] Enabling user values for resolution (%f), "
        "frequency (%f) and min/max angle (%f/%f)... [done]",
        angular_resolution_, scanning_frequency_, min_angle_, max_angle_);
    }

    return 0;
  }

  /**
   * @brief Stop LMS
   *
   * @return int
   */
  int stop()
  {
    // Stop the measurement process
    lms_.StopMeasurement();
    // Set back to userlevel 0
    lms_.TerminateConfiguration();
    // Disconnect from the laser unit
    lms_.Disconnect();

    RCLCPP_INFO(this->get_logger(), "> [SickLMS400] SICK LMS400 driver shutting down... [done]");
    return 0;
  }

  // Set new measurement values and restart scanning
  void restartMeasurementWithNewValues(
    float scanning_frequency, float angular_resolution, float min_angle, float diff_angle,
    int intensity, bool laser_enabled)
  {
    // Stop the measurement process
    lms_.StopMeasurement();

    // Change userlevel to 3
    if (lms_.SetUserLevel(4, password_.c_str()) != 0) {
      RCLCPP_WARN(
        this->get_logger(), "> Unable to change userlevel to 'Service' using %s",
        password_.c_str());
      if (laser_enabled) lms_.StartMeasurement(intensity);
    } else {
      // Set the angular resolution and frequency
      if (
        lms_.SetResolutionAndFrequency(
          scanning_frequency, angular_resolution, min_angle, diff_angle) == 0)
        // Re-start the measurement process
        if (laser_enabled) lms_.StartMeasurement(intensity);
    }
  }

  // 大根
  // void getParametersFromServer()
  // {
  //   // LMS400 related parameters
  //   bool laser_enabled;
  //   nh_.getParam("enable_laser", laser_enabled);
  //   // New value specified
  //   if (laser_enabled != laser_enabled_) {
  //     RCLCPP_INFO(this->get_logger(), "New enable_laser parameter received (%d)", laser_enabled);
  //     laser_enabled_ = laser_enabled;

  //     if (!laser_enabled_)
  //       lms_.StopMeasurement();
  //     else
  //       lms_.StartMeasurement(intensity_);
  //   }

  //   bool intensity;
  //   nh_.getParam("enable_intensity", intensity);
  //   // New value specified
  //   if (intensity != intensity_) {
  //     RCLCPP_INFO(this->get_logger(), "New enable_intensity parameter received (%d)", intensity);
  //     intensity_ = intensity;
  //   }

  //   double angular_resolution;
  //   nh_.getParam("angular_resolution", angular_resolution);
  //   // New value specified
  //   if (angular_resolution != angular_resolution_) {
  //     RCLCPP_INFO(
  //       this->get_logger(), "New angular_resolution parameter received (%f)", angular_resolution);
  //     angular_resolution_ = angular_resolution;
  //     restartMeasurementWithNewValues(
  //       scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_,
  //       laser_enabled_);
  //   }

  //   double min_angle;
  //   nh_.getParam("min_angle", min_angle);
  //   // New value specified
  //   if (min_angle != min_angle_) {
  //     RCLCPP_INFO(this->get_logger(), "New min_angle parameter received (%f)", min_angle);
  //     min_angle_ = min_angle;
  //     restartMeasurementWithNewValues(
  //       scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_,
  //       laser_enabled_);
  //   }

  //   double max_angle;
  //   nh_.getParam("max_angle", max_angle);
  //   // New value specified
  //   if (max_angle != max_angle_) {
  //     RCLCPP_INFO(this->get_logger(), "New max_angle parameter received (%f)", max_angle);
  //     max_angle_ = max_angle;
  //     restartMeasurementWithNewValues(
  //       scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_,
  //       laser_enabled_);
  //   }

  //   double scanning_frequency;
  //   nh_.getParam("scanning_frequency", scanning_frequency);
  //   // New value specified
  //   if (scanning_frequency != scanning_frequency_) {
  //     RCLCPP_INFO(
  //       this->get_logger(), "New scanning_frequency parameter received (%f)", scanning_frequency);
  //     scanning_frequency_ = scanning_frequency;
  //     restartMeasurementWithNewValues(
  //       scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_,
  //       laser_enabled_);
  //   }
  // }
};