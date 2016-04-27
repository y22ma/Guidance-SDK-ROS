/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/PoseStamped.h> //IMU
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <signal.h>


#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return;}}


class GuidanceMultiCamNode
{
public:
  GuidanceMultiCamNode();
  ~GuidanceMultiCamNode();
  int guidanceCb(int data_type, int data_len, char *content);

private:
  const int CAM_COUNT;

  ros::NodeHandle nh_;
  std::vector<ros::Publisher> depth_image_pubs_;
  std::vector<ros::Publisher> left_image_pubs_;
  std::vector<ros::Publisher> right_image_pubs_;
  ros::Publisher imu_pub_;
  ros::Publisher obstacle_distance_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher ultrasonic_pub_;
  ros::Publisher position_pub_;
  ros::Publisher pose_pub_;
  e_vbus_index cam_vbus_indices_[5];

  DJI_lock dji_lock_;
  DJI_event dji_event_;

  bool visualize_;
  bool imu_received_;
  int32_t width_;
  int32_t height_;
  int32_t image_len_;
  cv::Mat g_greyscale_image_left;
  cv::Mat g_greyscale_image_right;
  cv::Mat g_depth;
  cv::Mat depth8;

  sensor_msgs::Imu curr_imu_;
};

GuidanceMultiCamNode* instance;

int globalCb(int data_type, int data_len, char *content)
{
  instance->guidanceCb(data_type, data_len, content);
}


int GuidanceMultiCamNode::guidanceCb(int data_type, int data_len, char *content)
{
  dji_lock_.enter();

  /* image data */
  for (uint32_t i = 0; i < CAM_COUNT; i++)
  {
    if (e_image == data_type && NULL != content)
    {
      image_data* data = (image_data*)content;

      if (data->m_greyscale_image_left[cam_vbus_indices_[i]])
      {
        memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[cam_vbus_indices_[i]], image_len_);
        if (visualize_)
        {
          cv::imshow("left" + std::to_string(i),  g_greyscale_image_left);
        }

        // publish left greyscale image
        cv_bridge::CvImage left_8;
        g_greyscale_image_left.copyTo(left_8.image);
        left_8.header.frame_id  = "guidance";
        left_8.header.stamp = ros::Time::now();
        left_8.encoding   = sensor_msgs::image_encodings::MONO8;
        left_image_pubs_[i].publish(left_8.toImageMsg());
      }
      if (data->m_greyscale_image_right[cam_vbus_indices_[i]])
      {
        memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[cam_vbus_indices_[i]], image_len_);
        if (visualize_)
        {
	  cv::imshow("right" + std::to_string(i), g_greyscale_image_right);
        }

        // publish right greyscale image
        cv_bridge::CvImage right_8;
        g_greyscale_image_right.copyTo(right_8.image);
        right_8.header.frame_id  = "guidance";
        right_8.header.stamp   = ros::Time::now();
        right_8.encoding     = sensor_msgs::image_encodings::MONO8;
        right_image_pubs_[i].publish(right_8.toImageMsg());
      }
      if (data->m_depth_image[cam_vbus_indices_[i]])
      {
        memcpy(g_depth.data, data->m_depth_image[cam_vbus_indices_[i]], image_len_ * 2);
        g_depth.convertTo(depth8, CV_8UC1);
        if (visualize_)
        {
	  cv::imshow("depth" + std::to_string(i), depth8);
        }

        // publish depth image
        cv_bridge::CvImage depth_16;
        g_depth.copyTo(depth_16.image);
        depth_16.header.frame_id  = "guidance";
        depth_16.header.stamp   = ros::Time::now();
        depth_16.encoding   = sensor_msgs::image_encodings::MONO16;
        depth_image_pubs_[i].publish(depth_16.toImageMsg());
      }

      if (visualize_)
      {
        cv::waitKey(1);
      }
    }
  }

  /* imu */
  if (e_imu == data_type && NULL != content)
  {
    imu *imu_data = (imu*)content;
    ROS_INFO("frame index: %d, stamp: %d", imu_data->frame_index, imu_data->time_stamp);
    ROS_INFO("imu: [%f %f %f %f %f %f %f]", imu_data->acc_x, imu_data->acc_y,
        imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3]);

    //_.publish imu data
    imu_received_ = true;
    curr_imu_.header.frame_id = "guidance";
    curr_imu_.header.stamp    = ros::Time::now();
    curr_imu_.linear_acceleration.x = imu_data->acc_x;
    curr_imu_.linear_acceleration.y = imu_data->acc_y;
    curr_imu_.linear_acceleration.z = imu_data->acc_z;
    curr_imu_.orientation.w = imu_data->q[0];
    curr_imu_.orientation.x = imu_data->q[1];
    curr_imu_.orientation.y = imu_data->q[2];
    curr_imu_.orientation.z = imu_data->q[3];
    imu_pub_.publish(curr_imu_);
  }
  /* velocity */
  if (e_velocity == data_type && NULL != content)
  {
    velocity *vo = (velocity*)content;
    ROS_INFO("frame index: %d, stamp: %d", vo->frame_index, vo->time_stamp);
    ROS_INFO("vx:%f vy:%f vz:%f", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz);

    //_.publish velocity
    geometry_msgs::Vector3Stamped g_vo;
    g_vo.header.frame_id = "guidance";
    g_vo.header.stamp    = ros::Time::now();
    g_vo.vector.x = 0.001f * vo->vx;
    g_vo.vector.y = 0.001f * vo->vy;
    g_vo.vector.z = 0.001f * vo->vz;
    velocity_pub_.publish(g_vo);
  }

  /* obstacle distance */
  if (e_obstacle_distance == data_type && NULL != content)
  {
    obstacle_distance *oa = (obstacle_distance*)content;
    ROS_INFO("frame index: %d, stamp: %d", oa->frame_index, oa->time_stamp);
    ROS_INFO("obstacle distance:");
    for (int i = 0; i < CAM_COUNT; ++i)
    {
      ROS_INFO(" %f ", 0.01f * oa->distance[i]);
    }

    //_.publish obstacle distance
    sensor_msgs::LaserScan g_oa;
    g_oa.ranges.resize(CAM_COUNT);
    g_oa.header.frame_id = "guidance";
    g_oa.header.stamp    = ros::Time::now();
    for (int i = 0; i < CAM_COUNT; ++i)
      g_oa.ranges[i] = 0.01f * oa->distance[i];
    obstacle_distance_pub_.publish(g_oa);
  }

  /* ultrasonic */
  if (e_ultrasonic == data_type && NULL != content)
  {
    ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
    ROS_INFO("frame index: %d, stamp: %d", ultrasonic->frame_index, ultrasonic->time_stamp);
    for (int d = 0; d < CAM_COUNT; ++d)
    {
      ROS_INFO("ultrasonic distance: %f, reliability: %d",
          ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d]);
    }

    // publish ultrasonic data
    sensor_msgs::LaserScan g_ul;
    g_ul.ranges.resize(CAM_COUNT);
    g_ul.intensities.resize(CAM_COUNT);
    g_ul.header.frame_id = "guidance";
    g_ul.header.stamp    = ros::Time::now();
    for (int d = 0; d < CAM_COUNT; ++d)
    {
      g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
      g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
    }
    ultrasonic_pub_.publish(g_ul);
  }

  if (e_motion == data_type && NULL != content)
  {
    motion* m = (motion*)content;
    ROS_INFO("frame index: %d, stamp: %d", m->frame_index, m->time_stamp);
    ROS_INFO("(px,py,pz)=(%.2f,%.2f,%.2f)", m->position_in_global_x,
        m->position_in_global_y, m->position_in_global_z);

    //_.publish position
    geometry_msgs::Vector3Stamped g_pos;
    g_pos.header.frame_id = "guidance";
    g_pos.header.stamp = ros::Time::now();
    g_pos.vector.x = m->position_in_global_x;
    g_pos.vector.y = m->position_in_global_y;
    g_pos.vector.z = m->position_in_global_z;
    position_pub_.publish(g_pos);

    if (imu_received_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "guidance";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = g_pos.vector.x;
      pose.pose.position.y = g_pos.vector.y;
      pose.pose.position.z = g_pos.vector.z;
      pose.pose.orientation.w = curr_imu_.orientation.w;
      pose.pose.orientation.x = curr_imu_.orientation.x;
      pose.pose.orientation.y = curr_imu_.orientation.y;
      pose.pose.orientation.z = curr_imu_.orientation.z;
      pose_pub_.publish(pose);
    }
  }

  dji_lock_.leave();
  dji_event_.set_event();

  return 0;
}


GuidanceMultiCamNode::~GuidanceMultiCamNode()
{
  /* release data transfer */
  int err_code = stop_transfer();
  RETURN_IF_ERR(err_code);

  //make sure the ack packet from GUIDANCE is received
  sleep(1);
  ROS_INFO("release_transfer");
  err_code = release_transfer();
  RETURN_IF_ERR(err_code);
}


GuidanceMultiCamNode::GuidanceMultiCamNode() : CAM_COUNT(5), depth_image_pubs_(CAM_COUNT),
    left_image_pubs_(CAM_COUNT), right_image_pubs_(CAM_COUNT), imu_received_(false)
{
  cam_vbus_indices_[0] = e_vbus1;
  cam_vbus_indices_[1] = e_vbus2;
  cam_vbus_indices_[2] = e_vbus3;
  cam_vbus_indices_[3] = e_vbus4;
  cam_vbus_indices_[4] = e_vbus5;

  bool lr_auto_exposure, pub_left, pub_right, pub_depth;
  ros::param::param<bool>("~lr_auto_exposure", lr_auto_exposure, true);
  ros::param::param<bool>("~visualize", visualize_, false);
  ros::param::param<bool>("~pub_left", pub_left, true);
  ros::param::param<bool>("~pub_right", pub_right, false);
  ros::param::param<bool>("~pub_depth", pub_depth, false);

  imu_pub_      = nh_.advertise<sensor_msgs::Imu>("/guidance/imu", 1);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity", 1);
  position_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/guidance/position", 1);
  pose_pub_     = nh_.advertise<geometry_msgs::PoseStamped>("/guidance/pose", 1);
  ultrasonic_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic", 1);
  obstacle_distance_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance", 1);

  reset_config();
  int err_code = init_transfer();
  RETURN_IF_ERR(err_code);

  // selecting camera streams
  for (uint32_t i = 0; i < CAM_COUNT; i++)
  {
    if (pub_left)
    {
      left_image_pubs_[i]  = nh_.advertise<sensor_msgs::Image>("/guidance/cam" +
          std::to_string(i) + "/left_image", 1);
      err_code = select_greyscale_image(cam_vbus_indices_[i], true);
      RETURN_IF_ERR(err_code);
    }

    if (pub_right)
    {
      right_image_pubs_[i] = nh_.advertise<sensor_msgs::Image>("/guidance/cam" +
          std::to_string(i) + "/right_image", 1);
      err_code = select_greyscale_image(cam_vbus_indices_[i], false);
      RETURN_IF_ERR(err_code);
    }

    if (pub_depth)
    {
      depth_image_pubs_[i] = nh_.advertise<sensor_msgs::Image>("/guidance/cam" +
          std::to_string(i) + "/depth_image", 1);
      err_code = select_depth_image(cam_vbus_indices_[i]);
      RETURN_IF_ERR(err_code);
    }
  }

  // selecting guidance info stream
  select_imu();
  select_ultrasonic();
  select_obstacle_distance();
  select_velocity();
  select_motion();

  /* start data transfer */
  get_image_size(&width_, &height_);
  image_len_ = width_ * height_;
  ROS_INFO("width %d height %d", width_, height_);
  g_greyscale_image_left = cv::Mat(height_, width_, CV_8UC1);
  g_greyscale_image_right = cv::Mat(height_, width_, CV_8UC1);
  g_depth = cv::Mat(height_, width_, CV_16SC1);
  depth8 = cv::Mat(height_, width_, CV_8UC1);

  //err_code = set_sdk_event_handler(guidanceCb);
  err_code = set_sdk_event_handler(globalCb);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  int online_status[CAM_COUNT];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  ROS_INFO("Sensor online status: ");
  for (int i = 0; i < CAM_COUNT; i++)
  {
    ROS_INFO("%d", online_status[i]);
  }

  // get cali param
  stereo_cali cali[CAM_COUNT];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  ROS_INFO("cu\tcv\tfocal\tbaseline");
  for (int i = 0; i < CAM_COUNT; i++)
  {
    ROS_INFO_STREAM(cali[i].cu << "\t" << cali[i].cv << "\t" << cali[i].focal
        << "\t" << cali[i].baseline);
  }

  instance = this;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  // All the default sigint handler does is call shutdown()
  delete instance;
  ros::shutdown();
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "guidance", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigintHandler);

  instance = new GuidanceMultiCamNode();
  ros::spin();
  delete instance;
}
