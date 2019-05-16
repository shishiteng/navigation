#ifndef COSTMAP_2D_VIRTUAL_LAYER_H_
#define COSTMAP_2D_VIRTUAL_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>

namespace costmap_2d
{

class VirtualLayer : public CostmapLayer
{
public:
  VirtualLayer();
  virtual ~VirtualLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

private:
  void incomingPoints(const sensor_msgs::PointCloudConstPtr& points_msg);

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  bool has_updated_data_;
  ros::Subscriber points_sub_;
  sensor_msgs::PointCloud msg_;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_VIRTUAL_LAYER_H_
