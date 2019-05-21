/*********************************************************************
 * Author: shishiteng@126.com
 *********************************************************************/
#include <costmap_2d/virtual_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::VirtualLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

VirtualLayer::VirtualLayer() : dsrv_(NULL) {}

VirtualLayer::~VirtualLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void VirtualLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;

  points_sub_ = g_nh.subscribe("virtualwall_points", 1, &VirtualLayer::incomingPoints, this);
  has_updated_data_ = false;

  if (dsrv_)
    delete dsrv_;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VirtualLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VirtualLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
    enabled_ = config.enabled;

  if (enabled_)
    has_updated_data_ = true;
}

void VirtualLayer::matchSize()
{
}

void VirtualLayer::activate()
{
  onInitialize();
}

void VirtualLayer::deactivate()
{
  points_sub_.shutdown();
}

void VirtualLayer::reset()
{
  onInitialize();
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                double *max_x, double *max_y)
{
  if (!has_updated_data_)
    return;

  if (!enabled_)
    return;

  //如果virtual wall用occupacyGrid更好，直接得到地图信息就可以更新bounds了
  //这里先用一种取巧的方法，直接使用costmap的信息
  if (layered_costmap_)
  {
    Costmap2D *costmap = layered_costmap_->getCostmap();
    if (costmap)
    {
      *min_x = costmap->getOriginX();
      *min_y = costmap->getOriginY();
      *max_x = costmap->getSizeInMetersX() + costmap->getOriginX();
      *max_y = costmap->getSizeInMetersY() + costmap->getOriginY();
    }
  }
}

void VirtualLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!has_updated_data_)
    return;

  if (!enabled_)
    return;

  ROS_DEBUG("VirtualLayer_updateCosts[%d ]", msg_.points.size());

  // 为了减小计算量，这里的点应该降采样
  for (int i = 0; i < msg_.points.size(); i++)
  {
    unsigned int mx, my;
    float x = msg_.points[i].x;
    float y = msg_.points[i].y;
    if (master_grid.worldToMap(x, y, mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE); //INSCRIBED_INFLATED_OBSTACLE LETHAL_OBSTACLE
    }
    else
    {
      //ROS_WARN("master_grid.worldToMap failed[%.1f %.1f]",x,y);
    }
  }

  //has_updated_data_ = false;
}

void VirtualLayer::incomingPoints(const sensor_msgs::PointCloudConstPtr &points_msg)
{
  if (!enabled_)
    return;

  msg_ = *points_msg;
  has_updated_data_ = true;
}

} // namespace costmap_2d
