#include "CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

using std::string;
using std::vector;

using costmap_2d::Costmap2D;
using costmap_2d::Costmap2DROS;
using geometry_msgs::PoseStamped;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planning_node");

  // 下面三行总体构建tf变换，储存10s内的数据
  tf2_ros::Buffer tf(ros::Duration(10));
  // 第一行开启 Buffer 专用线程，让它能独立接收和处理 TF 数据。
  // 第二行创建监听器，让 Buffer 真正开始接收 TF 消息
  tf.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf);

  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
  // planner_costmap_ros_->pause();
  /*修改了膨胀半径为0.3（更换了机器人）。
  创建这个对象之后自动做的事情，在ros服务器里加载地图参数（定义了是cleaning_costmap命名空间），使用tf监听器，默认开启一个后台线程，周期性更新地图。
  如何更新？：使用tf监听器得到tf变换数据，使用命名空间里的参数定义的更新细节（膨胀半径，更新和发布频率等），
  自动监听/cleaning_costmap/observation_sources的参数，观测的传感器话题来源，如果没有，不进行障碍物检测和动态地图更新
  */
  //！！之后变更动态的入口在这里。

  
  // 创建路径规划对象，传入costmap；传入的是指针，因此是得到每次更新后的最新数据。
  CleaningPathPlanning clr(&lcr);
  // 调用路径获取接口
  clr.GetPathInROS();
  // clr.GetBorderTrackingPathInROS();
  ros::Rate r(10);
  while (ros::ok())
  {
    // 不断发布路径
    clr.PublishCoveragePath();
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  //安全退出释放资源
  
  return 0;
}
