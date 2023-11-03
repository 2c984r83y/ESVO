/*
 * @Author: 2c984r83y 1422512438@qq.com
 * @Date: 2023-11-03 16:38:55
 * @LastEditors: 2c984r83y 1422512438@qq.com
 * @LastEditTime: 2023-11-03 16:38:57
 * @FilePath: \ESVO\esvo_time_surface\src\TimeSurface_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <esvo_time_surface/TimeSurface.h>

int main(int argc, char* argv[])
{
  // 初始化ROS节点
  ros::init(argc, argv, "esvo_time_surface");
  // nh是ROS节点的句柄（handle）
  // 它是一个ROS节点与ROS系统之间的接口。
  // 通过句柄，节点可以与ROS系统中的其他节点、话题、服务等进行通信。
  // 创建ROS节点Handle
  ros::NodeHandle nh;
  // 访问私有命名空间
  ros::NodeHandle nh_private("~");

  // 创建TimeSurface对象
  esvo_time_surface::TimeSurface ts(nh, nh_private);

  // 运行ROS节点
  ros::spin();

  return 0;
}