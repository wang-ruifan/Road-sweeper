#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerCommonDef.h"

class LanePublisherNode
{
public:
    LanePublisherNode()
    {
        // 初始化发布器
        lane_pub_ = nh_.advertise<std_msgs::Int32>("current_lane_id", 10);

        // 初始化订阅器
        pose_sub_ = nh_.subscribe("/current_pose", 10, &LanePublisherNode::poseCallback, this);

        // 设置定时器，每秒调用一次publishLaneID
        timer_ = nh_.createTimer(ros::Duration(1.0), &LanePublisherNode::publishLaneID, this);

        // 加载地图数据
        loadMapData();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher lane_pub_;
    ros::Subscriber pose_sub_;
    ros::Timer timer_;
    PlannerHNS::RoadNetwork map_;
    PlannerHNS::WayPoint current_position_;

    void loadMapData()
    {
        // 在这里加载你的vectormap数据并构建RoadNetwork对象
        // 例如：PlannerHNS::MappingHelpers::LoadMapData("path_to_map_file", map_);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 更新当前位置
        current_position_.pos.x = msg->pose.position.x;
        current_position_.pos.y = msg->pose.position.y;
        current_position_.pos.z = msg->pose.position.z;
    }

    void publishLaneID(const ros::TimerEvent &)
    {
        // 获取最近的车道
        PlannerHNS::Lane *current_lane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(current_position_, map_, 10.0);
        if (current_lane)
        {
            std_msgs::Int32 msg;
            msg.data = current_lane->id;
            lane_pub_.publish(msg);
            ROS_INFO("Published lane ID: %d", current_lane->id);
        }
        else
        {
            ROS_WARN("No lane found within the search distance.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_publisher_node");
    LanePublisherNode node;
    ros::spin();
    return 0;
}