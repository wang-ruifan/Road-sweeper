#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <vector_map_msgs/PointArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

class AutoSweep
{
private:
    // 可配置参数
    double search_threshold_;  // 搜索阈值：当车辆移动距离超过此值时重新搜索最近车道

public:
    AutoSweep()
    {
        // 从参数服务器获取搜索阈值参数
        ros::NodeHandle private_nh("~");
        private_nh.param("search_threshold", search_threshold_, 0.5);
        ROS_INFO("Search threshold set to: %.2f meters", search_threshold_);

        lane_sub_ = nh_.subscribe("/vector_map_info/lane", 10, &AutoSweep::laneCallback, this);
        node_sub_ = nh_.subscribe("/vector_map_info/node", 10, &AutoSweep::nodeCallback, this);
        point_sub_ = nh_.subscribe("/vector_map_info/point", 10, &AutoSweep::pointCallback, this);
        pose_sub_ = nh_.subscribe("/current_pose", 10, &AutoSweep::poseCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_markers", 1, true);
        sweep_pub_ = nh_.advertise<std_msgs::Bool>("sweep_control", 1);
        enable_service_ = nh_.advertiseService("enable_auto_sweep", &AutoSweep::enableCallback, this);
    }

private:
    struct LaneData
    {
        int lnid;
        int bnid;
        int fnid;
        int issweep;
        std::vector<double> start_coords;
        std::vector<double> end_coords;
    };

    struct LastPoseInfo {
        double x;
        double y;
        int lane_id;
        double distance;
        bool valid;
    } last_pose_;
    
    ros::NodeHandle nh_;
    ros::Subscriber lane_sub_;
    ros::Subscriber node_sub_;
    ros::Subscriber point_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher sweep_pub_;
    ros::ServiceServer enable_service_;

    std::unordered_map<int, LaneData> lanes_;
    std::unordered_map<int, int> nodes_;

    bool enable_auto_sweep_ = false;
    bool sweep_status_ = false;

    std::unordered_map<int, std::vector<int>> lane_connections_;

    bool enableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enable_auto_sweep_ = req.data;
        res.success = true;
        res.message = enable_auto_sweep_ ? "Auto sweep enabled" : "Auto sweep disabled";

        if (!enable_auto_sweep_) {
            sweepUpdate(false);
        } else {
            sweep_status_ = false;
        }

        ROS_INFO("%s", res.message.c_str());
        return true;
    }

    // 在 laneCallback 中构建车道连接关系
    void laneCallback(const vector_map_msgs::LaneArray::ConstPtr &msg)
    {
        lanes_.clear();
        lane_connections_.clear();
        
        for (const auto &lane : msg->data)
        {
            LaneData lane_data;
            lane_data.lnid = lane.lnid;
            lane_data.bnid = lane.bnid;
            lane_data.fnid = lane.fnid;
            lane_data.issweep = lane.issweep;
            lanes_[lane.lnid] = lane_data;

            // 记录相邻车道关系
            if (lane.blid > 0) {  // 有前向连接
                lane_connections_[lane.lnid].push_back(lane.blid);
            }
            if (lane.flid > 0) {  // 有后向连接
                lane_connections_[lane.lnid].push_back(lane.flid);
            }
        }
        ROS_INFO("Loaded %lu lanes", lanes_.size());
        publishLaneMarkers();
    }

    void nodeCallback(const vector_map_msgs::NodeArray::ConstPtr &msg)
    {
        for (const auto &node : msg->data)
        {
            nodes_[node.nid] = node.pid;
        }
        ROS_INFO("Loaded %lu nodes", nodes_.size());
    }

    void pointCallback(const vector_map_msgs::PointArray::ConstPtr &msg)
    {
        for (const auto &point : msg->data)
        {
            for (auto &lane : lanes_)
            {
                if (nodes_[lane.second.bnid] == point.pid)
                {
                    lane.second.start_coords = {point.ly, point.bx}; // Swap x and y
                }
                if (nodes_[lane.second.fnid] == point.pid)
                {
                    lane.second.end_coords = {point.ly, point.bx}; // Swap x and y
                }
            }
        }
        ROS_INFO("Loaded %lu points", msg->data.size());
        publishLaneMarkers();
    }

    // 优化后的相邻车道搜索逻辑
    void searchNearbyLanes(const double& current_x, const double& current_y, 
                      const int current_lane_id,
                      double& min_distance, int& closest_lane_id)
    {
        std::vector<int> lanes_to_check;
        
        // 将当前车道加入检查列表
        lanes_to_check.push_back(current_lane_id);
        
        // 将相邻车道加入检查列表
        if (lane_connections_.find(current_lane_id) != lane_connections_.end()) {
            const auto& connected_lanes = lane_connections_[current_lane_id];
            lanes_to_check.insert(lanes_to_check.end(), 
                                connected_lanes.begin(), 
                                connected_lanes.end());
        }

        // 只检查相邻车道
        for (int lane_id : lanes_to_check)
        {
            if (lanes_.find(lane_id) == lanes_.end()) continue;
            
            const auto& lane_data = lanes_[lane_id];
            if (!lane_data.start_coords.empty() && !lane_data.end_coords.empty())
            {
                double start_distance = calculateDistance(current_x, current_y,
                                                       lane_data.start_coords[0],
                                                       lane_data.start_coords[1]);
                if (start_distance < min_distance)
                {
                    min_distance = start_distance;
                    closest_lane_id = lane_id;
                }
            }
        }
    }

    // 处理位置更新的主要逻辑
    void handlePoseUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg, 
                        bool& is_sweep)
    {
        double current_x = msg->pose.position.x;
        double current_y = msg->pose.position.y;

        // 检查是否需要重新搜索
        if (shouldPerformNewSearch(current_x, current_y))
        {
            performLaneSearch(current_x, current_y, is_sweep);
        }
        else
        {
            // 使用缓存的结果
            useCachedResult(is_sweep);
        }
    }

    // 检查是否需要执行新的搜索
    bool shouldPerformNewSearch(double current_x, double current_y) const
    {
        if (!last_pose_.valid)
            return true;

        double move_distance = calculateDistance(current_x, current_y, 
                                              last_pose_.x, last_pose_.y);
        return move_distance >= search_threshold_;
    }

    // 执行车道搜索
    void performLaneSearch(double current_x, double current_y, bool& is_sweep)
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_lane_id = -1;

        // 1. 先搜索相邻车道
        if (last_pose_.valid)
        {
            searchNearbyLanes(current_x, current_y, last_pose_.lane_id,
                            min_distance, closest_lane_id);
            if (closest_lane_id != -1 && min_distance < search_threshold_)
            {
                updateLastPose(current_x, current_y, closest_lane_id, min_distance);
                if(lanes_.at(closest_lane_id).issweep == 1) {
                    is_sweep = true;
                }
                return;
            }
        }

        // 2. 如果相邻车道搜索失败，执行全局搜索
        performGlobalSearch(current_x, current_y, min_distance, closest_lane_id, is_sweep);
    }

    // 执行全局车道搜索
    void performGlobalSearch(double current_x, double current_y,
                           double& min_distance, int& closest_lane_id,
                           bool& is_sweep)
    {
        for (const auto &lane : lanes_)
        {
            const auto &lane_data = lane.second;
            if (!lane_data.start_coords.empty() && !lane_data.end_coords.empty())
            {
                double distance = calculateDistance(current_x, current_y,
                                                 lane_data.start_coords[0],
                                                 lane_data.start_coords[1]);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_lane_id = lane_data.lnid;
                }
            }
        }

        if (closest_lane_id != -1)
        {
            updateLastPose(current_x, current_y, closest_lane_id, min_distance);
            if(lanes_.at(closest_lane_id).issweep == 1) {
                is_sweep = true;
            }
        }
        else
        {
            last_pose_.valid = false;
            ROS_WARN("No valid lane found in global search");
        }
    }

    // 更新上次位置信息
    void updateLastPose(double current_x, double current_y, 
                       int lane_id, double distance)
    {
        last_pose_.x = current_x;
        last_pose_.y = current_y;
        last_pose_.lane_id = lane_id;
        last_pose_.distance = distance;
        last_pose_.valid = true;

        const auto &lane_data = lanes_[lane_id];
        ROS_INFO("Updated position - Lane ID: %d, distance: %.2f m, issweep: %d",
                 lane_id, distance, lane_data.issweep);
    }

    // 使用缓存的结果
    void useCachedResult(bool &is_sweep) const
    {
        const auto& lane_data = lanes_.at(last_pose_.lane_id);
        ROS_INFO("Using cached result - Lane ID: %d, issweep: %d, distance: %.2f m",
                 last_pose_.lane_id, lane_data.issweep, last_pose_.distance);
        if (lane_data.issweep == 1) {
            is_sweep = true;
        }
    }

    void sweepUpdate(bool is_sweep)
    {
        if(is_sweep != sweep_status_) {
            std_msgs::Bool msg;
            msg.data = is_sweep;
            sweep_pub_.publish(msg);
            sweep_status_ = is_sweep;
            ROS_INFO("Sweep status changed to: %s", is_sweep ? "ON" : "OFF");
        }
    }


    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        try {
            if (!enable_auto_sweep_) {
                return;
            }

            if (!msg) {
                ROS_WARN("Received null pose message");
                return;
            }
            
            if (lanes_.empty()) {
                ROS_WARN_THROTTLE(1, "No lanes data available");
                return;
            }

            ros::Time start_time = ros::Time::now();
            
            bool is_sweep = false;

            // 执行主要搜索逻辑
            handlePoseUpdate(msg, is_sweep);

            // 更新清扫状态
            sweepUpdate(is_sweep);

            // 记录处理时间
            ros::Duration processing_time = ros::Time::now() - start_time;
            ROS_DEBUG("Pose processing time: %.3f ms", 
                     processing_time.toSec() * 1000);

        } catch (const std::exception& e) {
            ROS_ERROR("Error in poseCallback: %s", e.what());
        }
    }

    void publishLaneMarkers() const
    {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for (const auto &lane : lanes_)
        {
            const auto &lane_data = lane.second;
            if (!lane_data.start_coords.empty() && !lane_data.end_coords.empty())
            {
                // Line marker
                visualization_msgs::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = ros::Time::now();
                line_marker.ns = "lanes";
                line_marker.id = id++;
                line_marker.type = visualization_msgs::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::Marker::ADD;
                line_marker.scale.x = 0.1;

                if (lane_data.issweep == 1) {
                    line_marker.color.r = 0.0;
                    line_marker.color.g = 0.0;
                    line_marker.color.b = 1.0;
                } else {
                    line_marker.color.r = 1.0;
                    line_marker.color.g = 0.0;
                    line_marker.color.b = 0.0;
                }
                line_marker.color.a = 1.0;

                geometry_msgs::Point start_point;
                start_point.x = lane_data.start_coords[0];
                start_point.y = lane_data.start_coords[1];
                start_point.z = 0.0;
                line_marker.points.push_back(start_point);

                geometry_msgs::Point end_point;
                end_point.x = lane_data.end_coords[0];
                end_point.y = lane_data.end_coords[1];
                end_point.z = 0.0;
                line_marker.points.push_back(end_point);

                marker_array.markers.push_back(line_marker);

                // Text marker
                visualization_msgs::Marker text_marker;
                text_marker.header.frame_id = "map";
                text_marker.header.stamp = ros::Time::now();
                text_marker.ns = "lane_ids";
                text_marker.id = id++;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;
                text_marker.scale.z = 0.5;
                text_marker.color.r = 0.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 0.0;
                text_marker.color.a = 1.0;
                text_marker.text = std::to_string(lane_data.lnid);

                text_marker.pose.position.x = (start_point.x + end_point.x) / 2.0;
                text_marker.pose.position.y = (start_point.y + end_point.y) / 2.0;
                text_marker.pose.position.z = 0.5;

                marker_array.markers.push_back(text_marker);
            }
        }

        marker_pub_.publish(marker_array);
        ROS_INFO("Published %lu markers", marker_array.markers.size());
    }

    double calculateDistance(double x1, double y1, double x2, double y2) const
    {
        return std::hypot(x2 - x1, y2 - y1);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_sweep_node");
    AutoSweep auto_sweep_node;
    ros::spin();
    return 0;
}