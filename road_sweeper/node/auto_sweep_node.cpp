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

class AutoSweep
{
public:
    AutoSweep()
    {
        lane_sub_ = nh_.subscribe("/vector_map_info/lane", 10, &AutoSweep::laneCallback, this);
        node_sub_ = nh_.subscribe("/vector_map_info/node", 10, &AutoSweep::nodeCallback, this);
        point_sub_ = nh_.subscribe("/vector_map_info/point", 10, &AutoSweep::pointCallback, this);
        pose_sub_ = nh_.subscribe("/current_pose", 10, &AutoSweep::poseCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_markers", 10);
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

    ros::NodeHandle nh_;
    ros::Subscriber lane_sub_;
    ros::Subscriber node_sub_;
    ros::Subscriber point_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer enable_service_;

    std::unordered_map<int, LaneData> lanes_;
    std::unordered_map<int, int> nodes_;

    bool enable_auto_sweep_ = true;

    bool enableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enable_auto_sweep_ = req.data;
        res.success = true;
        res.message = enable_auto_sweep_ ? "Auto sweep enabled" : "Auto sweep disabled";
        ROS_INFO("%s", res.message.c_str());
        return true;
    }

    void laneCallback(const vector_map_msgs::LaneArray::ConstPtr &msg)
    {
        for (const auto &lane : msg->data)
        {
            LaneData lane_data;
            lane_data.lnid = lane.lnid;
            lane_data.bnid = lane.bnid;
            lane_data.fnid = lane.fnid;
            lane_data.issweep = lane.issweep;
            lanes_[lane.lnid] = lane_data;
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

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!enable_auto_sweep_)
        {
            return;
        }

        double min_distance = std::numeric_limits<double>::max();
        int closest_lane_id = -1;
        int closest_issweep = -1;
        std::vector<double> closest_start_coords;
        std::vector<double> closest_end_coords;

        for (const auto &lane : lanes_)
        {
            const auto &lane_data = lane.second;
            if (!lane_data.start_coords.empty() && !lane_data.end_coords.empty())
            {
                double distance = calculateDistance(msg->pose.position.x, msg->pose.position.y, lane_data.start_coords[0], lane_data.start_coords[1]);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_lane_id = lane_data.lnid;
                    closest_issweep = lane_data.issweep;
                    closest_start_coords = lane_data.start_coords;
                    closest_end_coords = lane_data.end_coords;
                }
            }
        }

        if (closest_lane_id != -1)
        {
            ROS_INFO("Closest Lane ID: %d, issweep: %d, distance: %f", closest_lane_id, closest_issweep, min_distance);
            ROS_INFO("Current Pose: x: %f, y: %f\n Lane start: x: %f, y: %f; end: x: %f, y: %f", msg->pose.position.x, msg->pose.position.y, closest_start_coords[0], closest_start_coords[1], closest_end_coords[0], closest_end_coords[1]);
        }
        else
        {
            ROS_WARN("No valid lane found.");
        }
    }

    void publishLaneMarkers()
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
                line_marker.color.r = 1.0;
                line_marker.color.g = 0.0;
                line_marker.color.b = 0.0;
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

    double calculateDistance(double x1, double y1, double x2, double y2)
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