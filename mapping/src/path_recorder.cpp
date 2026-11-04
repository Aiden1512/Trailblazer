
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>

struct JunctionNode
{   
    int id;
    double x;
    double y;
    double yaw;
};

struct TrailWaypoint{
    double x;
    double y;
    double yaw;
};

struct Trail{
    int id;
    int start_junction;
    int end_junction = -1;

    std::vector<TrailWaypoint> waypoints;
};


class JunctionRecorder : public rclcpp::Node
{
public:
    JunctionRecorder()
        : Node("junction_recorder")
    {
    
        
        this->declare_parameter<double>("detection_radius", 1.5);
        this->declare_parameter<std::string>("save_file", "junction_nodes.yaml");
        this->declare_parameter<double>("waypoint_spacing", 0.7);

        detection_radius_ = this->get_parameter("detection_radius").as_double();
        save_file_ = this->get_parameter("save_file").as_string();
        waypoint_spacing_ = this->get_parameter("waypoint_spacing").as_double();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/husky1/pose", 10, std::bind(&JunctionRecorder::poseCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/husky1/map", 10, std::bind(&JunctionRecorder::mapCallback, this, std::placeholders::_1));
        ray_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/husky1/junction_rays", 10);
        graph_pub_ = this->create_publisher<std_msgs::msg::String>("/husky1/trail_graph", 10);
        reset_service_ = this->create_service<std_srvs::srv::Trigger>("/husky1/reset_trail_graph", [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
                {
                    (void)request;

                    // Clear and reset all 
                    nodes_.clear();
                    trails_.clear();
                    next_junction_id_ = 1;
                    next_trail_id_ = 1;
                    current_trail_index_ = -1;
                    active_junction_id_ = -1;
                    consecutive_detections_ = 0;
                    has_last_waypoint_ = false;
                    last_waypoint_x_ = 0.0;
                    last_waypoint_y_ = 0.0;
                    recording_enabled_ = false;

                    // Save empty graph and publish to GUI
                    saveNodes();

                    RCLCPP_INFO(this->get_logger(), "Trail graph reset");

                    response->success = true;
                    response->message = "Trail graph reset";
                });
        start_service_ = this->create_service<std_srvs::srv::Trigger>("/husky1/start_trail_recording",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            recording_enabled_ = true;
            response->success = true;
            response->message = "Recording started";
            RCLCPP_INFO(this->get_logger(), "Recording started");
        });
        // Stops adding junctions and waypoints (e.g. once Phase 1 is finished); the graph so far is kept and saved
        stop_service_ = this->create_service<std_srvs::srv::Trigger>("/husky1/stop_trail_recording",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            recording_enabled_ = false;
            saveNodes();
            response->success = true;
            response->message = "Recording stopped";
            RCLCPP_INFO(this->get_logger(), "Recording stopped");
        });
    }

    ~JunctionRecorder()
    {
        saveNodes();
    }


private:

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ray_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::vector<JunctionNode> nodes_;

    double detection_radius_; //How far from the robot to check for open paths
    double angle_resolution_deg_ = 5.0; //Angle frequency of checks for open paths 
    double ray_step_ = 0.10; //Distance between samples along each ray
    double minimum_opening_deg_ = 25.0; //Minimum size of an opening
    double minimum_node_distance_ = 1.5; //Minimum distance between nodes
    int confirmation_required_ = 3; //Number of consecutive detections required to be considered a node
    int occupied_threshold_ = 50; //How confident before consider occupancy map occupied
    int consecutive_detections_ = 0;
    std::string save_file_;

    std::vector<Trail> trails_; 
    int next_trail_id_ = 1; //next unused trail id
    int next_junction_id_ = 1; //next unused junction id
    int current_trail_index_ = -1;
    int active_junction_id_ = -1;
    double waypoint_spacing_ = 0.7; //distance we want to know when to place next waypoint (0.5 m so Phase 2 follows curves, not straight cuts across them)
    double junction_exit_distance_ = 1.8; //distance from a junction before it can trigger again
    double last_waypoint_x_ = 0.0; //last recorded waypoint position
    double last_waypoint_y_ = 0.0; //last recorded waypoint position
    bool has_last_waypoint_ = false; // if there is a waypoint reference position
    bool recording_enabled_ = false;
    
    

   
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {   if (!recording_enabled_) {return;}
        if (!latest_map_){return;}

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = getYaw(msg->pose.pose.orientation);

        if (trails_.empty())
        {
            Trail trail;
            trail.id = next_trail_id_++;
            trail.start_junction = -1;  // No junction at the starting position
            trails_.push_back(trail);

            current_trail_index_ = 0;

            last_waypoint_x_ = x;
            last_waypoint_y_ = y;
            has_last_waypoint_ = true;

            trails_[0].waypoints.push_back({x, y, yaw});

            saveNodes();
        }


        int openings = detectOpenings(x, y);
        checkJunctionExit(x, y);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Open directions detected: %d", openings);

        // We only care about junctions.
        if (openings >= 3)
        {
            consecutive_detections_++;
            RCLCPP_INFO(this->get_logger(), "Possible junction: %d openings (%d/%d)", openings, consecutive_detections_, confirmation_required_);
        }
        else
        {
            consecutive_detections_ = 0;
        }

        // Junction must be detected repeatedly.
        if (consecutive_detections_ >= confirmation_required_){

            if (active_junction_id_ == -1){
                handleJunction(x, y, yaw);
            }

            consecutive_detections_ = 0;
            }
        recordTrailWaypoint(x, y, yaw);
    }

   int detectOpenings(double robot_x, double robot_y)
{
    int number_of_rays =
        static_cast<int>(360.0 / angle_resolution_deg_);

    std::vector<bool> ray_is_open(number_of_rays, false);

    // Marker containing all of our rays
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "husky1_map";
    marker.header.stamp = this->now();

    marker.ns = "junction_detection";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;

  
    for (int i = 0; i < number_of_rays; i++)
    {
        double angle_deg = i * angle_resolution_deg_;

        double angle_rad = angle_deg * M_PI / 180.0;

        bool open = isDirectionOpen(robot_x, robot_y, angle_rad);
        ray_is_open.at(i) = open;

        // Start of ray
        geometry_msgs::msg::Point start;

        start.x = robot_x;
        start.y = robot_y;
        start.z = 0.1;

        // End of ray
        geometry_msgs::msg::Point end;

        end.x = robot_x + detection_radius_ * std::cos(angle_rad);
        end.y = robot_y + detection_radius_ * std::sin(angle_rad);
        end.z = 0.1;

        std_msgs::msg::ColorRGBA colour;

        if (open)
        {
            // Green, algorithm thinks this direction is open
            colour.r = 0.0;
            colour.g = 1.0;
            colour.b = 0.0;
            colour.a = 1.0;
        }
        else
        {
            // Red, algorithm thinks this direction is blocked
            colour.r = 1.0;
            colour.g = 0.0;
            colour.b = 0.0;
            colour.a = 1.0;
        }

        marker.colors.push_back(colour);
        marker.colors.push_back(colour);

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    ray_marker_pub_->publish(marker);

    return countOpenSectors(ray_is_open);
}

bool isDirectionOpen(double robot_x, double robot_y, double angle)
{
    for (double distance = ray_step_; distance <= detection_radius_; distance += ray_step_)
    {
        double world_x = robot_x + distance * std::cos(angle);
        double world_y = robot_y + distance * std::sin(angle);

        int grid_x;
        int grid_y;

        if (!worldToGrid(world_x, world_y, grid_x, grid_y))
        {
            return false;
        }

        int index = grid_y * latest_map_->info.width + grid_x;
        int8_t value = latest_map_->data[index];

        // Unknown map cell
        //if (value == -1)
        //{
        //    return false;
        //}

        // Occupied map cell
        if (value >= occupied_threshold_)
        {
            return false;
        }
    }

    return true;
}
    



   
    int countOpenSectors(const std::vector<bool> &open_rays)
    {
        if (open_rays.empty()) {return 0;}

        int minimum_rays = static_cast<int>(std::ceil(minimum_opening_deg_ /angle_resolution_deg_ ));


        // Considering that angle directions form circle, its easiest to start with a known blocked angle index first
        int blocked_index = -1;

        for (size_t i = 0; i < open_rays.size(); i++)
        {
            if (!open_rays[i])
            {
                blocked_index = static_cast<int>(i);
                break;
            }
        }

        
        if (blocked_index == -1)
        {
            return 1; // Everything around the robot is open so treat it as one large open region rather than many branches.
        }

        int openings = 0;
        int current_open_length = 0;
        int size = static_cast<int>(open_rays.size());

        for (int offset = 1; offset <= size; offset++)
        {
            int index = (blocked_index + offset) % size;

            if (open_rays.at(index))
            {
                current_open_length++;
            }
            else
            {
                if (current_open_length >= minimum_rays)
                {
                    openings++;
                }

                current_open_length = 0;
            }
        }

        // Deal with a sector ending at the final checked ray.
        if (current_open_length >= minimum_rays)
        {
            openings++;
        }


        return openings;
    }

    bool worldToGrid(double world_x, double world_y, int &grid_x, int &grid_y)
    {
        double origin_x = latest_map_->info.origin.position.x;
        double origin_y = latest_map_->info.origin.position.y;
        double resolution = latest_map_->info.resolution;


        grid_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
        grid_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));

        if (grid_x < 0 || grid_y < 0 || grid_x >=static_cast<int>(latest_map_->info.width) || grid_y >= static_cast<int>(latest_map_->info.height))
        {
            return false;
        }

        return true;
    }


    bool nodeNearby(double x, double y)
    {
        for (const auto &node : nodes_)
        {
            double dx = x - node.x;
            double dy = y - node.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < minimum_node_distance_)
            {
                return true;
            }
        }
        return false;
    }



    int findExistingJunction(double x, double y)
    {
        for (const auto &node : nodes_)
        {
            double dx = x - node.x;
            double dy = y - node.y;

            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < minimum_node_distance_)
            {
                return node.id;
            }
        }

        return -1; //No existing junction nearby
    }

    const JunctionNode* getJunction(int id)
    {
        for (const auto &node : nodes_)
        {
            if (node.id == id)
            {
                return &node;
            }
        }

        return nullptr;
    }


    void startNewTrail(int junction_id)
    {
        Trail trail;
        trail.id = next_trail_id_++;
        trail.start_junction = junction_id;
        trails_.push_back(trail);

        current_trail_index_ = static_cast<int>(trails_.size()) - 1;

        // wait until the robot leaves the junction before recording waypoints.
        has_last_waypoint_ = false;

        RCLCPP_INFO(this->get_logger(), "Started Trail %d at Junction %d", trail.id, junction_id);
        saveNodes();
    }

    void finishCurrentTrail(int junction_id)
    {
        if (current_trail_index_ == -1)
        {
            return;
        }

        Trail &trail = trails_[current_trail_index_];
        trail.end_junction = junction_id;
        RCLCPP_INFO(this->get_logger(), "Finished Trail %d at Junction %d", trail.id, junction_id);
        current_trail_index_ = -1;

        saveNodes();
    }
    void recordTrailWaypoint(double x, double y, double yaw)
    {
        if (current_trail_index_ == -1)
        {
            return;
        }
        
        // Keep recording inside the junction zone too: the junction is saved on the approach (its rays see the
        // openings ~2 m early), so the turn itself happens in the zone. Skipping it left a ~2 m straight line from
        // the junction to the first waypoint that cut the corner in Phase 2 (and clipped trees).

        if (!has_last_waypoint_)
        {
            const Trail &trail = trails_[current_trail_index_];

            if (trail.start_junction != -1)
            {
                const JunctionNode* start = getJunction(trail.start_junction);

                if (start){
                    last_waypoint_x_ = start->x;
                    last_waypoint_y_ = start->y;
                }
                else{
                    last_waypoint_x_ = x;
                    last_waypoint_y_ = y;
                }
            }
            else{
                last_waypoint_x_ = x;
                last_waypoint_y_ = y;
            }

            has_last_waypoint_ = true;
        }

        double dx = x - last_waypoint_x_;
        double dy = y - last_waypoint_y_;

        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance >= waypoint_spacing_)
        {
            TrailWaypoint waypoint;
            waypoint.x = x;
            waypoint.y = y;
            waypoint.yaw = yaw;
            trails_[current_trail_index_].waypoints.push_back(waypoint);
            last_waypoint_x_ = x;
            last_waypoint_y_ = y;

            RCLCPP_INFO(this->get_logger(), "Trail %d: Waypoint saved at x=%.2f y=%.2f", trails_[current_trail_index_].id, x, y);
            saveNodes();
        }
    }

    void handleJunction(double x, double y, double yaw)
    {
        int junction_id = findExistingJunction(x, y);

        if (junction_id == -1)
        {
            saveJunctionNode(x, y, yaw);
            junction_id = nodes_.back().id;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Revisited Junction %d", junction_id);
        }
        if (active_junction_id_ == junction_id) //so not constantly saving junction
        {
            return;
        }
        //end old trail and start new trail
        finishCurrentTrail(junction_id);
        startNewTrail(junction_id);
        active_junction_id_ = junction_id;
        saveNodes();
    }

    void checkJunctionExit(double x, double y)
    {
        if (active_junction_id_ == -1){
            return;
        }
        const JunctionNode* junction = getJunction(active_junction_id_);

        if (!junction){
            return;
        }

        double dx = x - junction->x;
        double dy = y - junction->y;

        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance >= junction_exit_distance_){

            active_junction_id_ = -1;
        }
    }

    void saveJunctionNode(double x, double y, double yaw)
    {
        JunctionNode node;
        node.id = next_junction_id_++;
        node.x = x;
        node.y = y;
        node.yaw = yaw;
        nodes_.push_back(node);

        RCLCPP_INFO(this->get_logger(),"JUNCTION SAVED"); 
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f y=%.3f", x, y);
        RCLCPP_INFO(this->get_logger(), "Junction ID: %d", node.id);

        saveNodes();
    }



void saveNodes()
{

    std::ofstream file(save_file_);

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open %s", save_file_.c_str());
        return;
    }

    file << "frame_id: husky1_map\n";
    file << "\njunctions:\n";

    for (const auto &node : nodes_)
    {
        file << "  - id: " << node.id << "\n";
        file << "    x: " << node.x << "\n";
        file << "    y: " << node.y << "\n";
        file << "    yaw: " << node.yaw << "\n";
    }

    file << "\ntrails:\n";
    for (const auto &trail : trails_)
    {
        file << "  - id: " << trail.id << "\n";

        file << "    start_junction: " << trail.start_junction << "\n";

        if (trail.end_junction == -1)
        {
            file << "    end_junction: null\n";
        }
        else
        {
            file << "    end_junction: " << trail.end_junction << "\n";
        }

        if (trail.waypoints.empty())
        {
            file << "    waypoints: []\n";
        }
        else
        {
            file << "    waypoints:\n";

            for (const auto &waypoint : trail.waypoints)
            {
                file << "      - x: " << waypoint.x << "\n";
                file << "        y: " << waypoint.y << "\n";
                file << "        yaw: " << waypoint.yaw << "\n";
            }
        }
    }

    file.close();

    RCLCPP_INFO( this->get_logger(),"Saved %zu junctions and %zu trails to %s", nodes_.size(), trails_.size(), save_file_.c_str());
    publishGraph();

    }



    double getYaw(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2.0 *(q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 *(q.y * q.y + q.z * q.z);

        return std::atan2(siny_cosp, cosy_cosp);
    }

    void publishGraph()
{
    std::ostringstream out;

    out << "{\"junctions\":[";

    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        if (i > 0) out << ",";

        out << "{\"id\":" << nodes_[i].id
            << ",\"x\":" << nodes_[i].x
            << ",\"y\":" << nodes_[i].y << "}";
    }

    out << "],\"trails\":[";

    for (size_t i = 0; i < trails_.size(); ++i)
    {
        if (i > 0) out << ",";

        const auto &trail = trails_[i];

        out << "{\"id\":" << trail.id
            << ",\"start_junction\":" << trail.start_junction
            << ",\"end_junction\":" << trail.end_junction
            << ",\"waypoints\":[";

        for (size_t j = 0; j < trail.waypoints.size(); ++j)
        {
            if (j > 0) out << ",";

            out << "{\"x\":" << trail.waypoints[j].x
                << ",\"y\":" << trail.waypoints[j].y << "}";
        }

        out << "]}";
    }

    out << "]}";

    std_msgs::msg::String msg;
    msg.data = out.str();
    graph_pub_->publish(msg);
}

};






int main(int argc,char * argv[])
{
    rclcpp::init(argc, argv);


    auto node = std::make_shared<JunctionRecorder>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}
