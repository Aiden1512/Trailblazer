#ifndef TRAIL_DECISION_MAKING__TRAIL_GRAPH_HPP_
#define TRAIL_DECISION_MAKING__TRAIL_GRAPH_HPP_

#include <string>
#include <cmath>
#include <vector>

namespace trail_decision_making
{

    // Plain structs (no ROS types) so the graph can be unit tested without ROS.
    // Convert to geometry_msgs in the MissionManager.
    enum class EdgeStatus
    {
        UNINSPECTED,
        INSPECTED,
        BLOCKED
    };

    struct Waypoint // x, y, yaw in global frame
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct TrailNode
    {
        int id = -1; // index into TrailGraph::nodes_
        Waypoint position;
    };

    struct TrailEdge // from_node -> to_node, with waypoints in between
    {
        int id = -1; // index into TrailGraph::edges_
        int from_node = -1; 
        int to_node = -1;
        std::string trail_name;
        std::vector<Waypoint> waypoints; // ordered from from_node to to_node
        EdgeStatus status{EdgeStatus::UNINSPECTED}; 
    };

    struct ClosestEdgeResult
    {
        int edge_id = -1;
        double distance = std::numeric_limits<double>::max();
        double t = 0.0; // fraction along the *whole* edge, 0 at from_node, 1 at to_node
    };

    class TrailGraph
    {
    public:
        // Load the YAML file produced by the mapping team. Returns false on failure.
        bool loadFromYaml(const std::string &file_path);

        const std::vector<TrailNode> &nodes() const { return nodes_; }
        const std::vector<TrailEdge> &edges() const { return edges_; }

        // nullptr if the id does not exist
        const TrailEdge *getEdge(int edge_id) const;

        // All edges touching this node (trails are traversable in both directions)
        std::vector<int> getConnectedEdges(int node_id) const;

        void setEdgeStatus(int edge_id, EdgeStatus status);
        EdgeStatus getEdgeStatus(int edge_id) const;

        // Returns -1 if the graph is empty
        int findNearestNode(double x, double y) const;

        // (inspected + blocked) / total, 0.0 if graph is empty
        double explorationPercentage() const;

        void resetAllEdgesToUninspected();
        std::pair<double, double> pointToSegment(double px, double py, double ax, double ay, double bx, double by);

    private:
        std::vector<TrailNode> nodes_;
        std::vector<TrailEdge> edges_;

        // Reuse an existing node within a tolerance, otherwise create a new one
        int getOrCreateNode(double x, double y, double yaw);
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__TRAIL_GRAPH_HPP_