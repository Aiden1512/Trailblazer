#ifndef TRAIL_DECISION_MAKING__TRAIL_GRAPH_H_
#define TRAIL_DECISION_MAKING__TRAIL_GRAPH_H_

#include <cstddef>
#include <string>
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
        int junction_id = -1; // id from the YAML junctions list, -1 for a trail dead end
        Waypoint position;
    };

    struct TrailEdge // from_node -> to_node, with waypoints in between
    {
        int id = -1; // index into TrailGraph::edges_
        int trail_id = -1; // "id" of the trail in the YAML file (what the GUI shows), -1 if it has none
        int from_node = -1;
        int to_node = -1;
        std::string trail_name;
        std::vector<Waypoint> waypoints; // ordered from from_node to to_node
        EdgeStatus status{EdgeStatus::UNINSPECTED};
        // BLOCKED part way through inspecting it: the end it still has to be inspected from, up to the
        // obstacle (-1: nothing left to check, e.g. checked from both ends)
        int far_side_node = -1;
        // BLOCKED from both ends: the stretch between the two obstacles, which the robot can't reach to
        // inspect from either side (empty if there is none)
        std::vector<Waypoint> inaccessible;
    };

    class TrailGraph
    {
    public:
        // Load a trail file (path_recorder's junction_nodes.yaml format). Returns false on failure.
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

        // Share of trail checked, 0.0 if graph is empty: inspected and fully checked blocked edges count 1,
        // blocked edges still to be checked from their far side count 0.5
        double explorationPercentage() const;

        void setFarSideNode(int edge_id, int node_id);
        size_t pendingFarSides() const; // blocked edges whose far side hasn't been checked yet

        // Distance (m) along the edge's waypoints, from from_node, to the point on the edge closest to (x, y)
        double distanceAlongEdge(int edge_id, double x, double y) const;
        // The edge's waypoints between two distances along it (from from_node), the ends interpolated
        std::vector<Waypoint> edgeSection(int edge_id, double start, double end) const;
        void setInaccessibleSection(int edge_id, const std::vector<Waypoint> &section);
        size_t countInaccessibleSections() const;

        size_t countEdges(EdgeStatus status) const;

    private:
        std::vector<TrailNode> nodes_;
        std::vector<TrailEdge> edges_;

        // Reuse an existing node within a tolerance, otherwise create a new one
        int getOrCreateNode(double x, double y, double yaw);
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__TRAIL_GRAPH_H_