"""
Shared data structures for the TrailBlazer GUI.

These classes hold the state that both the Phase 1 (teleop mapping) and
Phase 2 (autonomous inspection) tabs read from and write to, so a trail
mapped in Phase 1 can be handed straight over to Phase 2.
"""
import math


class Node:
    """A single recorded point on the trail (a waypoint or a junction)."""

    _next_id = 1

    def __init__(self, x, y, kind="waypoint"):
        self.id = Node._next_id
        Node._next_id += 1
        self.order = self.id           # creation order, shown on the map
        self.x = x
        self.y = y
        self.kind = kind                # "waypoint" or "junction"

    def pos(self):
        return (self.x, self.y)


class Edge:
    """A trail segment between two consecutively created nodes."""

    STATUS_COLORS = {
        "unvisited": "#111111",   # black
        "clear": "#2e8b32",       # green
        "blocked": "#c0392b",     # red
    }

    def __init__(self, node_a: Node, node_b: Node):
        self.node_a = node_a
        self.node_b = node_b
        self.status = "unvisited"

    def color(self):
        return self.STATUS_COLORS[self.status]

    def distance_to_point(self, px, py):
        """Perpendicular distance from (px, py) to this segment, used to
        test whether a placed obstacle actually blocks this route."""
        ax, ay = self.node_a.x, self.node_a.y
        bx, by = self.node_b.x, self.node_b.y
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        closest_x, closest_y = ax + t * dx, ay + t * dy
        return math.hypot(px - closest_x, py - closest_y)


class TrailState:
    """The single source of truth shared across both GUI phases."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.trail_points = []   # fine-grained (x, y) samples of the travelled path
        self.nodes = []          # list[Node], in creation order
        self.edges = []          # list[Edge], consecutive-node segments
        self.obstacles = []      # list[(x, y)] placed for the Phase 2 simulation
        Node._next_id = 1

    def add_node(self, x, y, kind="waypoint"):
        node = Node(x, y, kind=kind)
        self.nodes.append(node)
        if len(self.nodes) >= 2:
            self.edges.append(Edge(self.nodes[-2], self.nodes[-1]))
        return node

    def build_edges_from_nodes(self):
        """(Re)build the ordered edge list from self.nodes."""
        self.edges = [Edge(a, b) for a, b in zip(self.nodes, self.nodes[1:])]

    def exploration_percentage(self):
        if not self.edges:
            return 0.0
        done = sum(1 for e in self.edges if e.status != "unvisited")
        return 100.0 * done / len(self.edges)
