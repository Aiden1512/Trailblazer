# Trailblazer GUI

Tkinter operator GUI for the Trailblazer trail-inspection Husky. It is started by
`41068_ignition.launch.py`; see the top-level README for the full walkthrough.

- **Start screen** (`start_screen.py`): start a new simulation, or continue from the trail map saved in `saved_map.json`.
- **Phase 1 – Teleop mapping** (`phase1_teleop.py`): W/A/S/D driving while `path_recorder` records junctions and trails.
- **Phase 2 – Autonomous inspection** (`phase2_inspection.py`): start / e-stop / resume / return-home controls for
  `mission_manager`, the trail network coloured by inspection status, simulated obstacle placement, and the
  obstacle snapshot and live camera.

ROS access is kept out of the Tk widgets: `robot_pose.py`, `mission_status.py` and `camera_feed.py` each wrap a
small ROS node (sharing one context via `ros_context.py`) that the UI polls.
