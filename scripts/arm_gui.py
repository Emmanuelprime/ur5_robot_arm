#!/usr/bin/env python3
"""
Simple Tkinter GUI to jog UR5 joints by publishing to
/joint_trajectory_controller/joint_trajectory

Usage:
    ros2 run <your_package> joint_jog_gui.py
    (or just: python3 joint_jog_gui.py, with ROS 2 environment sourced)
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Match this exactly to your robot's joint order (check with: ros2 topic echo /joint_states --once)
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Per-joint (min, max) limits in radians. Adjust to match your ur_type's joint_limits.yaml
JOINT_LIMITS = [
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
]

# Starting slider positions, matching config/initial_positions.yaml
INITIAL_POSITIONS = [
    0.0,    # shoulder_pan_joint
    -1.57,  # shoulder_lift_joint
    0.0,    # elbow_joint
    -1.57,  # wrist_1_joint
    0.0,    # wrist_2_joint
    0.0,    # wrist_3_joint
]

TOPIC = "/joint_trajectory_controller/joint_trajectory"
MOVE_TIME_SEC = 1.0  # time_from_start for each commanded move


class JointJogPublisher(Node):
    def __init__(self):
        super().__init__("joint_jog_gui")
        self.publisher = self.create_publisher(JointTrajectory, TOPIC, 10)

    def send_positions(self, positions):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(MOVE_TIME_SEC)
        nanosec = int((MOVE_TIME_SEC - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {positions}")


class JogGUI:
    def __init__(self, root, ros_node: JointJogPublisher):
        self.node = ros_node
        self.root = root
        self.root.title("UR5 Joint Jog")

        self.sliders = []
        self.value_labels = []

        for i, (name, (lo, hi)) in enumerate(zip(JOINT_NAMES, JOINT_LIMITS)):
            frame = ttk.Frame(root, padding=5)
            frame.grid(row=i, column=0, sticky="ew")

            label = ttk.Label(frame, text=name, width=20)
            label.grid(row=0, column=0, sticky="w")

            init_val = INITIAL_POSITIONS[i]

            # Create the value label FIRST and register it before the slider exists,
            # since ttk.Scale fires its command callback as soon as .set() is called below.
            value_label = ttk.Label(frame, text=f"{init_val:.3f}", width=8)
            value_label.grid(row=0, column=2)
            self.value_labels.append(value_label)

            slider = ttk.Scale(
                frame, from_=lo, to=hi, orient="horizontal",
                length=350, command=lambda val, idx=i: self.on_slider_change(idx, val)
            )
            slider.grid(row=0, column=1, padx=10)
            self.sliders.append(slider)
            slider.set(init_val)

        # Buttons
        btn_frame = ttk.Frame(root, padding=10)
        btn_frame.grid(row=len(JOINT_NAMES), column=0)

        send_btn = ttk.Button(btn_frame, text="Send Trajectory", command=self.send_current)
        send_btn.grid(row=0, column=0, padx=5)

        reset_btn = ttk.Button(btn_frame, text="Reset to Initial", command=self.reset_to_initial)
        reset_btn.grid(row=0, column=1, padx=5)

    def on_slider_change(self, idx, val):
        if idx < len(self.value_labels):
            self.value_labels[idx].config(text=f"{float(val):.3f}")

    def get_current_positions(self):
        return [slider.get() for slider in self.sliders]

    def send_current(self):
        positions = self.get_current_positions()
        self.node.send_positions(positions)

    def reset_to_initial(self):
        for slider, val in zip(self.sliders, INITIAL_POSITIONS):
            slider.set(val)
        self.send_current()


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as e:
        node.get_logger().warn(f"Spin thread exiting: {e}")


def main():
    rclpy.init()
    node = JointJogPublisher()

    # Spin ROS in a background thread so Tkinter's mainloop can own the main thread
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = JogGUI(root, node)

    def shutdown():
        """Cleanly tear down ROS and close the window, guarded against double-calls."""
        if getattr(shutdown, "_done", False):
            return
        shutdown._done = True

        node.get_logger().info("Shutting down joint_jog_gui...")

        # Stop rclpy.spin() in the background thread
        try:
            rclpy.shutdown()
        except Exception:
            pass

        # Wait for the spin thread to actually exit
        spin_thread.join(timeout=2.0)

        # Destroy the node explicitly (releases publishers/subscriptions)
        try:
            node.destroy_node()
        except Exception:
            pass

        root.destroy()

    # Handle window "X" close button
    root.protocol("WM_DELETE_WINDOW", shutdown)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()