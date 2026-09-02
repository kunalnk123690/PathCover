#!/usr/bin/env python3
import numpy as np
from pynput import keyboard
import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import TrajectoryCommand


class UnifiedKeyboardController(Node):
    def __init__(self):
        super().__init__('unified_keyboard_controller')

        # --- Parameters ---
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('start_z', 1.0)
        self.declare_parameter('horizontal_speed', 0.8)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        start_pos_z = float(self.get_parameter('start_z').value)
        self.vlocal = float(self.get_parameter('horizontal_speed').value)

        # --- State Variables ---
        self.target_pose = [0.0, 0.0, start_pos_z, 0.0]  # [x, y, z, yaw]
        self.commanded_velocity = [0.0, 0.0, 0.0, 0.0]   # [vx_w, vy_w, vz, vyaw]
        self.keys_pressed = set()

        # Key mapping for vertical and yaw control
        self.key_map = {
            'i': ('vz', 0.8),    # Rise
            'k': ('vz', -0.8),   # Descend
            'j': ('vyaw', 0.8),  # Turn Left
            'l': ('vyaw', -0.8)  # Turn Right
        }

        # Publisher
        self.trajectory_pub = self.create_publisher(TrajectoryCommand, '/quadrotor/trajectory', 10)

        self.print_instructions()

        # Keyboard listener runs in its own thread
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Timer for publishing at publish_rate
        self.dt = 1.0 / max(self.publish_rate, 1e-3)
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.publish_command)

    # --- Input handling ---
    def print_instructions(self):
        print("""
        Unified Drone Keyboard Controller (ROS 2)
        -----------------------------------------
        Move relative to current yaw:
           W              (forward)       
        A  S  D    (strafe left/back/right)

        Up/Down: I/K
        Turn(left/right): J/L

        Commands:
        H : Hover (stops all movement)
        ESC : Quit
        """)

    def on_press(self, key):
        try:
            char_key = key.char.lower()

            # Horizontal keys
            if char_key in ('w', 's', 'a', 'd'):
                self.keys_pressed.add(char_key)

            # Vertical & yaw controls
            elif char_key in self.key_map:
                v_type, value = self.key_map[char_key]
                if v_type == 'vz':
                    self.commanded_velocity[2] = value
                elif v_type == 'vyaw':
                    self.commanded_velocity[3] = value

            # Hover
            elif char_key == 'h':
                self.keys_pressed.clear()
                self.commanded_velocity = [0.0, 0.0, 0.0, 0.0]

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            char_key = key.char.lower()

            if char_key in ('w', 's', 'a', 'd') and char_key in self.keys_pressed:
                self.keys_pressed.remove(char_key)

            elif char_key in self.key_map:
                v_type, _ = self.key_map[char_key]
                if v_type == 'vz':
                    self.commanded_velocity[2] = 0.0
                elif v_type == 'vyaw':
                    self.commanded_velocity[3] = 0.0

        except AttributeError:
            # ESC to shutdown
            if key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed: shutting down.')
                # Stop listener and shutdown ROS gracefully
                try:
                    self.listener.stop()
                except Exception:
                    pass
                rclpy.shutdown()
                return False

    # --- Control/publish loop ---
    def publish_command(self):
        # Compute dt from clock to avoid timer jitter
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:   # guard if debugger pauses or first tick
            dt = self.dt
        self.last_time = now

        # Compute horizontal world velocities based on keys pressed and current yaw
        vx_local = 0.0
        vy_local = 0.0
        for k in self.keys_pressed:
            if k == 'w':
                vx_local += self.vlocal
            elif k == 's':
                vx_local -= self.vlocal
            elif k == 'a':
                vy_local += self.vlocal
            elif k == 'd':
                vy_local -= self.vlocal

        yaw = self.target_pose[3]

        # Transform local -> world
        self.commanded_velocity[0] = np.cos(yaw) * vx_local - np.sin(yaw) * vy_local
        self.commanded_velocity[1] = np.sin(yaw) * vx_local + np.cos(yaw) * vy_local

        # Integrate to update target pose
        self.target_pose[0] += self.commanded_velocity[0] * dt
        self.target_pose[1] += self.commanded_velocity[1] * dt
        self.target_pose[2] += self.commanded_velocity[2] * dt
        self.target_pose[3] += self.commanded_velocity[3] * dt

        # Optional: wrap yaw to [-pi, pi] to avoid drift
        self.target_pose[3] = (self.target_pose[3] + np.pi) % (2.0 * np.pi) - np.pi

        # Construct and publish TrajectoryCommand
        msg = TrajectoryCommand()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'

        msg.position.x = float(self.target_pose[0])
        msg.position.y = float(self.target_pose[1])
        msg.position.z = float(self.target_pose[2])

        msg.velocity.x = float(self.commanded_velocity[0])
        msg.velocity.y = float(self.commanded_velocity[1])
        msg.velocity.z = float(self.commanded_velocity[2])

        msg.yaw = float(self.target_pose[3])
        msg.yaw_velocity = float(self.commanded_velocity[3])

        self.trajectory_pub.publish(msg)


def main():
    rclpy.init()
    node = UnifiedKeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop keyboard listener on shutdown
        try:
            node.listener.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
