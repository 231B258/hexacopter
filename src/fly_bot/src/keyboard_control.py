#!/usr/bin/env python

import rospy
import sys, select, termios, tty
from std_msgs.msg import Float64MultiArray

class KeyboardControl:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.throttle = 50.0  # Base throttle value (0-100)
        self.roll = 0.0        # Roll control (-100 to 100)
        self.pitch = 0.0       # Pitch control (-100 to 100)
        self.yaw = 0.0         # Yaw control (-100 to 100)
        self.control_pub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=1)
        self.msg = """
        Hexacopter Keyboard Control
        ---------------------------
        Controls:
          'w' : Increase throttle
          's' : Decrease throttle
          'a' : Roll left
          'd' : Roll right
          'i' : Pitch forward
          'k' : Pitch backward
          'j' : Yaw left
          'l' : Yaw right
          ' ' : Reset all controls
          'q' : Quit
        """
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_motor_commands(self):
        f = Float64MultiArray()
        
        # Calculate motor velocities using hexacopter mixing
        esc_br = 1500 + self.throttle + self.pitch + 0.5*self.roll + self.yaw
        esc_fr = 1500 + self.throttle - self.pitch + 0.5*self.roll + self.yaw
        esc_fl = 1500 + self.throttle - self.pitch - 0.5*self.roll - self.yaw
        esc_bl = 1500 + self.throttle + self.pitch - 0.5*self.roll - self.yaw
        esc_r = 1500 + self.throttle + self.roll - self.yaw
        esc_l = 1500 + self.throttle - self.roll + self.yaw
        
        # Convert ESC values to motor velocities
        br_motor_vel = ((esc_br - 1500)/25) + 50
        bl_motor_vel = ((esc_bl - 1500)/25) + 50
        fr_motor_vel = ((esc_fr - 1500)/25) + 50
        fl_motor_vel = ((esc_fl - 1500)/25) + 50
        r_motor_vel = ((esc_r - 1500)/25) + 50
        l_motor_vel = ((esc_l - 1500)/25) + 50
        
        # Apply motor directions (match your PID script)
        f.data = [fr_motor_vel, -fl_motor_vel, l_motor_vel, -bl_motor_vel, br_motor_vel, -r_motor_vel]
        self.control_pub.publish(f)

    def run(self):
        print(self.msg)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # Throttle control
                if key == 'w':
                    self.throttle = min(self.throttle + 2.0, 100.0)
                elif key == 's':
                    self.throttle = max(self.throttle - 2.0, 0.0)
                
                # Roll control
                elif key == 'a':
                    self.roll = max(self.roll - 5.0, -100.0)
                elif key == 'd':
                    self.roll = min(self.roll + 5.0, 100.0)
                
                # Pitch control
                elif key == 'i':
                    self.pitch = min(self.pitch + 5.0, 100.0)
                elif key == 'k':
                    self.pitch = max(self.pitch - 5.0, -100.0)
                
                # Yaw control
                elif key == 'j':
                    self.yaw = max(self.yaw - 5.0, -100.0)
                elif key == 'l':
                    self.yaw = min(self.yaw + 5.0, 100.0)
                
                # Reset controls
                elif key == ' ':
                    self.throttle = 50.0
                    self.roll = 0.0
                    self.pitch = 0.0
                    self.yaw = 0.0
                
                # Quit
                elif key == 'q':
                    break
                else:
                    if key != '':
                        print(f"Invalid key: {key}")
                
                # Print current values
                print(f"Throttle: {self.throttle:.1f} | Roll: {self.roll:.1f} | Pitch: {self.pitch:.1f} | Yaw: {self.yaw:.1f}")
                
                # Publish motor commands
                self.publish_motor_commands()
                
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node("keyboard_control")
    controller = KeyboardControl()
    controller.run()