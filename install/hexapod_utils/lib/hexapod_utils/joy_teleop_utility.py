#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
class JoyToFloat64MultiArray(Node):
    def __init__(self):
        super().__init__('joy_to_float64_multiarray')

        self.mode = 0

        self.linear_vel_x = 0.0
        self.linear_vel_y = 0.0
        self.angular_vel_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0

        self.gait = 0

        self.lin_vel_constant = 0.1
        self.angular_vel_constant = 1.0
        self.rpy_constant = 0.15
        self.xyz_constant = 0.025

        self.rpy_limit = 0.15
        self.xy_limit = 0.025
        self.z_limit_low = 0.0
        self.z_limit_high = 0.04

        # Subscribes to the /joy topic where joystick data is published
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publishes modified joystick data to the Float64MultiArray topic
        self.publisher_ = self.create_publisher(Float64MultiArray, 'hexapod_controller/joy_commands', 10)

    def joy_callback(self, joy_msg):
        # Prepare the Float64MultiArray message
        msg = Float64MultiArray() # lin_vel_x, lin_vel_y, ang_vel_z trans_x, trans_y, trans_z, orient_roll, orient_pitch, orient_yaw, gait, mode

        if joy_msg.buttons[11]==1 or joy_msg.buttons[12]==1 or joy_msg.buttons[13]==1 or joy_msg.buttons[14]==1:
            self.reset_values()
            self.update_mode(joy_msg.buttons[11], joy_msg.buttons[12], joy_msg.buttons[13], joy_msg.buttons[14])
        if joy_msg.buttons[0]==1 or joy_msg.buttons[1]==1 or joy_msg.buttons[2]==1 or joy_msg.buttons[3]==1:
            self.update_gait(joy_msg.buttons[0], joy_msg.buttons[1], joy_msg.buttons[2] , joy_msg.buttons[3])

        if self.mode == 0:
            self.activate_orientation_lock(joy_msg)
        elif self.mode == 1:
            self.activate_orientation_dance(joy_msg)
        elif self.mode == 2:
            self.activate_turret_translation_lock(joy_msg)
        elif self.mode == 3:
            self.activate_tracking(joy_msg)

        msg.data = [float(self.linear_vel_x), 
                    float(self.linear_vel_y), 
                    float(self.angular_vel_z), 
                    float(self.pos_x), 
                    float(self.pos_y), 
                    float(self.pos_z), # dist btw ground and 1st femur servo axle
                    float(self.roll), 
                    float(self.pitch), 
                    float(self.yaw), 
                    float(self.gait), 
                    float(self.mode)]
        
        self.publisher_.publish(msg)
    
    def reset_values(self):
        self.linear_vel_x = 0.0
        self.linear_vel_y = 0.0
        self.angular_vel_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        
    def update_mode(self, up, down, left, right):
        if up == 1:
            self.mode = 0
        elif down == 1:
            self.mode = 2
        elif left == 1:
            self.mode = 3
        elif right == 1:
            self.mode = 1
        else:
            pass
    
    def update_gait(self, amble, wave, ripple, tri):
        if tri == 1:
            self.gait = 0
        elif wave == 1:
            self.gait = 1
        elif ripple == 1:
            self.gait = 2
        elif amble == 1:
            self.gait = 3
        else:
            pass

    def activate_orientation_lock(self, joy_msg):
        self.linear_vel_x = self.lin_vel_constant * -joy_msg.axes[2]
        self.linear_vel_y = self.lin_vel_constant * joy_msg.axes[3]
        self.angular_vel_z = 0.5 * self.angular_vel_constant * (joy_msg.axes[4] - joy_msg.axes[5])

        if (self.roll <= self.rpy_limit and self.roll >= -self.rpy_limit) or \
            (self.roll > self.rpy_limit and joy_msg.axes[0] > 0) or \
            (self.roll < -self.rpy_limit and joy_msg.axes[0] < 0):
            self.roll += -joy_msg.axes[0] * 0.01 
        if (self.pitch <= self.rpy_limit and self.pitch >= -self.rpy_limit) or \
            (self.pitch > self.rpy_limit and joy_msg.axes[1] < 0) or \
            (self.pitch < -self.rpy_limit and joy_msg.axes[1] > 0):
            self.pitch += joy_msg.axes[1] * 0.01 
        
        if joy_msg.buttons[7] > 0:
            self.roll = 0
            self.pitch = 0
            self.yaw = 0

    def activate_orientation_dance(self, joy_msg):
        self.pos_x = -joy_msg.axes[2] * self.xyz_constant
        self.pos_y = joy_msg.axes[3] * self.xyz_constant
        if (self.pos_z >= self.z_limit_low and self.pos_z <= self.z_limit_high) or \
            (self.pos_z<self.z_limit_low and (0.01 * (joy_msg.buttons[6] - joy_msg.buttons[4])) > 0) or \
            (self.pos_z>self.z_limit_low and (0.01 * (joy_msg.buttons[6] - joy_msg.buttons[4])) < 0):
            self.pos_z += 0.002 * (joy_msg.buttons[6] - joy_msg.buttons[4])
        
        self.roll = -joy_msg.axes[0] * self.rpy_constant
        self.pitch = joy_msg.axes[1] * self.rpy_constant
        if (self.yaw >= -self.rpy_limit and self. yaw <= self.rpy_limit) or \
            (self.yaw < -self.rpy_limit and (joy_msg.buttons[10] - joy_msg.buttons[9])>0) or \
            (self.yaw > self.rpy_limit and (joy_msg.buttons[10] - joy_msg.buttons[9])<0) :
            self.yaw += (joy_msg.buttons[10] - joy_msg.buttons[9]) * 0.01 

        if joy_msg.buttons[7] > 0:
            self.roll = 0
            self.pitch = 0
            self.yaw = 0

        
    def activate_turret_translation_lock(self, joy_msg):

        self.linear_vel_x = self.lin_vel_constant * -joy_msg.axes[2]
        self.linear_vel_y = self.lin_vel_constant * joy_msg.axes[3]
        self.angular_vel_z = 0.5 * self.angular_vel_constant * (joy_msg.axes[4] - joy_msg.axes[5])

        self.roll = -joy_msg.axes[0] * self.rpy_constant
        self.pitch = joy_msg.axes[1] * self.rpy_constant

    
    def activate_tracking(self, joy_msg):
        self.reset_values()


def main(args=None):
    rclpy.init(args=args)
    node = JoyToFloat64MultiArray()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
