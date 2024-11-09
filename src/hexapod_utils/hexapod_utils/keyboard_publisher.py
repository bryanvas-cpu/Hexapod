#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard  # Import pynput for capturing key presses

class KeyboardToFloat64MultiArray(Node):
    def __init__(self):
        super().__init__('keyboard_to_float64multiarray')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'hexapod_controller/array_commands', 10)
        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 10

        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel_z = 0.0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.trans_z = 0.0
        self.orient_pitch = 0.0
        self.orient_roll = 0.0
        self.orient_yaw = 0.0
        self.gait = 0

        # Map keys to specific float values
        self.key_map = {
            '8': 0.2,      
            '2': -0.2,
                     
            '4': -0.2,      
            '6': 0.2, 

            '7': 0.5,       
            '9': -0.5,
                       
            '0': 0.0,

            

            'w': 0.05,      
            'x': -0.05, 
                   
            'a': -0.05,      
            'd': 0.05, 
                    
            'e': 0.05,     
            'c': -0.05,

            's': 0.0,


                        
            'i': 0.15,      
            ',': -0.15, 
                    
            'j': -0.15,     
            'l': 0.15,  
                  
            'u': 0.174533,
            'o': -0.174533,
                  
            'k': 0.0,
        }

        # Initialize listener for the keyboard
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            # Convert the key to string
            key_str = key.char
        except AttributeError:
            # Handle special keys like arrow keys
            key_str = str(key)

        if key_str == "8" or key_str == "2":
            self.lin_vel_y = self.key_map[key_str]
        elif key_str == "4" or key_str == "6":
            self.lin_vel_x = self.key_map[key_str]
        elif key_str == "7" or key_str == "9":
            self.ang_vel_z = self.key_map[key_str]
        elif key_str == "0":
            self.lin_vel_y = self.key_map[key_str]
            self.lin_vel_x = self.key_map[key_str]
            self.ang_vel_z = self.key_map[key_str]


        elif key_str == "i" or key_str == ",":
            self.trans_y = self.key_map[key_str]
        elif key_str == "j" or key_str == "l":
            self.trans_x = self.key_map[key_str]
        elif key_str == "u" or key_str == "o":
            self.trans_z = self.key_map[key_str]
        elif key_str == "k":
            self.trans_y = self.key_map[key_str]
            self.trans_x = self.key_map[key_str]
            self.trans_z = self.key_map[key_str]


        elif key_str == "w" or key_str == "x":
            self.orient_pitch = self.key_map[key_str]
        elif key_str == "a" or key_str == "d":
            self.orient_roll = self.key_map[key_str]
        elif key_str == "e" or key_str == "c":
            self.orient_yaw = self.key_map[key_str]
        elif key_str == "s":
            self.orient_roll = self.key_map[key_str]
            self.orient_pitch = self.key_map[key_str]
            self.orient_yaw = self.key_map[key_str]
        
        elif key_str == "z":
            self.gait = (self.gait + 1 if self.gait<3 else 0)

        self.msg.data = [
            self.lin_vel_x, self.lin_vel_y, self.ang_vel_z, 
            self.trans_x, self.trans_y, self.trans_z, 
            self.orient_roll, self.orient_pitch, self.orient_yaw, self.gait
        ]
        
        self.publisher_.publish(self.msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToFloat64MultiArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
