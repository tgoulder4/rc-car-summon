#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class RobotCommandPublisher:
    def __init__(self):
        print("Initializing RobotCommandPublisher")
        self.pub = rospy.Publisher('robot_commands', String, queue_size=10)
        rospy.init_node('command_publisher', anonymous=True)
        self.rate = rospy.Rate(1)  # 1 Hz
        print("RobotCommandPublisher initialized with topic 'robot_commands'")
    
    def send_command(self, command):
        print(f"Publishing command: {command}")
        self.pub.publish(command)
        print("Command published, sleeping for rate control")
        self.rate.sleep()
        print("Rate sleep completed")
    
    def send_path_commands(self, path):
        print(f"Sending commands for path with {len(path)} points")
        for i in range(len(path) - 1):
            current = path[i]
            next_pos = path[i + 1]
            
            print(f"Processing path segment {i}: {current} -> {next_pos}")
            
            # Determine direction
            dr = next_pos[0] - current[0]
            dc = next_pos[1] - current[1]
            
            if dr == 1:
                print(f"Movement is DOWN (dr={dr})")
                self.send_command("DOWN")
            elif dr == -1:
                print(f"Movement is UP (dr={dr})")
                self.send_command("UP")
            elif dc == 1:
                print(f"Movement is RIGHT (dc={dc})")
                self.send_command("RIGHT")
            elif dc == -1:
                print(f"Movement is LEFT (dc={dc})")
                self.send_command("LEFT")
            else:
                print(f"WARNING: Invalid movement detected: dr={dr}, dc={dc}")
        
        print("All path commands sent")

class RobotCommandSubscriber:
    def __init__(self):
        print("Initializing RobotCommandSubscriber")
        rospy.init_node('command_subscriber', anonymous=True)
        rospy.Subscriber('robot_commands', String, self.callback)
        print("RobotCommandSubscriber initialized and subscribed to 'robot_commands'")
    
    def callback(self, data):
        command = data.data
        print(f"Received command: {command}")
        print(f"Processing command: {command}")
        # Process command here
    
    def listen(self):
        print("Starting to listen for robot commands...")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print("ROS interrupted while listening")
        finally:
            print("Stopped listening for robot commands")

