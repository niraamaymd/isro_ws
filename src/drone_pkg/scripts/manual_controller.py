#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class ManualController:
    def __init__(self):
        rospy.init_node('manual_controller')
        self.publisher = rospy.Publisher('manual_drone_cmds', String, queue_size=10)
        
    def run_interactive(self):
        print("Manual Drone Controller")
        print("Type your command and press Enter:")
        print("Type 'quit' to exit")
        
        while not rospy.is_shutdown():
            try:
                command = raw_input(">> ").strip()  # Use input() if using Python 3
                
                if command.lower() == 'quit':
                    break
                
                if command:
                    msg = String()
                    msg.data = command
                    self.publisher.publish(msg)
                    rospy.loginfo('Published: {}'.format(command))
                    
            except KeyboardInterrupt:
                break
                
        print("Goodbye!")

if __name__ == '__main__':
    try:
        controller = ManualController()
        controller.run_interactive()
    except rospy.ROSInterruptException:
        pass
