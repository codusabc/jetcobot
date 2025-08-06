import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from pymycobot.mycobot280 import MyCobot280       

import time

class GripperFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('gripper_feedback_publisher')
        self.mc = MyCobot280('/dev/ttyUSB0', 1000000) #mc.get_gripper_value() : 열리면 100, 닫히면 0
        self.gripper_pub = self.create_publisher(Int32, '/gripper_feedback', 10)
        
        # Timer to publish gripper value every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.publish_gripper_value)
    
    def publish_gripper_value(self):
        try:
            gripper_value = self.mc.get_gripper_value()  # 열리면 100, 닫히면 0
            
            msg = Int32()
            msg.data = gripper_value
            self.gripper_pub.publish(msg)
            self.get_logger().info(f'Gripper value: {gripper_value}')
                
        except Exception as e:
            self.get_logger().error(f"Failed to read/publish gripper value: {e}")


def main(): 
    rp.init()

    gripper_feedback_publisher = GripperFeedbackPublisher()

    try:
        rp.spin(gripper_feedback_publisher)
    finally:
        gripper_feedback_publisher.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()


    # joint_control.py 에서 수정한 부분
    #     self.pub = self.create_publisher(JointState, "real_joint_states", 1)
    #     self.gripper_feedback_pub = self.create_publisher(Int32, "gripper_feedback", 1)
    #     self.gripper_command = None
    
    # def gripper_callback(self, msg): ## Int 메세지 수신 # gripper_command는 0~100 범위
    #     self.gripper_command = int(msg.data) ## close(true) or open(false) -> Int
    #     for attempt in range(5):
    #         # time.sleep(0.1)
    #         self.mc.set_gripper_value(self.gripper_command, 100) # set_gripper_state 대신 set_gripper_value 사용  # gripper_value, speed
    #         result = self.mc.get_gripper_value() # gripper_value값 받아오기
    #         if result != -1:
    #             # result를 Int32 메시지로 publish
    #             gripper_feedback_msg = Int32()
    #             gripper_feedback_msg.data = int(result)
    #             self.gripper_feedback_pub.publish(gripper_feedback_msg)
                
    #             self.get_logger().info(f"Set gripper command: {self.gripper_command}, Gripper state: {result}")
    #             # self.get_logger().info(f"Gripper state successfully set to {result} on attempt {attempt + 1}")'
    #             break
    #         else:
    #             self.get_logger().warn(f"Failed to set gripper state, attempt {attempt + 1}/5")
    #             if attempt == 4:
    #                 self.get_logger().error("Failed to set gripper state after 5 attempts")