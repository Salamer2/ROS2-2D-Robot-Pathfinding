import rclpy
from rclpy.node import Node

from .Grid_World import Grid_World

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

import json

class WorldNode(Node):
    def __init__(self):
        super().__init__('grid_world_node')

        self.world = Grid_World(rows=10, cols=10, obs=40)

        self.img_pub = self.create_publisher(
            Image,
            'world_img',
            1
        )

        self.bridge = CvBridge()

        self.state_pub = self.create_publisher(String, 'world_state', 1)
        self.score_pub = self.create_publisher(String, 'world_score', 1)
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            1
        )

        self.create_timer(0.5, self.state_score_callback)


        # Current visualization
        self.img = self.world.world_update()

        # Image timer (DO NOT MODIFY)
        self.create_timer(0.1, self.image_callback)

        self.get_logger().info("World node started")

    def command_callback(self, msg):
        mapping = {
            "up": (-1, 0),
            "down": (1, 0),
            "left": (0,-1),
            "right": (0, 1),
        }
        cmd = msg.data.strip().lower()
        if cmd not in mapping:
            return
        dx, dy = mapping[cmd]
        reward = self.world.move(dx, dy)
        self.img = self.world.world_update()
        self.get_logger().info(f"I'm moving {cmd}! reward={reward}, score={self.world.get_score()}")

    def state_score_callback(self):
        pos, obs = self.world.get_state()
        state_msg = String()
        state_msg.data = json.dumps({
            "position": list(pos),
            "observation": obs,
            "done": self.world.is_done(),
        })
        self.state_pub.publish(state_msg)

        score_msg = String()
        score_msg.data = str(self.world.get_score())
        self.score_pub.publish(score_msg)

    def image_callback(self):
        msg = self.bridge.cv2_to_imgmsg(
            self.img,
            "bgr8"
        )
        self.img_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()