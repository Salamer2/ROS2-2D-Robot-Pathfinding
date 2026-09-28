import json
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CommandNode(Node):
    def __init__(self):
        super().__init__('grid_world_solver')

        self.known_map = [[2] * 10 for _ in range(10)]
        self.known_map[0][0] = 0
        self.goal = (9, 9)
        self.known_map[9][9] = 0

        # TODO 1:
        # Create a subscriber to receive the robot observation.
        #
        # Topic name:
        #   world_state
        #
        # The received message contains:
        #   - current robot position
        #   - 3x3 local map around the robot
        #
        # Example observation:
        #
        # [
        #   ['.', 'o', '.'],
        #   ['.', 'a', '.'],
        #   ['.', '.', 'g']
        # ]
        #
        # where:
        #   a = robot position
        #   o = obstacle
        #   g = goal
        #   . = free space
        #   # = outside the world
        #
        # Your callback function should analyze this information
        # and decide the next movement.
        self.state_sub = self.create_subscription(
            String,
            'world_state',
            self.state_callback,
            1
        )

        self.command_pub = self.create_publisher(String, 'command', 1)
        self.view_pub = self.create_publisher(Image, 'robot_view_img', 1)
        self.bridge = CvBridge()
        self.current_pos = (0, 0)


        self.get_logger().info("Solver node started")

    def state_callback(self, msg):
        data = json.loads(msg.data)
        pos = tuple(data["position"])
        obs = data["observation"]
        self.current_pos = pos

        if data.get("done", False):
            self.draw_robot_view()
            return

        self.update_map(pos, obs)
        path = self.astar(pos)
        if not path:
            return

        next_cell = path[0]
        dr = next_cell[0] - pos[0]
        dc = next_cell[1] - pos[1]
        
        cmd = {(-1, 0): "up", (1, 0): "down", (0, -1): "left", (0, 1): "right"}.get((dr, dc), "")
        if cmd:
            out = String()
            out.data = cmd
            self.command_pub.publish(out)
            self.current_pos = next_cell
        self.draw_robot_view()

    def update_map(self, pos, obs):
        r, c = pos
        for i in range(3):
            for j in range(3):
                nr, nc = r + i - 1, c + j - 1
                if 0 <= nr < 10 and 0 <= nc < 10:
                    cell = obs[i][j]
                    if cell == 'o':
                        self.known_map[nr][nc] = 1
                    elif cell in ('.', 'a', 'g'):
                        self.known_map[nr][nc] = 0

    def astar(self, start):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dr, current[1] + dc)
                if not (0 <= neighbor[0] < 10 and 0 <= neighbor[1] < 10):
                    continue
                if self.known_map[neighbor[0]][neighbor[1]] == 1:
                    continue

                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + abs(neighbor[0] - 9) + abs(neighbor[1] - 9)
                    heapq.heappush(open_set, (f_score, neighbor))
        return None

    # This function is just a gridworld drawer copy, but it draws cells undiscovered by the robot as black.
    def draw_robot_view(self):
        rows = 10
        cols = 10
        scale = 4
        block_size = 30 * scale
        block_boarders = 2 * scale
        
        block = np.zeros((block_size, block_size, 3), dtype=np.uint8)
        shift = block_boarders // 2
        block[shift:block_size-shift, shift:block_size-shift] = 255
        
        img_boarders = (block_boarders + 1) * 2
        img = np.zeros((rows * block_size + img_boarders, cols * block_size + img_boarders, 3), dtype=np.uint8)
        
        for r in range(rows):
            for c in range(cols):
                y = img_boarders//2 + r * block_size
                x = img_boarders//2 + c * block_size
                val = self.known_map[r][c]
                
                if val == 2:
                    continue
                elif (r, c) == self.goal:
                    img[y:y+block_size, x:x+block_size, 1] = block[:,:,1]
                elif val == 1:
                    img[y:y+block_size, x:x+block_size, 2] = block[:,:,2]
                elif val == 0:
                    img[y:y+block_size, x:x+block_size] = block
                
                if (r, c) == self.goal:
                    img[y:y+block_size, x:x+block_size, 1] = block[:,:,1]
        
        r_pos, c_pos = self.current_pos
        y = img_boarders//2 + r_pos * block_size
        x = img_boarders//2 + c_pos * block_size
        robot_shift = block_size // 4
        img[y+robot_shift:y+block_size-robot_shift, x+robot_shift:x+block_size-robot_shift, :] = 0
        
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.view_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()