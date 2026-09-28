import numpy as np
import copy
from random import sample
from collections import deque

class Grid_World:
    def __init__(self, rows=5, cols=6, obs=8):
        self.rows = rows
        self.cols = cols
        self.obs = obs

        self.start = (0, 0)
        self.end = (rows - 1, cols - 1)

        self.player = self.start

        self.score = 0
        self.done = False

        self.generate_world()
        self.init_world()


    ############################################################
    # Generate random obstacles with guaranteed solution
    ############################################################
    def generate_world(self):
        possible = [
            (i, j)
            for i in range(self.rows)
            for j in range(self.cols)
            if (i, j) != self.start and (i, j) != self.end
        ]

        while True:
            self.obs_loc = sample(possible, self.obs)

            if self.is_solvable():
                break


    ############################################################
    # Check path exists using BFS
    ############################################################

    def is_solvable(self):
        queue = deque([self.start])
        visited = {self.start}

        while queue:
            current = queue.popleft()
            if current == self.end:
                return True
            for dx, dy in [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1)
            ]:
                nx = current[0] + dx
                ny = current[1] + dy
                if not (0 <= nx < self.rows):
                    continue
                if not (0 <= ny < self.cols):
                    continue
                if (nx, ny) in self.obs_loc:
                    continue
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return False


    ############################################################
    # Reset environment
    ############################################################
    def reset(self):
        self.player = self.start
        self.score = 0
        self.done = False

        return self.get_state()


    ############################################################
    # Move agent
    # dx,dy:
    # (-1,0) up
    # (1,0) down
    # (0,-1) left
    # (0,1) right
    ############################################################
    def move(self, dx, dy):
        if self.done:
            return 0

        new_position = (
            self.player[0] + dx,
            self.player[1] + dy
        )
        # Outside grid
        if not (
            0 <= new_position[0] < self.rows and
            0 <= new_position[1] < self.cols
        ):
            self.score -= 1
            return -1


        # Hit obstacle
        if new_position in self.obs_loc:

            self.score -= 10
            return -10
        # Move
        self.player = new_position
        self.score -= 1
        # Goal reached
        if self.player == self.end:
            self.score += 100
            self.done = True
            return 100

        return -1


    ############################################################
    # Return local 3x3 observation
    ############################################################
    def get_state(self):
        observation = []
        for dx in [-1, 0, 1]:
            row = []
            for dy in [-1, 0, 1]:
                x = self.player[0] + dx
                y = self.player[1] + dy
                if not (
                    0 <= x < self.rows and
                    0 <= y < self.cols
                ):
                    row.append('#')
                elif (x, y) == self.player:
                    row.append('a')
                elif (x, y) == self.end:
                    row.append('g')
                elif (x, y) in self.obs_loc:
                    row.append('o')
                else:
                    row.append('.')
            observation.append(row)
        return self.player, observation

    ############################################################
    # Return score
    ############################################################
    def get_score(self):
        return self.score

    ############################################################
    # Check goal
    ############################################################
    def is_done(self):
        return self.done

    ############################################################
    # Visualization
    ############################################################
    def init_world(self,
                   scale=4,
                   block_size=30,
                   block_boarders=2):
        self.block_size = block_size * scale

        block_size *= scale
        block_boarders *= scale
        self.block = np.zeros(
            (block_size, block_size, 3),
            dtype=np.uint8
        )
        shift = block_boarders // 2
        self.block[
            shift:block_size-shift,
            shift:block_size-shift
        ] = 255
        self.img_boarders = (block_boarders + 1) * 2
        self.world = np.zeros(
            (
                self.rows * block_size + self.img_boarders,
                self.cols * block_size + self.img_boarders,
                3
            ),
            dtype=np.uint8
        )
        for i in range(self.rows):
            for j in range(self.cols):
                r = self.img_boarders//2 + i * block_size
                c = self.img_boarders//2 + j * block_size
                if (i,j) in self.obs_loc:
                    self.world[
                        r:r+block_size,
                        c:c+block_size,
                        2
                    ] = self.block[:,:,2]
                elif (i,j) == self.end:
                    self.world[
                        r:r+block_size,
                        c:c+block_size,
                        1
                    ] = self.block[:,:,1]
                else:
                    self.world[
                        r:r+block_size,
                        c:c+block_size
                    ] = self.block

    def world_update(self):
        img = copy.deepcopy(self.world)
        r = self.img_boarders//2 + self.player[0]*self.block_size
        c = self.img_boarders//2 + self.player[1]*self.block_size
        shift = self.block_size//4
        img[
            r+shift:r+self.block_size-shift,
            c+shift:c+self.block_size-shift,
            :
        ] = 0
        return img

if __name__ == "__main__":
    env = Grid_World()

    print("Obstacles:")
    print(env.obs_loc)

    print("\nInitial state:")
    pos, state = env.get_state()

    print(pos)

    for row in state:
        print(row)

    print("\nMoving right")
    reward = env.move(0,1)
    print("Reward:", reward)
    print("Score:", env.get_score())
    print("\nNew state:")
    pos, state = env.get_state()
    print(pos)
    for row in state:
        print(row)