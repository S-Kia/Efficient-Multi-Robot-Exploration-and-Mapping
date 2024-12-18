import random
import os

def generate_maze(width, height):
    # Initialize the maze with all walls (1 = wall, 0 = path)
    maze = [[1 for _ in range(width)] for _ in range(height)]

    # Helper function to check if a cell is within bounds
    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    # Depth-First Search (DFS) function to generate paths
    def dfs(x, y):
        # Mark the current cell as a path
        maze[y][x] = 0

        # Shuffle directions for randomness (right, down, left, up)
        directions = [(0, 2), (2, 0), (0, -2), (-2, 0)]
        random.shuffle(directions)

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            # Check if the neighboring cell is within bounds and is a wall
            if in_bounds(nx, ny) and maze[ny][nx] == 1:
                # Knock down the wall between the current cell and the neighbor
                maze[y + dy // 2][x + dx // 2] = 0
                dfs(nx, ny)

    # Start maze generation from the top-left corner (0,0)
    dfs(0, 0)

    return maze

def maze_to_sdf(maze, wall_size=0.75, wall_height=2, gap=2):
    # Calculate the offsets to center the maze
    width = len(maze[0])
    height = len(maze)
    
    # Outer wall boundary dimensions
    outer_width = width + gap * 2
    outer_height = height + gap * 2

    x_offset = -(outer_width * wall_size) / 2
    y_offset = -(outer_height * wall_size) / 2

    sdf_content = """
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <world name="default">
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
    """

    # Create outer walls
    for y in range(outer_height):
        for x in range(outer_width):
            if x == 0 or x == outer_width - 1 or y == 0 or y == outer_height - 1:
                # Place walls around the boundary (leave a gap inside)
                x_position = x * wall_size + x_offset
                y_position = y * wall_size + y_offset
                sdf_content += f"""
                <model name="outer_wall_{x}_{y}">
                  <static>true</static>
                  <link name="link">
                    <collision name="collision">
                      <geometry>
                        <box>
                          <size>{wall_size} {wall_size} {wall_height}</size>
                        </box>
                      </geometry>
                    </collision>
                    <visual name="visual">
                      <geometry>
                        <box>
                          <size>{wall_size} {wall_size} {wall_height}</size>
                        </box>
                      </geometry>
                    </visual>
                  </link>
                  <pose>{x_position} {y_position} {wall_height / 2} 0 0 0</pose>
                </model>
                """

    # Create maze walls
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1:  # Wall cell in maze
                # Calculate the position of each wall segment with offset
                x_position = (x + gap) * wall_size + x_offset
                y_position = (y + gap) * wall_size + y_offset
                sdf_content += f"""
                <model name="wall_{x}_{y}">
                  <static>true</static>
                  <link name="link">
                    <collision name="collision">
                      <geometry>
                        <box>
                          <size>{wall_size} {wall_size} {wall_height}</size>
                        </box>
                      </geometry>
                    </collision>
                    <visual name="visual">
                      <geometry>
                        <box>
                          <size>{wall_size} {wall_size} {wall_height}</size>
                        </box>
                      </geometry>
                    </visual>
                  </link>
                  <pose>{x_position} {y_position} {wall_height / 2} 0 0 0</pose>
                </model>
                """

    sdf_content += """
      </world>
    </sdf>
    """
    return sdf_content

# Parameters for maze generation
width, height = 12, 12  # Maze dimensions
wall_size = 0.75        # Wall size in meters
wall_height = 1         # Wall height in meters
gap = 2                 # Gap between maze and outer wall

maze = generate_maze(width, height)
sdf_content = maze_to_sdf(maze, wall_size, wall_height, gap)

# Define the save directory
save_directory = "/home/s-kia/catkin_ws/src/random_map"

# Create the directory if it doesn't exist
os.makedirs(save_directory, exist_ok=True)

# Path to save the world file
save_path = os.path.join(save_directory, "random_maze.world")

# Write the generated world content to the specified file
with open(save_path, "w") as f:
    f.write(sdf_content)

print(f"Maze world file saved to: {save_path}")

