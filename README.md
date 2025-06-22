# astar_ros

## Quick Start with Docker

### Prerequisites
- Docker and Docker Compose installed on your system

### Running the Project
1. Clone the repository:
   ```bash
   git clone <your-repo-url>
   cd astar_ros
   ```

2. Build and run with Docker:
   ```bash
   docker-compose up --build
   ```

3. In the container, you can run ROS2 commands:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

### Development
To rebuild after making changes:
```bash
docker-compose down
docker-compose up --build
```