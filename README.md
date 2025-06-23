# A* Path Planning for TurtleBot3 in ROS2

**Implementasi Algoritma A* untuk Perencanaan Jalur Robot dalam ROS2 dan Gazebo**

<!-- Placeholder for demo GIF -->
![Demo GIF](assets/demo.gif)
*Demo: Robot TurtleBot3 navigasi menggunakan algoritma A* di lingkungan labirin*

---

## 📋 Deskripsi

Program ini merupakan implementasi algoritma A* untuk perencanaan jalur robot yang dikembangkan sebagai tugas mata kuliah **Strategi Algoritma IF2211**. Sistem bekerja dengan:

- **Input**: Occupancy Grid Map, posisi robot saat ini (`/odom`), dan titik tujuan (`/goal_pose`)
- **Output**: Path planning (`/plan`) untuk navigasi robot
- **Kontroler**: Open-loop sequential control tanpa feedback menggunakan node `goto_waypoint`

### 🏗️ Arsitektur Sistem

```
Occupancy Grid ──┐
                 ├──► A* Algorithm ──► Path ──► goto_waypoint ──► Robot Movement
Robot Odometry ──┘
Goal Pose ───────┘
```

## 👨‍💻 Identitas Pengembang

- **Nama**: Karol Yangqian Poetracahya
- **NIM**: 13523093
- **Institusi**: Institut Teknologi Bandung
- **Mata Kuliah**: Strategi Algoritma IF2211

## 🛠️ Spesifikasi Teknologi

- **OS**: Linux Ubuntu 22.04 LTS
- **Framework**: ROS2 Humble Hawksbill
- **Simulator**: Gazebo Classic
- **Robot Platform**: TurtleBot3 Burger (ROBOTIS)

## 📦 Dependencies

- **Maze Generator**: [Geckostya/maze_generator](https://github.com/Geckostya/maze_generator)
- **TurtleBot3 Simulation**: [ROBOTIS Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## 🚀 Instalasi dan Menjalankan

### Prerequisites

1. **Install ROS2 Humble**:
   ```bash
   # Follow official guide: https://docs.ros.org/en/humble/Installation.html
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update && sudo apt install ros-humble-desktop
   ```

2. **Install Gazebo Classic**:
   ```bash
   # Install Gazebo 11: https://classic.gazebosim.org/tutorials?tut=install_ubuntu
   sudo apt install gazebo libgazebo-dev
   ```

3. **Source ROS2**:
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Setup Workspace

1. **Buat workspace ROS2**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone repository**:
   ```bash
   git clone https://github.com/karolyangqian/stimabot_ros.git
   cd ..
   ```

3. **Install dependencies**:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

### Menjalankan Simulasi

Jalankan perintah berikut dalam **3 terminal berbeda** secara berurutan:

#### Terminal 1: Launch Gazebo World
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch astar_pathplanner empty_world.launch.py
```

#### Terminal 2: Launch Path Planning System
```bash
ros2 launch astar_pathplanner path_planning.launch.py
```

#### Terminal 3: Launch Map Server
```bash
ros2 launch astar_pathplanner map_server.launch.py
```

### 🎮 Cara Menggunakan

1. **Tunggu** hingga RViz2 terbuka dan menampilkan peta labirin
2. **Klik** tombol "**2D Goal Pose**" di toolbar RViz2
3. **Pilih** titik tujuan di dalam peta labirin
4. **Amati** robot bergerak mengikuti jalur yang dihasilkan algoritma A*

## 📁 Struktur Project

```
astar_pathplanner/
├── include/astar_pathplanner/
│   ├── astar.hpp              # Core A* algorithm
│   ├── astar_pathplanner.hpp  # ROS2 node wrapper
│   └── goto_waypoint.hpp     # Path following controller
├── src/
│   ├── astar.cpp              # A* implementation
│   ├── astar_pathplanner.cpp  # Path planning node
│   └── goto_waypoint.cpp     # Movement controller
├── launch/                    # Launch files
├── map/                       # Maze maps
├── config/                    # Configuration files
└── rviz/                      # RViz configurations
```

## 🔧 Konfigurasi

Parameter algoritma A* dapat disesuaikan di:
- `config/astar_pathplanner.yaml`

## 📺 Topics dan Services

### Subscribed Topics
- `/odom` (nav_msgs/Odometry) - Posisi robot saat ini
- `/map` (nav_msgs/OccupancyGrid) - Peta lingkungan
- `/goal_pose` (geometry_msgs/PoseStamped) - Titik tujuan

### Published Topics
- `/plan` (nav_msgs/Path) - Jalur yang dihasilkan A*
- `/cmd_vel` (geometry_msgs/Twist) - Perintah gerakan robot

## 🏆 Fitur

- ✅ **Pure A* Algorithm**: Implementasi algoritma A* dari nol
- ✅ **ROS2 Integration**: Terintegrasi penuh dengan ekosistem ROS2
- ✅ **Modular Design**: Pemisahan core algorithm dan ROS2 wrapper
- ✅ **Real-time Visualization**: Visualisasi path planning di RViz2
- ✅ **Configurable Parameters**: Parameter algoritma dapat disesuaikan
- ✅ **Maze Environment**: Lingkungan labirin untuk testing

## 📚 Referensi

- [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)

---

**📧 Kontak**: karolyangqian14@gmail.com