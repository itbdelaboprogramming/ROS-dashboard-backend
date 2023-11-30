# ROS Dashboard Backend

## Installation
1. Clone the repository. Please make sure that catkin_ws directory is already prepared.
```bash
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/ROS-dashboard-backend.git
cd ~/catkin_ws
catkin_make
```

2. Install dependencies (Node.js v18.18.2)
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend
npm install
```

3. Run
```bash
cd ~/catkin_ws
roslaunch ros_dashboard_backend ros_dashboard_backend.launch
```

4. The app will run at all interfaces (0.0.0.0) on port 5000.