# ROS Dashboard Backend

Backend service for ROS Dashboard. This service is responsible for handling HTTP REST API requests from the frontend and also handling ROS communication with the robot.

## Installation
1. The backend application requires some application from SLAM_ITBdeLabo ROS Package. Please install SLAM_ITBdeLabo ros pacakge first. Refer to this [link](https://github.com/itbdelaboprogramming/SLAM_ITBdeLabo) for installation guide.

2. Clone the repository. Please make sure that catkin_ws directory is already prepared. Also make sure Node.js is installed. Version that is used for development is v18.18.2. If there are issues, refer to this version (v18.18.2).
```bash
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/ROS-dashboard-backend.git
cd ~/catkin_ws
catkin_make
```

3. Install MySQL server and set root password
```bash
sudo apt install mysql-server
sudo mysql
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'root';
FLUSH PRIVILEGES;
exit
```

4. Set user and password for remote access from Jetson through zerotier
```bash
# expose mysql to all network interfaces
sudo nano /etc/mysql/mysql.conf.d/mysqld.cnf
# find bind-address and add another bind-address = "0.0.0.0"
# save and exit
ctrl + s
ctrl + x

# add new accounts for accesss from Jetson/remote pc
mysql -u root -p
# enter root password, in this case from previous step is 'root'
# enter these commands, ONE BY ONE separated by ";" (without quotes)
CREATE USER 'your_username'@'ip_zerotier' IDENTIFIED BY 'your_password';
GRANT ALL PRIVILEGES ON *.* TO 'your_username'@'ip_zerotier' WITH GRANT OPTION;
FLUSH PRIVILEGES;
exit

# now that user can access the database from remote/other pc assigned with that IP address.
```

5. Prepare database
```bash
mysql -u root -p
# enter root password, in this case from previous step is 'root'
# copy all the commands from sql/init.sql ONE BY ONE separated by ";" (without quotes)
```
6. Fill MySQL authentication
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend/scripts
nano backend_node
# Fill the MySQL authentication
host: "localhost",
user: "root",
password: "root",
database : "ROS_DB"
```
7. Install dependencies (Node.js v18.18.2)
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend
npm install
```

8. Run
```bash
cd ~/catkin_ws
roslaunch ros_dashboard_backend ros_dashboard_backend.launch
```

9. The app will run at all interfaces (0.0.0.0) on port 5000.

## Postman
Postman collection and environment filees are provided to test the HTTP REST API using Postman.