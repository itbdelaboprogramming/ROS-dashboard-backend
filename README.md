# ROS Dashboard Backend

## Installation
1. Clone the repository. Please make sure that catkin_ws directory is already prepared. Also make sure Node.js is installed. Version that is used for development is v18.18.2. If there are issues, refer to this version (v18.18.2).
```bash
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/ROS-dashboard-backend.git
cd ~/catkin_ws
catkin_make
```

2. Install MySQL server and set root password
```bash
sudo apt install mysql-server
sudo mysql
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'root';
FLUSH PRIVILEGES;
exit
```

3. (Optional) set user for remote password through zerotier
```bash
mysql -u root -p
# enter root password, in this case from previous step is 'root'

CREATE USER 'your_username'@'ip_zerotier' IDENTIFIED BY 'your_password';
GRANT ALL PRIVILEGES ON *.* TO 'your_username'@'ip_zerotier' WITH GRANT OPTION;
FLUSH PRIVILEGES;
exit

# now that user can access the database from remote/other pc assigned with that IP address.
```

4. Prepare database
```bash
mysql -u root -p
# enter root password, in this case from previous step is 'root'
# copy all the commands from sql/init.sql ONE BY ONE
```
5. Fill MySQL authentication
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend/scripts
nano backend_node
# Fill the MySQL authentication
```
6. Install dependencies (Node.js v18.18.2)
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend
npm install
```

7. Run
```bash
cd ~/catkin_ws
roslaunch ros_dashboard_backend ros_dashboard_backend.launch
```

8. The app will run at all interfaces (0.0.0.0) on port 5000.

## Postman
Postman collection and environment filees are provided to test the HTTP REST API using Postman.