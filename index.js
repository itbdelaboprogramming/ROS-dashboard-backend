const express = require('express');
const bodyParser = require('body-parser');
const app = express();
const PORT = process.env.PORT || 5000;
const rosnodejs = require('rosnodejs');

rosnodejs.initNode('/backend_node');
const std_msgs = rosnodejs.require('std_msgs').msg;
const nh = rosnodejs.nh;
const sub = nh.subscribe('/chatter', 'std_msgs/String', (msg) => {
    console.log('Got msg on chatter: %j', msg);
});
const pub = nh.advertise('/chatter', 'std_msgs/String');

app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));

app.get('/', (req, res) => {
    pub.publish({ data: "Hello ros from http nodejs"});
    res.status(200).json({ success: true, data: "hello world sent to ROS"});
});

app.listen(PORT,"0.0.0.0", () => console.log(`Server running at port: ${PORT}`));