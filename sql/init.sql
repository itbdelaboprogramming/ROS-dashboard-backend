CREATE DATABASE ROS_DB;

USE ROS_DB;

CREATE TABLE pgm_data (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, map_name varchar(100), file_size varchar(255), created_time datetime, modified_time datetime, IP_Address varchar(15), file_type varchar(255), data longtext);

CREATE TABLE yaml_data(id int NOT NULL AUTO_INCREMENT PRIMARY KEY, name varchar(100), age int, email varchar(100), file_name varchar(255), file_size varchar(255), created_time datetime, modified_time datetime, author varchar(255), file_type varchar(255), data text);
