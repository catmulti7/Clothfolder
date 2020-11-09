#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include<iostream>
#include "iomanip"
#include <stdlib.h>
#define DATA_LEN 2

serial::Serial ser;//声明串口对象
//serial_common::gimbal receive_msg;

unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) {
    unsigned char byte_crc = 0;
    for (unsigned char i = 0; i < data_lenth; i++) {
        byte_crc += InputBytes[i];
    }
    return byte_crc;
}

void Data_disintegrate(unsigned int Data, unsigned char *LData,
                       unsigned char *HData) {
    *LData = Data & 0XFF;          // 0xFF = 1111 1111
    *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}

void write_callback(const std_msgs::Int16ConstPtr& msg)
{
    uint8_t Buffer[DATA_LEN];
    Buffer[0] = 0xFF;
    if(msg->data==0)
        Buffer[1] = 0;
    if(msg->data==1)
        Buffer[1] = 1;
    if(msg->data==2)
        Buffer[1] = 2;
    //Data_disintegrate(msg->data, &Buffer[2], &Buffer[3]);
    ser.write(Buffer,DATA_LEN);   //发送串口数据
}

void receive_process(std::string &read_buffer)
{
  if(read_buffer[0]!=0XFF)
  {
    return;
  }
  int patch_num=(int)read_buffer[1];
}

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_common_node");
    
    //声明节点句柄
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("/result", 33, write_callback);

    //设置串口属性，并打开串口
    //const char *usb_ttl=getenv("usb_ttl");
    const char *usb_ttl=NULL;
    if(usb_ttl==NULL)
    {
      ROS_WARN_STREAM("SYSTEM INIT");
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
      ROS_WARN_STREAM("SYSTEM USB DETECTED");
      if(!ser.isOpen())
      {
          ser.setPort("/dev/ttyUSB1");
          ser.open();
      }
    }
    else
    {
      ROS_WARN_STREAM("usb name is"<<usb_ttl);
      ser.setPort(usb_ttl);
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
}
