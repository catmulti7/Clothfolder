#include"ros/ros.h"
#include"std_msgs/Int16.h"
#include <iostream>
#include<string>
#include <stdlib.h>
//lib
#include <librealsense2/rs.hpp>

#include<librealsense2/rsutil.h>

#include <librealsense2/hpp/rs_processing.hpp>

#include <librealsense2/hpp/rs_types.hpp>

#include <librealsense2/hpp/rs_sensor.hpp>

#include <opencv2/opencv.hpp>

#include <math.h>


using namespace std;

using namespace cv;

#define WIDTH 640 
#define HEIGHT 480 
#define FPS 30

int stableCount=0;
ros::Publisher res_pub;
std_msgs::Int16 res;

void imgcap(Mat& color, Mat& depth);
int mindist,maxdist;
int main (int argc, char** argv)
{
	ros::init(argc, argv, "recg_node");
    //声明节点句柄
    ros::NodeHandle nh;
	res_pub = nh.advertise<std_msgs::Int16>("/result", 1);
	nh.getParam("/min_dist",mindist);
	nh.getParam("/max_dist",maxdist);
	cout<<mindist<<" "<<maxdist<<endl;
	Mat depth,color;
	while(ros::ok())
	{
		imgcap(color,depth);
		//imgprocess(color,depth);
	}
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

void imgprocess(Mat& color, Mat& depth)
{
	
	Mat binDepth;
	inRange(depth,mindist,maxdist,binDepth);
	Mat kernel=getStructuringElement(MORPH_RECT,Size(7,5));
	erode(binDepth,binDepth,kernel);
	erode(binDepth,binDepth,kernel);
	Mat kernel2=getStructuringElement(MORPH_RECT,Size(7,7));
	dilate(binDepth,binDepth,kernel2);
	//threshold(depth,binDepth,100,255,THRESH_BINARY);
	//imshow("binImg",binDepth);
	Rect mask(30,0,580,480);
	Mat depimg=binDepth(mask).clone();
	Rect cloth=boundingRect(binDepth);

	if(cloth.area()<20000)
	{
		cout<<"no cloth"<<endl;
		stableCount=0;
		res.data=0;
		res_pub.publish(res);
		return;
	}

	stableCount++;

	rectangle(color,cloth,Scalar(20,20,255),2);
	rectangle(depth,cloth,Scalar(100),2);
	imshow("Display deep", depth);
	imshow("Display Image", color);
	waitKey(1);

	
	float density1=0;
	float density2=0;
	float density3=0;
	for(int i=0;i<cloth.height;i++)
	{
		uchar* data = binDepth.ptr<uchar>(cloth.y+i);
		if(data[cloth.x+cloth.width/2]!=0)
		{
			density1=density1+1;
			data[cloth.x+cloth.width/2]=70;
		}
		if(data[cloth.x+cloth.width*2/5]!=0)
		{
			density2=density2+1;
			data[cloth.x+cloth.width*2/5]=50;
		}
		if(data[cloth.x+cloth.width*3/5]!=0)
		{
			density3=density3+1;
			data[cloth.x+cloth.width*3/5]=70;
		}
	}
	density1=density1/cloth.height;
	density2=density2/cloth.height;
	density3=density3/cloth.height;
	imshow("deep res", binDepth);
	waitKey(5);
	if(stableCount>10)
	{
		if(density1>0.5 && density2>0.5 &&density3>0.5)
		{
			cout<<"cloth"<<endl;
			res.data=1;
		}
		
		else
		{
			cout<<"trousers"<<endl;
			res.data=2;
		}
		cout<<"height is "<<cloth.height<<endl;
		cout<<"width is "<<cloth.width<<endl;
		res_pub.publish(res);
	}
}

void imgcap(Mat& color, Mat& depth)
{
	// judge whether devices is exist or not 
	rs2::context ctx;
	rs2::colorizer color_map;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();

	float depthScale=get_depth_scale(dev);

	rs2::pipeline pipe;     //Contruct a pipeline which abstracts the device
	rs2::config cfg;    //Create a configuration for configuring the pipeline with a non default profile
	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile selection = pipe.start(cfg);
	

	int k = 0;
	
	rs2_stream align_to = RS2_STREAM_COLOR;
	rs2::align align(align_to);

	while (ros::ok())
	 {
		rs2::frameset frames;
		frames = pipe.wait_for_frames();

		// rs2::align align_to_depth(RS2_STREAM_DEPTH);
        // frames = align_to_depth.process(frames);

		auto processed = align.process(frames);
		//Get each frame
		auto color_frame = frames.get_color_frame();
		auto depth_frame = frames.get_depth_frame();
		auto depth_colored_frame = frames.get_depth_frame().apply_filter(color_map);
		

		//create cv::Mat from rs2::frame
		Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		Mat deep(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
		Mat depth_colored(Size(640, 480), CV_8UC3, (void*)depth_colored_frame.get_data(), Mat::AUTO_STEP);

		if (!deep.empty())
		{
			deep=deep*70;
			double min,max;
			minMaxLoc(deep,&min,&max);
			double scale=255/(max-min);
			deep.convertTo(depth,CV_8UC1,scale,(-1)*min*scale);

			if(!color.empty()&&!depth.empty())
			{
				// imshow("Display Image", color);
				 imshow("Display colored depth", depth_colored);
				 waitKey(1);
				imgprocess(color,depth);
			}
		}
	}
}