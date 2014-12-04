#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <iostream>
#include "OniSampleUtilities.h"
#include <OpenNI.h>
#include <stdlib.h> 
#include <string>

using namespace openni;
using namespace std;



int main(int argc, char **argv){

	string topic;
	string frame_id;
	int resolution;
	int rate;
	int _depth;
	int _rgb;
	int _registration;
	int _sync;

	printf("starting\n");
	fflush(stdout);
	ros::init(argc, argv, "xtionberry");
	ros::NodeHandle n("~");

	//Base topic name
	n.param("topic", topic, string("/xtionberry"));
	//Resolution
	//0 = 160x120
	//1 = 320x240
	n.param("resolution", resolution, 1);
	n.param("depth", _depth, 1);
	n.param("sync", _sync, 0);
	n.param("rgb", _rgb, 0);
	n.param("registration", _registration,0);
	n.param("rate", rate, 30);
	n.param("frame_id", frame_id, string("xtionberry"));

	if(_rgb==0 && _depth==0){
		printf("You should consider to request at least one sensor duuuuuude\n");
		fflush(stdout);
	}

	printf("Launched with params:\n");
	printf("_topic:= %s\n",topic.c_str());
	printf("_depth:= %d\n",_depth);
	printf("_sync:= %d\n",_sync);
	printf("_registration:= %d\n",_registration);
	printf("_rgb:= %d\n",_rgb);
	printf("_resolution:= %d\n",resolution);
	printf("_rate:= %d\n",rate);
	printf("_frame_id:= %s\n",frame_id.c_str());
	fflush(stdout);
	ros::Publisher xtionberry_pub_depth = n.advertise<sensor_msgs::Image>(topic+"/depth", 1);
	ros::Publisher xtionberry_pub_color = n.advertise<sensor_msgs::Image>(topic+"/color", 1);
	ros::Rate loop_rate(rate);

	

	
	//OPENNI2 STUFF
	//===================================================================
	VideoStream depth;
	VideoStream color;
	openni::VideoStream** streams;
	streams = new openni::VideoStream*[2];
	streams[0]=&depth;
	streams[1]=&color;

	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		fflush(stdout);
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK){
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		fflush(stdout);
		return 2;
	}

	if(_depth==1){
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL){
			rc = depth.create(device, SENSOR_DEPTH);
			if (rc != STATUS_OK){
				printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
				fflush(stdout);
				return 3;
			}
		}
		rc = depth.start();
	}

	if(_rgb==1){
		if (device.getSensorInfo(SENSOR_COLOR) != NULL){
			rc = color.create(device, SENSOR_COLOR);
			if (rc != STATUS_OK){
				printf("Couldn't create rgb stream\n%s\n", OpenNI::getExtendedError());
				fflush(stdout);
				return 3;
			}
		}
		rc = color.start();
	}

	if(_depth==1 && _rgb==1 && _registration==1){
		device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	if(_depth==1 && _rgb==1 && _sync==1){
		device.setDepthColorSyncEnabled(true);
	}
	
	if(resolution==1){
		if(_depth){
			rc = depth.setVideoMode(device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[0]);
		}
	}
	
	


	VideoFrameRef frame;
	VideoFrameRef colorframe;
	int changedStreamDummy;
	VideoStream* pStream;
	VideoStream* pColorStream;
	//33 ms as delay
	if(_depth==1 && _rgb==0){
		printf("getting first depth data\n");
		fflush(stdout);
		pStream = &depth;
		OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy,-1);
		depth.readFrame(&frame);
		printf("ready\n");
		fflush(stdout);
	}
	if(_rgb==1 && _depth==0){
		printf("getting first rgb data\n");
		fflush(stdout);
		pColorStream = &color;
		OpenNI::waitForAnyStream(&pColorStream, 1, &changedStreamDummy,-1);
		color.readFrame(&colorframe);
		printf("ready\n");
		fflush(stdout);

	}
	//==================================================


	sensor_msgs::Image image;
	image.header.frame_id=frame_id;

	
	image.is_bigendian=0;
	printf("i'm here,_depth=%d _rgb=%d\n",_depth,_rgb);
	fflush(stdout);

	if(_depth==1 && _rgb==0){
		printf("publishing depth data\n");
		fflush(stdout);
		image.height=frame.getHeight();
		image.width=frame.getWidth();
		image.encoding="mono16";
		image.step=frame.getWidth()*2;
		image.data.resize(image.step*image.height);
		while (ros::ok()){
			memcpy((char*)(&image.data[0]),frame.getData(),image.height*image.width*2);
			xtionberry_pub_depth.publish(image);
			ros::spinOnce();
			loop_rate.sleep();
			OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy,33);
			depth.readFrame(&frame);
		}
	}
	else if(_rgb==1 && _depth==0){
		printf("publishing rgb data\n");
		fflush(stdout);
		image.height=colorframe.getHeight();
		image.width=colorframe.getWidth();
		image.encoding="rgb8";
		image.step=colorframe.getWidth()*3;
		image.data.resize(image.step*image.height);
		while (ros::ok()){
			memcpy((char*)(&image.data[0]),colorframe.getData(),image.height*image.width*3);
			xtionberry_pub_color.publish(image);
			ros::spinOnce();
			loop_rate.sleep();
			OpenNI::waitForAnyStream(&pColorStream, 1, &changedStreamDummy,33);
			color.readFrame(&colorframe);
		}
	}
	else if(_rgb==1 && _depth==1){
		printf("publishing rgb and depth data\n");
		fflush(stdout);
		while (ros::ok()){
			openni::OpenNI::waitForAnyStream(streams, 2, &changedStreamDummy,33);
			switch (changedStreamDummy)
			{
			case 0:
				depth.readFrame(&frame);
				if(!frame.isValid()) break;
				image.height=frame.getHeight();
				image.width=frame.getWidth();
				image.encoding="mono16";
				image.step=frame.getWidth()*2;
				image.data.resize(image.step*image.height);
				memcpy((char*)(&image.data[0]),frame.getData(),image.height*image.width*2);
				xtionberry_pub_depth.publish(image);
				break;
			case 1:
				color.readFrame(&colorframe);
				if(!colorframe.isValid()) break;
				image.height=colorframe.getHeight();
				image.width=colorframe.getWidth();
				image.encoding="rgb8";
				image.step=colorframe.getWidth()*3;
				image.data.resize(image.step*image.height);
				memcpy((char*)(&image.data[0]),colorframe.getData(),image.height*image.width*3);
				xtionberry_pub_color.publish(image);
				break;
			default:
				printf("Error in wait\n");
			}

			/*
			//DEPTH
					
			OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy,-1);
			depth.readFrame(&frame);
			image.height=frame.getHeight();
			image.width=frame.getWidth();
			image.encoding="mono16";
			image.step=frame.getWidth()*2;
			xtionberry_pub_depth.publish(image);
			//COLOR
			printf("c");
			fflush(stdout);			
			OpenNI::waitForAnyStream(&pColorStream, 1, &changedStreamDummy,-1);
			color.readFrame(&colorframe);
			image.height=colorframe.getHeight();
			image.width=colorframe.getWidth();
			image.encoding="rgb8";
			image.step=colorframe.getWidth()*3;
			memcpy((char*)(&image.data[0]),colorframe.getData(),image.height*image.width*3);
			xtionberry_pub_color.publish(image);
			
			printf(".");
			fflush(stdout);			
			
*/
			ros::spinOnce();
			loop_rate.sleep();
			

		}
	}
	
	
	depth.stop();
	depth.destroy();
	color.stop();
	color.destroy();
	device.close();
	OpenNI::shutdown();
	
	return 0;
}


















	//INFO ON DEVICE
	/*cout << "Depth modes" << endl;
    const openni::SensorInfo* sinfo = device.getSensorInfo(openni::SENSOR_DEPTH); // select index=4 640x480, 30 fps, 1mm
    const openni::Array< openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
    for (int i = 0; i<modesDepth.getSize(); i++) {
        printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(),
            modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
    }
	*/
	
	//320x240 = 0
	//160x120 = 6
