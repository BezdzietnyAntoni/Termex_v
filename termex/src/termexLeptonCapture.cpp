#include <iostream>
#include <stdio.h>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

#include "LeptonIntercept.h"

//If u want publish raw image uncoment
#define RAW_IMAGE

//If u want to publish scaled image to topics "/lepton_scaled" uncomment 
//#define SCALED_IMAGE

//If u want resize image from "/lepton_scaled" uncomment 
//#define RESIZE


void scaleImageU8(uint16_t*, uint8_t*, int, int);
void comadCallback(const std_msgs::String::ConstPtr&);
void handleComand(std::string);

LeptonIntercept cameraLepton;

int main(int argc, char **argv)
{       
    //frame rate
    int FREQ;
    
    //initialize node 
    ros::init(argc, argv, "lepton_capture");
    //set private namespace
    ros::NodeHandle n("~");
    
    //get parameters command line _freq:=8
    n.getParam("freq", FREQ);
    
    //FREQ must have value between 0 to 8 
    if(FREQ > 8 || FREQ < 1){
        FREQ = 8;
        ROS_INFO("[WARNING] _freq must be a value between [0, 8]");
    }
    
    ROS_INFO("[INFO] Running termexLeptonCaptur, freq=%d",FREQ);
    
    ros::Subscriber sub = n.subscribe("/comand_console", 10, comadCallback);

    
#ifdef RESIZE
    //resize value 
    int resize = 5;
#endif
  
#ifdef RAW_IMAGE    
    image_transport::ImageTransport it_(n);
    image_transport::Publisher image_pub = it_.advertise("/lepton_output", 10);
    
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr -> encoding = "mono16";
#endif
    
#ifdef SCALED_IMAGE
    image_transport::ImageTransport it2_(n);
    image_transport::Publisher image_scaled_pub = it2_.advertise("/lepton_scaled", 10);
    
    cv_bridge::CvImagePtr cv_ptr_scaled(new cv_bridge::CvImage);
    cv_ptr_scaled -> encoding = "mono8";
#endif

	
	cameraLepton = LeptonIntercept();
    cameraLepton.setSpiSpeedMHz(20);  
    cameraLepton.connect();
    if( !cameraLepton.isConnected() )
        ROS_INFO("[Error] Failed connect SPI");
    
    int image_columns = cameraLepton.getImageColumns();
    int image_rows    = cameraLepton.getImageRows();
    uint16_t *image_pointer; //here can be use cameraLepton.getImagePointer();
    
    cv::Mat raw_image(image_rows, image_columns, CV_16U);
    
#ifdef SCALED_IMAGE
    uint8_t scaled_image_pointer[image_columns*image_rows];
    cv::Mat scaled_image(image_rows, image_columns, CV_8U);
    #ifdef RESIZE
    cv::Mat resized_scaled_image;
    #endif
    
#endif
    
    ros::Rate loop_rate(FREQ);
    
    while (ros::ok())
    {
        
		//its bad name function (refresh stored vector in LeptonIntercept)
		cameraLepton.getFrame();
        image_pointer = cameraLepton.getImagePointer();

#ifdef RAW_IMAGE
        std::memcpy(raw_image.data, image_pointer, image_rows*image_columns*sizeof(uint16_t));
        cv_ptr -> header.stamp = ros::Time::now();
        cv_ptr -> header.frame_id = "/lepton_output";
        cv_ptr -> image = raw_image;
        
        image_pub.publish(cv_ptr->toImageMsg());            
        //ROS_INFO("Raw image Send!");	
#endif 
        
#ifdef SCALED_IMAGE
        scaleImageU8(image_pointer, scaled_image_pointer, image_columns, image_rows);
        std::memcpy(scaled_image.data, scaled_image_pointer, image_rows*image_columns*sizeof(uint8_t));
        
    #ifdef RESIZE
        cv::resize(scaled_image, resized_scaled_image, cv::Size(scaled_image.cols*5, scaled_image.rows*5));
    #endif
        
        cv_ptr_scaled -> header.stamp = ros::Time::now();
        cv_ptr_scaled -> header.frame_id = "/lepton_scaled";
    #ifdef RESIZE
        cv_ptr_scaled -> image = resized_scaled_image;
    #else
        cv_ptr_scaled -> image = scaled_image;
    #endif

        image_scaled_pub.publish(cv_ptr_scaled->toImageMsg());            
        //ROS_INFO("Scaled image Send!");
        
#endif

  
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void comadCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Get command: [%s]", msg->data.c_str());
    handleComand(msg->data);
}

void handleComand(std::string com){
    if(com == "FFC"){
        ROS_INFO("Reset FFC");
        cameraLepton.performFFC();
    }else{
        ROS_INFO("Unknown command");
    }          
            
}

#ifdef SCALED_IMAGE
void scaleImageU8(uint16_t* imgU16, uint8_t* imgU8, int column, int row){
    int min = imgU16[0];
    int max = imgU16[0];
    
    for(int i=1; i<column*row; ++i){
        if(imgU16[i] < min)
            min = imgU16[i];
        if(imgU16[i] > max)
            max = imgU16[i];
    }
    
    float diff = max - min;
    
    for(int i=0; i<column*row; ++i){
        imgU8[i] = uint8_t(((imgU16[i]-min)/diff)*255);
    }
}
#endif 
