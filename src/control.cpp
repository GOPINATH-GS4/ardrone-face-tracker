//
//  Control.cpp
//  opencv
//
//  Created by janakiraman gopinath on 2/3/15.
//  Copyright (c) 2015 gopi. All rights reserved.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include "PID.h"
using namespace std;
using namespace cv;
bool no_face = true;
/**
 * @function main
 */

#include "control.h"
String fcn = "./haarcascade_frontalface_alt.xml";
String upperbody = "./haarcascade_frontalface_default.xml";
CascadeClassifier fc, up;

PID pid(.8,0,.5);
PID pidx(.5,0,.35);
PID pidy(.5,0,.35);
PID pidz(.3,0,.35);
bool firstTime = true;
bool prevSet = false;
markers bounds;
struct COMMAND prev;
int frames_without_image = 0;

markers fd(cv::Mat src)
{
	no_face = true;
    std::vector<Rect> faces;
    cv::Mat frame_gray;
    
    cvtColor( src, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    //-- Detect UpperBody
//    fc.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	fc.detectMultiScale( frame_gray, faces, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    
    Rect largestface;
	Mat faceROI;
	std::vector<Rect> f;
    CvPoint point;
    markers m;
	m.x = m.y = m.radius = m.area = 0;

    for( size_t i = 0; i < faces.size(); i++ ) {

        if(i == 0) 
			largestface = faces[i];
        else if ((largestface.width * largestface.height) < (faces[i].width * faces[i].height))
            largestface = faces[i];

		faceROI =  frame_gray(faces[i]);
	}

	//printf("Number of faces detected %d\n", faces.size());
	/***
	if(faceROI.rows > 0 && faceROI.cols > 0) {

   		 //-- Detect face from upper Body
    	fc.detectMultiScale( faceROI, f, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		//printf("face size %d\n", f.size());
		if(f.size() !=1 ) { return m;}
		no_face = false;		
	}
	else {
		//printf("faceROI empty\n");
		return m;
	}
	**/

    if(largestface.height > 0 && largestface.width > 0) {
        Point center( largestface.x + largestface.width*0.5, largestface.y + largestface.height*0.5 );
        int radius = largestface.width > largestface.height ? largestface.width/2 : largestface.height/2;
        circle( src, center, radius, Scalar( 255, 0, 255 ), 4, 8 );
		/***
        //ellipse( src, center, Size( largestface.width*0.5, largestface.height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
//        printf("Area = %lf\n", M_PI * radius * radius);
		for( size_t j = 0; j < f.size(); j++ ) {
			Point center1( largestface.x + f[j].x + f[j].width*0.5, largestface.y + f[j].y + f[j].height*0.5 );
			int radius = cvRound( (f[j].width + f[j].height)*0.25 );
			circle( src, center1, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
		}
		**/
        point = center;
        m.x = center.x;
        m.y = center.y;
		//printf("Radius %d\n" , radius);
        m.area = M_PI * radius * radius;
		m.radius = radius;
		no_face = false;
    }

    return m;
}

void debug(PID pid) {
    
    printf("******************************************************************\n");
    printf("Set Point : X->%d, Y->%d area->%lf\n", pid.getSetPoint().x, pid.getSetPoint().y, pid.getSetPoint().area);
    printf("Output : X->%d, Y->%d area->%lf\n", pid.getOutput().x, pid.getOutput().y, pid.getOutput().area);
    printf("Direction : %d\n", pid.getDirection());
    printf("Input : %f\n", pid.getInput());
    printf("Area : %f\n", pid.getArea());
    printf("******************************************************************\n");
	fflush(stdout);
    
}
void debug(struct COMMAND c1) {
    printf("******************************************************************\n");
    printf("X = %lf, Y=%lf, Z=%lf\n", c1.commandx, c1.commandy, c1.commandz);
    printf("******************************************************************\n");
}
struct COMMAND calculate_command(PID pid_lr, PID pid_ud, PID pid_fb) {
    
	struct COMMAND c1;
    
	if(no_face)  {
		frames_without_image++;	
		if(prevSet) {
			c1.commandx = -1 * prev.commandx;
			c1.commandy = -1 * prev.commandy;
			c1.commandz = -1 * prev.commandz;
			prevSet = false;
		} 
		else 
			c1.commandx = c1.commandy = c1.commandz = 0;
	}
	else {
    	c1.commandx = pid_lr.getDirection() *  pid_lr.getInput() / bounds.x;
    	c1.commandy = pid_ud.getDirection() * pid_ud.getInput() / bounds.y;
    	c1.commandz = pid_fb.getDirection() * pid_fb.getInput() / bounds.radius;
		prev = c1;
		prevSet = true;
		frames_without_image = 0;
	}
    return c1;
}
void reset_frames_without_image() {
	frames_without_image = 0;
}
int get_frames_without_image() {
	return frames_without_image;
}

struct COMMAND control(cv::Mat src) {
    
  
       
	//printf("Cols %d, Rows %d\n" , src.cols, src.rows);
	if (firstTime) {

		struct COMMAND c;
		c.commandx = c.commandy = c.commandz = 0.0;
    	if( !fc.load( fcn ) ){ printf("--(!)Error loading\n"); return c;}
    	if( !up.load( upperbody ) ){ printf("--(!)Error loading\n"); return c;}
		markers p;

		bounds.x = src.cols;
		bounds.y = src.rows;
		bounds.area = (src.rows/ 6) * (src.rows/ 6) * 3.14; 
		bounds.radius = (src.rows / 6);
		printf("Bounds area : %f\n", bounds.area);
		printf("bounds radius: %f\n" , bounds.radius);
		p.x = src.cols/2;
		p.y = src.rows/2;
		p.area = bounds.area;
		p.radius = bounds.radius;
		firstTime = false;
		pid.setPoint(p);
		pidx.setPoint(p);
		pidy.setPoint(p);
		pidz.setPoint(p);
		pidx.setBounds(bounds);
		pidy.setBounds(bounds);
		pidz.setBounds(bounds);
	}
    
    /* Detect the face and show it on the screen */
        
     markers position = fd(src);
        
     /* Make sure the center point of the detected face is within the image */
        
     if(position.x != 0 && position.y != 0 && position.x <= src.cols
     && position.y <= src.rows) {
        
        // Exit loop pid which monitors the goal state
		markers setPoint, tmp;
		tmp = position;

		setPoint = pid.getSetPoint();
            
		pid.setOutput(position);
        pid.compute(POINTS);
        // debug(pid);
            
		position = tmp;
        // Control Drones Roll (left/right)
        setPoint = pidx.getSetPoint();
        position.y = -1;
        pidx.setOutput(position);
        pidx.compute(POINTS);
        //debug(pidx);
            
		position = tmp;
        // Control drones faz (climb/desent)
        setPoint = pidy.getSetPoint();
        position.x = -1;
        pidy.setOutput(position);
        pidy.compute(POINTS);
        //debug(pidy);
            
		position = tmp;
		position.x = -1;
		position.y = -1;
        // Control drones pitch (forward/backward)
        setPoint = pidz.getSetPoint();
        pidz.setOutput(position);
        pidz.compute(POINTS);
        //debug(pidz);
            
	} else {
	}

   return calculate_command(pidx, pidy, pidz);
}
