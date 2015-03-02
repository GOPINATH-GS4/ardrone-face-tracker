//
//  PID.h
//  opencv
//
//  Created by janakiraman gopinath on 1/4/15.
//  Copyright (c) 2015 gopi. All rights reserved.
//
#include <iostream>
#include <stdio.h>
#ifndef __opencv__PID__
#define __opencv__PID__



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#endif /* defined(__opencv__PID__) */
/* A wrapper of type T */

struct markers {
    int x;
    int y;
    int z;
	float radius;
    double area;
};

enum pidElement {POINTS, AREA};

class PID {
    
    unsigned long lastTime;
    double input;
    markers  output, setpoint;
    double errSum, lastErr;
    double kp, ki, kd;
    int edistance(markers point1, markers point2);
    float angle(markers point1, markers point2);
    short direction;
    int distance;
	float radius;
	float area;
    markers bounds;
    bool boundSet;
	bool debug;
public:
    PID(double kp, double ki, double kd);
    void setPoint(markers setpoint);
    void setOutput(markers output);
    void compute(pidElement p);
    double getInput();
    short getDirection();
    void setBounds(markers m);
    markers getSetPoint();
    static float getRMSE(markers m1, markers m2);
    markers getOutput();
    int getDistance();
	float getRadius();
	float getArea();
	void setDebug(bool debug);
    void init(double kp, double ki, double kd);
};


