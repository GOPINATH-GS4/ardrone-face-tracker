//
//  PID.cpp
//  opencv
//
//  Created by janakiraman gopinath on 1/4/15.
//  Copyright (c) 2015 gopi. All rights reserved.
//

#include "PID.h"




PID::PID(double Kp, double Ki, double Kd)
{
    this->init(Kp, Ki, Kd);
}

void PID::init(double Kp, double Ki, double Kd)
{
    this->kp = Kp;
    this->ki = Ki;
    this->kd = Kd;
    this->lastTime = 0;
    this->lastErr = INFINITY;
    this->errSum = this->lastErr = 0;
    this->boundSet = false;
	this->input = 0;
	this->distance = 0;
	this->direction = 0;
	this->area = 0.0;
}
int PID::edistance(markers point1, markers point2) {
    int diff = 0;
	if (point2.x == -1 && point2.y == -1) {

        if (point2.radius < 0) {
            this->distance = 0;
        }
        else {
            diff = (point1.radius - point2.radius);
            this->direction = diff / (diff !=0 ? abs(diff) : 1);
            this->distance = abs(point1.radius - point2.radius);
        }
        return this->distance;
	}
    else if(point2.x == -1) {
        
        if (this->boundSet && (point2.y > this->bounds.y || point2.y < 0)) {
            this->distance = 0;
        }
        else {
            diff = (point1.y - point2.y);
            this->direction = diff / (diff != 0 ? abs(diff) : 1);
            this->distance = abs(point1.y - point2.y);

        }
        return this->distance;
    }
    else if(point2.y == -1){
        
        if (this->boundSet && (point2.x > this->bounds.x || point2.x < 0)) {
            this->distance = 0;
        }
        else {
            diff = (point1.x - point2.x);
            this->direction = diff / (diff !=0 ? abs(diff) : 1);
            this->distance = abs(point1.x - point2.x);
        }
        return this->distance;
    }
    else {
        
        if (this->boundSet && (point2.y > this->bounds.y || point2.y < 0 || point2.x > this->bounds.x || point2.x < 0)) {
            this->distance = 0;
        }
        else {
            this->direction = 0;
            this->distance =  abs(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y,2)));
        }
        return this->distance;
    }
    
}
void PID::setBounds(markers m) {
    this->bounds = m;
    this->boundSet = true;
    
}

int PID::getDistance() {
    return this->distance;
}
float PID::angle(markers point1, markers point2) {
    std::cout << point1.y << ":" << point2.y << ":" << abs(point1.y - point2.y) << std::endl;
    std::cout << point1.x << ":" << point2.x << ":" << abs(point1.x - point2.x) << std::endl;
    return atanf(abs(point1.y - point2.y)/ abs(point1.x -point2.x)) * 180 / 3.14;

 
}
void PID::setDebug(bool debug) {
	this->debug = debug;
}
void PID::compute(pidElement p)
{
    unsigned long now = time(NULL);
    
    long dt = (now - this->lastTime);
    
    double de = 0;
    double error = 0;
	
    if (this->lastTime != 0 && dt > 0) {
         // Compute de (error derivation)
        if (this->lastErr < INFINITY) {
            if (p == POINTS) {
                 error = edistance(this->setpoint, this->output);
            }
            else {
                this->direction = 0;
                error = abs(this->setpoint.area - this->output.area);
                this->direction = error / (error != 0 ? abs(error) : 1);
				this->area = error;
            }
           
            de  = (error - this->lastErr) / dt;
        }
        // Integrate error
        this->errSum += error * dt;
    }

	if(debug) {
		printf("Error: %lf, Input:%lf, Setpoint(x): %d, Setpoint(y):%d,Setpoint(radius): %lf, Setpoint(area):%lf,Output(x):%d,Output(y):%d,Output(radius):%f,Output(area): %f, kp: %f,ki:%f,kd:%f\n", error, this->input, this->setpoint.x, this->setpoint.y, this->setpoint.radius, this->setpoint.area , this->output.x, this->output.y, this->output.radius,this->output.area,this->kp, this->ki, this->kd);
	}
	this->lastTime= now;
	this->lastErr = error;
	this->input = this->kp * error + this->ki * this->errSum  + this->kd * de;
}

float PID::getArea() {
	return this->area;
}
float PID::getRMSE(markers m1, markers m2) {
    
    //return sqrt(powf(m1.x - m2.x,2) + powf(m1.y-m2.y,2) + powf(m1.area-m2.area,2));
    return sqrt(powf(m1.area-m2.area,2));
}
markers PID::getSetPoint() {
    return this->setpoint;
}
markers PID::getOutput() {
    return this->output;
}

void PID::setPoint(markers setpoint) {
    this->setpoint = setpoint;
}
void PID::setOutput(markers output) {
    this->output = output;
}
double PID::getInput() {
    return this->input;
}
short PID::getDirection() {
    return this->direction;
}
