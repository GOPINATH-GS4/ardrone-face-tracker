//
//  PIDTest.h
//  opencv
//
//  Created by janakiraman gopinath on 2/3/15.
//  Copyright (c) 2015 gopi. All rights reserved.
//

#ifndef __opencv__PIDTest__
#define __opencv__PIDTest__

#include <stdio.h>
#include "opencv2/core/core.hpp"

int Control(cv::Mat);
#endif /* defined(__opencv__PIDTest__) */

struct COMMAND {
	float commandx;
	float commandy;
	float commandz;
};
