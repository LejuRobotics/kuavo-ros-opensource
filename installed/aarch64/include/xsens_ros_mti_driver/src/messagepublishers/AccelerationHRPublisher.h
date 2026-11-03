
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef ACCELERATIONHRPUBLISHER_H
#define ACCELERATIONHRPUBLISHER_H

#include "packetcallback.h"

#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9))

struct AccelerationHRPublisher : public PacketCallback
{

    AccelerationHRPublisher()
    {
        int pub_queue_size = 5;
    }

    void operator()(const XsDataPacket &packet, struct timespec timestamp)
    {
        if (packet.containsAccelerationHR())
        {
            static struct timespec t0_sol, t1_sol;
            clock_gettime(CLOCK_MONOTONIC, &t1_sol);
            printf("acc HR period(ms): %f\n",
                    TIME_DIFF(t0_sol, t1_sol) * 1000.0);
            clock_gettime(CLOCK_MONOTONIC, &t0_sol);

            // geometry_msgs::Vector3Stamped msg;

            // msg.header.stamp = timestamp;
            // msg.header.frame_id = frame_id;

            XsVector accel = packet.accelerationHR();
            printf("acc: %f\n", accel[1]);
            // msg.vector.x = accel[0];
            // msg.vector.y = accel[1];
            // msg.vector.z = accel[2];

            // pub.publish(msg);
        }
    }
};

#endif
