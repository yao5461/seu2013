/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_GYRORATESENSOR_H_
#define _ROBOT_DEVICE_SENSOR_GYRORATESENSOR_H_

#include "Sensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            class GyroRateSensor : public Sensor
            {
            public:
                GyroRateSensor();
            };
            
            
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_SENSOR_GYRORATESENSOR_H_ */
