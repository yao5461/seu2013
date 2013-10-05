/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _CONTROLLER_ARMMOTION_H_
#define _CONTROLLER_ARMMOTION_H_

#include <boost/shared_ptr.hpp>
#include "action/JointAction.h"

namespace controller{

    class ArmMotion
    {
    public:
        static boost::shared_ptr<action::Action> control();

        static boost::shared_ptr<action::Action> control(boost::shared_ptr<action::JointAction> ja);

        static void control(std::map<unsigned int, seumath::AngDeg>& angles);

        static void control(std::map<unsigned int, seumath::AngDeg>& angles,
                            const seumath::Vector3f& pl, const seumath::Vector3f& pr, const seumath::Vector3f& pt);
        
    private:
        // do not try to create an instance
        ArmMotion();
    };
    
} // namespace controller

#endif /* _CONTROLLER_ARMMOTION_H_ */
