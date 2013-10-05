
/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 *
 * Author Gao HaiDan
 *
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_DYNAMICKICK_H
#define TASK_DYNAMICKICK_H

#define ENABLE_TASK_KICKTASK_LOG

#include "Task.h"
#include "math/Math.hpp"
#ifdef ENABLE_TASK_KICKTASK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task {

using namespace std;

class DynamicKick: public Task
{
public:
    DynamicKick( const seumath::Vector2f& Target,float bodyHeight,Task* primary =NULL,bool useBest =true);

    virtual boost::shared_ptr<action::Action> perform();

    bool isDone () const;

private:


    seumath::Vector2f mTarget;
    
    float mIsFirstTime;

    bool mUseBest;

    bool mIsLeft;
    
    //kick
    seumath::Vector2f mSwingVec;
    
    //ready to kick
    seumath::Vector2f mReadVec;
    
    seumath::Vector2f mMaxVec;

};


}

#endif // TASK_DYNAMICKICK_H

