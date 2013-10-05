
/***************************************************************************
*                              SEU RoboCup Simulation Team
*                     -------------------------------------------------
* Copyright (c) Southeast University , Nanjing, China.
*
* Author GaoHaiDan
*
* All rights reserved.
*
* $Id$
*
****************************************************************************/

#include "Task.h"
#include "DynamicKick.h"
#include "SwingFoot.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include <fstream>
#include <sstream>

namespace task {

using namespace boost;
using namespace action;
using namespace std;
using namespace robot::humanoid;
using namespace controller;
using namespace perception;

//DEFINE_STATIC_GRAPHIC_LOGGER(KickTask)


DynamicKick::DynamicKick( const seumath::Vector2f& Target,float bodyHeight,Task* primary,bool useBest):
    Task(-1, primary ),
    mTarget(Target),
    mUseBest(useBest),
    mIsFirstTime(true),
    mMaxVec(seumath::Vector2f(0.1f,0.19f)),
    mIsLeft(false)
{
    seumath::Vector2f ballRelPos =WM.getBallRelPos2D();
    seumath::AngDeg ballRelDir =ballRelPos.angle();
    seumath::AngDeg ballAng =-seumath::atan2Deg<AngDeg>(ballRelPos.x() , ballRelPos.y());
    
    if(ballAng >0.0f)
      mIsLeft =true;
    else 
      mIsLeft =false;
    
    mSwingVec.x() =0.17f * seumath::cosDeg<AngDeg>(ballRelDir);
    mSwingVec.y() =0.17f * seumath::sinDeg<AngDeg>(ballRelDir);
//     cout<<mSwingVec<<endl;
    
    mReadVec = -0.5f * mSwingVec;
      
    
  
    float height =0.04f;
    shared_ptr<Task> swingFoot8(new SwingFoot(!mIsLeft, mIsLeft ? seumath::Vector2f(0.08f,0) :	seumath::Vector2f(-0.08f,0) ,//right foot,just for test  --ghd
                                0,
                                0.0f,
                                0,
                                bodyHeight, 0.05f, this));
    mSubTaskList.push_back(swingFoot8);
    
    shared_ptr<Task> swingFoot9(new SwingFoot(mIsLeft, seumath::Vector2f(0,0),	//right foot,just for test  --ghd
                                0,
                                0.0f,
                                0,
                                bodyHeight, 0.05f, this , mIsLeft ? RIGHTSUPPORT : LEFTSUPPORT));
    mSubTaskList.push_back(swingFoot9);
    
    shared_ptr<Task> swingFoot0(new SwingFoot(mIsLeft, mReadVec,	//right foot,just for test  --ghd
                                height,
                                0,
                                0,
                                bodyHeight, 0.08f, this , mIsLeft ? RIGHTSUPPORT : LEFTSUPPORT));
    mSubTaskList.push_back(swingFoot0);
    
    shared_ptr<Task> swingFoot1(new SwingFoot(mIsLeft, mSwingVec,	//right foot,just for test  --ghd
                                height,
                                0,
                                0,
                                bodyHeight, 0.04f, this , mIsLeft ? RIGHTSUPPORT : LEFTSUPPORT));
    mSubTaskList.push_back(swingFoot1);
    
    shared_ptr<Task> swingFoot2(new SwingFoot(mIsLeft, mSwingVec,	//right foot,just for test  --ghd
                                0.00f,
                                0.0f,
                                0,
                                bodyHeight, 0.1f, this));
    mSubTaskList.push_back(swingFoot2);
    
    shared_ptr<Task> swingFoot3(new SwingFoot(mIsLeft, seumath::Vector2f(0.0f,0.0f),	//right foot,just for test  --ghd
                                0.00f,
                                0.0f,
                                0,
                                bodyHeight, 0.5f, this));
    mSubTaskList.push_back(swingFoot3);
}


shared_ptr<Action> DynamicKick::perform()
{
    shared_ptr<Action> act = Task::perform();

    if (0 == act.get()) {
        // there is no sub task
        shared_ptr<JointAction> jact(new JointAction);
        jact->fill(0);
        act = shared_static_cast<Action > (jact);
    }

    return act;
} //end of KickTask::perform()
bool DynamicKick::isDone () const
{
    return Task::isDone();
}



} //end of namespace task

