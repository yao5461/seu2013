/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WalkRel.cpp 2011-03-07 TT $
 *
 ****************************************************************************/

#include <list>
#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "Step.h"
#include "WalkRel.h"


namespace task{

using namespace std;
using namespace boost;
using namespace seumath;
using namespace serversetting;
using namespace action;

float WalkRel::mBodyHeight0 =0.325f;
float WalkRel::mBodyHeight1 =0.35f;
float WalkRel::mBodyHeight2 =0.33f;

///float WalkRel::mBodyHeight0 =0.33f;
///float WalkRel::mBodyHeight1 =0.38f;
///float WalkRel::mBodyHeight2 =0.33f;
///the origin value  by   ghd   change  by visen


//float WalkRel::mWalkHeight = 0.35f;

DEFINE_STATIC_GRAPHIC_LOGGER(WalkRel)


WalkRel::WalkRel(const Vector2f& target, AngDeg direction,bool goStop, bool avoidBall ,Task* primary)
	:Task(-1, primary), // -1: as soon as possible
	mTarget(target),
	mDirection(direction),
	mSizeErrorThreshold(0.01f,0.01f),
	mDirErrorThreshold(3),
	mIsWalking(false),
	mGoStop(goStop),
	mAvoidBall(avoidBall)
{
	BEGIN_ADD_STATIC_LOG_LAYER(WalkRel)
		ADD_LOG_LAYER("newWalk")
		ADD_LOG_LAYER("newStep")
		ADD_LOG_LAYER("planPath")
		ADD_LOG_LAYER("blocks")
	END_ADD_STATIC_LOG_LAYER(WalkRel)

	mPreSize.zero();
	m2PreSize.zero();
	mShouldStop=false;
}


bool WalkRel::isDone() const
{
	if(!mIsWalking)
		return false;

	return mSubTaskList.empty();
}


bool WalkRel::revise( shared_ptr<const Task> rt )
{
	///////////////////////////////// test
	//printf("WalkRel: size= %d\n",getSubTaskListSize());
	/////////////////////////////////

	shared_ptr<const WalkRel> wrt=shared_dynamic_cast<const WalkRel>(rt);
	if( NULL != wrt.get() )
	{
		// accept another WalkRel
		mTarget = wrt->mTarget;
		mDirection = wrt->mDirection;
		mSizeErrorThreshold = wrt->mSizeErrorThreshold;
		mDirErrorThreshold = wrt->mDirErrorThreshold;
		mAvoidBall = wrt->mAvoidBall;
		mGoStop =wrt->mGoStop;
		//std::cout<<"!!"<<mTarget.length()<<std::endl;
		return true;
	}

	else
		return false;
}


void WalkRel::updateSubTaskList()
{
	//if it's too early to calculate next step, then return
	Task::updateSubTaskList();
	if( false==isSubTaskOfAllLessThanTwo() )
		return;
	
	//...
	mIsWalking=true;

	//...
	bool isLeft=true;
	AngDeg preDir=0;
	shared_ptr<const Step> cStep;  //current step
	if( !mSubTaskList.empty() )
	{
		cStep=shared_dynamic_cast<const Step>( mSubTaskList.front() ); //get the last step
		if( 0!=cStep.get() )
		{
			isLeft= !cStep->isLeft(); //change foot
			preDir=cStep->dir();
			mPreSize=cStep->size();
			
// 			c2PreStep = cStep->getPreStep() ;	//ghd
// 			if(c2PreStep.get() !=0)
// 			{
// 			    m2PreSize =c2PreStep ->size();
// 			}
		}
	}

	Vector2f size;
	//TT, for Walk and WalkRel
	
	if(mShouldStop){
		return;
	}
	else{
	size=mTarget*0.5; //NOTE: the real step size is double "size"
		}
	
	switch(FM.getMy().heteroType)
	{
	  case 0 :mBodyHeight =mBodyHeight0;
		  break;
	  case 1 :mBodyHeight =mBodyHeight1;
		  break;
	  case 2 :mBodyHeight =mBodyHeight2;
		  break;
	}
	
		//cout<<mGoStop<<endl;
	//std::cout<<size<<std::endl;
	shared_ptr<Task> mstep( new Step(isLeft,
									 size,
									 mDirection,
									 cStep,
									 mBodyHeight,
									 mGoStop,
									 this
									 ) );
	mSubTaskList.push_back(mstep);
}


} //end of namespace task

