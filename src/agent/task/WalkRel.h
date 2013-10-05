/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WalkRel.h 2011-03-07 TT $
 *
 ****************************************************************************/


#ifndef TASK_WALKREL_H
#define TASK_WALKREL_H


#include "Task.h"
#include "math/Math.hpp"

//#define ENABLE_TASK_WALKREL_LOG

#ifdef ENABLE_TASK_WALKREL_LOG
	#include "logger/Logger.h"
#else
	#include "logger/NoLogger.h"
#endif


namespace task{

class WalkRel: public Task
{
public:

	/**
	 * create a walk task by walking target and
	 * the direction while reach the target position
	 *
	 * @param target target position
	 * @param direction the direction while reach the target position
	 * @param primary the primary task which create this task
	 *
	 */
	WalkRel(const seumath::Vector2f& target,
			seumath::AngDeg direction=0,
			bool goStop =false,
			bool avoidBall=false,
			Task* primary=NULL);       //add by g&q

	virtual bool isDone() const;

	virtual bool revise( boost::shared_ptr<const Task> rt );

	virtual void updateSubTaskList();
/*
	static void setWalkHeight( float h )
	{
		mWalkHeight = h;
	}

	static float getWalkHeight()
	{
		return mWalkHeight;
	}
*/
	///////////////////////////////// TT test
	void stopWalk()
	{
		mShouldStop=true;
		mTarget=seumath::Vector2f(0,0);
		mDirection=0;
	}

	float getStepLength()
	{
		return mPreSize.length();
	}



private:
	/// the target of the walk task
	seumath::Vector2f mTarget;

	/// the desired driection while reach the target
	seumath::AngDeg mDirection;

	seumath::Vector2f m2PreSize;
	seumath::Vector2f mPreSize;

	/// vaiables of path planning
	seumath::Vector2f mPlanTarget;
	seumath::AngDeg mPlanDir;

	/// the error threshold for isDone
	seumath::Vector2f mSizeErrorThreshold;
	seumath::AngDeg mDirErrorThreshold;

	//static float mWalkHeight;
	float mBodyHeight;
	
	static float mBodyHeight0;
	static float mBodyHeight1;
	static float mBodyHeight2;

	float mIsWalking;

	/// should I avoid the ball
	bool mAvoidBall;

	//TT, for Walk and WalkRel
	bool mShouldStop;
	//dpf, mShouldStop is to stop, but need one more step.mMustStop is no more steps;
	//bool mMustStop;
	// this class handls logging
	/// for reduce my walk Vec
	bool mGoStop;
	
	DECLARE_STATIC_GRAPHIC_LOGGER;
}; //end of class WalkRel


} //end of namespace task

#endif //TASK_WALKREL_H

