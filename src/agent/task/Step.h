/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Step.h 1894 2008-07-02 13:09:05Z xy $
 *
 ****************************************************************************/

#ifndef TASK_STEP_H
#define TASK_STEP_H

#define ENABLE_TASK_STEP_LOG

#include "Task.h"
#include "math/Math.hpp"
#ifdef ENABLE_TASK_STEP_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task {

    class Step: public Task
    {
    public:
        Step( bool isLeft,
              const seumath::Vector2f& size, seumath::AngDeg dir,
              boost::shared_ptr<const Step> preStep,
              float bodyHeight,
	      bool goStop,
              Task* primary
	      );

        virtual bool isTerminable() const;

        virtual bool isDone() const;

        virtual boost::shared_ptr<action::Action> perform();

        /**
         * @return moving which foot
         */
        bool isLeft() const
            {
                return mIsLeft;
            }

        /**
         * @return the size of the step
         */
        const seumath::Vector2f& preSize() const
            {
                return mPreSize;
            }

        const seumath::Vector2f& preSizeAcc() const
            {
                return mPreSizeAcc;
            }

        seumath::AngDeg predir() const
            {
                return mPreDir;
            }

        /**
         * @return the size of the step
         */
        const seumath::Vector2f& size() const
            {
                return mSize;
            }

        const seumath::Vector2f& sizeAcc() const
            {
                return mSizeAcc;
            }

        /**
         * @return the direction of the step
         */
        seumath::AngDeg dir() const
            {
                return mDir;
            }
            
        seumath::AngDeg dirAcc() const 
	    {
		return mDirAcc;
	    }
            
        int walkType() const
	    {
		return mWalkType;
	    }
        
         void setMaxSizeAcc( const seumath::Vector2f& v )
            {
                mMaxSizeAcc = v;
            }

         void setMaxSize( const seumath::Vector2f& v )
            {
                mMaxSize = v;
            }

         void setMaxDirAcc(seumath::AngDeg a)
            {
                mMaxDirAcc = a;
            }

         void setMaxDir(seumath::AngDeg a)
            {
                mMaxDir = a;
            }

         const seumath::Vector2f& getMaxSizeAcc()
            {
                return mMaxSizeAcc;
            }

         const seumath::Vector2f getMaxSize()
            {
                return mMaxSize;
            }

         seumath::AngDeg getMaxDirAcc()
            {
                return mMaxDirAcc;
            }

         seumath::AngDeg getMaxDir()
            {
                return mMaxDir;
            }

//         static seumath::AngDeg getStepTime()
//             {
//                 return mStepTime;
//             }
           
//
    protected:

        /// configuration of the walking step
        seumath::Vector2f mMaxSizeAcc;
        seumath::Vector2f mMinSize;
        seumath::Vector2f mMaxSize;
        seumath::AngDeg mMaxDirAcc;
        seumath::AngDeg mMaxDir;
        seumath::AngDeg mMinDir;
        float mStep1Time;
	float mStep2Time;
	float mStep3Time;
	float mh1;
	float mh2;
	int mi1;
	int mi2;
	
	//ghd
	seumath::Vector2f mMaxSize2Acc;
	seumath::AngDeg mMaxDir2Acc;
	
	/// next step
	/// steping left foot or right foot
        bool mIsLeft;

        /// the step size
        seumath::Vector2f mSize;

        /// the size accerelation
        seumath::Vector2f mSizeAcc;

        /// the direction change
        seumath::AngDeg mDir;
	
	//ghd
	seumath::AngDeg mDirAcc;
	
	///current step
	seumath::Vector2f mPreSize;
        seumath::Vector2f mPreSizeAcc;
        seumath::AngDeg mPreDir;
	seumath::AngDeg mPreDirAcc;
	
	seumath::Vector2f mTempSize;
	
	///0 : front-side
	///1 : front
	///2 : side
	///3 : turn (don't walk)
	///4 : stop (when the match begin)
	int mWalkType;
	
	bool mHaveLast;	//is it have last step
	
	///for hetero	--ghd
	///hetero 0
	static seumath::Vector2f mMaxSizeAcc0;
        static seumath::Vector2f mMinSize0;
        static seumath::Vector2f mMaxSize0;
        static seumath::AngDeg mMaxDirAcc0;
        static seumath::AngDeg mMaxDir0;
        static seumath::AngDeg mMinDir0;
        static float mStep1Time0;
	static float mStep2Time0;
	static float mStep3Time0;
	static float mh10;
	static float mh20;
	static int mi10;
	static int mi20;
	static seumath::Vector2f mMaxSize2Acc0;
	static seumath::AngDeg mMaxDir2Acc0;
	
	///hetero 1
	static seumath::Vector2f mMaxSizeAcc1;
        static seumath::Vector2f mMinSize1;
        static seumath::Vector2f mMaxSize1;
        static seumath::AngDeg mMaxDirAcc1;
        static seumath::AngDeg mMaxDir1;
        static seumath::AngDeg mMinDir1;
        static float mStep1Time1;
	static float mStep2Time1;
	static float mStep3Time1;
	static float mh11;
	static float mh21;
	static int mi11;
	static int mi21;
	static seumath::Vector2f mMaxSize2Acc1;
	static seumath::AngDeg mMaxDir2Acc1;
	
	///hetero 2
	static seumath::Vector2f mMaxSizeAcc2;
        static seumath::Vector2f mMinSize2;
        static seumath::Vector2f mMaxSize2;
        static seumath::AngDeg mMaxDirAcc2;
        static seumath::AngDeg mMaxDir2;
        static seumath::AngDeg mMinDir2;
        static float mStep1Time2;
	static float mStep2Time2;
	static float mStep3Time2;
	static float mh12;
	static float mh22;
	static int mi12;
	static int mi22;
	static seumath::Vector2f mMaxSize2Acc2;
	static seumath::AngDeg mMaxDir2Acc2;
	
	

    private:
        // this class handls logging
        DECLARE_STATIC_GRAPHIC_LOGGER;
    };

} // namespace task


#endif // TASK_STEP_H

