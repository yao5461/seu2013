/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WorldModel.h 2733 2009-03-31 12:24:31Z cs $
 *
 ****************************************************************************/

#ifndef CORE_WORLD_MODEL
#define CORE_WORLD_MODEL

#include "configuration/Configuration.h"
#include "Singleton.hpp"
#include "perception/Perception.h"
#include "perception/Vision.h"
#include "math/TConvexPolygon.hpp"
#include "robot/humanoid/Humanoid.h"
#include "perception/ExtenedKalmanFilter.hpp"
#include "PassModel.h"
#include <set>

namespace core {

    using namespace perception;
    using namespace std;
    using namespace boost;
    using namespace seumath;
    using namespace serversetting;
    using namespace robot;
    using namespace robot::humanoid;
    using namespace action;
    using namespace perception;

#define MAX_VEL_DIFF 0.15f
#define MAX_POSTION_DIFF 0.03f
#define number 20

    class WorldModel : public Singleton<WorldModel> {
    public:
	enum BlockState {IsBlocking,FailedBlock,NoBlock};
        /** constructor */
        WorldModel();

        /** Destructor */
        ~WorldModel();

        /**
         * update from the perception and store it
         *
         * @param p the smart pointer to the perception
         *
         * @return successfully updated or not
         */
        bool update(boost::shared_ptr<perception::Perception> p);

	const int getTheStatusToTurnOnDynamicCast() const{ // add by esh
	    return this->statusToturnOnDynamicCast;
	    
	}
	
	void turnOnTheDynamicCast(); // esh
	void turnOffTheDynamicCast(); // esh
        // ===========================
        // Get the golbal information
        // ===========================

        /**
         * @return the global position of the eye (camera)
         */
        const seumath::Vector3f& getMyGlobalPos() const {
            return getMyTorsoGlobalPos();
            //return getVisionTrans().pos();
        }

        /**added by dpf
         * get my torso global pos, not the eye (ie camera)
         * */
        const seumath::Vector3f& getMyTorsoGlobalPos() const {
            return getBoneTrans(robot::humanoid::Humanoid::HEAD).pos();
        }

        const seumath::Vector2f& getMyTorsoGlobalPos2D() const {
            return *(const seumath::Vector2f*)(getBoneTrans(robot::humanoid::Humanoid::HEAD).pos().get());
        }

        const seumath::Vector2f& getMyGlobalPos2D() const {
            return getMyTorsoGlobalPos2D();
            //return *(const seumath::Vector2f*)(getVisionTrans().pos().get());
        }

        //dpf test
        //attention, its p() is not glboal pos, but matrix is global matrix
        //use getBoneTrans(TORSO) as soon as possible;

        const seumath::TransMatrixf getDpfBodyTransMatrix() const {
            return mDpfBodyTransMatrix;
        }

        //dpf test
        const seumath::TransMatrixf updateDpfBodyTransMatrix() const;
        /**
         * @return the global velocity of the upper torso
         */
        ///dpf test, calculate the mVisionBodyAngZ, if no flags can be seen, return false;
        ///attention this method is not very good to use, espcially when the body is not stand up straitly.
        bool calcVisionBodyAngZ(const Vision &p);

        const seumath::Vector3f& getMyGlobalVel() const {
            return mMyGlobalVel;
        }

        /**
         * @return the local coordination's origin of the agent self
         * in Vector3f formate
         */
        const seumath::Vector3f& getMyOrigin() const {
            return mMyOriginMatrix.pos();
        }

        /**
         * @return the local coordination's origin of the agent self
         * in Vector2f formate
         */
        const seumath::Vector2f& getMyOrigin2D() const {
            return *(const seumath::Vector2f*)(mMyOriginMatrix.pos().get());
        }

        /**
         * @return the local coordination's matrix of the agent self
         */
        const seumath::TransMatrixf& getMyOriginTrans() const {
            return mMyOriginMatrix;
        }

        seumath::AngDeg getMyFaceDirection() const {
            return mMyFaceDirection;
        }

        const seumath::Vector3f& getMyCenterOfMass() const {
            return mMyCenterOfMass;
        }

        /**
         * by dpf
         * get my Rel com, not lean rel!!!
         */
        const seumath::Vector3f& getMyRelCenterOfMass() const {
            return mMyRelCenterOfMass;
        }

        const seumath::Vector3f& getMyGyroRate() const {
            return lastPerception().gyroRate().rate(0);
        }

        /***********************************end******************************************/
        /////////////////////////add by allen 2010.3.15

        const seumath::Vector3f& getMyAcc() const {
            return mMyAcc;
        }
        //test by dpf reset the global pos from acc-sensor to pos given

        void accGlobalPosRest(const seumath::Vector3f pos) {
            mMyAccPosGlobal = pos;
        }
        //test by dpf reset the global vec from acc-sensor to zero;

        void accVecGlobalReset() {
            mMyAccVelGlobal.zero();
        }
        ////////////////////////////////////////////////

        seumath::AngDeg getMyBodyDirection() const {
            return mMyBodyDirection;
        }

        robot::humanoid::Humanoid::ESkeleton getMySupportFoot() const {
            return mMySupportBone;
        }

        bool isDoubleSupport() {
            return lastPerception().forceResistance().isDoubleSupport();
        }

        /**
         * get our teammates' global position
         *
         * @param i the number of the teammate
         *
         * @return the 3D position of teammate's torso
         */
        const seumath::Vector3f& getOurGlobalPos(unsigned int i) const {
            std::map<unsigned int, seumath::Vector3f>::const_iterator iter = mTeammateGlobalPos.find(i);
            if (mTeammateGlobalPos.end() != iter) return iter->second;
            return mIllegalPos;
        }

        const std::map<unsigned int, seumath::Vector3f>& getOurGlobalPos() const {
            return mTeammateGlobalPos;
        }

        const seumath::Vector2f& getOurGlobalPos2D(unsigned int i) const {
            return *(const seumath::Vector2f*)(getOurGlobalPos(i).get());
        }

        /**
         * get opponent's global position
         *
         * @param i the number of the opponent
         *
         * @return the 3D position of opponent's torso
         */
        const seumath::Vector3f& getOppGlobalPos(unsigned int i) const {
            std::map<unsigned int, seumath::Vector3f>::const_iterator iter = mOpponentGlobalPos.find(i);
            if (mOpponentGlobalPos.end() != iter) return iter->second;
            return mIllegalPos;
        }

        const std::map<unsigned int, seumath::Vector3f>& getOppGlobalPos() const {
            return mOpponentGlobalPos;
        }

        const seumath::Vector2f& getOppGlobalPos2D(unsigned int i) const {
            return *(const seumath::Vector2f*)(getOppGlobalPos(i).get());
        }
        //////////////////////add by allen 2010.6.7////////////////////

        const int getOurPlayerNumber() {
            return mOurPlayerNum;
        }

        const int getOppPlayerNumber() {
            return mOppPlayerNum;
        }
        ////////////////////////////////////////////////////

        const seumath::Vector3f& getBallGlobalPos() const {
            /*
              if (seenFlagsNum() >= 3)
                  return mBallGlobalPos;
              else {
                  return mBallGlobalPos3DWithRelInfo;
              }
             */
            //dpf comment the above old lines, i think it is useless and may make mistakes.
            return mBallGlobalPos;
        }

        const seumath::Vector3f& getBallAveragePos() const {
            return mBallAveragePos;
        }

        const seumath::Vector2f& getBallGlobalPos2D() const {
            /*
              if (seenFlagsNum() >= 3)
                  return *(const seumath::Vector2f*)(mBallGlobalPos.get());
              else
                  return mBallGlobalPos2DWithRelInfo;
             */
            return *(const seumath::Vector2f*)(mBallGlobalPos.get());
        }

        const seumath::Vector3f& getBallGlobalVel() const {
            return mBallGlobalVel;
        }

        const seumath::Vector3f& getBallGlobalStopPos() const {
            return mBallGlobalStopPos;
        }

        const seumath::Vector2f& getBallGlobalStopPos2D() const {
            return *(const seumath::Vector2f*)(mBallGlobalStopPos.get());
        }

        const seumath::Vector3f& getInterceptBallGlobalPos() const {
            return mInterceptBallGlobalPos;
        } //camera

        const seumath::Vector2f& getInterceptBallGlobalPos2D() const //add it
        {
            return *(const seumath::Vector2f*)(mInterceptBallGlobalPos.get());
        }

        const seumath::TransMatrixf& getVisionTrans() const {
            //test by dpf
            return mDpfMyVisionMatrix;
            // return mMyVisionMatrix;
        }
        //dpf test

        void bodyReset() {
            //rotationZ -90 degrees because the x-y plane not same between two coordination systems;
            mDpfBodyTransMatrix.rotationZ(-90);
            ;
            return;
        }
        //dpf test, reset to bodyAng Vector3f bodyAng(xAng,yAng,zAng),order Y-X-Z, in degrees

        int bodyReset(seumath::Vector3f bodyAng) {
            mDpfBodyTransMatrix.rotationZ(-90);
            mDpfBodyTransMatrix.rotateLocalY(bodyAng.y());
            mDpfBodyTransMatrix.rotateLocalX(bodyAng.x());
            mDpfBodyTransMatrix.rotateLocalZ(bodyAng.z() + 90);
            return 0;
        }
        const seumath::TransMatrixf& getLeftFootTrans() const;

        const seumath::TransMatrixf& getRightFootTrans() const;

        const std::map<unsigned int, seumath::TransMatrixf>& getBoneTrans() const {
            return mBoneTrans;
        }
        //added by dpf, lean rel trans

        const std::map<unsigned int, seumath::TransMatrixf>& getBoneLocalTrans() const {
            return mBoneLocalTrans;
        }
        //rel trans, not lean rel

        const std::map<unsigned int, seumath::TransMatrixf>& getBoneRelTrans() const {
            return mBoneRelTrans;
        }
        // this function name is out date, do not use!!!

        const seumath::TransMatrixf& getJointTrans(unsigned int id) const {
            return getBoneTrans(id);
        }

        const seumath::TransMatrixf& getBoneTrans(unsigned int id) const {
            return mBoneTrans.find(id)->second;
        }
        //added by dpf, lean rel trans

        const seumath::TransMatrixf& getBoneLocalTrans(unsigned int id) const {
            return mBoneLocalTrans.find(id)->second;
        }
        //added by dpf, rel trans, not lean rel
        //added by dpf

        const seumath::TransMatrixf& getBoneRelTrans(unsigned int id) const {
            return mBoneRelTrans.find(id)->second;
        }

        const seumath::TransMatrixf& getBoneTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneTrans(id);
        }
        //added by dpf, lean rel trans

        const seumath::TransMatrixf& getBoneLocalTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneLocalTrans(id);
        }
        //rel trans, not lean rel

        const seumath::TransMatrixf& getBoneRelTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneRelTrans(id);
        }

        seumath::AngDeg getJointAngle(unsigned int jid) {
            //const perception::Perception& p=lastPerception();
            //const perception::JointPerception& jp=p.joints();
            return lastPerception().joints().jointAng(jid);
        }

        seumath::AngDeg getJointRate(unsigned int jid) {
            return lastPerception().joints().jointRate(jid);
        }

        /**
         * get the sum force that applied to the feet
         *
         * @return the vector of force, in the lean rel coordination;
         */
        const seumath::Vector3f& getFeetForce() const {
            return mFeetForce;
        }

        /**
         * get the left foot force, in the lean rel coordination
         */
        const seumath::Vector3f& getLeftFootForce() const {
            return mLeftFootForce;
        }

        /**
         * get the right foot force, in the lean rel coordination
         */
        const seumath::Vector3f& getRightFootForce() const {
            return mRightFootForce;
        }

        /**
         * get the point of sum force that applied to the feet
         *
         * @return the force point
         */
        const seumath::Vector3f& getLeftFootForcePoint() const {
            return mLeftFootForceCenter;
        }

        /**
         * get the point of sum force that applied to the feet
         *
         * @return the force point
         */
        const seumath::Vector3f& getRightFootForcePoint() const {
            return mRightFootForceCenter;
        }

        /**
         * get the point of sum force that applied to the feet
         *
         * @return the force point
         */
        const seumath::Vector3f& getFeetForcePoint() const {
            return mFeetForcePoint;
        }

        /**
         * comment by dpf
         * x() is forward-backward lean angle, when we stand it is 0
         * y() is left-right angle
         * z() is bodyDirection, but when we face to opp goal, it is 90, not zero
         */
        const seumath::Vector3f& getMyBodyAng() const {
            return mMyBodyAng;
        }

        // ==========================
        // get simulation information
        // ==========================

        float getSimTime() const {
            return lastPerception().time().now();
        }

        unsigned int getSimCycle() const {
            return mSimCycle;
        }

        float getAverageStepTime() const;

        const perception::Perception& lastPerception() const {
            return *(mPerceptions.back().get());
        }

        const perception::Perception& predictedPerception() const {
            return mPredictedPerception;
        }

        typedef std::deque< boost::shared_ptr<perception::Perception> > TPerceptionDeque;

        const TPerceptionDeque& getPerceptions() const {
            return mPerceptions;
        }
        //dpf comment here, this function return
        //        seumath::Vector3f getBallPosToBone() const;
        //global pos to body rel pos, but it maybe useless, take care before you use it
        //because you are more often use rel pos to cal global pos
        seumath::Vector3f getPosToBone(seumath::Vector3f) const;
        // ==========================
        // get game information
        // ==========================

        serversetting::TPlayMode getPlayMode() const {
            return lastPerception().getPlayMode();
        }

        serversetting::TTeamIndex getOurTeamIndex() const {
            return perception::GameState::getOurTeamIndex();
        }

        const std::string& getOurTeamName() const {
            return perception::Vision::ourTeamName();
        }

        unsigned int getMyUnum() const {
            return perception::GameState::unum();
        }

        float getGameTime() const {
            return lastPerception().gameState().gameTime();
        }

        int getOurGoal() const {
            return mOurGoal;
        }

        int getOppGoal() const {
            return mOppGoal;
        }

        /**
         * calculate the closest position to a given position in the array
         * this is useful in calculate who is the closest to a given object
         * @note consider the 2D only!!
         *
         * @param pos the given position
         * @param data the array of candinate position
         * @param min min Id of the array
         * @param max max Id of the array
         * @param minDist the min distance
         *
         * @return the Id of the closest position in array
         */
        unsigned int calClosestIdToPosition(const seumath::Vector3f& pos,
                const std::map<unsigned int, seumath::Vector3f>& data,
                float& minDist) const;

        // ==========================
        // prediction
        // ==========================
        /**
         * predict the movement of ball, this function is really simple,
         * just use a model of ball movement
         *
         * @param p the position of ball
         * @param v the velocity of ball
         */
        void predictBall(seumath::Vector3f& p, seumath::Vector3f& v) const;

        unsigned int getOppClosestToMe() const {
            return mOppClosestToMe;
        }

        float getOppClosestToMeDistance() const {
            return mOppClosestToMeDist;
        }

        // ==========================
        // high levle judgement, don't use them (comment by dpf)
        // ==========================
        bool isGroundBall() const;

        bool isStable() const;

        bool isLeftRolling() const;

        bool isLeftRolled() const;

        bool isRightRolling() const;

        bool isRightRolled() const;

        bool isDiving() const;

        bool isDived() const;

        bool isDived(seumath::AngDeg angX) const;

        bool isLying() const;

        bool isLied() const;

        bool isLied(seumath::AngDeg angX) const;

        bool isCloseToGoal() const;

        bool isLeftFall() const;

        bool isLeftFall(seumath::AngDeg angle) const;

        bool isRightFall() const;

        bool isRightFall(seumath::AngDeg angle) const;

        bool isFall() const;
///new code
	int isOppFall(int i) const
	{
	    float head = getOppBodyInformation(i,Vision::HEAD).z();
	    float Lfoot = getOppBodyInformation(i,Vision::L_FOOT).z();
	    float Rfoot = getOppBodyInformation(i,Vision::R_HAND).z();
	    if(head<=-1000||Lfoot<=-1000||Rfoot<=-1000)
	      return -1;//can't get correct  position
	    float foot = Lfoot>=Rfoot?Rfoot:Lfoot;
	    float delta = head - foot;
	    float fallHeight = 0.10f;
	    float stableHeight = 0.30f;
	    if(delta<=fallHeight)
	      return 1;//is fall
	    else if(delta>=stableHeight)
	      return 0;//steady
	    else 
	      return 2;//unknown
	    
	}
	
	Vector3f getOppBodyInformation(int i,Vision::PID name) const
	{
	 /* std::map<unsigned int, seumath::Vector3f>::const_iterator iter = mOpponentGlobalPos.find(i);
            if (mOpponentGlobalPos.end() != iter) return iter->second;
            return mIllegalPos;*/
	// std::map<unsigned int,seumath::Vector3f>::const_iterator iter = mOppnentHeadPos.find(10);
	std::map<unsigned int,  std::map<Vision::PID, seumath::Vector3f> >::const_iterator iter0 = mOppBodyInformation.find(i);
	std::map<Vision::PID, seumath::Vector3f>::const_iterator iter =  iter0->second.find(name);
	  //std::map<unsigned int,seumath::Vector3f>::const_iterator iter = mOppBodyInformation[i].find(name);
	 if( iter0->second.end()!=iter) return iter->second;
	 return mIllegalPos;
	}
	
	void updateMySpeed()
	{
	  int index1=0;int index2=0;
	  Vector3f pos1(0,0,0);Vector3f pos2(0,0,0);
	  for(int i=0;i<=number-1;i++)
	  {
	    if(GetmMyWalkPos(i)!=mIllegalPos)
	    {
	      index1=i;pos1=GetmMyWalkPos(i);
	      break;
	    }
	  }
	  for(int i=number-1;i>=0;i--)
	  {
	    if(GetmMyWalkPos(i)!=mIllegalPos)
	    {
	      index2=i;pos2=GetmMyWalkPos(i);
	      break;
	    }
	  }
	  if(index1<index2-2)
	    myspeed = (pos1-pos2).length()/(index2-index1);
	  else
	    return;
	  
	}
      
	void UpdateOppSpeed(int num)
	{
	    int index1=0;int index2=0;
	    Vector3f pos1(0,0,0);Vector3f pos2(0,0,0);
	    Vector3f oppPos;
	    for(int i=0;i<=number-1;i++)
	    {
	    oppPos = (GetmOppWalkPos(num,i,Vision::L_FOOT)+GetmOppWalkPos(num,i,Vision::R_FOOT))/2;
	      if(oppPos!=mIllegalPos)
	      {
		index1=i;pos1=oppPos;
		break;
	      }
	      if(i==number-1)
		return;
	    }
	    for(int i=number-1;i>=0;i--)
	    {
	      oppPos = (GetmOppWalkPos(num,i,Vision::L_FOOT)+GetmOppWalkPos(num,i,Vision::R_FOOT))/2;
	      if(oppPos!=mIllegalPos)
	      {
		index2=i;pos2=oppPos;
		break;
	      }
	      if(i==0)
		return;
	    }
	    if(index1<index2-2)
	      oppspeed[num] = (pos1-pos2).length()/(index2-index1);
	    else
	      return;
	}


	Vector3f GetOppWalkParam(int num)const
	{
	  Vector3f parameter_average(0,0,0);
	  Vector3f parameter_square_average(0,0,0);
	  Vector3f parameter_product_average(0,0,0);
	  Vector3f parameter(0,0,0);
	  ///extend to 1000 of its own
	  /*for(int i=0;i<=number-1;i++)
	  {
	  GetmOppWalkPos(num,i,Vision::L_FOOT).x()=1000*GetmOppWalkPos(num,i,Vision::L_FOOT).x();	    
	  GetmOppWalkPos(num,i,Vision::L_FOOT).y()=1000*GetmOppWalkPos(num,i,Vision::L_FOOT).y();	    
	  GetmOppWalkPos(num,i,Vision::R_FOOT).x()=1000*GetmOppWalkPos(num,i,Vision::R_FOOT).x();	    
	  GetmOppWalkPos(num,i,Vision::R_FOOT).y()=1000*GetmOppWalkPos(num,i,Vision::R_FOOT).y();	    
	  }*/
	  for(int i=0;i<=number-1;i++)
	  {
	    parameter_average.x()+=GetmOppWalkPos(num,i,Vision::L_FOOT).x();
	    parameter_average.y()+=GetmOppWalkPos(num,i,Vision::L_FOOT).y();
	    
	    parameter_square_average.x()+=pow2<float>((GetmOppWalkPos(num,i,Vision::L_FOOT).x()+
	    GetmOppWalkPos(num,i,Vision::R_FOOT).x())/2);
	  parameter_square_average.y()+=pow2<float>((GetmOppWalkPos(num,i,Vision::L_FOOT).y()+
	    GetmOppWalkPos(num,i,Vision::R_FOOT).y())/2);
	  
	    parameter_product_average.x()+=(GetmOppWalkPos(num,i,Vision::L_FOOT).x()+GetmOppWalkPos(num,i,Vision::R_FOOT).x())*
	    (GetmOppWalkPos(num,i,Vision::L_FOOT).y()+GetmOppWalkPos(num,i,Vision::R_FOOT).y())/4;
	    
	    parameter_average.x()+=GetmOppWalkPos(num,i,Vision::R_FOOT).x();
	    parameter_average.y()+=GetmOppWalkPos(num,i,Vision::R_FOOT).y();
	  }
	  parameter_average.x()/=number*2;
	  parameter_average.y()/=number*2;
	 parameter_square_average.x()/=number;
	 parameter_square_average.y()/=number;
	 parameter_product_average.x()/=number;
	  
	    parameter.y()=(parameter_average.x()*parameter_average.y()-parameter_product_average.x())/
	      (parameter_average.x()*parameter_average.x()-parameter_square_average.x());
	      parameter.x()=parameter_average.y()-parameter_average.x()*parameter.y();
	
	      
		return parameter;
	}
	
	/// make sure the direction of opp walk 
	

/*	bool GetballDirection() ///   
	{
	  Vector2f direction;
	    int length=ballpos.size();
	    if(length<5)
	    {
	       direction=mOriginBallDir;	      
	      return false;
	    }
	    else
	    {
	      std::list<Vector2f>::iterator it=ballpos.begin();
	      Vector2f temp1=*it;
	      for(unsigned int i=0;it!=ballpos.end();it++)
	      {
		i++;
		if(i==5)
		{
		  Vector2f temp2=*it;	    
		  direction.x()=temp1.x()-temp2.x();
		  direction.y()= temp1.y()-temp2.y();
		  break;
		}
	      }	 
	      mOriginBallDir=direction;
	      direction.normalize();
	      if(direction.x()<=-0.08||mOriginBallDir.x()<=-0.06)
	      {
		cout<<"defend"<<endl;
		return true;
	      
	      }else
	      {
		cout<<"attack"<<endl;
		return false;
	     // direction.normalize();
	      }
	    }
	}*/
	int GetOppWalkDirection(int num)const
	{
	      int lcount=0;
	      int rcount=0;
	      for(int i=0;i<=number-2;i++)
	      {
		      float oldx = GetmOppWalkPos(num,i+1,Vision::L_FOOT).x();
		      float newx = GetmOppWalkPos(num,i,Vision::L_FOOT).x();
		      if(oldx==100000||newx==100000)
			continue;
		      if(oldx>newx)
		      {
			lcount++;
		      }
		      else
			rcount++;
	      }
	      if(lcount+rcount>=10)
		return lcount>rcount?-1:1;
	      else
	      {
		lcount=0;rcount=0;
	      }
	      for(int i=0;i<=number-2;i++)
	      {
		    float oldy = GetmOppWalkPos(num,i+1,Vision::R_FOOT).x();
		    float newy = GetmOppWalkPos(num,i,Vision::R_FOOT).x();
		    if(oldy==100000||newy==100000)
		      continue;
		    if(oldy>newy)
		    {
		      lcount++;
		    }
		    else
		      rcount++;
	      }
		if(lcount+rcount>=10)
		return lcount>rcount?-1:1;
	      else
	      {
		lcount=0;rcount=0;
	      }
	      return 0;
	}
	
	
	Vector2f GetOppWalkVector(int num)const
	{
	    Vector2f WalkVector;  
	   // GetOppWalkParam(num).x()/=1000.0f;
	    Vector3f oppwalkParam=GetOppWalkParam(num);
	    WalkVector.x()=oppwalkParam.x()/oppwalkParam.y();
	    WalkVector.y()=oppwalkParam.x();
	    int a =GetOppWalkDirection(num);
	    if(WalkVector.x()*a<0)
	      //return WalkVector*GetOppWalkDirection(num)*(-1);
	      return WalkVector*(-1);
	    else
	      return WalkVector;
	   //return oppspeed[num];
	}
	
	Vector3f GetmOppWalkPos(int num,int times,Vision::PID name)const
	{
	  /*std::map<unsigned int ,std::map<unsigned int,  std::map<Vision::PID, seumath::Vector3f> > >::const_iterator iter0 = mOppWalkPosition.find(times);
	  std::map<unsigned int, std::map<Vision::PID, seumath::Vector3f> >::const_iterator iter1=iter0->second.find(num);
	  std::map<Vision::PID, seumath::Vector3f>::const_iterator iter2 =  iter1->second.find(name);
	  //std::map<unsigned int,seumath::Vector3f>::const_iterator iter = mOppBodyInformation[i].find(name);
	  if( iter1->second.end()!=iter2) return iter2->second;
		return mIllegalPos;*/
	  if(name==Vision::L_FOOT)
	    return mOppWalkPos[times][num][0];
	  else
	    return mOppWalkPos[times][num][1];
	}
	
	Vector3f GetmMyWalkPos(int times)const
	{
	   return mMyWalkPos[times];
	}




	float GetMetoOppDistance(int num)const
	{
	    Vector3f OppPos=getOppGlobalPos(num);
	    Vector3f MyPos=getMyGlobalPos();
	    float dis=sqrt(pow2<float>(OppPos.x()-MyPos.x())+pow2<float>(OppPos.y()-MyPos.y()));
	    return dis;
	}

	float GetOppWalkVec(int num)
	{
	   return oppspeed[num]+0.002;
	  float oppvec;
	  Vector3f endpos=(GetmOppWalkPos(num,number-1,Vision::L_FOOT)+GetmOppWalkPos(num,number-1,Vision::R_FOOT))/2;
	  Vector3f startpos=(GetmOppWalkPos(num,0,Vision::L_FOOT)+GetmOppWalkPos(num,0,Vision::R_FOOT))/2;
	  double temp=sqrt((double)pow2<float>(endpos.x()-startpos.x())+pow2<float>(endpos.y()-startpos.y()));
	 oppvec=(float)temp/number;
	  return oppvec;						///need a map
	
	}


	float GetMyWalkVec()
	{
	   return myspeed;
	  float myvec;
	  Vector3f endpos=GetmMyWalkPos(number-1);
	  Vector3f startpos=GetmMyWalkPos(0);  
	  double temp=sqrt((double)pow2<float>(endpos.x()-startpos.x())+pow2<float>(endpos.y()-startpos.y()));
	  myvec=(float)temp/number;
	  return myvec;
	
	}

	unsigned int calOurBlockNumber();/// the ID of our players to block

	float getPosAngle ( unsigned int oppNumber,unsigned int ourNumber );

	bool IsInBlockRange ( unsigned int oppNumber,unsigned int ourNumber,float preDis );
	
	bool IsOnTheBlock ( unsigned int oppNumber,unsigned int ourNumber ); //whether block has completed
	
	unsigned int getOurDribbleNum()
	{	return ourDribbleNumber;
	}
	
	int getOppDribbleNum() const;//if opp is dribbling,return the number of the dribbler; if not,return 0;
	///end of new code

        /**
         * check if I am the closest one to the ball
         */

        int GetOppCount(Vector2f);

        bool amIFastestToBallOfOurTeam() const;

        float howIFasterToBallThanOpps() const;

        float getMyInterceptBallTime() const {
            return mMyInterceptBallTime;
        }

        bool isTouch(unsigned int id) const {
            return lastPerception().forceResistance().isTouch(id);
        }

        /**
         * This function tells whether the specified foot is contact with the ball.
         */
        bool isTouchBall(unsigned int id) const;

        /**
         * @return the position of opponent who is closest in their team
         */
        const seumath::Vector3f& getOppFastestToBallPos() const {
            return getOppGlobalPos(mOppFastestToBall);
        }

        float getOppFastestToBallDistance() const {
            return mOppFastestToBallDist;
        }
        
        float getOppFastestToBallTime() const {
	    return mOppFastestToBallTime;
	}
	
	unsigned int GetOppFastestToBallNum() const
	{	return mOppFastestToBall;
	}

        float getOurFastestToBallDistance() const {
            return mOurFastestToBallDist;
        }

        const seumath::Vector3f& getOurFastestToBallPos() const {
            return getOurGlobalPos(mOurFastestToBall);
        }

        unsigned int getOurFastestToBallNum() const {
            return mOurFastestToBall;
        }

        unsigned int getHearOurFastestToBallNum() const {
            return mHearOurFastestToBall;
        }

        void setHearOurFastestToBallNum(unsigned int num) {
            mHearOurFastestToBall = num;
        }

        bool isPlayerInField(const seumath::Vector2f& pos, float margin = 0);

        float getLostSimTime() const {
            return mLostSimTime;
        }

        bool isHaveOppInArea(seumath::TConvexPolygon<float> area);

        void setDribbleSide(bool isLeft) /////terrymimi
        {
            isLeftDribble = isLeft;
        }

        bool getDribbleSide() const {
            return isLeftDribble;
        }

        bool isBallToOurGoal(void);

        bool isGameStateChanged() const {
            if (mPerceptions.size() > 1)
                return lastPerception().gameState().getPlayMode() != mPerceptions[mPerceptions.size() - 2]->gameState().getPlayMode();
            else
                return false;
        }

        int getFlagNumbersISee()const //vision-me
        {
            const Vision* v = lastPerception().vision().get();
            if (v != NULL) {
                set<Vision::FID> flags = (*v).getStaticFlagSet();
                int num = flags.size();

                return num;
            }

            return 0;
        }

        const seumath::Vector3f& getBallLaPol()const //jia
        {
            return ballLaPol; //get ball's pol pos.
        }

        void setSearchSpeedX(float speed) /////terry
        {
            mSearchSpeed.x() = speed;
        }

        void setSearchSpeedY(float speed) /////terry
        {
            mSearchSpeed.y() = speed;
        }

        const seumath::Vector2f& getSearchSpeed() const /////terry
        {
            return mSearchSpeed;
        }

        float getLatestFlagNumISee() //terry
        {
            const Vision* v = lastPerception().vision().get();
            if (v != NULL) {
                set<Vision::FID> flags = (*v).getStaticFlagSet();
                latestFlagNum = flags.size();
            }
            return latestFlagNum;
        }

        float timeAfterLastSeeEnoughFlags() const {
            return getSimTime() - mLastTimeSeeEnoughFlags;
        }

        void setNowFormation(configuration::Formation::FormationType p_ft) {
            nowFT = p_ft;
        }

        configuration::Formation::FormationType getNowFormation() {
            return nowFT;
        }

                /** comment by dpf, i don't know what's the below function
         * don't use them before ask allen, maybe allen's work.
         * i think it needs deleted!
         * */
        void setBestZone(int p_X, int p_Y) {
            bestZoneX = p_X;
            bestZoneY = p_Y;
        }

        int getBestZoneX() {
            return bestZoneX;
        }

        int getBestZoneY() {
            return bestZoneY;
        }

        bool getIsAttacking() {
            return isAttacking;
        }

        void setIsAttacking(bool att) {
            isAttacking = att;
        }

        //=================================================================TT REL

        struct ObjectRelInfo {
            bool canSee;
            Vector2f relPos2D; //+x: right, +y: forward
            Vector2f pol2D; //(dist,ang); ang:left+, right-
        };

        /**	this function is used to set data to a new ObjectRelInfo
         *	it is to overwrite the {} method of struct's build function
         *	because this method is not supported on some OSs like FreeBSD
         */
        ObjectRelInfo newObjectRelInfo(bool a, Vector2f b, Vector2f c) {
            ObjectRelInfo ret;
            ret.canSee = a;
            ret.relPos2D = b;
            ret.pol2D = c;
            return ret;
        }

        struct BlockInfo {
            float dist;
            float angC;
            float angL; //angC+theta
            float angR; //angC-theta
            bool isOurTeam; 
            //bool operator < (const BlockInfo& a);
        };

        /*bool BlockInfo::operator < (const BlockInfo& a){
                return dist<a.dist;
        }*/

        bool canSeeBall() {
            return mBallRelInfo.canSee;
        }

        const Vector2f& getBallRelPos2D() {
            return mBallRelInfo.relPos2D;
        }

        const Vector2f& getBallPol2D() {
            return mBallRelInfo.pol2D;
        }

        const seumath::Vector2f& getBallRelVel2D() {
            return mBallRelVel2D;
        }

        bool canSeeFlag(Vision::FID fid) {
            return mFlagRelInfoMap[fid].canSee;
        }

        const Vector2f& getFlagRelPos2D(Vision::FID fid) {
            return mFlagRelInfoMap[fid].relPos2D;
        }

        const Vector2f& getFlagPol2D(Vision::FID fid) {
            return mFlagRelInfoMap[fid].pol2D;
        }

        /*const map<Vision::FID,ObjectRelInfo>& getFlagRelInfoMap() const {
                return mFlagRelInfoMap;
        }*/

        const BlockInfo& getBallBlock() const {
            return mBallBlock;
        }

	const BlockInfo& getPlayerBlock() const 
        {
	    return mPlayerBlock;
	}

        const std::list<BlockInfo>& getBlockList() const {
            return mBlockList;
        }

        unsigned int getOurState()			//add by ghd
	{
	    return mOurCastType;
	}
	
	unsigned int getHearCastType()	//ghd
	{
	    return mHearCastType;
	}
	
	unsigned int getHearGoodPosID()	//ghd
	{
	    return mHearGoodPos;
	}
	
	std::vector<int> getOurCast()	//ghd
	{
	    return mOurCast;
	}
	
	bool isOppNearMe();	//ghd
	
        // Vector2f calMyGlobalPos2DWithTwoFlags();
        //dpf added, one could in fact get the pos when only 1 flags is seen
        //Vector3f calDpfGlobalPosWithOneFlags(const Vision& v);
        //dpf added, use new method to calc vision global pos
        Vector3f calDpfVisionGlobalPos(const Vision&v);
        //dpf added, one could get the pos when only ball is seen, we guess ball is not moved
        Vector3f calDpfGlobalPosWithOnlyBall();

        int seenFlagsNum() const {
            int num = 0;

            FOR_EACH(iter, mFlagRelInfoMap) {
                if (true == iter->second.canSee)
                    num++;
            }
            return num;
        }

        Vector2f calObjRelPos2D(const Vector3f& objPolToVisionSensor);
        Vector3f calObjRelPos(const Vector3f& objPolToVisionSensor);

        /**
         * calculate originGlobalPos's rel pos relPos 's globalPos;
         * used in goTo before kick, by TT
         */
        seumath::Vector2f transRelPosToGlobalPos(const Vector2f& originGlobalPos, const Vector2f& relPos) {
            AngDeg theta = getMyBodyDirection() - 90.0f;
            return originGlobalPos + Vector2f(relPos.x() * cosDeg(theta) - relPos.y() * sinDeg(theta),
                    relPos.x() * sinDeg(theta) + relPos.y() * cosDeg(theta));

        }

        seumath::Vector2f transGlobalPosToRelPos(const Vector2f& originPos, const Vector2f& destPos) {
            AngDeg theta = 90.0f - getMyBodyDirection();
            Vector2f deltaV2f = destPos - originPos;
            return Vector2f(deltaV2f.x() * cosDeg(theta) - deltaV2f.y() * sinDeg(theta),
                    deltaV2f.x() * sinDeg(theta) + deltaV2f.y() * cosDeg(theta));
        }

        /**
         * dpf rewrite
         */
        seumath::Vector2f transRelPosToGlobalPos(const Vector2f& relPos) {
            return *(seumath::Vector2f*)(transRelPosToGlobalPos(Vector3f(relPos.x(), relPos.y(), 0.0f))).get();
        }

        /**
         * dpf rewrite
         */
        seumath::Vector2f transGlobalPosToRelPos(const Vector2f& globalPos) {
            return *(seumath::Vector2f*)(transGlobalPosToRelPos(Vector3f(globalPos.x(), globalPos.y(), 0.0f))).get();
        }

        /**dpf rewrite it using transfer matrix
         *from the global pos to the eye-rel pos, ie position relative to the camera
         *there are three types of position
         *first, global position with the center of the playground is zero point
         *second, camera rel pos, position relative to the camera
         *third, body rel pos, which is often called rel pos, with is relative to the \
         *body,
         *there are two kinds of body rel pos,
         *1. only consider body's rotationZ
         *2. also consider other's rotation
         *below two funcitons is about 2, but it works more near to 1, maybe it is more stable.
         **/
        seumath::Vector3f transGlobalPosToRelPos(const Vector3f &destPos) {
            seumath::TransMatrixf relToGlobalTrans;
            float bodyDirZ = getMyBodyAng().z(); //rotation angle of body;
            //relToGlobalTrans.rotationZ(-90);
            relToGlobalTrans.rotationZ(bodyDirZ);
            relToGlobalTrans.p() = getMyOrigin();
            relToGlobalTrans.p().z() = 0;
            return relToGlobalTrans.inverseTransform(destPos);
        }

        /**added by dpf
         * look above, you konws...^_^
         * the rel is not  lean rel
         * */
        seumath::Vector3f transRelPosToGlobalPos(const Vector3f &destPos) {
            seumath::TransMatrixf relToGlobalTrans;
            float bodyDirZ = getMyBodyAng().z(); //rotation angle of body;
            //relToGlobalTrans.rotationZ(-90);
            relToGlobalTrans.rotationZ(bodyDirZ);
            relToGlobalTrans.p() = getMyOrigin();
            relToGlobalTrans.p().z() = 0;
            return relToGlobalTrans.transform(destPos);

        }

        /**
         * added by dpf
         * from lean rel pos to rel pos
         */
        seumath::Vector3f transLeanRelPosToRelPos(const Vector3f&destPos) {
            seumath::TransMatrixf leanRelToRelTrans = getBoneRelTrans(robot::humanoid::Humanoid::TORSO);
            return leanRelToRelTrans.transform(destPos);
        }

        /**
         * from rel to lean rel pos
         */
        seumath::Vector3f transRelPosToLeanRelPos(const Vector3f&destPos) {
            //rotated trans from rel to lean rel, is leanRelToRelTrans, see our documents for details
            //this name means, it can transfer the lean rel pos to rel pos, so call it leanRelToRelTrans;
            //leanRelPos*leanRelToRelTrans=relPos,so relPos*(~leanRelToRelTrans)=leanRelPos;
            seumath::TransMatrixf leanRelToRelTrans = getBoneRelTrans(robot::humanoid::Humanoid::TORSO);
            return leanRelToRelTrans.inverseTransform(destPos);
        }
        //TT for test
        void logPrintSeenFlags();
	
	bool IsJoin();
	
	

    private:
        // =====================
        //  private functions
        // =====================

        // ============================
        //  world model update functions
        // ============================
        /** Update the information from server */
        bool updatePerception(boost::shared_ptr<perception::Perception> newP);

        void localization();
	
	int statusToturnOnDynamicCast; // add by esh

        //test by dpf
        seumath::TransMatrixf calcDpfVisionSensorRotation() const;
        ///old function using flags
        ///set mMyVisionMatrixUsingFlags, if no enough flags is seen return flase
        bool calcVisionSensorRotationUsingFlags(const Vision&v);
        ///the lower function
        seumath::Matrix3x3f calcVisionSensorRotationUsingFlags(const seumath::Vector3f& myPos,
                const Vision& v,
                Vision::FID flagA,
                Vision::FID flagB,
                Vision::FID flagC) const;

        /**
         * calculate the global position of vision sensor
         *
         * @param v the vision information
         *
         * @return the global position of vision sensor
         */
        void updateBall();

        void updateSelf();

        void updatePlayers();
	
	void updateCast();
	
	bool updateState(); //is opp attaking --ghd


        /**
         * predicted the real intercepted position and who is the fastest
         * to the ball
         *
         * @param data the position of player
         * @param speed the speed of player
         * @param minTime the time of fastest player to intercepted the ball
         *
         * @return the number of fastest player
         */
        unsigned int calFastestIdToBall(const std::map<unsigned int, seumath::Vector3f>& data,
                float speed, float& minTime);

        float predictInterceptBall(seumath::Vector3f& posBall, seumath::Vector3f& velBall,
                const seumath::Vector3f& pos, float speed, float maxTime) const;


        //TT, MMXI
        //for relative information
        void updateObjectRelInfo();

        Vector2f calObjPol2D(const Vector2f& relPos2D);
        void buildBlocks();

        //test by dpf

        void setDpfBodyTransMatrix(const seumath::TransMatrixf& rel) {
            mDpfBodyTransMatrix = rel;
        }
        /**
         * calculate the Normal Vector of the playground in the coordinate system of the camera 
         * @author allen
         * not use
         */
        void calcNVectorOfGround();
    private:
        unsigned int mSimCycle;

        float mStartSimTime;

        float mLostSimTime;

        float mLastTimeSeeEnoughFlags;

        /** the deque of perception from the server */
        TPerceptionDeque mPerceptions;
        const static unsigned int max_perception_size;

        std::map<unsigned int, seumath::TransMatrixf> mBoneTrans;

        //added by dpf local transfer matrix of jonits. ie. torso is identify matrix,lean rel trans
        std::map<unsigned int, seumath::TransMatrixf> mBoneLocalTrans;
        //added by dpf rel transfer matrix of joints, rel trans, not lean rel
        std::map<unsigned int, seumath::TransMatrixf> mBoneRelTrans;

        /////////// Illegal Position ///////
        const static seumath::Vector3f mIllegalPos;

        /////////// Ball ///////////////
        seumath::Vector3f mBallGlobalPos;
        //seumath::Vector2f mBallGlobalPos2DWithRelInfo; //TT
        //seumath::Vector3f mBallGlobalPos3DWithRelInfo; //TT
        //seumath::Vector3f mG2RGlobalPos; //terry
        //seumath::Vector3f mG1RGlobalPos; //terry
        seumath::Vector3f mBallGlobalVel;
        seumath::Vector3f mBallAveragePos;
        seumath::TVector<PVExtenedKalmanFilter<float>, 3 > mBallPvekf;
        seumath::Vector3f mBallGlobalStopPos;
        seumath::Vector3f mInterceptBallGlobalPos;
        float mMyInterceptBallTime;

	unsigned int mOurCastType;			//add by ghd
	
	unsigned int mHearGoodPos;			//add by ghd
	
	unsigned int mHearCastType;			//add by ghd
	
	std::vector<int> mOurCast;		//add by ghd
	
        ////////// Player /////////////
        std::map<unsigned int, seumath::Vector3f> mTeammateGlobalPos;
        std::map<unsigned int, seumath::Vector3f> mOpponentGlobalPos;
        int mOurPlayerNum; //our team players
        int mOppPlayerNum; //opp team players
        unsigned int mOppClosestToMe;
        float mOppClosestToMeDist;

        /// the matrix of the vision sensor using flags
        seumath::TransMatrixf mMyVisionMatrixUsingFlags;
        ///test by dpf
        seumath::TransMatrixf mDpfMyVisionMatrix;
        ///test by dpf, the body ang Z caculated from vision, init pos is -90 degree
        seumath::AngDeg mVisionBodyAngZ;
        seumath::Vector3f mMyGlobalVel;
        seumath::AngDeg mMyFaceDirection;
        seumath::Vector3f mMyCenterOfMass;
        seumath::Vector3f mMyRelCenterOfMass;
        seumath::Vector3f mMyBodyAng; //torso's rotation angles
        seumath::Vector3f mMyLastBodyAng; //dubai
        seumath::Vector3f cameraRot; //vision-me
        seumath::Vector3f mMyGlobalPos; //vision-me
        seumath::Vector3f ballLaPol; //jia

        /// the coordination for self motion
        seumath::TransMatrixf mMyOriginMatrix; //it is just mDpfBodyTransMatrix;
        seumath::AngDeg mMyBodyDirection;
        seumath::AngDeg mMyBodyDirWithFlags; //TT
        seumath::TransMatrixf mDpfBodyTransMatrix; //dpf test, torso global transMatrix


        /// the lean rel position of the force center in feet
        seumath::Vector3f mLeftFootForceCenter;
        seumath::Vector3f mRightFootForceCenter;
        seumath::Vector3f mLeftFootForce;
        seumath::Vector3f mRightFootForce;
        /// the force of feet, dpf change the coordination to lean rel coordination;
        seumath::Vector3f mFeetForcePoint;
        seumath::Vector3f mFeetForce;

        robot::humanoid::Humanoid::ESkeleton mMySupportBone;

        /// the predicted perception
        perception::Perception mPredictedPerception;

        ///////////////////// for soccer game ///////////////////
        unsigned int mOurFastestToBall;
        unsigned int mHearOurFastestToBall;
        float mOurFastestToBallDist;
        float mOurFastestToBallTime;
        unsigned int mOppFastestToBall;
        float mOppFastestToBallDist;
        float mOppFastestToBallTime;

        int mOurGoal;
        int mOppGoal;

        bool isLeftDribble;

        seumath::Vector2f mSearchSpeed; //terry

        boost::shared_ptr<const perception::Vision> mLatestV; //terry

        seumath::Vector3f preBallPol; //terry

        seumath::Vector3f preG2RPol; //terry

        float latestFlagNum; //terry

        //-------------------------------------------TT xxxxxxxxxxxxxxxxxxxxxxxxxx
        // bool localWalkSign;
        //bool rushToGoalSign;
        //bool isLeftFootKick;
        //----------------------------------------------

        //==================================================TT REL
        ObjectRelInfo mBallRelInfo; //for ball
        std::map<Vision::FID, ObjectRelInfo> mFlagRelInfoMap; //for 8 flags

        BlockInfo mBallBlock; //for ball, but we dont know whether he see ball or not from here
        std::list<BlockInfo> mBlockList; //for players' body

        seumath::Vector2f mBallRelVel2D;
	BlockInfo mPlayerBlock;

        //===========================================================
        seumath::Vector3f mMyAcc;

        //now formationtype add by allen
        configuration::Formation::FormationType nowFT;
        int bestZoneX;
        int bestZoneY;
        bool isAttacking;
        ///dpf test acc
        //test acc by dpf, rel
        seumath::Vector3f mMyRealAccRel;
        //global
        seumath::Vector3f mMyRealAccGlobal;
        //pos from acc calculate, global
        seumath::Vector3f mMyAccPosGlobal;
        //velocity from acc-sensor,global
        seumath::Vector3f mMyAccVelGlobal;

        /////////////////////////
        //use to calculate normal vector of ground allen 
        typedef std::map<unsigned int, seumath::Vector3f> TSegmentPointPolMap;
        typedef std::map<unsigned int, seumath::Vector3f> TSegmentPointMap;
        typedef std::map<unsigned int, perception::SegmentPol> TSegmentPolMap;
        typedef std::map<unsigned int, core::FPID> TFPIDMap;
        typedef std::map<core::FPID, TFPIDMap> TTFPIDMap;
        TSegmentPointMap mPointsXYZ;
        TSegmentPolMap mFieldLines;
        seumath::Vector3f mNVector;
        ///////////////////////////////////allen
	std::map<unsigned int,  std::map<Vision::PID, seumath::Vector3f> > mOppBodyInformation;
	Vector3f mOppWalkPos[number][12][2];//0 left //1 right
	Vector3f mMyWalkPos[number];
	int Tcount;
	float myspeed;
	float oppspeed[12];
	
	
	public:
	int ourBlockNumber;
	BlockState state;
	bool IsBlockFailed();
	std::list<Vector2f>ballpos;
	Vector2f mOriginBallDir;
	float calBlockXDIs ( unsigned int OurNumber,unsigned oppNumber );
	//int ourDribbleOrder;
	unsigned int ourDribbleNumber;
    }; //end of class WorldModel

#define WM core::WorldModel::GetSingleton()

} //end of namespace core

#endif // CORE_WORLD_MODEL

