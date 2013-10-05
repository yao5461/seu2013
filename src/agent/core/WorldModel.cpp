/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WorldModel.cpp 2731 2009-03-30 14:26:08Z zyj $
 *
 ****************************************************************************/

#include <fstream>
#include "configuration/Configuration.h"
#include "soccer/TeamPlayer.h"
#include "WorldModel.h"
#include "math/TLine2.hpp"
#include "robot/humanoid/Humanoid.h"
#include "PassModel.h"
#include "SayAndHearModel.h"
#include<time.h>

bool sortByDist(const core::WorldModel::BlockInfo& a, const core::WorldModel::BlockInfo& b) {
    return a.dist < b.dist;
}


namespace core {

    using namespace std;
    using namespace boost;
    using namespace seumath;
    using namespace serversetting;
    using namespace robot;
    using namespace robot::humanoid;
    using namespace action;
    using namespace perception;

    const unsigned int WorldModel::max_perception_size = 100;
    const seumath::Vector3f WorldModel::mIllegalPos(100000, 100000, -100000);

    WorldModel::WorldModel() {
        mMyAcc = Vector3f(0.0f, 0.0f, 9.81f);
	Tcount=0;
	myspeed=0;
	for(int i=0;i<=11;i++)
	{
	  oppspeed[i]=0;
	}

        //===========================================TT Rel
        mFlagRelInfoMap.clear();
        //rewrited by dpf to avoid errors on somecout< OSs like FreeBSD
        ObjectRelInfo tmpObjectRelInfo = newObjectRelInfo(false, Vector2f(0, 0), Vector2f(0, 0));
        mFlagRelInfoMap[Vision::F1L] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::F2L] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::F1R] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::F2R] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::G1L] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::G2L] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::G1R] = tmpObjectRelInfo;
        mFlagRelInfoMap[Vision::G2R] = tmpObjectRelInfo;
        //cout<<"mFlagRelInfoMap.size()="<<mFlagRelInfoMap.size()<<endl;
        //===================================================

        /////////////allen add//////////////
        mOurPlayerNum = 0;
        mOppPlayerNum = 0;
        /////////////////////////
        shared_ptr<Perception> p(new Perception());

        // perception
        mPerceptions.push_back(p);
        mSimCycle = 0;
        mStartSimTime = 0;

        // vision
        Vision::setOurTeamName(OPTS.arg<string > ("teamname"));
        Vision::setupObjectsVision(TI_NULL);
        Vision::setupPlayerVision();
        Vision::setupStaticObjectsGlobalPos();

        // ball
        mBallGlobalPos = Vector3f(0, 0, ball_radius);
        mBallGlobalVel.zero();
        mBallGlobalStopPos = mBallGlobalPos;
        mInterceptBallGlobalPos = mBallGlobalPos;
        mBallAveragePos = mBallGlobalPos;
        nowFT = configuration::Formation::FT_HOME; //////////////////////////////////////////////
        bestZoneX = 8;
        bestZoneY = 2;
        mHearOurFastestToBall = 0;
        isAttacking = false;
        // set up ball filter
        const Vector3f errVar(1, 1, 1);
        for (int i = 0; i < 3; i++) {
            mBallPvekf[i].init(0, 0, sim_step, 0.03f,
                    ball_mass, errVar[i] / sim_step, errVar[i]);
        }

        // game info (score)
        mOurGoal = 0;
        mOppGoal = 0;
	
	//dpf test
	bodyReset();
	
	mMyVisionMatrixUsingFlags.identity();
	
	//dpf test acc
	accVecGlobalReset();
    }

    WorldModel::~WorldModel() {
    }

    int WorldModel::GetOppCount(Vector2f center) {
        int count = 0;
        int length = 2;
        std::map<unsigned int, seumath::Vector3f>::const_iterator iter;

        FOR_EACH(iter, mOpponentGlobalPos) {
            Vector3f OppPos = iter->second;
            bool condition = (OppPos.x() < center.x() + length) && (OppPos.x() > center.x() - length) && (OppPos.y() > OppPos.y() - length / 2) && (OppPos.y() < center.y() + length);
            if (condition)
                count++;
        }
        return count;
    }

    bool WorldModel::update(shared_ptr<Perception> p) {
        if (!updatePerception(p))
            return false;

        //TT REL
        updateObjectRelInfo(); //this info may be used in localization
        buildBlocks();

        updateSelf();
        updateBall();
	updateState();
        SHM.update();
        PM.update();
        updatePlayers();
	updateCast();
	updateMySpeed();
	if(Tcount>=number)
	  for(int i = 1;i<=11;i++)
        UpdateOppSpeed(i);

        mSimCycle++;
        if (1 == mSimCycle) {
            mStartSimTime = getSimTime();
        }
        //cout<<getBallGlobalPos2D().x()<<'\t'<<getBallGlobalPos2D().y()<<endl;
        return true;
    }

    void WorldModel::updateCast()
    {
	mHearGoodPos =SHM.getHearGoodPosID();
        mHearCastType =SHM.getHearCastType();
	mOurCast =PM.getOurCast();
    }
      
    void WorldModel::turnOffTheDynamicCast()
    {
	statusToturnOnDynamicCast = 1;
    }
    
    void WorldModel::turnOnTheDynamicCast()
    {
	statusToturnOnDynamicCast = 0;
    }

    
    bool WorldModel::updateState()      //maybe this have some errors
    {
	float ourDist =PM.getOurMinTimeToBall();
	float oppDist =PM.getOppMinTimeToBall();
	Vector2f ballPos =WM.getBallGlobalPos2D();
// 	if(getOurFastestToBallNum() <4)mOurCastType = 2;
// 	else
// 	{
// 	    mOurCastType =(ourDist -oppDist) >1.0f;
// 	    if(ballPos.x() <0)mOurCastType = 1;
// 	}
	if(ballPos.x() < 0)
	{
	  mOurCastType = 1;
	}
	else 
	{
	  mOurCastType = 2;
	}
	return true;
    }
    
    bool WorldModel::updatePerception(shared_ptr<Perception> p) {
        if (NULL == p.get()) return false;

        const Perception& lp = lastPerception();

        float deltaTime = p->time().now() - lp.time().now();
        mLostSimTime = deltaTime - sim_step;

        if (deltaTime > sim_step / 1000) //TT: don't use "deltaTime>0", because it's a floating number
        {
            //1. update goal(score)
            if (lp.getPlayMode() != serversetting::PM_GOAL_LEFT &&
                    lp.getPlayMode() != serversetting::PM_GOAL_RIGHT) {
                if ((p->getPlayMode() == serversetting::PM_GOAL_LEFT
                        && WM.getOurTeamIndex() == serversetting::TI_LEFT)
                        || (p->getPlayMode() == serversetting::PM_GOAL_RIGHT
                        && WM.getOurTeamIndex() == serversetting::TI_RIGHT)) {
                    //cout<<"[WorldModel] our team got a goal\n";
                    mOurGoal++;
                } else if ((p->getPlayMode() == serversetting::PM_GOAL_RIGHT
                        && WM.getOurTeamIndex() == serversetting::TI_LEFT)
                        || (p->getPlayMode() == serversetting::PM_GOAL_LEFT
                        && WM.getOurTeamIndex() == serversetting::TI_RIGHT)) {
                    //cout<<"[WorldModel] our team lost a goal\n";
                    mOppGoal++;
                }
            }

            //2. TT mark: update joints rate in p by lp
            p->update(lp);

            //3. TT mark: predict new perception by last action
            shared_ptr<const Action> lastAct = AGENT.getLastAction();
            if (0 != lastAct.get()) {
                mPredictedPerception = *(p.get());
                float aveStepTime = getAverageStepTime();
                mPredictedPerception.predict(lastAct, aveStepTime); //TT test remove//////////////////////////////////
            }

            //4. update mPerceptions(deque)
            mPerceptions.push_back(p);
            if (mPerceptions.size() > max_perception_size) {
                mPerceptions.pop_front();
            }

            return true;
        }
        return false;
    }

    void WorldModel::updateSelf() {
        // 0. vision info
        shared_ptr<const Vision> vp = lastPerception().vision();
        if (vp.get() != NULL) {
            mLatestV = vp; //terry
        }
        int flagsNumber = getFlagNumbersISee();
	
	//if(calcVisionBodyAngZ(*vp)){
	  //cout<<"vision body angZ\t"<<mVisionBodyAngZ<<endl;
	//}
	// 0.4 added by dpf update local transMatrix of joints;
	//see it below
	 map<unsigned int, AngDeg> angles = lastPerception().joints().jointAngles();
	 //lean rel trans
	 
	 TransMatrixf zeroTrans;
	 zeroTrans.identity();
	 mBoneLocalTrans.clear();
	 HUMANOID.forwardKinematics(robot::humanoid::Humanoid::TORSO, zeroTrans, angles, mBoneLocalTrans);
	 
	 // 0.5 added by dpf update the body transfer matrix
	mDpfBodyTransMatrix=updateDpfBodyTransMatrix();
	 // 1. calculate the global position and rotation of torso
        localization();
        const TransMatrixf& vt = getVisionTrans();
	//face direction maybe meaningless. comment by dpf
        mMyFaceDirection = atan2Deg(vt.o().y(), vt.o().x());


        // 2. forward Kinematics, then we got position and rotation of every body
        mBoneTrans.clear();
	//mBoneLocalTrans.clear();
      //  map<unsigned int, AngDeg> angles = lastPerception().joints().jointAngles();
        TransMatrixf visionTrans = getVisionTrans(); //vision-me
	//TransMatrixf zeroTrans;
	//zeroTrans.identity();
        HUMANOID.forwardKinematics(robot::humanoid::Humanoid::HEAD, visionTrans, angles, mBoneTrans); //vision-me
	//HUMANOID.forwardKinematics(robot::humanoid::Humanoid::TORSO, zeroTrans, angles, mBoneLocalTrans);

        // 3. calculate the center of mass
         mMyCenterOfMass = HUMANOID.calcCenterOfMass(mBoneTrans);
	 //dpf rewrite it
	 mMyBodyAng.x() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngX();
         mMyBodyAng.y() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngY();
         mMyBodyAng.z() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngZ();
	
	 // dpf ,calc rel trans, not lean rel
	 
	 mBoneRelTrans.clear();
	 TransMatrixf relTorsoTrans;
	 relTorsoTrans.identity();
	 //the order (Z-X-Y) is revised with create action (Y-X-Z)
	 //relTorsoTrans.rotationZ(90);//when we init body, we rotate Z with a -90 angle, so we inverse do this
	 //relTorsoTrans.rotationX(mMyBodyAng.x());
	 relTorsoTrans.rotateLocalY(mMyBodyAng.y());
	 relTorsoTrans.rotateLocalX(mMyBodyAng.x());
	 relTorsoTrans.p().z()=getBoneTrans(humanoid::Humanoid::TORSO).p().z();//may have noise
	 HUMANOID.forwardKinematics(robot::humanoid::Humanoid::TORSO, relTorsoTrans, angles, mBoneRelTrans);
	 // dpf, calc rel center of mass, not lean rel
	 mMyRelCenterOfMass=HUMANOID.calcCenterOfMass(mBoneRelTrans);
	//rewrited by dpf use gyro-rate sensor to calc them
	mMyOriginMatrix=getBoneTrans(humanoid::Humanoid::TORSO);
	mMyBodyDirection=mMyOriginMatrix.rotatedAngZ();
	mMyBodyDirection=normalizeAngle(mMyBodyDirection+90);
        //Acc
        //April, MMXI
        const Vector3f& newAcc = lastPerception().accelerometer().rate(0);
        mMyAcc = mMyAcc * 0.9f + newAcc * 0.1f;
	//using acc calculate vec and pos, it is only right in 3-5 secs
        mMyRealAccRel = newAcc;
        seumath::TransMatrixf accTrans=mDpfBodyTransMatrix;
        mMyRealAccGlobal = mDpfBodyTransMatrix.transform(mMyRealAccRel)-Vector3f(0,0,9.81f);
	Vector3f oldAccVelGlobal=mMyAccVelGlobal;
	mMyAccVelGlobal+=mMyRealAccGlobal*0.02f;
	mMyAccPosGlobal+=(oldAccVelGlobal+mMyAccVelGlobal)*0.5f*0.02;
	/*
	///old vision matrix using flags
	if (calcVisionSensorRotationUsingFlags(*vp)){
	///test cout by dpf
	cout<<"gyro vision matrix angz\n"<<getBoneTrans(robot::humanoid::Humanoid::HEAD).rotatedAngZ()<<endl;
	cout<<"vision matrix direct angz\n"<<mMyVisionMatrixUsingFlags.rotatedAngZ()<<endl;
      */
	///old feet force part, need more work!!!
	mMySupportBone = Humanoid::ILLEGAL;
        mFeetForce.zero();
        mFeetForcePoint.zero();
        const ForceResistance& forceResistance = lastPerception().forceResistance();
        unsigned int lfid = HUMANOID.getBoneId(Humanoid::L_FOOT);
        unsigned int rfid = HUMANOID.getBoneId(Humanoid::R_FOOT);
        if (forceResistance.isTouch(lfid)) {
            mMySupportBone = Humanoid::L_FOOT;
            const perception::ForceResistance::FeedBack& lfrp = forceResistance.feedBack(lfid);
            TransMatrixf lfm = getBoneLocalTrans(Humanoid::L_FOOT);//dpf change it to lean rel trans
            mLeftFootForceCenter = lfm.transform(lfrp.pos);
	    mLeftFootForce=lfm.transform(lfrp.force);
            mFeetForce = mLeftFootForce;
            mFeetForcePoint = mLeftFootForceCenter;
        }

        if (forceResistance.isTouch(rfid)) {
            const perception::ForceResistance::FeedBack& rfrp = forceResistance.feedBack(rfid);
            TransMatrixf rfm = getBoneLocalTrans(Humanoid::R_FOOT);//dpf change it to lean rel trans
	    mRightFootForceCenter = rfm.transform(rfrp.pos);
	    mRightFootForce=rfm.transform(rfrp.pos);
            if (Humanoid::ILLEGAL == mMySupportBone) {
                mMySupportBone = Humanoid::R_FOOT;
                mFeetForce = mRightFootForce;
                mFeetForcePoint = mRightFootForceCenter;
            } else {
                // double support, choose the bigger force foot as support foot
                float lf = forceResistance.feedBack(lfid).force.length();
                float rf = forceResistance.feedBack(rfid).force.length();
                if (lf < rf) {
                    mMySupportBone = Humanoid::R_FOOT;
                }
                mFeetForce += mRightFootForce;
                mFeetForcePoint = (mLeftFootForceCenter * lf + mRightFootForceCenter * rf) / (lf + rf);
            }
        }
    }

    void WorldModel::updateBall() {
        static const float minZ = ball_radius - 0.0018f;

        shared_ptr<const Vision> vp = lastPerception().vision();
        if (NULL == vp.get() || false == canSeeBall()) return;

        // record the postion of ball before,to calculate the velocity of ball
        Vector3f oldPos = mBallGlobalPos;
        const Vector3f& localRelPosBall = vp->pos(Vision::BALL);
        //const Vector3f& localRelPosG2R = vp->pos(Vision::G2R);
        //const Vector3f& localRelPosG1R = vp->pos(Vision::G1R);
        ballLaPol = vp->pol(perception::Vision::BALL);

        Vector3f posSee = getVisionTrans().transform(localRelPosBall);
	//cout<<localRelPosBall.x()<<'\t'<<localRelPosBall.y()<<endl;
        //Vector3f posSeeG2R = getVisionTrans().transform(localRelPosG2R); //terry
        //Vector3f posSeeG1R = getVisionTrans().transform(localRelPosG1R); //terry
        Vector3f velSee = (posSee - oldPos) / (3 * sim_step);

        // simple simulation
        Vector3f velSim = mBallGlobalVel;
        Vector3f posSim = mBallGlobalPos;
        predictBall(posSim, velSim);
        Vector3f visionError(1, 1, 10);
        Vector3f diffVel = velSee - velSim;
        if (pow2(diffVel.x()) + pow2(diffVel.y()) > 16 / 9.0f) {
            // the ball is kicked or be moved
            mBallGlobalPos = posSee;
            //mG2RGlobalPos = posSeeG2R; //terry
            //mG1RGlobalPos = posSeeG1R; //terry
            mBallGlobalVel = velSee;
            if (mBallGlobalVel.squareLength() > 2500) {
                mBallGlobalVel.zero();
            }
            for (int i = 0; i < 3; i++)
                mBallPvekf[i].setP(visionError[i] / (sim_step * 3), 0, 0, visionError[i]);
        }
        else {
            mBallGlobalPos = posSim;
            mBallGlobalVel = velSim;
            for (int i = 0; i < 3; i++)
                mBallPvekf[i].update(mBallGlobalVel[i], mBallGlobalPos[i], velSim[i], sim_step * 3,
                    velSee[i], posSee[i], visionError[i]);
        }
      //comment by dpf, maybe useless!
      //  mBallGlobalPos.z() = max(mBallGlobalPos.z(), minZ);

        posSim = mBallGlobalPos;
        velSim = mBallGlobalVel;

        mMyInterceptBallTime = predictInterceptBall(posSim, velSim, getMyOrigin(), 0.4f, 60);
        //mInterceptBallGlobalPos = posSim;
        mInterceptBallGlobalPos = posSee + velSee * 0.06f;

        while (velSim.squareLength() > 0.01f) {
            predictBall(posSim, velSim); //camera cancel(dangerous)
            posSim.z() = max(posSim.z(), minZ);
        }

        mBallGlobalStopPos = posSim;

        // average position of ball
        mBallAveragePos = (mBallAveragePos * mSimCycle + mBallGlobalPos) / (mSimCycle + 1);

        //TT
        //calBallGlobalPos2DWithRelInfo();
        //mBallGlobalPos3DWithRelInfo.set(mBallGlobalPos2DWithRelInfo.x(), mBallGlobalPos2DWithRelInfo.y(), ball_radius);
    }
//use flags only
bool WorldModel::calcVisionSensorRotationUsingFlags(const perception::Vision& v)
{
  if(getFlagNumbersISee()<3)return false;//if no enough flags is seen
  else{//flagsNumber>=3
    mMyVisionMatrixUsingFlags.identity();
    Vector3f myVisionPos=getBoneTrans(robot::humanoid::Humanoid::HEAD).p();
    mMyVisionMatrixUsingFlags.p()=myVisionPos;
	    set<Vision::FID>::iterator flags;
	    int i;
	    for(flags=v.getStaticFlagSet().begin(),i=3;// i used to use pointer, but i failed, may because the pointer++ is in mem, not in data_Structure;
		i<=v.getStaticFlagSet().size();
		flags++,i++){
            Vision::FID f1 = *flags;
            Vision::FID f2 = *(++flags);
            Vision::FID f3 = *(++flags);
            mMyVisionMatrixUsingFlags.R() += calcVisionSensorRotationUsingFlags(myVisionPos, v, f1, f2, f3);
	      }// is there any good method except this to get its average? dpf ask here;
	    mMyVisionMatrixUsingFlags.R()*=1.0f/(v.getStaticFlagSet().size()-2); 
  }
  return true;
}

//use flags only, myPos is visionPos
Matrix3x3f WorldModel::calcVisionSensorRotationUsingFlags(const seumath::Vector3f& myPos, const perception::Vision& v, Vision::FID flagA, Vision::FID flagB, Vision::FID flagC) const
{
	Vector3f relPosFlagA = Vision::getFlagGlobalPos(flagA) - myPos;
        Vector3f relPosFlagB = Vision::getFlagGlobalPos(flagB) - myPos;
        Vector3f relPosFlagC = Vision::getFlagGlobalPos(flagC) - myPos;
        Matrix3x3f relMat(relPosFlagA,
                relPosFlagB,
                relPosFlagC);
        relMat.inv();
        Vector3f localRelPosFlagA = v.pos(flagA);
        Vector3f localRelPosFlagB = v.pos(flagB);
        Vector3f localRelPosFlagC = v.pos(flagC);
        Matrix3x3f localRelMat(localRelPosFlagA,
                localRelPosFlagB,
                localRelPosFlagC);
        Matrix3x3f rotateMat = relMat * localRelMat;
        rotateMat.transpose();
        return rotateMat;
}

    void WorldModel::updatePlayers() {
        shared_ptr<const Vision> vp = lastPerception().vision();
        if (NULL == vp.get()) return;

        const TransMatrixf& eyeMat = getVisionTrans();

        // 1. teammates
        mOurPlayerNum = 0;
        const Vision::TTeamPolMap& ourPol = vp->ourPolMap();
        mTeammateGlobalPos.clear();

        FOR_EACH(iter, ourPol) {
            Vector3f p = Vision::calLocalRelPos(iter->second.begin()->second);
            mTeammateGlobalPos[iter->first] = eyeMat.transform(p);
        }

        // update myself information in team, for correcting illegal value
        mTeammateGlobalPos[getMyUnum()] = eyeMat.pos();

        // calculate the fastest teammate to the ball
        //float player_speed = 0.4f;
        //  mOurFastestToBall = calFastestIdToBall(mTeammateGlobalPos, player_speed, mOurFastestToBallTime);
        mOurFastestToBall = PM.getOurFastestID();
        mOurFastestToBallTime = PM.getOurMinTimeToBall();

        // 2. opponents
        mOppPlayerNum = 0;
        const Vision::TTeamPolMap& oppPol = vp->oppPolMap();
        mOpponentGlobalPos.clear();
	mOppBodyInformation.clear();
        FOR_EACH(iter, oppPol) {
            //  Vector3f p = Vision::calLocalRelPos(iter->second.find(Vision::HEAD)->second);
            Vector3f p = Vision::calLocalRelPos(iter->second.begin()->second);
            mOpponentGlobalPos[iter->first] = eyeMat.transform(p);
	    p = Vision::calLocalRelPos(iter->second.find(Vision::HEAD)->second); 
	    mOppBodyInformation[iter->first][Vision::HEAD] =eyeMat.transform(p);
	    p = Vision::calLocalRelPos(iter->second.find(Vision::L_FOOT)->second); 
	     mOppBodyInformation[iter->first][Vision::L_FOOT] =eyeMat.transform(p);
	    p = Vision::calLocalRelPos(iter->second.find(Vision::L_HAND)->second); 
	     mOppBodyInformation[iter->first][Vision::L_HAND] =eyeMat.transform(p);
	    p = Vision::calLocalRelPos(iter->second.find(Vision::R_FOOT)->second); 
	     mOppBodyInformation[iter->first][Vision::R_FOOT] =eyeMat.transform(p);
	    p = Vision::calLocalRelPos(iter->second.find(Vision::R_HAND)->second); 
	     mOppBodyInformation[iter->first][Vision::R_HAND] =eyeMat.transform(p);
            
            mOppPlayerNum++;
        }
        
        if(Tcount<number)
	{
	  //mOppWalkPos[Tcount][0][0] = getOppBodyInformation(1,Vision::L_FOOT);
	  //mOppWalkPos[Tcount][0][1] = getOppBodyInformation(1,Vision::R_FOOT);
	  for(int j=1;j<=11;j++)
	  {
	  mOppWalkPos[Tcount][j][0] = getOppBodyInformation(j,Vision::L_FOOT);
	  mOppWalkPos[Tcount][j][1] = getOppBodyInformation(j,Vision::R_FOOT);
	  //mOppWalkPos[Tcount][j][0]=mIllegalPos;
	  //mOppWalkPos[Tcount][j][1]=mIllegalPos;
	  
	  }
	  mMyWalkPos[Tcount]=getMyGlobalPos();
	  
	}
	
	if(Tcount>=number)
	{
	  for(int j=1;j<=11;j++)
	  {
	    for(int i=number-1;i>=1;i--)
	    {
	      mOppWalkPos[i][j][0]=mOppWalkPos[i-1][j][0];
	      mOppWalkPos[i][j][1]=mOppWalkPos[i-1][j][1];
	      if(i==1)
	      {
		mOppWalkPos[0][j][0]=getOppBodyInformation(j,Vision::L_FOOT);
		mOppWalkPos[0][j][1]=getOppBodyInformation(j,Vision::R_FOOT);
	      }
	    }
	    
	  }
	  for( int q=number-1;q>=1;q--)
	    { 
		  mMyWalkPos[q]=mMyWalkPos[q-1];
		  if(q==1)
		    mMyWalkPos[0]=getMyGlobalPos();
	 
	    }
	}
	if(Tcount<number)
	  Tcount++;

        //   mOppFastestToBall = calFastestIdToBall(mOpponentGlobalPos, 0.6f, mOppFastestToBallTime);
        mOppFastestToBall = PM.getOppFastestID();
        mOppFastestToBallTime = PM.getOppMinTimeToBall();
        mOurPlayerNum = PM.getOurTeammates().size();
        mOppClosestToMe = calClosestIdToPosition(getMyOrigin(), mOpponentGlobalPos, mOppClosestToMeDist);
    }

    void WorldModel::updateObjectRelInfo() {
        shared_ptr<const Vision> v = lastPerception().vision();
        if (NULL == v.get())
            return;

        //init
        mBallRelInfo.canSee = false;

        FOR_EACH(iter, mFlagRelInfoMap) {
            (iter->second).canSee = false;
        }

        //get map in Vision
        const map<Vision::FID, Vector3f>& objPolMap = v->objectPolMap(); //Vision didn't add lines into this map

        FOR_EACH(iter, objPolMap) {
            Vision::FID fid = iter->first;

            //ball
            if (Vision::BALL == fid) {
                Vector2f oldpos = mBallRelInfo.relPos2D;
                mBallRelInfo.canSee = true;
                mBallRelInfo.relPos2D = calObjRelPos2D(iter->second);
                mBallRelInfo.pol2D = calObjPol2D(mBallRelInfo.relPos2D);
                mBallRelVel2D = (mBallRelInfo.relPos2D - oldpos) / (3 * sim_step);
                continue;
            }//flags
            else if (fid == Vision::F1L ||
                    fid == Vision::F2L ||
                    fid == Vision::F1R ||
                    fid == Vision::F2R ||
                    fid == Vision::G1L ||
                    fid == Vision::G2L ||
                    fid == Vision::G1R ||
                    fid == Vision::G2R) {
                mFlagRelInfoMap[fid].canSee = true;
                mFlagRelInfoMap[fid].relPos2D = calObjRelPos2D(iter->second);
                mFlagRelInfoMap[fid].pol2D = calObjPol2D(mFlagRelInfoMap[fid].relPos2D);
            }
        }
    }

        /**
     * calc the body rel pos from vision pol pos, take care is not vision rel pos.
     * vision rel pos is always useless!!!
     * rewritten by dpf
     * */
    Vector2f WorldModel::calObjRelPos2D(const Vector3f& objPolToVisionSensor) {
        return *(seumath::Vector2f*)(calObjRelPos(objPolToVisionSensor).get());
    }
    //added by dpf,calculate the object body rel pos from vision pol pos, not vision rel pos.
    //not body lean rel pos
    //but attention, lean rel pos works better, I don't know why. may because when we want to kick ball
    //the ball rel  pos is more accurate!!!

    Vector3f WorldModel::calObjRelPos(const seumath::Vector3f& objPolToVisionSensor) {
        TransMatrixf visionLocalRelTrans = getBoneLocalTrans(humanoid::Humanoid::HEAD); //lean rel is not rel
        //visionLocalRelTrans.p().z()=getBoneTrans(humanoid::Humanoid::HEAD).p().z();
        Vector3f objRelToVisionSensor = Vision::calLocalRelPos(objPolToVisionSensor);
        return visionLocalRelTrans.transform(objRelToVisionSensor);
    }

    Vector2f WorldModel::calObjPol2D(const Vector2f& relPos2D) {
        float x = relPos2D.x();
        float y = relPos2D.y();
        float dist = sqrt(x * x + y * y);
        float ang = -atan2Deg(x, y); //atan2Deg returns -180 to 180 angle-degree

        return Vector2f(dist, ang);
    }

    void WorldModel::buildBlocks() {
	seumath::Vector2f centerVec, sideVec;
        shared_ptr<const Vision> v = lastPerception().vision();
        if (NULL == v.get())
            return;

        //===================================for ball
        if (mBallRelInfo.canSee) {
            mBallBlock.dist = mBallRelInfo.pol2D.x();
            mBallBlock.angC = mBallRelInfo.pol2D.y();
            //float theta = asinDeg(0.4f / mBallBlock.dist); //ball block size ////////////////////////////////////
            centerVec=mBallRelInfo.relPos2D;
	    sideVec=centerVec+Vector2f(0.3f,0);
	    float theta = fabs(centerVec.angle()-sideVec.angle());
	    mBallBlock.angL = mBallBlock.angC + theta;
            mBallBlock.angR = mBallBlock.angC - theta;
	    mBallBlock.isOurTeam =0;                             //ghd for avoid player
        }

        //===================================players' body
        mBlockList.clear();
        list<Vector2f> tempV2fList; //body parts relPos2D list
        Vector2f centerOfCircle;
        float radius;
        float tempFloat;
        Vector2f tempV2f;
        float theta;
	float theta2 =8.0f;

        //====================our players
        const Vision::TTeamPolMap& ourPlayersMap = v->ourPolMap();

        FOR_EACH(iterPlayer, ourPlayersMap) {
            if (getMyUnum() == iterPlayer->first) //myself
                continue;
	
	    //if(mOurNearstToMeDistance>(iterPlayer->second -getMyGlobalPos())
	    
            //========================one player's body
            const Vision::TPlayerPolMap& bodyPartMap = iterPlayer->second;
            //add body parts to the list
            tempV2fList.clear();

            FOR_EACH(iterBodyPart, bodyPartMap) {
                tempV2fList.push_back(calObjRelPos2D(iterBodyPart->second));
            }
            if (tempV2fList.empty())
                continue;

            //calculate the center of circle
            centerOfCircle.set(0, 0);

            FOR_EACH(iterV2f, tempV2fList) {
                centerOfCircle += *iterV2f;
            }
            centerOfCircle /= tempV2fList.size();

            //calculate the radius of circle
            radius = 0;

            FOR_EACH(iterV2f, tempV2fList) {
                tempFloat = ((*iterV2f) - centerOfCircle).length();
                if (tempFloat > radius)
                    radius = tempFloat;
            }

            //build a block
            BlockInfo playerBlock;
            tempV2f = calObjPol2D(centerOfCircle);
            playerBlock.dist = tempV2f.x();
            playerBlock.angC = tempV2f.y();
	    centerVec=centerOfCircle;
	    sideVec=centerOfCircle+Vector2f(radius+0.3,0);
            theta=fabs(centerVec.angle()-sideVec.angle());
	    //theta = asinDeg(radius + 0.4f / playerBlock.dist); /////////////////////////////////////// 0 ?
            playerBlock.angL = playerBlock.angC + theta;
            playerBlock.angR = playerBlock.angC - theta;
	    playerBlock.isOurTeam =1;                    // --ghd
	    
 	    if (mOurFastestToBall == iterPlayer ->first) //fastest player --ghd
	    {
		mPlayerBlock =playerBlock;
		mPlayerBlock.angL +theta2;
		mPlayerBlock.angR -theta2;
	    }
	    
            mBlockList.push_back(playerBlock);
        } //loop for our players

        //====================opp players
        const Vision::TTeamPolMap& oppPlayersMap = v->oppPolMap();

	mOppFastestToBall;
	
        FOR_EACH(iterPlayer, oppPlayersMap) {
            //========================one player's body
            const Vision::TPlayerPolMap& bodyPartMap = iterPlayer->second;

            //add body parts to the list
            tempV2fList.clear();
            FOR_EACH(iterBodyPart, bodyPartMap) {
                tempV2fList.push_back(calObjRelPos2D(iterBodyPart->second));
            }
            if (tempV2fList.empty())
                continue;

            //calculate the center of circle
            centerOfCircle.set(0, 0);

            FOR_EACH(iterV2f, tempV2fList) {
                centerOfCircle += *iterV2f;
            }
            centerOfCircle /= tempV2fList.size();

            //calculate the radius of circle
            radius = 0;

            FOR_EACH(iterV2f, tempV2fList) {
                tempFloat = ((*iterV2f) - centerOfCircle).length();
                if (tempFloat > radius)
                    radius = tempFloat;
            }

            //build a block
            BlockInfo playerBlock;
            tempV2f = calObjPol2D(centerOfCircle);
            playerBlock.dist = tempV2f.x();
            playerBlock.angC = tempV2f.y();
	    centerVec=centerOfCircle;
	    sideVec=centerOfCircle+Vector2f(radius+0.4,0);
            theta=fabs(centerVec.angle()-sideVec.angle());
            //theta = asinDeg(radius + 0.4f / playerBlock.dist); /////////////////////////////////////// 0 ?
            playerBlock.angL = playerBlock.angC + theta;
            playerBlock.angR = playerBlock.angC - theta;
	    playerBlock.isOurTeam =0;            //--ghd
            mBlockList.push_back(playerBlock);
        } //loop for opp players

        //============================sort by dist
        mBlockList.sort(sortByDist);
    }
    //all in one function

    Vector3f WorldModel::calDpfVisionGlobalPos(const perception::Vision& v) {
        //Pr*mR+mP=Pg; so mP=Pg-Pr*mR; the right-product is not a common understanding way;
        //now vision Trans's R() is updated, but the p() is not
        seumath::TransMatrixf notGoodTrans;
        notGoodTrans.identity();
        notGoodTrans.R() = getVisionTrans().R();
        set<Vision::FID> flags = v.getStaticFlagSet();
        set<Vision::FID>::iterator It = flags.begin();
        Vector3f flagGlobalPos, flagLocalRelPos, ret = Vector3f(0, 0, 0);
        for (; It != flags.end(); It++) {
            flagGlobalPos = Vision::getFlagGlobalPos(*It);
            flagLocalRelPos = v.pos(*It);
            ret += flagGlobalPos - notGoodTrans.transform(flagLocalRelPos);
        }
        ret *= 1.0f / flags.size();
        return ret;
    }

    Vector3f WorldModel::calDpfGlobalPosWithOnlyBall() {
        //Pr*mR+mP=Pg; so mP=Pg-Pr*mR; the right-product is not a common understanding way;
        //now vision Trans's R() is updated, but the p() is not
        seumath::TransMatrixf notGoodTrans;
        notGoodTrans.identity();
        notGoodTrans.R() = getVisionTrans().R();
        Vector3f ballGlobalPos = getBallGlobalPos();
        Vector3f ballRelPos = Vector3f(getBallRelPos2D().x(), getBallRelPos2D().y(), ball_radius);
        Vector3f ret = ballGlobalPos - notGoodTrans.transform(ballRelPos);
        return ret;
    }

    float WorldModel::getAverageStepTime() const {
        shared_ptr<const Perception> pe = mPerceptions.back();
        shared_ptr<const Perception> pb = mPerceptions.front();
        float time = pe->time().now() - pb->time().now();
        int cycle = mPerceptions.size() - 1;
        return floor(time / cycle / serversetting::sim_step + 0.8f) * serversetting::sim_step;
    }

    void WorldModel::localization() {
        //if there is no vision message, do nothing
        const Vision* v = lastPerception().vision().get();
        if (NULL == v) return;
        //test by dpf
        mDpfMyVisionMatrix.R() = calcDpfVisionSensorRotation().R();
        //calculate the golbal position
        int flagNumbers = seenFlagsNum();
        if (flagNumbers > 0) {
            mLastTimeSeeEnoughFlags = WM.getSimTime();
            mMyGlobalPos = calDpfVisionGlobalPos(*v);
        } else if (canSeeBall()) {
            mMyGlobalPos = calDpfGlobalPosWithOnlyBall();
        } else return;
        mDpfMyVisionMatrix.p() = mMyGlobalPos;

    }

    /**test by dpf
     *only consider mR here excluing mP
     **/
    TransMatrixf WorldModel::calcDpfVisionSensorRotation() const {
        //the zero point, only take rotation into consideration, bodyTrans
        seumath::TransMatrixf headLocalTrans = getBoneLocalTrans(robot::humanoid::Humanoid::HEAD);
        seumath::TransMatrixf myBodyTrans = getDpfBodyTransMatrix();
        //the zero point , only take rotation into consideration, headTrans
        return myBodyTrans.transfer(headLocalTrans);
    }

    void WorldModel::predictBall(Vector3f& p, Vector3f& v) const {
        static const float k = exp(-0.03f / ball_mass * sim_step);
        v *= k;
        if (p.z() > ball_radius * 1.5f) {
            v.z() -= acceleration_of_gravity*sim_step;
        }
        p += v*sim_step;
    }

    Vector3f WorldModel::getPosToBone(Vector3f pos) const {
        const TransMatrixf& tempMat = WM.getBoneTrans(robot::humanoid::Humanoid::TORSO);
        return tempMat.inverseTransform(pos);

    }

    unsigned int WorldModel::calClosestIdToPosition(const Vector3f& pos,
            const map<unsigned int, Vector3f>& data,
            float& minDist) const {
        if (data.empty()) return 0; // should not happen

        map<unsigned int, Vector3f>::const_iterator iter = data.begin();
        map<unsigned int, Vector3f>::const_iterator endIter = data.end();
        unsigned int minNum = iter->first;
        minDist = (pos - iter->second).squareLength();
        for (++iter; endIter != iter; ++iter) {
            float d = (pos - iter->second).squareLength();
            if (d < minDist) {
                minNum = iter->first;
                minDist = d;
            }
        }
        minDist = sqrt(minDist);
        return minNum;
    }

    unsigned int WorldModel::calFastestIdToBall(const std::map<unsigned int, seumath::Vector3f>& data,
            float speed,
            float& minTime) {
        if (data.empty()) return 0; // should not happens

        const float maxtime = 60;
        map<unsigned int, Vector3f>::const_iterator iter = data.begin();
        map<unsigned int, Vector3f>::const_iterator endIter = data.end();
        unsigned int minNum = iter->first;
        Vector3f pb = getBallGlobalPos();
        Vector3f vb = getBallGlobalVel();
        minTime = predictInterceptBall(pb, vb, iter->second, speed, maxtime);
        for (++iter; endIter != iter; ++iter) {
            pb = getBallGlobalPos();
            vb = getBallGlobalVel();
            float t = predictInterceptBall(pb, vb, iter->second, speed, maxtime);
            if (iter->second.z() < 0.3f) // fallen
            {
                //t += 2;
            }

            if (t < minTime) {
                minNum = iter->first;
                minTime = t;
            }
        }
        return minNum;
    }

    float WorldModel::predictInterceptBall(Vector3f& posBall, Vector3f& velBall, const Vector3f& pos,
            float speed, float maxTime) const {
        float time = 0;
        while (time < maxTime) {
            float dist = sqrt(pow2(posBall.x() - pos.x()) + pow2(posBall.y() - pos.y()));
            if (time * speed > dist) {
                return time;
            }

            if (velBall.squareLength() < 0.01f) {
                // the ball stopped
                return time + dist / speed;
            }

            predictBall(posBall, velBall);
            posBall.z() = max(posBall.z(), ball_radius);

            time += sim_step;
        }
        return -1;
    }

    // =====================================
    // high level judgement
    // =====================================

    bool WorldModel::isOppNearMe()		//ghd
    {
	float myLength=getBallRelPos2D().length();
	float oppLength =getOppFastestToBallDistance();
	/*if(mBallGlobalPos.x() >0)*/return oppLength-myLength <1.1f;
// 	else return oppLength-myLength <0.3f;
    }
    
    bool WorldModel::isGroundBall() const {
        return (abs(WM.getBallGlobalPos().z()) < MAX_POSTION_DIFF);
    }

    bool WorldModel::isStable() const {
        // accodring to rotation-Z, rotation-Y, rotation-X order
        /* | cz*cy          sz*cy          -sy   0 |
           | cz*sy*sx-sz*cx sz*sy*sx+cz*cx cy*sx 0 |
           | cz*sy*cx+sz*sx sz*sy*cx-cz*sx cy*cx 0 |
           | 0              0              0     1 | */
        return ( abs(mMyBodyAng.x()) < 20 && abs(mMyBodyAng.y()) < 20);
    }

    bool WorldModel::isLeftRolling() const {
        return mMyBodyAng.y() < -20 && getMyGyroRate().length() > 2;
    }

    bool WorldModel::isLeftRolled() const {
        return ( abs(mMyBodyAng.x()) < 45 &&
                mMyBodyAng.y() < -70 &&
                getMyGyroRate().length() < 5);
    }

    bool WorldModel::isRightRolling() const {
        return ( mMyBodyAng.y() > 20 &&
                getMyGyroRate().length() > 2);
    }

    bool WorldModel::isRightRolled() const {
        return ( abs(mMyBodyAng.x()) < 45 &&
                mMyBodyAng.y() > 70 &&
                getMyGyroRate().length() < 5);
    }

    bool WorldModel::isDiving() const {
        return ((mMyBodyAng.x()<-20 && mMyBodyAng.x()>-90 && getMyGyroRate().x()<-5) ||
                (mMyBodyAng.x()<-90 && getMyGyroRate().x() > 5));
    }

    bool WorldModel::isDived() const {
        return ( getMyAcc().y() < -6); //TT acc
    }

    bool WorldModel::isDived(AngDeg angX) const {
        return ( mMyBodyAng.x() < -angX);
    }

    bool WorldModel::isLying() const {
        return ((mMyBodyAng.x() > 20 && mMyBodyAng.x() < 90 && getMyGyroRate().x() > 2) ||
                (mMyBodyAng.x() > 90 && getMyGyroRate().x()<-2));
    }

    bool WorldModel::isLied() const {
        return ( getMyAcc().y() > 6); //TT acc
    }

    bool WorldModel::isLied(AngDeg angX) const {
        return ( mMyBodyAng.x() > angX);
    }

    bool WorldModel::isLeftFall() const {
        return ( getMyAcc().x() > 8); //TT: graz is mMyBodyAng.y()<-60
    }

    bool WorldModel::isLeftFall(AngDeg angle) const {
        return ( mMyBodyAng.y() < -angle);
    }

    bool WorldModel::isRightFall() const {
        return ( getMyAcc().x() < -8); //TT: graz is mMyBodyAng.y()>60
    }

    bool WorldModel::isRightFall(AngDeg angle) const {
        return ( mMyBodyAng.y() > angle);
    }

    bool WorldModel::isFall() const {
        return ( isLied() || isDived());
    }

    bool WorldModel::amIFastestToBallOfOurTeam() const {
        return ( mOurFastestToBall == getMyUnum());
    }

    float WorldModel::howIFasterToBallThanOpps() const {
        return mOppFastestToBallTime - mMyInterceptBallTime;
    }

    bool WorldModel::isTouchBall(unsigned int footID) const {
        // We assume that no more than one object touches the specified foot.
        // Of course, this is not accurate, however, very easy to achieve.
        // We must make it more accurate in the future!!

        const ForceResistance& forceResistance = lastPerception().forceResistance();

        if (!forceResistance.isTouch(footID)) {
            return false;
        }

        const Vector3f& posBall = WM.getBallGlobalPos();
        if (/*FRID_LEFT_FOOT*/0 == footID) {
            float d = (mLeftFootForceCenter - posBall).squareLength();
            return d < pow2(ball_radius);
        }

        if (/*FRID_RIGHT_FOOT*/1 == footID) {
            float d = (mRightFootForceCenter - posBall).squareLength();
            return d < pow2(ball_radius);
        }

        return false;
    }

    //return true if opp player is in the ball field

    bool WorldModel::isPlayerInField(const Vector2f& pos, float margin) {
        return (fabs(pos[0]) < (field_length / 2.0f + margin)
                && fabs(pos[1]) < (field_width / 2.0f + margin));
    }

    bool WorldModel::isCloseToGoal() const {
        float size = 1;
        const Vector3f& posMe = getMyGlobalPos();
        float x = abs(posMe.x());
        if (x > half_field_length - size && x < half_field_length + goal_depth + size) {
            float y = abs(posMe.y());
            if (y < half_goal_width + size) {
                return true;
            }
        }
        return false;
    }

    bool WorldModel::isHaveOppInArea(TConvexPolygon<float> area) {

        Vector2f opp2D;
        Vector3f opp;
        for (unsigned int i = 1; i <= 11; i++) {
            opp = getOppGlobalPos(i);
            opp2D = Vector2f(opp[0], opp[1]);
            if (!isPlayerInField(opp2D, 4))
                continue;
            if (area.isInside(opp2D))
                return true;
        }
        return false;
    }
    //not good, don't use it

    const seumath::TransMatrixf& WorldModel::getRightFootTrans() const {
        return getBoneTrans(robot::humanoid::Humanoid::R_FOOT);
    }
    //not good, don't use it

    const seumath::TransMatrixf& WorldModel::getLeftFootTrans() const {
        return getBoneTrans(robot::humanoid::Humanoid::L_FOOT);
    }

    bool WorldModel::IsJoin()
    {
	return SHM.IsJoin();
    }
    
    bool WorldModel::isBallToOurGoal(void) {
        Vector3f ballVel = WM.getBallGlobalVel();
        Vector2f ballVel2D(ballVel.x(), ballVel.y());
        if (ballVel.length() < 0.01f) return false;
        Vector3f ballPos = WM.getBallGlobalPos();
        Vector3f ballToLeftGoal = Vector3f(-half_field_length, half_goal_width, 0) - ballPos;
        AngDeg angOfBallAndLeftGoal = ballToLeftGoal.angle();
        Vector3f ballToRightGoal = Vector3f(-half_field_length, -half_goal_width, 0) - ballPos;
        AngDeg angOfBallAndRightGoal = ballToRightGoal.angle();
        AngDeg angOfBallVel = ballVel2D.angle();
        return isAngInInterval(angOfBallVel, angOfBallAndLeftGoal, angOfBallAndRightGoal);
    }

    //dpf test

    /**X =
     *
     *[ 1,      0,       0]
     *[ 0, cos(a), -sin(a)]
     *[ 0, sin(a),  cos(a)]
     *
     *Y =
     *
     *[  cos(b), 0, sin(b)]
     *[       0, 1,      0]
     *[ -sin(b), 0, cos(b)]
     *Z =
     *
     *[ cos(c), -sin(c), 0]
     *[ sin(c),  cos(c), 0]
     *[      0,       0, 1]
     *so the y-x-z convetion is Z*X*Y=
     *[ cos(b)*cos(c) - sin(a)*sin(b)*sin(c), -cos(a)*sin(c), cos(c)*sin(b) + cos(b)*sin(a)*sin(c)]
     *[ cos(b)*sin(c) + cos(c)*sin(a)*sin(b),  cos(a)*cos(c), sin(b)*sin(c) - cos(b)*cos(c)*sin(a)]
     *[                       -cos(a)*sin(b),         sin(a),                        cos(a)*cos(b)]
     * attention here, I write the transfer matrix in a common way, not the way used in our \
     * source code; but in the below source code, i don't use the common way but our \
     * behavior way. the two method are equal.(i have tested this using matlab)
     * */
    const seumath::TransMatrixf WorldModel::updateDpfBodyTransMatrix() const {
        seumath::TransMatrixf tmp, rel = getDpfBodyTransMatrix();
        seumath::Vector3f myGyroRate = getMyGyroRate()*0.02;
        tmp.rotationY(myGyroRate.y());

        rel.transfer(tmp);

        tmp.rotationX(myGyroRate.x());
        rel.transfer(tmp);

        tmp.rotationZ(myGyroRate.z());
        rel.transfer(tmp);
        return rel;
    }

    bool WorldModel::calcVisionBodyAngZ(const Vision&v) {
        if ((getFlagNumbersISee()) == 0)return false; //if no flags are seen, return false;
        cout << "seeflagnumbers:\t" << getFlagNumbersISee() << endl;
        seumath::AngDeg nowFlagAngZ, initFlagAngZ; //the present and init pos's flag's angZ to body;
        seumath::TransMatrixf initBodyRelToGlobalTrans;
        initBodyRelToGlobalTrans.identity(); //initBodyRelToGlobalTrans.rotationZ(-90);
        initBodyRelToGlobalTrans.p() = getMyGlobalPos();
        mVisionBodyAngZ = 0;
        seumath::TransMatrixf visionRelTrans = getBoneRelTrans(robot::humanoid::Humanoid::HEAD);
        set<Vision::FID> flags = v.getStaticFlagSet();
        set<Vision::FID>::iterator It = flags.begin();
        Vector3f flagLocalRelPos, flagGlobalPos, flagInitRelPos;
        for (; It != flags.end(); It++) {
            flagGlobalPos = Vision::getFlagGlobalPos(*It);
            flagLocalRelPos = v.pos(*It); //rel to vision
            flagLocalRelPos = visionRelTrans.transform(flagLocalRelPos);
            nowFlagAngZ = atan2Deg(flagLocalRelPos.y(), flagLocalRelPos.x());
            flagInitRelPos = initBodyRelToGlobalTrans.inverseTransform(flagGlobalPos);
            initFlagAngZ = atan2Deg(flagInitRelPos.y(), flagInitRelPos.x());
            mVisionBodyAngZ += nowFlagAngZ - initFlagAngZ;
        }
        mVisionBodyAngZ *= -1.0f / flags.size(); //attention here is negative
        return true;
    }

    void WorldModel::logPrintSeenFlags() {
        char str[10];
        str[9] = '\0';
        int i = 0;
        str[i++] = WM.canSeeFlag(Vision::F1L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G1L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G2L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::F2L) ? '1' : '0';
        str[i++] = ' ';
        str[i++] = WM.canSeeFlag(Vision::F1R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G1R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G2R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::F2R) ? '1' : '0';
    }

    /**
     * calculate the Normal Vector of the playground in the coordinate system of the camera
     * @author allen
     *
     * This function uses the points of the lines in the ground to  calculate the normal vector by Least squares method.
     * 
     * not use
     *
     */

    void WorldModel::calcNVectorOfGround() {
        cout << "begin";
        shared_ptr<const Vision> v = lastPerception().vision();
        if (NULL == v.get()) {
            return;
        }
        unsigned int pointId = 0;
        mFieldLines.clear();
        mPointsXYZ.clear();

        mFieldLines = v->getFieldLines();
        TSegmentPolMap::const_iterator iterFL;
        //        if(mFieldLines.size()<=1){
        //            cout<<"only one line"<<endl;
        //            return false;
        //        }

        for (iterFL = mFieldLines.begin();
                iterFL != mFieldLines.end();
                ++iterFL) {
            //   cout <<"mFieldLines"<<endl;
            mPointsXYZ[pointId] = pol2xyz(iterFL->second.p0());
            pointId++;
            mPointsXYZ[pointId] = pol2xyz(iterFL->second.p1());
            pointId++;
        }
        if (pointId < 3) {
            cout << pointId << "too less" << endl;
            return;
        }
        cout << "points" << pointId << endl;
        float *A = new float[pointId * 3];
        float *b = new float[pointId];
        int i = 0;
        int j = 0;
        for (i = 0; i < pointId; i++) {
            A[i * 3 + 0] = mPointsXYZ[i].x();
            A[i * 3 + 1] = mPointsXYZ[i].y();
            A[i * 3 + 2] = 1;
            b[i] = mPointsXYZ[i].z();
        }
        //x=(A^T*A)^(-1)*A^T*b
        float C[3][3];
        float d[3];
        int z = 0;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                C[i][j] = 0;
                for (z = 0; z < pointId; z++) {
                    C[i][j] += A[z * 3 + i] * A[z * 3 + j];
                }
                //    cout << "C["<<i<<"]["<<j<<"]="<<C[i][j];
            }
        }
        TMatrix<float, 3, 3 > MatC(C);
        TMatrix<float, 3, 3 > invMatC;
        cout << "MatC" << MatC;
        if (MatC.det() != 0) {
            invMatC = MatC.inv();
        } else {
            cout << "detis 0!!!!!!!!!!!!!!!!!!!!!11" << endl;
            delete [] A;
            delete [] b;
            return;
        }
        for (i = 0; i < 3; i++) {
            d[i] = 0;
            for (z = 0; z < pointId; z++) {
                d[i] += A[z * 3 + i] * b[z];
            }
        }
        float res[3];
        for (i = 0; i < 3; i++) {
            res[i] = 0;
            for (j = 0; j < 3; j++) {
                res[i] += invMatC[i][j] * d[j];
            }
        }
        mNVector.zero();
        mNVector.x() = -res[0];
        mNVector.y() = -res[1];
        mNVector.z() = 1;
        delete [] A;
        delete [] b;
        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%NVector:" << mNVector << endl;
        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%NVectorNormal:" << mNVector.normalized() << endl;
        cout << "Vision trans " << getVisionTrans() << endl;

    }
    
    bool WorldModel::IsInBlockRange ( unsigned int oppNumber, unsigned int ourNumber, float preDis )
{	Vector2f myPos = getOurGlobalPos2D ( ourNumber );
	Vector2f ballPos = getBallGlobalPos2D();
	Vector2f oppPos = getOppGlobalPos2D ( oppNumber );
	Vector2f oppVector=GetOppWalkVector ( oppNumber );
	oppVector.normalize();
	Vector2f oppPos2=Vector2f ( oppPos.x() +preDis*oppVector.x(),oppPos.y() +preDis*oppVector.y() );
	Vector2f vec1 ( oppPos2.x()-oppPos.x(),oppPos2.y()-oppPos.y() );
	Vector2f vec2 ( myPos.x()-oppPos2.x(),myPos.y()-oppPos2.y() );
	float dis = ( myPos-ballPos ).length();
	vec1.normalize();
	vec2.normalize();
	float delta_angle=fabs ( vec1.angle()-vec2.angle() );
	if ( delta_angle>180 )
		delta_angle=360-delta_angle;
	if ( delta_angle<=80 )
		return true;
	else
		return false;
}

float WorldModel::getPosAngle ( unsigned int oppNumber,unsigned int ourNumber ) ///whether block has completed
{	//oppNumber=11;
	Vector2f myPos = getOurGlobalPos2D ( ourNumber );
	Vector2f ballPos = getBallGlobalPos2D();
	Vector2f oppPos = getOppGlobalPos2D ( oppNumber );
	Vector2f vec1 ( ballPos.x()-oppPos.x(),ballPos.y()-oppPos.y() );
	Vector2f vec2 ( myPos.x()-oppPos.x(),myPos.y()-oppPos.y() );
	float dis = ( myPos-ballPos ).length();
	vec1.normalize();
	vec2.normalize();
	float delta_angle=fabs ( vec1.angle()-vec2.angle() );
	if ( delta_angle>180 )
		delta_angle=360-delta_angle;
	return delta_angle;
}

unsigned int WorldModel::calOurBlockNumber()
{	if ( getMyUnum() !=1 )
	{	return SHM.getHearBlockNumber();
	}

	unsigned int oppNum= getOppDribbleNum();
	if ( oppNum==0 )
	{	state=NoBlock;
		return 0;
	}
	if ( IsOnTheBlock ( oppNum,getMyUnum() ) )
		return 0;
	if ( IsBlockFailed() ==true&&state!=NoBlock )
		state=FailedBlock;
	if ( state==IsBlocking )
		return ourBlockNumber;
	else
	{	Vector2f oppDribblePos=getOppGlobalPos2D ( oppNum );
		Vector2f ballPos = getBallGlobalPos2D();
		///float posAngle = getPosAngle ( oppNum,2 );
		float blockDis = calBlockXDIs ( 2,oppNum );
		float dis= ( getOurGlobalPos2D ( 2 )-oppDribblePos ).length();
		int blockNum=2;
		for ( int i=2; i<=11; i++ )
		{	if ( true )
			{	
			  if (IsInBlockRange ( oppNum,i,2 ) ==true )
				{	float temp= ( getOurGlobalPos2D ( i )-oppDribblePos ).length();
					if ( temp<dis )
					{	dis=temp;
						blockNum=i;
					}
				}
			}
			else
			{	if ( getOurGlobalPos2D ( i ).x() -oppDribblePos.x() <=-1.0f &&getOurGlobalPos2D ( i ).x() -oppDribblePos.x() >=-8.0f )
				{	//if ( posAngle>getPosAngle ( oppNum,i ) )
					//{	posAngle=getPosAngle ( oppNum,i );
					//	blockNum=i;
					//}
					float temp=calBlockXDIs ( i,oppNum );
					if ( blockDis<temp&&temp<=8 )
					{	blockDis=temp;
						blockNum=i;
					}
				}
			}
		}
		ourBlockNumber=blockNum;
		state=IsBlocking;
		return blockNum;
	}


}

///tested,can be used
bool WorldModel::IsOnTheBlock ( unsigned int oppNumber,unsigned int ourNumber ) ///whether block has completed
{	//oppNumber=11;
	Vector2f myPos = getOurGlobalPos2D ( ourNumber );
	Vector2f ballPos = getBallGlobalPos2D();
	Vector2f oppPos = getOppGlobalPos2D ( oppNumber );
	Vector2f vec1 ( ballPos.x()-oppPos.x(),ballPos.y()-oppPos.y() );
	Vector2f vec2 ( myPos.x()-oppPos.x(),myPos.y()-oppPos.y() );
	float dis = ( myPos-ballPos ).length();
	float maxDis=2.0f;
	vec1.normalize();
	vec2.normalize();
	float delta_angle=fabs ( vec1.angle()-vec2.angle() );
	if ( delta_angle>180 )
		delta_angle=360-delta_angle;
	//cout<<"delta_angle="<<delta_angle<<endl;
	float ref_angle = 5.0f;
	if ( dis<=2.0f )
		ref_angle=7.5f;
	if ( dis<=1.0f )
		ref_angle=9.5f;
	if ( dis<0.5f )
		ref_angle=45.0f;
	bool IsOnline= ( delta_angle<ref_angle );
	if ( getOppGlobalPos ( oppNumber ) ==mIllegalPos )
		IsOnline=false;
	//cout<<"IsOnline="<<IsOnline<<endl;
	//cout<<"distrue="<< ( dis<maxDis ) <<endl;
	if ( IsOnline==true&&dis<=maxDis )
		return true;
	else
		return false;
}


int WorldModel::getOppDribbleNum() const//if opp is dribbling,return the number of the dribbler; if not,return 0;
{	unsigned int UNum = GetOppFastestToBallNum();
	Vector2f oppPos = getOppGlobalPos2D ( UNum );
	Vector2f ballPos = getBallGlobalPos2D();
	float dis = ( oppPos-ballPos ).length();
	//cout<<"dis="<<dis<<endl;
	Vector2f vector = GetOppWalkVector ( UNum );
	vector.normalize();
	float speed = oppspeed[UNum];
	///params to decide
	float maxDis = 1.0f;///the max of distance between dribbler and ball when dribbling
	float minSpeed = 0.01f;///
	float epsilon=0.1f;///the delta to judge parallel
	///end of params to decide
	if ( dis>=maxDis ) //if distance is too long, we can say opp isn't dribbling
		return 0;
	Vector2f ballOppVector ( ballPos.x()-oppPos.x(),ballPos.y()-oppPos.y() );
	ballOppVector.normalize();
	if ( ballOppVector.isParallelWith ( vector,epsilon ) ==false )
		return 0;
	if ( speed<=minSpeed )
		return 0;
	return UNum;
}

bool WorldModel::IsBlockFailed()
{	//Vector2f ourPos=getOurGlobalPos2D ( ourBlockNumber );
	Vector2f ourPos=getMyGlobalPos2D();
	unsigned int oppDribbleNumber= ( unsigned ) getOppDribbleNum();
	Vector2f oppPos=getOppGlobalPos2D ( oppDribbleNumber );
	float posAngle=getPosAngle ( oppDribbleNumber,ourBlockNumber );
	float xDis=oppPos.x()-ourPos.x();
	float yDis=oppPos.y()-ourPos.y();
	float Dis=xDis*xDis+yDis*yDis;
	if(posAngle>90)
	  return true;
	if ( Dis<=2&&posAngle>10 )
		return true;
	if ( xDis<=0.2 )
		return true;
	return false;
}

float WorldModel::calBlockXDIs ( unsigned int OurNumber, unsigned int oppNumber )
{	Vector2f myPos=getOurGlobalPos2D ( OurNumber );
	Vector2f oppPos=getOppGlobalPos2D ( oppNumber );
	Vector2f oppVector=GetOppWalkVector ( oppNumber );
	Vector2f oppToMe ( myPos.x()-oppPos.x(),myPos.y()-oppPos.y() );
	float deltaAngle=fabs ( oppVector.angle()-oppToMe.angle() );
	if ( deltaAngle>180 )
		deltaAngle=360-deltaAngle;
	float dis= ( myPos-oppPos ).length();
	float xDis=dis*cos ( deltaAngle*3.141592653/180 );
	return xDis;

}
} // namespace core
