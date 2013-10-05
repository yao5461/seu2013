/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Player.cpp 2755 2009-04-02 08:00:08Z zyj $
 *
 ****************************************************************************/

#include "Player.h"
#include "core/WorldModel.h"
#include "action/InitAction.h"
#include "action/BeamAction.h"
#include "action/Say.h"
#include "action/Actions.h"
#include "configuration/Configuration.h"
#include "controller/FixedAngleTrace.h"
#include "task/WalkRel.h"
#include "math/TLine2.hpp"
#include "math/Math.hpp"
#include "math/TConvexPolygon.hpp"
#include "task/KickTask.h"
#include "task/DynamicKick.h"
#include "task/CameraMotion.h"
#include "core/SayAndHearModel.h"
#include<fstream>
#include<time.h>
#include<algorithm>

namespace soccer {

using namespace std;
using namespace seumath;
using namespace boost;
using namespace serversetting;
using namespace controller;
using namespace perception;
using namespace action;
using namespace task;

Player::Player()
    : mTask(-1, NULL) {
    //load data of KickMotion
    mKickMotionVector.clear();
    
    mFirstKick =false;
    mKickCounter =0;
    
    ifstream inFile("data/kick_motion.txt", ios::in);
    if (NULL != inFile) {
        string oneLine;
        KickMotion tempKM;
        while (!inFile.eof()) {
            getline(inFile, oneLine); //maybe tempKM.firstTaskName
            if (oneLine.empty()) continue; //skip empty line
            if ('#' == oneLine[0] || ' ' == oneLine[0]) continue; //skip comment line
            tempKM.firstTaskName = oneLine;
            inFile >> tempKM.kickTargetRel.x() >> tempKM.kickTargetRel.y()
                   >> tempKM.myDesiredRelPosToBall.x() >> tempKM.myDesiredRelPosToBall.y()
                   >> tempKM.relPosToBallToStopWalk.x() >> tempKM.relPosToBallToStopWalk.y();

            mKickMotionVector.push_back(tempKM);
        }
        inFile.close();
    }

}

Player::~Player() {
}

bool Player::init() {
    if (!Agent::init()) return false;
    // get the respond (first message) from the server
    shared_ptr<Perception> p = sense();
    if (0 == p.get()) {
        return false;
    }
    if (!WM.update(p)) return false;
    // scened the init message
    shared_ptr<Action> iAct(new InitAction(OPTS.arg<string > ("teamname"),
                                           OPTS.arg<unsigned int>("unum")));
    perform(iAct);
    //Allen, for GUI under new server
    sense();
    shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(FM.getMy().beforeKickOffBeam));
    perform(bAct);
    times =0;
    simtime =100.0f;
    mIsFull =0;
    mIsLeft = false;
    return true;
}

/**
 * the entry of "Think" thread
 * 1-GK shout
 * 2-different play mode
 * 3-CameraMotion
 *
 * @author Xu Yuan
 *
 * @return boost::shared_ptr<Action>
 */
boost::shared_ptr<Action> Player::think() {
    boost::shared_ptr<Actions> actions(new Actions());
    if (SHM.IsCanSay()) {
        shared_ptr<Say> SHMsay(new Say(SHM.getSayString()));
        actions->add(SHMsay);
    }
    //TT add for controling CameraMotion
    mCameraMotionMode = -1; //mCameraMotionMode will be changed in "play mode"
    //play mode
    switch (WM.getPlayMode()) {
    case PM_BEFORE_KICK_OFF:
        actions->add(playBeforeKickOff());
        break;
    case PM_KICK_OFF_LEFT:
    case PM_KICK_OFF_RIGHT:
        actions->add(playKickOff());
        break;
    case PM_PLAY_ON:
        actions->add(playPlayOn());
        break;
    case PM_KICK_IN_LEFT:
    case PM_KICK_IN_RIGHT:
        actions->add(playKickIn());
        break;
    case PM_CORNER_KICK_LEFT:
    case PM_CORNER_KICK_RIGHT:
        actions->add(playCornerKick());
        break;
    case PM_GOAL_KICK_LEFT:
    case PM_GOAL_KICK_RIGHT:
        actions->add(playGoalKick());
        break;
    case PM_OFFSIDE_LEFT:
    case PM_OFFSIDE_RIGHT:
        actions->add(playOffSide());
        break;
    case PM_GAME_OVER:
        actions->add(playGameOver());
        break;
    case PM_GOAL_LEFT:
    case PM_GOAL_RIGHT:
        actions->add(playGoal());
        break;
    case PM_FREE_KICK_LEFT:
    case PM_FREE_KICK_RIGHT:
        actions->add(playFreeKick());
        break;
    default:
        cerr << "[WARNING] Player can not handle this Play Mode!\n";
        actions->add(playPlayOn());
        break;
    }

    //camera motion
    shared_ptr<JointAction> jact(new JointAction(false));
    if (NULL == WM.lastPerception().vision().get()) {
        jact->setForCamera(0, WM.getSearchSpeed().x());
        jact->setForCamera(1, WM.getSearchSpeed().y());
        actions->add(jact);
    } else {
        shared_ptr<CameraMotion> cm(new CameraMotion(mCameraMotionMode));
        actions->add(cm->perform());
    }
    return actions;
}

shared_ptr<Action> Player::playKickOff() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_KICK_OFF_LEFT == pm)
            || (TI_RIGHT == ti && PM_KICK_OFF_RIGHT == pm)) {
        return playOurKickOff();
    } else {
        return playOppKickOff();
    }
}

shared_ptr<Action> Player::playKickIn() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_KICK_IN_LEFT == pm)
            || (TI_RIGHT == ti && PM_KICK_IN_RIGHT == pm)) {
        return playOurKickIn();
    } else {
        return playOppKickIn();
    }
}

shared_ptr<Action> Player::playCornerKick() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_CORNER_KICK_LEFT == pm)
            || (TI_RIGHT == ti && PM_CORNER_KICK_RIGHT == pm)) {
        return playOurCornerKick();
    } else {
        return playOppCornerKick();
    }
}

shared_ptr<Action> Player::playGoalKick() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_GOAL_KICK_LEFT == pm)
            || (TI_RIGHT == ti && PM_GOAL_KICK_RIGHT == pm)) {
        return playOurGoalKick();
    } else {
        return playOppGoalKick();
    }
}

shared_ptr<Action> Player::playOffSide() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_OFFSIDE_LEFT == pm)
            || (TI_RIGHT == ti && PM_OFFSIDE_RIGHT == pm)) {
        return playOurOffSide();
    } else {
        return playOppOffSide();
    }
}

shared_ptr<Action> Player::playGoal() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_GOAL_LEFT == pm)
            || (TI_RIGHT == ti && PM_GOAL_RIGHT == pm)) {
        return playOurGoal();
    } else {
        return playOppGoal();
    }
}

shared_ptr<Action> Player::playFreeKick() {
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ((TI_LEFT == ti && PM_FREE_KICK_LEFT == pm)
            || (TI_RIGHT == ti && PM_FREE_KICK_RIGHT == pm)) {
        return playOurFreeKick();
    } else {
        return playOppFreeKick();
    }
}


shared_ptr<Action> Player::goTo(const Vector2f& destPos, AngDeg bodyDir, bool avoidBall, bool goStop) {
    Vector2f relTarget;
    //Vector2f tmpDestPos=Vector2f(-destPos.y(),destPos.x());//test by dpf
    float turnAng;
    //static float AddStep =0.5f;
    float ReduceStep =1;
    //float ReduceStep=1.0 + 0.5*sin(AddStep*1.5708 + 4.7124);
    //dpf change , comment: dpf want to use new transGlobalPosToRelPos() but it seems bad.
    //dpf once want to change myPos to torso's global pos, but failed, because we use myPos before
    // to test the FAT, now we must use it too, though it is good to undstand to use torso's global pos
    Vector2f myPos = WM.getMyGlobalPos2D();
    Vector2f destRelPos = WM.transGlobalPosToRelPos(destPos);

    float DisToTurn;

    turnAng = -atan2Deg(destRelPos.x(), destRelPos.y());

    //cout << endl;
    //cout<<WM.getMyGlobalPos().z()<<endl;
    //cout<<AddStep<<endl;
//-----------------------------------------------------------------------add by qxt
    /*if(destRelPos.length()>1.5)
    {
        AddStep +=0.04;
        if(AddStep >=1.0f )AddStep =1.0f;

        bool shouldIStop = false;
        shared_ptr<Task> curTask = mTask.getFirstSubTask();
        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);

        if (NULL == curWalkRel.get()) {
          //shouldIStop = curWalkRel->getShouldStop();
          //std::cout<<"aaaaaaaaaaaaaaaa"<<endl;
          shouldIStop = true;

        }
        //std::cout<<shouldIStop<<std::endl;
        if(shouldIStop|| (ReduceStep <0.55 && AddStep >=0.9)||WM.isFall()){
    	AddStep =0;
        }
    }
    else{*/

    /*
        if(destRelPos.length() < 1.0f && destRelPos.length() > 0.2f) {
    	ReduceStep =sqrt(1 -( destRelPos.length() -1 ) * ( destRelPos.length() -1 ) );//add by ghd

    	if (ReduceStep <= 0.5f) {
    	    ReduceStep =0.5f;
    	}
        } else {
    	ReduceStep =1.0f;//model : cycle
        }
    */
// 	AngDeg Dir =destRelPos.angle();
// 	if(destRelPos.length() > 2.0f && fabs(Dir - 90.0f)>60.0f )//turn big dir --ghd
// 	{
// 	    if(Dir > -90.0f && Dir < 90.0f)
// 	      return goToRel(Vector2f(0,0), -90.0f);
// 	    if(Dir > 90.0f && Dir < 180.0f || Dir > -180.0f && Dir < 90.0f )
// 	      return goToRel(Vector2f(0,0), 90.0f);
// 	}
    //cout<< fabs(turnAng)<<endl;
    if( WM.getOurFastestToBallNum() !=WM.getMyUnum() && destRelPos.length() <0.5f )
    {
        destRelPos =Vector2f(0,0);
        return goToRel(destRelPos,bodyDir - WM.getMyBodyDirection());
    }

    AngDeg myDir =WM.getMyBodyDirection();
    AngDeg OffAng =fabs(myDir - bodyDir);
    DisToTurn = 30.0f /OffAng;

    //cout<<fabs(turnAng)<<endl;

    if(destRelPos.length() >4.5f||OffAng >45.0f ||(fabs(turnAng) >30.0f && fabs(turnAng) <110.0f))
    {
        //cout<< turnAng<<'\t'<<OffAng<<endl;
        DisToTurn = min<float>(DisToTurn,0.4f);
    }

    //cout<<DisToTurn<<endl;

    if (destRelPos.length() > DisToTurn) //far enough, so don't care about body direction, just turn to destination
    {

        if (fabs(turnAng) > 15.0f) {
            //if(destRelPos.length()>0.6){
            relTarget = destRelPos / 4.0f / destRelPos.length();

            //}else{
            //               relTarget = Vector2f(0, 0);
            //}
            //std::cout<<relTarget<<std::endl;
        } else {

            relTarget = destRelPos;
            //cout<<relTarget<<endl;
        }


        /*a new walk line design by Liu Xiangxiao*/
        /*	    if(fabs(turnAng) > 60.0f) {

        		if(destRelPos.length() < 2.5) {
        		    relTarget = destRelPos / 4.0f / destRelPos.length();
        		} else {cpp
        		    Vector2f nowTarget = Vector2f(0, destRelPos.length());
        		    relTarget = (destRelPos + nowTarget * 7) / 16.0f / destRelPos.length();
        		}

        	    } else if (fabs(turnAng) > 30.0f) {

        		if(destRelPos.length() < 1.8) {
        		    relTarget = destRelPos / 4.0f / destRelPos.length();
        		} else {
        		    Vector2f nowTarget = Vector2f(0, destRelPos.length());
        		    relTarget = (destRelPos + nowTarget * 3) / 8.0f / destRelPos.length();
        		}

        	    } else if (fabs(turnAng) > 15.0f) {

        		if(destRelPos.length() < 1) {
        		    relTarget = destRelPos / 4.0f / destRelPos.length();
        		} else {
        		    Vector2f nowTarget = Vector2f(0, destRelPos.length());
        		    relTarget = (destRelPos + nowTarget) / 4.0f / destRelPos.length();
        		}

        	    } else {

        		if(destRelPos.length() < 1) {
        		  relTarget = destRelPos;
        		} else {
        		  Vector2f nowTarget = Vector2f(0, destRelPos.length());
        		  relTarget = (destRelPos + nowTarget) / 4.0f;
        		}

        	    }
        */
    } else {

// 	  cout<<destRelPos.angle()<<endl;
        if(destRelPos.length() >0.4f)
        {
            relTarget.x() =cosDeg(destRelPos.angle()) * 0.4f;
            relTarget.y() = sinDeg(destRelPos.angle()) * 0.4f;
        }
        else
        {
            relTarget = destRelPos;
        }
        turnAng = normalizeAngle(bodyDir - WM.getMyBodyDirection()); ///////////////////////////////
    }

    //cout<<relTarget<<endl;
    return goToAvoidBlocks(relTarget, turnAng, avoidBall, goStop);

}

shared_ptr<Action> Player::goToRel(const Vector2f& target, AngDeg dir ,bool goStop) {
    shared_ptr<Action> act = mBalance.perform();
    if (NULL != act.get()) {
        mIsFull =1;
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    shared_ptr<Task> walkTask(new WalkRel(target, dir,goStop)); //modify by ghd
    shared_ptr<Task> curTask = mTask.getFirstSubTask();

    //there's no task, append the walk directly
    if (NULL == curTask.get()) {

        mTask.append(walkTask);
        return mTask.perform();
    }

    shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
    if (NULL == curWalkRel.get())  //I am not walking, append the walk
    {
        if (mTask.getSubTaskListSize() <= 1)
            mTask.append(walkTask);
        return mTask.perform();
    } else      //I am walking now, revise the current walk
    {
        curWalkRel->revise(walkTask);
        return mTask.perform();
    }
}

shared_ptr<Action> Player::goTo(const Vector2f& stopPos, const Vector2f& lookAt, bool avoidBall) {
    Vector2f v = lookAt - stopPos;
    return goTo(stopPos, v.angle(), avoidBall);
}

shared_ptr<Action> Player::kickBetween(const Vector2f& goalLeft, const Vector2f& goalRight) {
    const Vector2f& posBall = WM.getBallGlobalPos2D();
    Vector2f vBL = goalLeft - posBall;
    Vector2f vBR = goalRight - posBall;
    const AngDeg angLeft = vBL.angle();
    const AngDeg angRight = vBR.angle();
    const AngDeg angBisector = calBisectorTwoAngles(angRight, angLeft);
    Vector2f goal = posBall + pol2xyz(Vector2f(10.0f, angBisector));

    return kickTo(goal);
}

shared_ptr<Action> Player::walkToBall() {
    return goTo(WM.getBallGlobalPos2D(), (WM.getBallGlobalPos2D() - WM.getMyOrigin2D()).angle());
}

/**
 * just beam to my initial position
 * and turn all joints to 'init' angle
 */
shared_ptr<Action> Player::beamAndInit(const Vector3f& beamPos) {
    mBalance.perform();
    mBalance.reset();
    mTask.clear();
    Vector3f initPos = Vector3f(beamPos.x(), beamPos.y(), 0.35f);

    //dpf test, bodyreset when squat
    WM.bodyReset(Vector3f(-9.665f, 0.0f, -90.0f + beamPos.z()));
    ///test by dpf, reset acc
    WM.accGlobalPosRest(initPos);
    WM.accVecGlobalReset();

    shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(beamPos));
    shared_ptr<Action> jAct = FAT.controlPreferThan("squat", "*");

    Vector2f posMe, beamPos2D;
    posMe = WM.getMyGlobalPos();
    beamPos2D = beamPos;

    shared_ptr<Actions> acts = shared_ptr<Actions > (new Actions);
    if ((beamPos2D - posMe).length() < 0.5) {
        // if we have beam to the given position, stop beaming
        jAct = FAT.controlPreferThan("squat", "*");
        acts->add(jAct);
    } else {
        acts->add(bAct);
        acts->add(jAct);
    }

    return acts;
}

//creat by ghd 
boost::shared_ptr< Action > Player::dynamicKick(const Vector2f& goal, bool useMaxForceMotion)
{
    
    shared_ptr<Action> act;
    //is kicking
    shared_ptr<Task> curTask = mTask.getFirstSubTask();
    shared_ptr<DynamicKick> curKick = shared_dynamic_cast<DynamicKick > (curTask);
    if (NULL != curKick.get()) {
        act = mTask.perform();
        if (NULL != act.get()) {
            mCameraMotionMode = 0;
            return act;
        } else {
            mKickLock = false;
        }
    }

    //keep balance
    act = mBalance.perform();
    if (NULL != act.get()) {
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    //choose kick type and calculate the target of walking
    Vector2f myDesiredPos;
    AngDeg myDesiredBodyDir;
    Vector2f relPosToStopWalk;
    shared_ptr<DynamicKick> kick;
    int motionNum = chooseKickType1(goal, 2, &myDesiredPos, &myDesiredBodyDir, &relPosToStopWalk,0);
  
    //calc in global , more flexible
    Vector2f ballPos =WM.getBallGlobalPos2D();
    Vector2f ballRelPos =WM.getBallRelPos2D();
    Vector2f myPos =WM.getMyGlobalPos2D();
    AngDeg ballAng =(ballPos - goal).angle();
  
//     Vector2f revise1(cosDeg<AngDeg>(ballAng) * 0.20f,sinDeg<AngDeg>(ballAng) * 0.20f);
//     float dist;
//     shared_ptr<DynamicKick> kick;
//     Vector2f myDesiredPos;
//     myDesiredPos =ballPos + revise1;


    /*std::cout<<"the distance to ball:"<<(WM.getBallRelPos2D() + relPosToStopWalk).length()<<std::endl;
    std::cout<<"the simtime:"<<simtime<<std::endl;
    if((WM.getBallRelPos2D() + relPosToStopWalk).length() < 0.016f){
        std::cout<<"times:"<<times<<std::endl;
        times ++;
        if(times==1)simtime =WM.getGameTime();
    }
    if(times==5)
    {
        simtime =WM.getGameTime()-simtime;
        times =0;
    }*/

// 	cout<<WM.getBallGlobalVel().length()<<endl;

// 	cout<<(WM.getBallRelPos2D() + relPosToStopWalk).length()<<'\t'<<fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection()))<<endl;
//     cout<<mKickCounter<<endl;
//     cout<<(WM.getBallRelPos2D()).length()<<endl;
   
   if (fabs(ballRelPos.x() + relPosToStopWalk.x()) < 0.03f &&(ballRelPos + relPosToStopWalk).length() <0.06f/*fabs(ballRelPos.x() + relPosToStopWalk.x()) <0.04f && fabs(ballRelPos.y() + relPosToStopWalk.y()) < 0.05f*///* && WM.getBallPol2D().y() <0.2f *///kick //0.01f 0.02f
   && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 60.0f && WM.getBallGlobalVel().length() <1.0f)
    {
	cout<<fabs(ballRelPos.x() + relPosToStopWalk.x())<<'\t'<<fabs(ballRelPos.y() + relPosToStopWalk.y())<<endl;
        int errAng=normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection());
        kick=shared_ptr<DynamicKick > (new DynamicKick(goal,0.36f));
        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
        if (NULL != curWalkRel.get()) {
            curWalkRel->stopWalk();
            act = mTask.perform();
            if (NULL != act.get()) {
                return act;
            }
        }
        //	printf("===========%s============\n","new DynamicKick in kickTo");
        //simtime =100.0f;
//         cout<<" ++ "<<endl;
	if(mKickCounter >30)
	{
	    mKickCounter =0;
	    mTask.clear();
	    mTask.append(kick);
	    return mTask.perform();
	}
	else{
	    mKickCounter ++;
	    return FAT.controlPreferThan("squat", "*");
	}
	
    } else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.3f //before modify --0.07f //0.03f
               && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 60.0f) //return goToRel
    {
        return goToAvoidBlocks(WM.getBallRelPos2D() + relPosToStopWalk, (goal -ballPos).angle() - WM.getMyBodyDirection(), true , true);
    } else //return goTo
    {
        return goTo(myDesiredPos, (goal -ballPos).angle(), true,true); /////////////////////////////////
    }
      
//       if (WM.getBallPol2D().x() < 0.3f/* && WM.getBallPol2D().y() <0.2f *///kick //0.01f 0.02f
//     /*&& fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 60.0f */&& WM.getBallGlobalVel().length() <1.0f)
//       {
// // 	  int errAng=normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection());
// 	  dist = 0.10f + fabs(cosDeg<AngDeg>(WM.getBallRelPos2D().angle()) * 0.06f);
// 	  cout<<dist<<endl;
// // 	  cout<<WM.getBallPol2D().x()<<endl;
// 	  Vector2f revise2(cosDeg<AngDeg>(ballAng) * dist,sinDeg<AngDeg>(ballAng) * dist);
// // 	  cout<<myDesiredPos<<'\t';
// 	  myDesiredPos =ballPos +revise2;
// //  	  cout<<myDesiredPos<<endl;
// // 	  cout<<"++"<<WM.getBallPol2D().x()<<endl;
// 	  if ((myPos - myDesiredPos).length() < 0.06f)
// 	  {
// // 	  
// 	    kick=shared_ptr<DynamicKick > (new DynamicKick(goal,0.36f));
// 	    shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
// 	    if (NULL != curWalkRel.get()) {
// 		curWalkRel->stopWalk();
// 		act = mTask.perform();
// 		if (NULL != act.get()) {
// 		    return act;
// 		}
// 	    }
// 	    //	printf("===========%s============\n","new DynamicKick in kickTo");
// 	    //simtime =100.0f;
//     //         cout<<" ++ "<<endl;
// 	    if(mKickCounter >30)
// 	    {
// 		mKickCounter =0;
// 		mTask.clear();
// 		mTask.append(kick);
// 		return mTask.perform();
// 	    }
// 	    else{
// 		mKickCounter ++;
// 		return FAT.controlPreferThan("squat", "*");
// 	    }
//  	  }
// 	  else 
// 	    return goTo(myDesiredPos, (ballPos - myPos).angle(), true,true); /////////////////////////////////
//       } /*else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.3f) //return goToRel
//       {
// 	  return goToAvoidBlocks(WM.getBallRelPos2D() + relPosToStopWalk, 0, true , true);
//       }*/ else //return goTo
//       {
// 	  return goTo(myDesiredPos, (ballPos - myPos).angle(), true,true); /////////////////////////////////
//       }
}


shared_ptr<Action> Player::kickTo(const Vector2f& goal, bool useMaxForceMotion) {
    shared_ptr<Action> act;
    //is kicking
    shared_ptr<Task> curTask = mTask.getFirstSubTask();
    shared_ptr<KickTask> curKick = shared_dynamic_cast<KickTask > (curTask);
    if (NULL != curKick.get()) {
        act = mTask.perform();
        if (NULL != act.get()) {
            mCameraMotionMode = 0;
            return act;
        } else {
            mKickLock = false;
        }
    }

    //keep balance
    act = mBalance.perform();
    if (NULL != act.get()) {
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    //choose kick type and calculate the target of walking
    Vector2f myDesiredPos;
    Vector2f ballPos =WM.getBallGlobalPos2D();
    AngDeg myDesiredBodyDir;
    Vector2f relPosToStopWalk;
    shared_ptr<KickTask> kick;
    int motionNum = chooseKickType1(goal, useMaxForceMotion, &myDesiredPos, &myDesiredBodyDir, &relPosToStopWalk,0);
    /*std::cout<<"the distance to ball:"<<(WM.getBallRelPos2D() + relPosToStopWalk).length()<<std::endl;
    std::cout<<"the simtime:"<<simtime<<std::endl;
    if((WM.getBallRelPos2D() + relPosToStopWalk).length() < 0.016f){
        std::cout<<"times:"<<times<<std::endl;
        times ++;
        if(times==1)simtime =WM.getGameTime();
    }
    if(times==5)
    {
        simtime =WM.getGameTime()-simtime;
        times =0;
    }*/

// 	cout<<WM.getBallGlobalVel().length()<<endl;

// 	cout<<(WM.getBallRelPos2D() + relPosToStopWalk).length()<<'\t'<<fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection()))<<endl;
   /// if ((WM.getBallRelPos2D() + relPosToStopWalk).length() < 0.016f  //kick //0.01f 0.02f
   if ((WM.getBallRelPos2D() + relPosToStopWalk).length() < 0.016f
         ///   && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 5.0f && WM.getBallGlobalVel().length() <1.0f)
	  && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 4.0f && WM.getBallGlobalVel().length() <1.0f)
    {
      
        int errAng=normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection());
        kick=shared_ptr<KickTask > (new KickTask(mKickMotionVector[motionNum].firstTaskName,NULL,errAng));
        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
        if (NULL != curWalkRel.get()) {
            curWalkRel->stopWalk();
            act = mTask.perform();
            if (NULL != act.get()) {
                return act;
            }
        }
        //	printf("===========%s============\n","new KickTask in kickTo");
        //simtime =100.0f;
        mTask.clear();
        mTask.append(kick);
        return mTask.perform();
 ///   } else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.07f //before modify --0.07f //0.03f
	 } else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.4f 
               && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 8.0f) //return goToRel
    {
        return goToAvoidBlocks(WM.getBallRelPos2D() + relPosToStopWalk,  myDesiredBodyDir - WM.getMyBodyDirection(), true,true);
    } else //return goTo
    {
        return goTo(myDesiredPos, myDesiredBodyDir, true,true); /////////////////////////////////
    }
}

boost::shared_ptr< Action > Player::DefendKick()
{

    shared_ptr<Action> act;
    //is kicking
    shared_ptr<Task> curTask = mTask.getFirstSubTask();
    shared_ptr<KickTask> curKick = shared_dynamic_cast<KickTask > (curTask);
    if (NULL != curKick.get()) {
        act = mTask.perform();
        if (NULL != act.get()) {
            mCameraMotionMode = 0;
            return act;
        } else {
            mKickLock = false;
        }
    }

    //keep balance
    act = mBalance.perform();
    if (NULL != act.get()) {
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    //calculate pos and dir to go
    int motionNum = 8;
    const Vector2f& ballPos = WM.getBallGlobalPos2D();

    Vector2f myDesiredRelPosToBall = mKickMotionVector[motionNum].myDesiredRelPosToBall;
    myDesiredRelPosToBall.y() -= 0.0037;

    Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, myDesiredRelPosToBall);
    AngDeg myDesiredBodyDir = (ballPos - WM.getMyGlobalPos2D()).angle();
    Vector2f relPosToStopWalk = mKickMotionVector[motionNum].relPosToBallToStopWalk;

    float x = mKickMotionVector[motionNum].relPosToBallToStopWalk.x();
    float y = mKickMotionVector[motionNum].relPosToBallToStopWalk.y();
    float canKickCircle = sqrt( x*x + y*y );

    shared_ptr< KickTask > kick = shared_ptr< KickTask >( new KickTask(mKickMotionVector[motionNum].firstTaskName) );

  ///  cout<<"!!!!!"<<endl;

    if( fabs( WM.getBallRelPos2D().length() - canKickCircle ) < 0.010f
            && WM.getBallRelPos2D().angle() > 50 && WM.getBallRelPos2D().angle() < 90 ) {

        //std::cout<<"Kick!!!!!!!"<<std::endl;

        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);

        if (NULL != curWalkRel.get()) {
            curWalkRel->stopWalk();
            act = mTask.perform();
            if (NULL != act.get()) {
                return act;
            }
        }

        mTask.clear();
        mTask.append(kick);

        return mTask.perform();
    } else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.10f ) {

        if ( fabs( normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection()) ) < 20.0f ) {

            Vector2f target = WM.getBallRelPos2D() + relPosToStopWalk;

            return goToAvoidBlocks(target, 0, true);
        } else {

            AngDeg turnAngle = myDesiredBodyDir - WM.getMyBodyDirection();
            return goToAvoidBlocks(Vector2f(0, 0), turnAngle, true);
        }

    } else {

        if( (myDesiredPos - WM.getMyGlobalPos2D()).length() > 0.10f ) {

            Vector2f tempTarget = WM.transGlobalPosToRelPos(myDesiredPos) * 0.90f;
            tempTarget = WM.transRelPosToGlobalPos(tempTarget);

            return goTo(tempTarget, myDesiredBodyDir, true);

        } else {

            return goTo(myDesiredPos, myDesiredBodyDir, true);
        }
    }

    return dribble();
}

boost::shared_ptr<task::KickTask> Player::chooseKickType(const seumath::Vector2f& goal, bool useMaxForceMotion, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk) {
    //compare dist and choose a motion
    int i = 0, motionNum = 0; //select motionNum
    float targetDistToGoal = 100.0f;
    float tempFloat;
    Vector2f kickTarget;

    if (useMaxForceMotion) {
        if(WM.getMyGlobalPos2D().y()>0)
            motionNum = 0;
        else
            motionNum = 1;
    } else {

        FOR_EACH(iter, mKickMotionVector) {
            kickTarget = WM.transRelPosToGlobalPos(iter->kickTargetRel);
            //kickTarget = WM.transRelPosToGlobalPos(WM.getMyGlobalPos2D(), iter->kickTargetRel);
            //dpf test it
            //kickTarget=*(Vector2f*)(WM.transRelPosToGlobalPos(Vector3f(iter->kickTargetRel.x(),iter->kickTargetRel.y(),0.0f)).get());
            tempFloat = (kickTarget - goal).length();
            if (tempFloat < targetDistToGoal) {
                targetDistToGoal = tempFloat;
                motionNum = i;
            }
            i++;
        }
    }

    //////////////////////////////////////////////
    //motionNum=7;//////////////////////////////////////test

    //calculate pos and dir to go
    const Vector2f& ballPos = WM.getBallGlobalPos2D();

    //for test
    /*	Vector2f myDesiredRelPosToBall = mKickMotionVector[motionNum].myDesiredRelPosToBall;
    	float length = myDesiredRelPosToBall.length();
     	AngDeg relDirToBall = atan2(mKickMotionVector[motionNum].kickTargetRel.x(), mKickMotionVector[motionNum].kickTargetRel.y());
    	relDirToBall += (goal - ballPos).angle();
    	std::cout<<"relDirToBall"<<relDirToBall<<endl;
    	myDesiredRelPosToBall.x() = 0 - length * cos(relDirToBall);
    	myDesiredRelPosToBall.y() = 0 - length * sin(relDirToBall);
    	std::cout<<"x:"<<myDesiredRelPosToBall.x();
    	std::cout<<"       y:"<<myDesiredRelPosToBall.y()<<std::endl;
    */

    Vector2f myDesiredRelPosToBall = mKickMotionVector[motionNum].myDesiredRelPosToBall;
    myDesiredRelPosToBall.y() += -0.0037; //0.155*sinDeg(-10);//a patch, old my glbal pos is eye pos, but now is torso.0.155 is length from eye to torso, -10 is bodyAng

    Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, myDesiredRelPosToBall);
    //Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, mKickMotionVector[motionNum].myDesiredRelPosToBall);
    //rewrited by dpf, just for test

    AngDeg myDesiredBodyDir = (goal - ballPos).angle();// + atan2Deg(mKickMotionVector[motionNum].kickTargetRel.x(),
    // mKickMotionVector[motionNum].kickTargetRel.y());
    if(WM.getPlayMode() != WM.getOurTeamIndex() + 1) {
        myDesiredBodyDir += atan2Deg(mKickMotionVector[motionNum].kickTargetRel.x(),
                                     mKickMotionVector[motionNum].kickTargetRel.y());
    }

    //add by liuyao
    /*choose awgood when we kick off ,keep go straight*/
// 	if(WM.getPlayMode() == WM.getOurTeamIndex() + 1) {
// 	    myDesiredBodyDir += 14;
// 	}

    *pPos = myDesiredPos;
    *pDir = myDesiredBodyDir;
    *pRelPosToStopWalk = mKickMotionVector[motionNum].relPosToBallToStopWalk;

    return shared_ptr<KickTask > (new KickTask(mKickMotionVector[motionNum].firstTaskName));
}

//add by FUJI_LIU	choose kicktype without kickact
int Player::chooseKickType1(const seumath::Vector2f& goal, int kickType, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk,int errAng)
{
    //compare dist and choose a motion
    int i = 0, motionNum = 0;  //select motionNum
    float targetDistToGoal = 100.0f;
    float tempFloat;
    Vector2f kickTarget;
    Vector2f myPos = WM.getMyGlobalPos2D();
    Vector2f ballPos =WM.getBallGlobalPos2D();
    AngDeg ballToGoalAng =( goal - ballPos).angle();
    AngDeg ghdAng = ballToGoalAng;

//     cout<<ghdAng<<endl;

    if (kickType == 1) {
        if( myPos.x() > 2.0f && myPos.y()>-2.0f ) {
            motionNum = 0;
        } else {
            motionNum = 1;
        }
    } else if(kickType == 2){

	if(ghdAng >0.0f)
	   motionNum =2;
	else 
	  motionNum =3;
//         if(myPos.y() <0.0f) {
//             motionNum = 3;
//         } else {
//             motionNum = 2;
//         }
    } else
      motionNum =0;

//     cout<<motionNum<<endl;
    //////////////////////////////////////////////
    //motionNum=7;//////////////////////////////////////test

    //calculate pos and dir to go
    if( WM.getPlayMode() == PM_KICK_OFF_LEFT || WM.getPlayMode() == PM_KICK_OFF_RIGHT ) {
        ballPos = Vector2f(0.0f, 0.0f);
    }

    //for test
    /*	Vector2f myDesiredRelPosToBall = mKickMotionVector[motionNum].myDesiredRelPosToBall;
    	float length = myDesiredRelPosToBall.length();
     	AngDeg relDirToBall = atan2(mKickMotionVector[motionNum].kickTargetRel.x(), mKickMotionVector[motionNum].kickTargetRel.y());
    	relDirToBall += (goal - ballPos).angle();
    	std::cout<<"relDirToBall"<<relDirToBall<<endl;
    	myDesiredRelPosToBall.x() = 0 - length * cos(relDirToBall);
    	myDesiredRelPosToBall.y() = 0 - length * sin(relDirToBall);
    	std::cout<<"x:"<<myDesiredRelPosToBall.x();
    	std::cout<<"       y:"<<myDesiredRelPosToBall.y()<<std::endl;
    */

    Vector2f myDesiredRelPosToBall = mKickMotionVector[motionNum].myDesiredRelPosToBall;
    myDesiredRelPosToBall.y() += -0.0037; //0.155*sinDeg(-10);//a patch, old my glbal pos is eye pos, but now is torso.0.155 is length from eye to torso, -10 is bodyAng

    TTeamIndex ti = WM.getOurTeamIndex();
    if ( (ti + 1 ==  WM.getPlayMode()) && ti < 2 ) {
        cout<<"!!!"<<endl;
        myDesiredRelPosToBall.y() += -0.02f;
//	   motionNum = 8;
    }

    Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, myDesiredRelPosToBall);
    //Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, mKickMotionVector[motionNum].myDesiredRelPosToBall);
    //rewrited by dpf, just for test

    AngDeg myDesiredBodyDir = (goal - ballPos).angle() + atan2Deg(mKickMotionVector[motionNum].kickTargetRel.x(),
                              mKickMotionVector[motionNum].kickTargetRel.y());

    *pPos = myDesiredPos;
    *pDir = myDesiredBodyDir;
    *pRelPosToStopWalk = mKickMotionVector[motionNum].relPosToBallToStopWalk;

    return motionNum;//shared_ptr<KickTask > (new KickTask(mKickMotionVector[motionNum].firstTaskName));
}

boost::shared_ptr<action::Action> Player::dribbleToDir(AngDeg angC, AngDeg angL, AngDeg angR) {

    Vector2f ballPos = WM.getBallGlobalPos2D();
    AngDeg ballAng =30.0f;
    
    if((ballPos -Vector2f(half_field_length ,0)).length() <3.0f)
      ballAng =20.0f;
    
//     Vector2f myPos = WM.getMyGlobalPos2D();
//     Vector2f ghdPos;
//     Vector2f adjust;			//ghd
//     AngDeg tempAng;
//     adjust.x() = 0.3f * cosDeg<AngDeg>((ballPos-myPos).angle());
//     adjust.y() = 0.3f * sinDeg<AngDeg>((ballPos-myPos).angle());
//     ghdPos =ballPos +adjust;
// 
// //     cout<<WM.getBallPol2D().y()<<endl;
//     
//     AngDeg ballDir =(ballPos -myPos).angle();
//     AngDeg myDirToBall = (ghdPos - myPos).angle();
//     Vector2f ballToMe = -ballPos +myPos;
//     
// //     cout<<angC<<endl;
// 
//     float turnAng = fabs(myDirToBall-angC);
//     
//     if(angL -angR >70.0f)
//     {
//         tempAng =70.0f *(angL -angC)/(angL -angR);
//         angL =angC +tempAng;
//         angR =angL -70.0f;
// // 	    cout<<angR<<'\t'<<angL<<endl;
//     }

    //cout<<"ballToMe.lenth   "<<ballToMe.length()<<endl;
//     cout<<WM.getMyBodyDirection()<<'\t'<<angR<<'\t'<<angL<<endl;
//     cout<<isAngInInterval(myDirToBall, angR, angL)<<endl;
//     cout<<fabs(WM.getBallPol2D().y())<<endl;
//     cout<<WM.getMyBodyDirection()<<'\t'<<angR<<'\t'<<angL<<endl;
//       cout<<angC<<endl;
    if (fabs(WM.getBallPol2D().y()) <ballAng
                                         && isAngInInterval(WM.getMyBodyDirection(),angR,angL))//TT note: angR<angL
    {
        return dribbleRel(angC);
    }
    else {

        
	float dist = 0.2f;//min(0.3f, WM.getBallPol2D().x()); //distance to ball
        
        float x = ballPos.x() - dist * seumath::cosDeg(angC);
        float y = ballPos.y() - dist * seumath::sinDeg(angC);
        Vector2f target= Vector2f(x, y);
        //Vector2f target= Vector2f(x-turnAng/90/2, y);

//         cout<<"goto "<<angC<<endl;
        //cout<<dist<<endl;
        //if((Vector2f(x,y)-myPos).length() <0.2f)return goToRel(Vector2f(0,0),angC - WM.getMyBodyDirection());
        return goTo(target, angC, true);
    }
}

void Player::ReviseAng(seumath::AngDeg& angC, seumath::AngDeg& angL, seumath::AngDeg& angR)   //only use in dribble --ghd
{
    AngDeg angOpp;
    float distOpp;
    AngDeg rotate;
    Vector2f oppPos;
    Vector2f ballPos =WM.getBallGlobalPos2D();
    unsigned int i;
    for(i =1; i<=WM.getOppPlayerNumber(); i++)
    {
        oppPos =WM.getOppGlobalPos2D(i);
        angOpp =(Vector2f(half_field_length,0) - oppPos).angle();
        distOpp =(ballPos -oppPos).length();
        if(isAngInInterval(angOpp,angR,angL)
                && distOpp <4.0f
                && oppPos.x() >ballPos.x())
        {
// 		cout<<"++++"<<endl;
            if(angOpp < angC)
            {
                rotate =15.0f;
                angL -=rotate;
                angR -=rotate;
                angC -=rotate;
            }
            else
            {
                rotate =15.0f;
                angL +=rotate;
                angR +=rotate;
                angC +=rotate;
            }
        }
        break;

    }
}

boost::shared_ptr<action::Action> Player::dribble() {


    //ghd  -- the best
    AngDeg myDir =WM.getMyBodyDirection();
    AngDeg goodDir =0.0f;
    Vector2f ballPos =WM.getBallGlobalPos2D();
    Vector2f myPos =WM.getMyGlobalPos2D();
    AngDeg maxDir =20.0f;
    AngDeg minDir = -maxDir;
    float EdgeY =half_field_width * 0.6f;
    float k;
    Vector2f goalTemp(half_field_length -1.0f,0);
    Vector2f goal(half_field_length,0.0f);
    Vector2f goalL(half_field_length,half_goal_width * 0.8f);
    Vector2f goalR(half_field_length,-half_goal_width * 0.8f);
    AngDeg delta;
    AngDeg angC;
    //cout<<angC<<endl;
    AngDeg angL =angC;
// 	  if(ballPos.y() > half_field_width*0.8f)
// 	    angL =min(angL,(goalLeft - myPos).angle());

    AngDeg angR =angC;
    AngDeg ballToMeDir =(ballPos -myPos).angle();
// 	  Vector2f goalLeft(half_field_length,half_field_width);
// 	  Vector2f goalRight(half_field_length,-half_field_width);
    if((ballPos -goalTemp).length() <2.0f)
    {
        
        //cout<<"+++"<<endl;
        if(ballPos.y() >0)
            angC =(goalL - ballPos).angle();
        else
            angC =(goalR - ballPos).angle();
        //cout<<"+++"<<endl;
// 	    angC =(goal -ballPos).angle();
        angL =(Vector2f(half_field_length,half_goal_width) - ballPos).angle();
        angR =(Vector2f(half_field_length,-half_goal_width) - ballPos).angle();
    }
    else if(ballPos.x() >0.7f * half_field_length)
    {
        //cout<<"!!!"<<endl;
        k = (goalTemp - ballPos).length() /1.5f;
        delta = atan2Deg<AngDeg>(1.2f,(goalTemp -myPos).length());
        angC =(goalTemp -ballPos).angle();
        delta *=k;
        angL = angC +delta;
        angR = angC -delta;
	
	if(ballPos.y() >0.0f)
	  angL =(Vector2f(half_field_length,half_goal_width) - ballPos).angle();
	else 
	  angR =(Vector2f(half_field_length,-half_goal_width) - ballPos).angle();

        //cout<<angC<<'\t'<<angL<<'\t'<<angR<<endl;
    }
    else
    {
        //EdgeY = (myDir >0) ? half_field_width*0.8f : -half_field_width*0.8f;
        //goodDir =0.0f;
// 	    cout<<mIsLeft<<endl;
        if(mIsLeft)	//(ballPos -myPos).angle() >0
        {
            if((ballPos -myPos).angle() <-20.0f && (ballPos - myPos).length() >1.0f)
            {
                goodDir = (-7.0f * (ballPos.y() +EdgeY)) ;
                mIsLeft =!mIsLeft;
            }
            else
                goodDir =(-7.0f * (ballPos.y() -EdgeY ));
        }
        else	//(ballPos -myPos).angle() <0
        {
            //cout<<(ballPos -myPos).angle()<<endl;
            if((ballPos -myPos).angle() >20.0f && (ballPos - myPos).length() >1.0f)
            {
                mIsLeft =!mIsLeft;
                goodDir =(-9.0f * (ballPos.y() -EdgeY ));
            }
            else
                goodDir = (-9.0f * (ballPos.y() +EdgeY)) ;
        }

        //cout<<goodDir<<endl;

        goodDir = clamp<AngDeg>(goodDir,-90.0f,90.0f);

        //cout<<goodDir<<endl;

        delta =clamp<AngDeg>(goodDir -myDir,minDir,maxDir);
        angC =myDir +delta;
	
	clamp<AngDeg>(angC , -60.0f,60.0f);
//         cout<<angC<<endl;
        angL =angC +(half_field_length - ballPos.x()) * 0.7f *6.0f;
        angR =angC -(half_field_length - ballPos.x()) * 0.7f *6.0f;
	
	
	
	angL = clamp<AngDeg>(angL,-90.0f,90.0f);
	angR = clamp<AngDeg>(angR,-90.0f,90.0f);
	
	
// 	cout<<angL << '\t'<<angR<<endl;
// 	if(ballPos.x() <-0.4f * half_field_length)
// 	{
// 	  if(ballPos.y() >0.0f)
// 	    angR =(Vector2f(half_field_length,-half_field_width) - ballPos).angle();
// 	  else 
// 	    angL =(Vector2f(half_field_length,half_field_width) - ballPos).angle();
// 	}else
// 	{
	  if(ballPos.y() >half_field_width *0.7f )
	    angL =(Vector2f(half_field_length,half_field_width) - ballPos).angle();
	  else if(ballPos.y() < half_field_width *-0.7f)
	    angR =(Vector2f(half_field_length,-half_field_width) - ballPos).angle();
// 	}
// 	    if(ballPos.x() < -0.7f * half_field_length)
// 	    {
// 	      Vector2f pointL(0,3);
// 	      Vector2f pointR(0,-3);
// 	      Vector2f pointC(3,0);
//
// 	      if(mIsLeft)
// 	      {
// 		angL =min<AngDeg>((pointL-myPos).angle(),(pointC-myPos).angle());
// 		angC +=( (angL -(angC +90.0f)))/2.0f;
// 	      }
// 	      else
// 	      {
// 		angR =max<AngDeg>((pointR-myPos).angle(),(pointC-myPos).angle());
// 		angC +=( (angR -(angR -90.0f)))/2.0f;
// 	      }
// 	    }
    }

    //cout<<EdgeY<<endl;


// 	  cout<<delta<<endl;

// 	  if(ballPos.y() <- half_field_width*0.8f)
// 	    angR =max(angR,(goalRight - myPos).angle());

    //cout<<"dribble"<<endl;
    //added by ghd  -- to avoid oppPlayer
    //const core::WorldModel::BlockInfo& ballBlock =WM.getBallBlock();
//  	  const std::list<core::WorldModel::BlockInfo>& tempList = WM.getBlockList();
//  	  std::list<core::WorldModel::BlockInfo>::const_iterator oppIter =tempList.begin();
//
// 	  Vector2f myPos = WM.getMyGlobalPos2D();
// 	  Vector2f ballPos = WM.getBallGlobalPos2D();
// 	  Vector2f oppGoal = Vector2f( half_field_length , 0.0f);
// 	  Vector2f oppGoalLeft =Vector2f (half_field_length, half_goal_width);
// 	  Vector2f oppGoalRight=Vector2f (half_field_length, -half_goal_width);
// 	  Vector2f dribbleVector= Vector2f (1.0f,1.0f);
// 	  Vector2f adjust;
// 	  float ballToGoalDistance=(WM.getBallGlobalPos2D()-oppGoal).length();
// 	  float ballPosX=ballPos.x();
// 	  float ballPosY=ballPos.y();
// 	  AngDeg myAng =(ballPos -myPos).angle();
// 	  AngDeg angC;
// 	  AngDeg angL;
// 	  AngDeg angR;
//
// 	  if(ballPosX >half_field_length * 0.7f)
// 	  {
// 	    float k =4.0f;
// 	    angC = (oppGoal - ballPos).angle();
// 	    angL = (oppGoalLeft - ballPos).angle();
// 	    angR = (oppGoalRight - ballPos).angle();
//
// 	    if(angL - angR <90.0f)
// 	    {
// 	      if(angC > 30.0f)
// 	      {
// 		angC += ballPosX * 2.0f;
// 		angL += ballPosX * 3.0f;
// 	      }
// 	      else if(angC < -30.0f)
// 	      {
// 		angC -= ballPosX * 2.0f;
// 		angR -=  ballPosX * 3.0f;
// 	      }
// 	    }
// 	  }
// 	  else
// 	  {
// 	    //ghd
// 	    bool isLeft;
// 	    float minCount;
// 	    unsigned int angCInt;
// 	    unsigned int i;
//
// 	    const AngDeg ang1 =-54.0f;
// 	    const AngDeg ang2 =-18.0f;
// 	    const AngDeg ang3 =18.0f;
// 	    const AngDeg ang4 =54.0f;
//
// 	    vector<float> counter(5,0.0f);
//
//   // 	  AngDeg myDir =WM.getMyBodyDirection();
//   //
// 	    while(oppIter != tempList.end())
// 	    {
// 	      if(oppIter ->isOurTeam ==false)	//is opp player --ghd
// 	      {
// 		if(oppIter ->angC <ang1)
// 		{
// 		    counter[0] += 10.0f/oppIter->dist;
// 		}
// 		else if(oppIter ->angC <ang2)
// 		{
// 		    counter[1] += 10.0f/oppIter->dist;
// 		}
// 		else if(oppIter ->angC <ang3)
// 		{
// 		    counter[2] += 10.0f/oppIter->dist;
// 		}
// 		else if(oppIter ->angC <ang4)
// 		{
// 		    counter[3] += 10.0f/oppIter->dist;
// 		}
// 		else
// 		{
// 		    counter[4] += 10.0f/oppIter->dist;
// 		}
// 	      }
// 	      oppIter ++;
// 	    }
//
// 	    isLeft = isAngInInterval( myAng,0.0f, 180.0f);
//
// 	    if(!isLeft)	//right
// 	    {
// 	      //0,1,2
// 	      minCount =counter[2];
// 	      for(i =0;i<2;i++)
// 		if(counter[i] <minCount)
// 		  minCount =counter[i];
//
// 	      for(i =2;i>=0;i--)
// 		if(counter[i] == minCount)
// 		  break;
//
// 	      angCInt =i;
// 	    }
// 	    else
// 	    {
// 	      minCount =counter[2];
// 	      for(i =3;i<5;i++)
// 		if(counter[i] <minCount)
// 		  minCount =counter[i];
//
// 	      for(i =2;i<5;i++)
// 		if(counter[i] == minCount)
// 		  break;
//
// 	      angCInt =i;
// 	    }
// 	    //cout<<angCInt<<endl;
//
// 	    switch(angCInt)
// 	    {
// 	      case 0:angC =(ang1 +-90.0f)/2.0f;
// 			  break;
// 	      case 1:angC =(ang2 +ang1)/2.0f;
// 			  break;
// 	      case 2:angC =(ang3 +ang2)/2.0f;
// 			  break;
// 	      case 3:angC =(ang4 +ang3)/2.0f;
// 			  break;
// 	      case 4:angC =(90.0f +ang4)/2.0f;
// 			  break;
// 	    }
//
// 	    //cout<<counter[0]<<'\t'<<counter[1]<<'\t'<<counter[2]<<'\t'<<counter[3]<<'\t'<<counter[4]<<endl;
// 	    //cout<<angCInt<<endl;
//
// 	    float k =(ballToGoalDistance +2.0f) * 3.0f ;
// 	    angL =angC +k;
// 	    angR =angC -k;
// 	  }

    // cout<<angL<<'\t'<<angC<<'\t'<<angR<<endl;

    //cout<<count1<<'\t'<<count2<<'\t'<<count3<<'\t'<<count4<<'\t'<<count5<<endl;
//
// 	  const Vector2f& myPos = WM.getMyGlobalPos2D();
// 	  const Vector2f& ballPos = WM.getBallGlobalPos2D();
// 	  Vector2f target(half_field_length, 0);
//
// 	  //calculate L and R
// 	  Vector2f pointL(half_field_length, half_goal_width);
// 	  Vector2f pointR(half_field_length, -half_goal_width);
//
// 	  Vector2f farPointL(half_field_length, half_field_width);
// 	  Vector2f farPointR(half_field_length, -half_field_width);
//
// 	  Vector2f MiddlePoinL(half_field_length, half_field_width/2.0f);
// 	  Vector2f MiddlePoinR(half_field_length, -half_field_width/2.0f);
//
// 	  AngDeg angC = (target - ballPos).angle();
// 	  AngDeg angL = (pointL - ballPos).angle();
// 	  AngDeg angR = (pointR - ballPos).angle();
// 	  AngDeg farL = (farPointL - ballPos).angle();
// 	  AngDeg farR = (farPointR - ballPos).angle();
//
// 	  float ballDistToTarget = (target - ballPos).length();
//
//
// 	  if(!(ballDistToTarget <5.0f || ballPos.x() >half_field_length -3.0f))
// 	  {
// 	    if(/*(ballPos -myPos).angle() */ballPos.y() >0.0f)
// 	    {
// 	      float k =(ballDistToTarget -5.0f )*4.0f;
//
// 	      oppIter =oppList.begin();
// 	      while(oppIter !=oppList.end())
// 	      {
// 		//cout<<oppIter ->angL <<endl;
// 		if(oppIter ->angL < farL && angC < oppIter ->angL )
// 		{
// 		  angC =oppIter ->angR ;
// 		  angL =angC +k;
// 		  angR =angC -k;
// 		}
// 		oppIter ++;
// 	      }
// 	    }
// 	    else
// 	    {
// 	      float k =(ballDistToTarget -5.0f )*4.0f;
//
// 	      oppIter =oppList.begin();
// 	      while(oppIter !=oppList.end())
// 	      {
// 		//cout<<oppIter ->angR <<endl;
// 		if(oppIter ->angR > farR && angC > oppIter ->angR )
// 		{
// 		  angC =oppIter ->angR ;
// 		  angL =angC +k;
// 		  angR =angC -k;
// 		}
// 		oppIter ++;
// 	      }
// 	    }
// 	  }
//
// 	  cout<<angL<<'\t'<<angC<<'\t'<<angR<<endl;
// 	  //cout<<angC<<endl;

    //added by ghd   --end

    //cout<<oppList.size()<<'\t'<<WM.getBlockList().size()<<endl;

    //cout<<WM.getBlockList()<<endl;
// 	//cout<<WM.seenFlagsNum()<<endl;
//         //if (WM.seenFlagsNum() >= 3) {
//             const Vector2f& myPos = WM.getMyGlobalPos2D();
//             const Vector2f& ballPos = WM.getBallGlobalPos2D();
//             Vector2f target(half_field_length, 0);
//
//             //calculate L and R
//             Vector2f pointL(half_field_length, half_goal_width);
//             Vector2f pointR(half_field_length, -half_goal_width);
//             AngDeg angC = (target - ballPos).angle();
//             AngDeg angL = (pointL - ballPos).angle();
//             AngDeg angR = (pointR - ballPos).angle();
//
//             float ballDistToTarget = (target - ballPos).length();
//             if (ballDistToTarget < 4.0f) //near enough to the target
//             {
// 		//if(ballDistToTarget >3.0f)ReviseAng(angC,angL,angR);                    //add by ghd,to avoid opp player
//                 return dribbleToDir(angC, angL, angR);5
//             } else {
//                 //=================enlarge the range
//                 float k = 0.96f; ///////////////////////////////////// 0.48f
//                 float deltaDist = ballDistToTarget - 4.0f;
//                 //L
//                 float deltaAngL = angL - angC;
//                 deltaAngL += deltaAngL * k * deltaDist;
//                 angL = normalizeAngle(angC + deltaAngL);
//                 //R
//                 float deltaAngR = angC - angR;
//                 deltaAngR += deltaAngR * k * deltaDist;
//                 angR = normalizeAngle(angC - deltaAngR);
//
//                 //=================restrict L and R according to the court info
//                 //...
// 		//ReviseAng(angC,angL,angR);                    //add by ghd,to avoid opp player
//                 return dribbleToDir(angC, angL, angR);
//             }
//         //} else {
//             //return dribbleToOppGoal();
//         //}

// 	  Vector2f myPos = WM.getMyGlobalPos2D();
// 	  Vector2f ballPos = WM.getBallGlobalPos2D();
// 	  Vector2f oppGoal = Vector2f( half_field_length , 0.0f);
// 	  Vector2f oppGoalLeft =Vector2f (half_field_length, half_goal_width);
// 	  Vector2f oppGoalRight=Vector2f (half_field_length, -half_goal_width);
// 	  Vector2f dribbleVector= Vector2f (1.0f,1.0f);
// 	  float ballToGoalDistance=(WM.getBallGlobalPos2D()-oppGoal).length();
// 	  float ballPosX=ballPos.x();
// 	  float ballPosY=ballPos.y();
// 	  AngDeg myAng =(ballPos -myPos).angle();
// 	  AngDeg angC = (oppGoal - ballPos).angle();
// 	  AngDeg angL = (oppGoalLeft - ballPos).angle();
// 	  AngDeg angR = (oppGoalRight - ballPos).angle();

// 	  AngDeg offset =8.0f;
//
// 	  //cout<<"pos        "<<myPos<<endl;
// 	  if(ballToGoalDistance <2.5f)			//ghd
// 	  {
// 	      angC = (oppGoal - ballPos).angle();
// 	      angL = (oppGoalLeft - ballPos).angle();
// 	      angR = (oppGoalRight - ballPos).angle();
// 	  }
// 	  else if(ballToGoalDistance<6.0f)//1
// 	    { //cout<<"111111111111"<<endl;
// 	      float k =ballToGoalDistance*4.0f;		//ghd
//
// 	      angC = (oppGoal - ballPos).angle();
// 	      angL = (oppGoalLeft - ballPos).angle();
// 	      angR = (oppGoalRight - ballPos).angle();
//
// 	      if(half_field_length - ballPosX >2.0f)	//ghd
// 	      {
// 		  angL +=k;
// 		  angR -=k;
// 	      }
// 	      else
// 	      {
// 		  if(ballPosY >0.0f)
// 		  {
// 		    angC -=offset;
// 		    angL +=offset;
// 		    angR -=k;
// 		  }
// 		  else
// 		  {
// 		    angC +=offset;
// 		    angL +=k;
// 		    angR -=offset;
// 		  }
// 	      }
//
//
// // 	      cout<<angL<<'\t'<<angR<<endl;
// // 	      cout<<angC<<endl;
// // 	      cout<<endl;
// 	    }
// 	  else  if((ballToGoalDistance<9.0f||ballPosX>9.0f)&&ballPosY<0.0f)//1
// 	    { //cout<<"2222222222"<<endl;
// 	      //angC = (oppGoal - ballPos).angle()*0.8f;
// 	      float k =ballToGoalDistance*6.0f;		//ghd
//
// 	      angC = (oppGoal - ballPos).angle();
// 	      angL = angC+k;
// 	      angR = angC-k;
// 	    }
// 	  else  if((ballToGoalDistance<9.0f||ballPosX>9.0f)&&ballPosY>0.0f)//1
// 	    {  //cout<<"3333333333"<<endl;
// 	     // angC = (oppGoal - ballPos).angle()*0.8f;
// 	      float k =ballToGoalDistance*6.0f;		//ghd
//
// 	      angC = (oppGoal - ballPos).angle();
// 	      angL = angC+k;
// 	      angR = angC-k;
// 	    }
//
// 	  else if(ballPosY>0.0f)
// 	  {
// 	    //cout<<"4444444444"<<endl;
// 	    float temp=0.0058f*ballPosY*ballPosY*ballPosY-0.0931f*ballPosY*ballPosY+0.1552*ballPosY+1.5f;
// 	    dribbleVector=Vector2f (1.0f,temp);
//
// 	    angC = dribbleVector.angle();
// 	    angL = angC +30.0f;
// 	    angR = angC -15.0f;
// 	  }
// 	  else
// 	  { //cout<<"5555555555555555"<<endl;
// 	    float temp=0.0058f*ballPosY*ballPosY*ballPosY+0.0931f*ballPosY*ballPosY+0.1552*ballPosY-1.5f;
// 	    dribbleVector=Vector2f(1.0f,temp);
// 	    angC = dribbleVector.angle();
// 	    angL = angC +15.0f;
// 	    angR = angC -30.0f;
// 	  }


    return dribbleToDir(angC,angL,angR);
}

shared_ptr<Action> Player::goToAvoidBlocks(seumath::Vector2f dest, seumath::AngDeg bodyDir,bool avoidBall, bool goStop,bool avoidPlayer ) {
    Vector2f AvoidDest;
    Vector2f DestRevise(cosDeg<AngDeg>(dest.angle())*0.2f,sinDeg<AngDeg>(dest.angle())*0.2f);
    AvoidDest =dest + DestRevise;
    float walkDir = -atan2Deg(dest.x(), dest.y());
    float revise;
    float k;
    float avoidDistance;
    const std::list<core::WorldModel::BlockInfo>& blockList = WM.getBlockList();
    std::list<core::WorldModel::BlockInfo>::const_iterator iter =blockList.begin();           //ghd
    const unsigned int outClosestToBall =WM.getOurFastestToBallNum();
    const core::WorldModel::BlockInfo& playerBlock =WM.getPlayerBlock();
    const core::WorldModel::BlockInfo& ballBlock = WM.getBallBlock();

    AngDeg AvoidAngL;
    AngDeg AvoidAngR;
    //cout<<playerIter->dist<<endl;
    //for deal with paste ball             --ghd
//     float erro = 0.01f;


//     if(fabs(WM.getBallRelPos2D().x()-0.13f)<erro &&fabs(WM.getBallRelPos2D().y()-0.13f)<erro
//             || fabs(WM.getBallRelPos2D().x()+0.14f)<erro &&fabs(WM.getBallRelPos2D().y()-0.13f)<erro
//       )
//     {
//         //cout<<"!!!"<<endl;
//         return goToRel(Vector2f(0,-1),0);
//     }

    //cout<<bodyDir<<endl;
    /*if(dest.length()<0.3f && bodyDir >45.0f)       //ghd
    {
        //cout<<"++"<<endl;
        return goToRel(Vector2f(0,0),90.0f);
    }
    else if(dest.length()<0.3f && bodyDir <-45.0f)       //ghd
    {
        //cout<<"++"<<endl;
        return goToRel(Vector2f(0,0),-90.0f);
    }*/
    //cout<<playerBlock.dist<<endl;
    //ball
    if (avoidBall && WM.canSeeBall() /*&& ballBlock.dist < nearestPlayerBlockDist*/) //can see?????????????????//avoid ball
    {
        do {
	    
            if ((ballBlock.dist) > dest.length()/**0.9f*/)
                break;
//             std::cout<<"I'm here !!!!"<<std::endl;
            /*||walkDir ballBlock.angC*/

            if (false == isAngInInterval(walkDir, ballBlock.angR, ballBlock.angL))
                break;

//             std::cout<<"I'm here !!!!"<<std::endl;
            //the ball blocks me
            walkDir = isAngInInterval(walkDir, ballBlock.angR, ballBlock.angC) ? ballBlock.angR: ballBlock.angL;
            //if(fabs(walkDir) > 45.0f )walkDir =45.0f * sign(walkDir);
            //

            dest.x() = -ballBlock.dist * sinDeg(walkDir);          //avoid ball
            dest.y() = ballBlock.dist * cosDeg(walkDir);

        } while (0);
    }

//         if(!WM.amIFastestToBallOfOurTeam())      //avoid player          --ghd
// 	{
// 	    /*if (fabs((Vector2f(half_field_length,0) - WM.getBallGlobalPos2D()).angle() -
// 	      (WM.getMyGlobalPos2D() - WM.getBallGlobalPos2D()).angle()) > 15.0f
// 	     && WM.getMyGlobalPos2D().x() > WM.getBallGlobalPos2D().x())
// 		    return goToRel(Vector2f(1,0),0);*/
// // 	    if (fabs(playerBlock.dist - dest.length()) <0.6f && dest.length() <1.0f)
// // 	    {
// // 		    //cout<<"IIIII"<<endl;
// // 		    return goToRel(Vector2f(0,0),0);
// // 	    }
// 
// 	    do {
// // 		cout<<playerBlock.dist<<'\t'<<dest.length()<<endl;
//                 if ((playerBlock.dist) > 2.5f/*dest.length()*//**0.9f*/)
//                     break;
// 
// 
//                 /*||walkDir ballBlock.angC*/
// // 		AvoidAngL =playerBlock.angL;
// // 		AvoidAngR =playerBlock.angR;
// // 		cout<<AvoidAngL<<'\t'<<AvoidAngR<<endl;
// 
// 		if(playerBlock.dist < 0.5f)
// 		{
// 		    walkDir = -playerBlock.angC;
// 		    dest.x() = -0.4f * sinDeg(walkDir)/*+revise*/;
// 		    dest.y() = 0.4f * cosDeg(walkDir);
// 		    break;
// 		}
// 		k =80.0f / playerBlock.dist;
// 
// 		AvoidAngL =playerBlock.angL + k;
// 		AvoidAngR =playerBlock.angR - k;
// 
// 		if (false == isAngInInterval(walkDir, AvoidAngR, AvoidAngL))
//                     break;
// // 		if (playerBlock.angC > 0 && playerBlock.angC < 25.0f && playerBlock.dist < 0.8f)
// // 		    return goToRel(Vector2f(1,0),0);
// // 		if (playerBlock.angC < 0 && playerBlock.angC > -25.0f && playerBlock.dist < 0.8f)
// // 		    return goToRel(Vector2f(-1,0),0);
//                 //the ball blocks me
//                 walkDir = isAngInInterval(walkDir, playerBlock.angR, playerBlock.angC) ?playerBlock.angR : playerBlock.angL;
// 		//cout<<AvoidAngL<<'\t'<<AvoidAngR<<endl;
// // 		cout<<walkDir<<endl;
// 		avoidDistance =min(dest.length(),playerBlock.dist);
// 		//revise = isAngInInterval(walkDir, ballBlock.angR, ballBlock.angC) ? -0.5f : 0.5f;
//                 dest.x() = avoidDistance * sinDeg(walkDir)/*+revise*/;
//                 dest.y() = avoidDistance * cosDeg(walkDir);
// // cout<<dest<<endl;
//             } while (0);
// 	}
// 	//'\t '<<ballBlock.dist<<std::endl;

    return goToRel(dest, bodyDir ,goStop); //bodyDir, instead of walkDir
}

shared_ptr<Action> Player::kickRel() {
    shared_ptr<Action> act;

    //is kicking
    shared_ptr<Task> curTask = mTask.getFirstSubTask();
    shared_ptr<KickTask> curKick = shared_dynamic_cast<KickTask > (curTask);

    if (NULL != curKick.get()) {
        act = mTask.perform();
        if (NULL != act.get()) {
            mCameraMotionMode = 0;
            return act;
        } else {
            mKickLock = false;
        }
    }

    //keep balance
    act = mBalance.perform();
    if (NULL != act.get()) {
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    //get closer to the ball
    Vector2f tempV2f(0, 0);
    float dir = 0.0f;
    const Vector2f& ballRelPos = WM.getBallRelPos2D();
    bool useRightFoot = (ballRelPos.x() > 0);

    if (ballRelPos.y() < 0.01f) //turn body
    {
        if (ballRelPos.x() < 0) dir = 20;
        else dir = -20;
    } else {
        if (ballRelPos.length() > 0.3f)
            dir = (-1) * atan2Deg(ballRelPos.x(), ballRelPos.y());

        if (fabs(dir) < 15.0f)
            tempV2f = ballRelPos + Vector2f((useRightFoot ? -0.07f : +0.07f), -0.19f); //-0.055f -0.19f
    }

    //decide to walk or kick
    if (tempV2f.length() > 0.013f || fabs(dir) > 8.0f) //////////////////////////////////// 0.01f
    {
        act = goToRel(tempV2f, dir);
        return act;
    } else {
        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
        if (NULL != curWalkRel.get()) {
            curWalkRel->stopWalk();
            act = mTask.perform();
            if (NULL != act.get()) {
                return act;
            }
        }
        printf("===========%s============\n", "new kick in kickRel");
        shared_ptr<KickTask> kick = useRightFoot ? shared_ptr<KickTask > (new KickTask("BTTL_t1")) :
                                    shared_ptr<KickTask > (new KickTask("BTT_t1"));
        //shared_ptr<KickTask> kick= shared_ptr<Shoot>( new Shoot(Vector2f(0,0)) );

        //printf("size=%d\n",mTask.getSubTaskListSize()); //1 or sometimes 0
        mTask.clear();
        //printf("size=%d\n",mTask.getSubTaskListSize()); //0
        mTask.append(kick);
        //printf("size=%d\n",mTask.getSubTaskListSize()); //1
        act = mTask.perform();
        //printf("size=%d\n",mTask.getSubTaskListSize()); //should be 1
        return act;
    }
}

boost::shared_ptr<action::Action> Player::sideWalk(bool isLeft) {
    if (isLeft) {
        Vector2f tar(-0.15, 0);
        return goToRel(tar, 0);
    } else {
        Vector2f tar(0.15, 0);
        return goToRel(tar, 0);
    }

}

shared_ptr<Action> Player::dribbleRel( AngDeg angC) {
    const Vector2f& ballRelPos = WM.getBallRelPos2D();
    //const Vector2f& ballGlobalPos =WM.getBallGlobalPos2D();
    Vector2f GlobalPos;
    Vector2f ballGlobalPos =WM.getBallGlobalPos2D();
    AngDeg dir;
    float ballDir =WM.getBallPol2D().y();
    float ballRelDir = WM.getBallRelPos2D().angle();

    //test by ghd
// 	if(ballRelPos.length() >0.15f)
// 	  dir =ballDir;
// 	else
// 	  dir =0;
//     if(fabs(ballGlobalPos.y()) >0.8f * half_field_width || ballGlobalPos.x() >0.8f * half_field_length)
//     {
//       dir =angC;
//       cout<<"   !!   "<<endl;
//     }else
//     {
//       dir =WM.getMyBodyDirection();
//     }//dir =angC -WM.getMyBodyDirection();

    //cout<<"!!!!!"<<endl;
    //added by ghd
    //cout<<(ballRelPos.x() >0)<<endl;

    Vector2f adjustV2f((ballRelPos.x() > 0 ?-0.07f :  0.07f), 0.08f); //before modify --0.05-- ghd
// 	float adjustLen =adjustV2f.length();
// 	AngDeg adjustAng =adjustV2f.angle();
// 	adjustAng -=ballRelDir;
// 	adjustV2f.x() =adjustLen * sinDeg<AngDeg>(adjustAng);
// 	adjustV2f.y() =adjustLen * cosDeg<AngDeg>(adjustAng);

    //cout<<ballDir<<endl;

    //cout<<adjustV2f<<endl;

    //cout<<"!!!!!!!!!!!!!!!!"<<endl;
    //for dribble FORWARD

    //cout<<angC<<endl;
    //===========

    //test by ghd
    //adjustV2f =Vector2f(0,0.10f);

    //cout<<adjustV2f<<endl;
    GlobalPos =WM.transRelPosToGlobalPos(ballRelPos + adjustV2f);
    //cout<<GlobalPos<<'\t'<<WM.getBallGlobalPos2D()<<endl;

    //ghd
// 	cout<<dir<<endl;
    //cout<<ballDir<<endl;

//     if(ballRelPos.length() <0.6f)
//         return goToRel(ballRelPos + adjustV2f,dir);
//     else
//     {
//         cout<<"!!!"<<endl;
        return goTo(GlobalPos, WM.getMyBodyDirection(),false);
//     }
}

//     shared_ptr<Action> Player::dribbleToOppGoal() {
//         mCameraMotionMode = 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//         float angBall = WM.getBallPol2D().y();
//         float angGoal1 = WM.getFlagPol2D(Vision::G1R).y();
//         float angGoal2 = WM.getFlagPol2D(Vision::G2R).y();
//
//         //=======================
//         Vector2f goalRelPos;
//         float ballDistToGoal;
//         float k = 0.96f; ////////////////////////////////////////
//         float angRange = 0;
//
//         if (WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R)) {
//             goalRelPos = Vector2f(WM.getFlagRelPos2D(Vision::G1R) + WM.getFlagRelPos2D(Vision::G2R)) / 2;
//             ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
//             angRange = k * ballDistToGoal;
//             if ((angGoal1 + angRange) > angBall && angBall > (angGoal2 - angRange))
//                 return dribbleRel();
//             else
//                 return goToBallBack();
//         } else if (WM.canSeeFlag(Vision::G1R)) {
//             goalRelPos = WM.getFlagRelPos2D(Vision::G1R);
//             ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
//             angRange = k * ballDistToGoal;
//             if ((angGoal1 + angRange) > angBall) ////////////////////////////////////////
//                 return dribbleRel();
//             else
//                 return goToBallBack();
//         } else if (WM.canSeeFlag(Vision::G2R)) {
//             goalRelPos = WM.getFlagRelPos2D(Vision::G2R);
//             ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
//             angRange = k * ballDistToGoal;
//             if (angBall > (angGoal2 - angRange)) ////////////////////////////////////////
//                 return dribbleRel();
//             else
//                 return goToBallBack();
//         } else {
//             return goToBallBack();
//         }
//     }

shared_ptr<Action> Player::goToBallBack() {
    mCameraMotionMode = 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    float xb = WM.getBallRelPos2D().x();
    float yb = WM.getBallRelPos2D().y();
    float angB = WM.getBallPol2D().y();
    float myDistToBall = WM.getBallPol2D().x();
    float xg = 0, yg = 0, angG = 0;
    float x = 0, y = 0, ang = 0;

    //========================info
    if (WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R)) {

        xg = (WM.getFlagRelPos2D(Vision::G1R).x() + WM.getFlagRelPos2D(Vision::G2R).x()) / 2;
        yg = (WM.getFlagRelPos2D(Vision::G1R).y() + WM.getFlagRelPos2D(Vision::G2R).y()) / 2;
        angG = -atan2Deg(xg, yg); //TT: this is accurate
    } else if (WM.seenFlagsNum() >= 3 && WM.canSeeBall()) {

        //don't want to use global position
        Vector2f deltaV2f = Vector2f(half_field_length, 0) - WM.getMyGlobalPos2D();
        angG = deltaV2f.angle() - WM.getMyBodyDirection();
        float myDistToGoal = deltaV2f.length();
        xg = -myDistToGoal * sinDeg(angG);
        yg = myDistToGoal * cosDeg(angG);
    } else if (WM.canSeeFlag(Vision::G1R)) {

        xg = WM.getFlagRelPos2D(Vision::G1R).x();
        yg = WM.getFlagRelPos2D(Vision::G1R).y();
        angG = WM.getFlagPol2D(Vision::G1R).y();
    } else if (WM.canSeeFlag(Vision::G2R)) {

        xg = WM.getFlagRelPos2D(Vision::G2R).x();
        yg = WM.getFlagRelPos2D(Vision::G2R).y();
        angG = WM.getFlagPol2D(Vision::G2R).y();
    } else {

        //mCameraMotionMode=3; //search flags
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f goal(half_field_length, 0);
        Vector2f goalL(half_field_length, half_goal_width);
        Vector2f goalR(half_field_length, -half_field_width);
        float dist = WM.getBallRelPos2D().length();
        float ballDistToTarget =(goal - ballPos).length();
        AngDeg desireBodyDir = (goal - ballPos).angle();
        AngDeg angL =(goalL - ballPos).angle();
        AngDeg angR =(goalR - ballPos).angle();
        AngDeg myDirToBall = (ballPos - WM.getMyGlobalPos2D()).angle();
        Vector2f target;

        /*
        if(ballDistToTarget >4.0f)
        {
        //=================enlarge the range
        	float k = 0.96f; ///////////////////////////////////// 0.48f
        	float deltaDist = ballDistToTarget - 4.0f;
        	//L
        	float deltaAngL = angL - desireBodyDir;
        	deltaAngL += deltaAngL * k * deltaDist;
        	angL = normalizeAngle(desireBodyDir + deltaAngL);
        	//R
        	float deltaAngR = desireBodyDir - angR;
        	deltaAngR += deltaAngR * k * deltaDist;
        	angR = normalizeAngle(desireBodyDir - deltaAngR);
            }
            cout<<angR<<'\t'<<angL<<endl;
            if (isAngInInterval(myDirToBall, angR, angL)) //TT note: angR<angL
            {

        	//cout<<"!!!"<<endl;
        	return dribbleRel();
            } else {
        	float dist = min(0.3f, WM.getBallPol2D().x()); //distance to ball
        	float x = ballPos.x() - dist * cosDeg(desireBodyDir);
        	float y = ballPos.y() - dist * sinDeg(desireBodyDir);
        	//if((Vector2f(x,y)-myPos).length() <0.2f)return goToRel(Vector2f(0,0),angC - WM.getMyBodyDirection());
        	return goTo(Vector2f(x, y), desireBodyDir, true);
            }*/

        /*a test by liuyao*/
        if(dist < 0.3f) {

            if( isAngInInterval(myDirToBall,angR,angL) ) {

                Vector2f ballRelPos = WM.getBallRelPos2D();
                float dir = WM.getBallPol2D().y();
                Vector2f adjustV2f((ballRelPos.x() > 0 ? -0.05f : 0.05f), 0.05f);

                return goToRel(ballRelPos + adjustV2f, dir);

            } else {

                dist = min(0.2f, dist);

                target.x() = ballPos.x() - dist * cosDeg(desireBodyDir);
                target.y() = ballPos.y() + ( ballPos.y() > 0 ? 1 : -1 ) * dist * sinDeg(desireBodyDir);

                return goToAvoidBlocks(target, desireBodyDir, true);
            }

        } else {

            target.x() =  ballPos.x() + (WM.getMyBodyDirection() > 0 ? -1 : 0 ) * 0.2f * cosDeg(desireBodyDir);
            target.y() = ballPos.y() + ( ballPos.y() > 0 ? 1 : -1 ) * 0.2f * sinDeg(desireBodyDir);

            return goTo(target, desireBodyDir, true);
        }
    }

    //======================================
    //get closer to the ball
    if (myDistToBall > 1.5f) {
        float D = min(0.7f, myDistToBall); //desired distance to ball
        float d = sqrt((xb - xg)*(xb - xg) + (yb - yg)*(yb - yg));
        if (d < EPSILON) d = EPSILON;

        x = (xb - xg) * D / d + xb;
        y = (yb - yg) * D / d + yb;
        ang = -atan2Deg(x, y);

        if (ang > 90.0f) ang -= 180.0f;
        else if (ang<-90.0f) ang += 180.0f;

        if (fabs(ang) < 30.0f)
            return goToAvoidBlocks(Vector2f(x, y), ang, true);
        else
            return goToRel(Vector2f(0, 0), ang);
    } else {
        float D = min(0.4f, myDistToBall); //desired distance to ball
        float d = sqrt((xb - xg)*(xb - xg) + (yb - yg)*(yb - yg));
        if (d < EPSILON) d = EPSILON;

        x = (xb - xg) * D / d + xb;
        y = (yb - yg) * D / d + yb;
        ang = angG;

        return goToAvoidBlocks(Vector2f(x, y), ang, true);
    }
}

shared_ptr<Action> Player::fallToGetBall(int dir) {
    shared_ptr<Action> act;
    shared_ptr<Task> curTask = mTask.getFirstSubTask();
    shared_ptr<KickTask> curKick = shared_dynamic_cast<KickTask > (curTask);

    if (NULL != curKick.get()) {
        act = mTask.perform();

        if (NULL != act.get()) {
            mCameraMotionMode = 0;
            return act;
        } else {
            ;
        }
    }

    //keep balance
    act = mBalance.perform();
    if (NULL != act.get()) {
        mKickLock = false;
        mCameraMotionMode = 0;
        mTask.clear();
        return act;
    }

    //new fall
    shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
    if (NULL != curWalkRel.get()) {
        curWalkRel->stopWalk();
        act = mTask.perform();
        if (NULL != act.get()) {
            return act;
        }
    }
    if (-1 == dir) {
        shared_ptr<KickTask> kick = shared_ptr<KickTask > (new KickTask("leftfall_pt_init_squat"));
        mTask.clear();
        mTask.append(kick);
        act = mTask.perform();
        return act;
    } else if (1 == dir) {
        shared_ptr<KickTask> kick = shared_ptr<KickTask > (new KickTask("rightfall_pt_init_squat"));
        mTask.clear();
        mTask.append(kick);
        act = mTask.perform();
        return act;
    } else if (-2 == dir) {
        shared_ptr<KickTask> kick = shared_ptr<KickTask > (new KickTask("LFToLie1_wcy"));
        mTask.clear();
        mTask.append(kick);
        act = mTask.perform();
        return act;
    } else if (2 == dir) {
        shared_ptr<KickTask> kick = shared_ptr<KickTask > (new KickTask("RFToLie1_wcy"));
        mTask.clear();
        mTask.append(kick);
        act = mTask.perform();
        return act;
    } else {
        shared_ptr<KickTask> kick = shared_ptr<KickTask > (new KickTask("squat"));
        mTask.clear();
        mTask.append(kick);
        act = mTask.perform();
        return act;
    }
}
} // namespace soccer
