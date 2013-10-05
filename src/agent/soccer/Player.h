/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_PLAYER_H
#define SOCCER_PLAYER_H

#include "configuration/Configuration.h"
#include "task/Task.h"
#include "core/Agent.h"
#include "task/KeepBalance.h"
#include "task/Fall.h"
#include "SoccerDefines.h"
#include "task/KickTask.h"
#include "task/DynamicKick.h"

namespace soccer {

using namespace seumath;
using namespace boost;

enum GoalKeeperState {
    LIED_STATE = 0, //lied state
    LYING_STATE, //lying state
    DIVED_STATE, //dived state
    DIVING_STATE, //diving state
    //LR_ROLLED_STATE,				//rocking state
    LEFTFALL_STATE,
    RIGHTFALL_STATE,
    BALANCE_STATE //balance state
};

struct KickMotion {
    std::string firstTaskName;
    seumath::Vector2f kickTargetRel;
    seumath::Vector2f myDesiredRelPosToBall; //used in calculation of myDesiredPos, related to WM.getBallGlobalPos2D()
    seumath::Vector2f relPosToBallToStopWalk; //used in judgement of stopping walk, related to WM.getBallRelPos2D()
};

class Player : public core::Agent {
public:
    Player();

    virtual ~Player();

    /** initalization the player */
    virtual bool init();

    /** think what need to do, i.e make the decision */
    virtual boost::shared_ptr<action::Action> think();
protected:
    ///////// interface for TeamPlayer ////////////
    /** the paly-on mode, mainly loop */
    virtual boost::shared_ptr<action::Action> playPlayOn() = 0;

    /** before kick off */
    virtual boost::shared_ptr<action::Action> playBeforeKickOff() = 0;

    /** kick off */
    boost::shared_ptr<action::Action> playKickOff();

    virtual boost::shared_ptr<action::Action> playOurKickOff() = 0;

    virtual boost::shared_ptr<action::Action> playOppKickOff() = 0;

    /** kick in */
    boost::shared_ptr<action::Action> playKickIn();

    virtual boost::shared_ptr<action::Action> playOurKickIn() = 0;

    virtual boost::shared_ptr<action::Action> playOppKickIn() = 0;

    /** corner kick */
    boost::shared_ptr<action::Action> playCornerKick();

    virtual boost::shared_ptr<action::Action> playOurCornerKick() = 0;

    virtual boost::shared_ptr<action::Action> playOppCornerKick() = 0;

    /** goal kick */
    boost::shared_ptr<action::Action> playGoalKick();

    virtual boost::shared_ptr<action::Action> playOurGoalKick() = 0;

    virtual boost::shared_ptr<action::Action> playOppGoalKick() = 0;

    /** offside */
    boost::shared_ptr<action::Action> playOffSide();

    virtual boost::shared_ptr<action::Action> playOurOffSide() = 0;

    virtual boost::shared_ptr<action::Action> playOppOffSide() = 0;

    /** game over */
    virtual boost::shared_ptr<action::Action> playGameOver() = 0;

    /** Gooooooooooooooooal */
    boost::shared_ptr<action::Action> playGoal();

    virtual boost::shared_ptr<action::Action> playOurGoal() = 0;

    virtual boost::shared_ptr<action::Action> playOppGoal() = 0;

    /** free kick */
    boost::shared_ptr<action::Action> playFreeKick();

    virtual boost::shared_ptr<action::Action> playOurFreeKick() = 0;

    virtual boost::shared_ptr<action::Action> playOppFreeKick() = 0;

public:
    /************* Skills **************/
    /**
     * let the robot walk to a desired position with desired direction
     *
     * @param stopPos the desired stop position
     * @param dir the direction when reached the desired position
     *
     * @return current action
     */
    boost::shared_ptr<action::Action> goTo(const seumath::Vector2f& destPos, seumath::AngDeg bodyDir, bool avoidBall = true,bool goStop =false);

    /**
     * walk to the desired position and look at a given position
     *
     * @param stopPos the desired stop position
     * @param lookAt the global position of looking at
     *
     * @return current action
     */
    boost::shared_ptr<action::Action> goTo(const seumath::Vector2f& stopPos, const seumath::Vector2f& lookAt, bool avoidBall = true);

    //TT, MMXI
    boost::shared_ptr<action::Action> goToRel(const seumath::Vector2f& target, seumath::AngDeg dir ,bool goStop =false);

    //bodyDir: body direction
    //one body direction may match any walk direction
    boost::shared_ptr<action::Action> goToAvoidBlocks(seumath::Vector2f dest, seumath::AngDeg bodyDir,bool avoidBall = true, bool goStop =false,bool avoidPlayer =1);

    boost::shared_ptr<action::Action> goToBallBack();

    /**
     * the function for challenge of walking
     */
    boost::shared_ptr<action::Action> walkToBall();

    boost::shared_ptr<action::Action> sideWalk(bool isLeft);

    boost::shared_ptr<action::Action> dribble();

    boost::shared_ptr<action::Action> dribbleRel(AngDeg angC);

    boost::shared_ptr<action::Action> dribbleToOppGoal();

    //TT
    //dribble to [angC] direction, between [angL] and [angR]
    //all of them are global parameters
    boost::shared_ptr<action::Action> dribbleToDir(AngDeg angC, AngDeg angL, AngDeg angR);

    boost::shared_ptr<action::Action> kickTo(const seumath::Vector2f& goal, bool useMaxForceMotion = true);

    boost::shared_ptr<action::Action> DefendKick();

    boost::shared_ptr<action::Action> kickRel();

    boost::shared_ptr<action::Action> kickBetween(const seumath::Vector2f& goalLeft, const seumath::Vector2f& goalRight);

    //TT
    // +-1, +-2
    boost::shared_ptr<action::Action> fallToGetBall(int dir);
    
    boost::shared_ptr<action::Action> dynamicKick(const seumath::Vector2f& goal, bool useMaxForceMotion =true);

    boost::shared_ptr<action::Action> beamAndInit(const seumath::Vector3f& beamPos);


    /**
     * choose the kick type according to current state
     *
     * @param goal where want to kick to
     *
     * @return a shared_ptr<KickTask>
     */
    boost::shared_ptr<task::KickTask> chooseKickType(const seumath::Vector2f& goal, bool useMaxForceMotion, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk);
    int chooseKickType1(const seumath::Vector2f& goal, int kickType, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk, int errAng);

    /**
     * add by ghd to revise angle when dribble,and just used in dribble to pass opp player
     *
     * @param angC
     * @param angL
     * @param angR
     *
     */
    void ReviseAng(AngDeg& angC,AngDeg& angL,AngDeg& angR);           //add by ghd


protected:

    /// cache the task
    task::Task mTask;

    task::KeepBalance mBalance;

    //TT, April, MMXI
    //-1: as CM wish
    //0: don't turn
    //1: search ball
    //2: stick to ball
    //3: search flags
    //4: for attacking (focus on ball and opp goal)
    //-2: for test
    int mCameraMotionMode;

    //true: kicking, don't want to change my mind
    bool mKickLock;

    //TT, April, MMXI
    //0: as LR wish
    //1: left
    //2: right
    //int mMovingDir;

private:
    seumath::Vector3f mBallPredictedPos;
    std::vector<KickMotion> mKickMotionVector;
    unsigned int times;
    float simtime;
    bool mIsFull;
    bool mIsLeft;
    
    bool mFirstKick;
    int mKickCounter;
}; //end of class Player
} //end of namespace soccer

#endif //SOCCER_PLAYER_H

