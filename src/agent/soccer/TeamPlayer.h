/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_TEAM_PLAYER_H
#define SOCCER_TEAM_PLAYER_H

#include "Player.h"

namespace soccer {
    using namespace std;
    using namespace boost;
    using namespace seumath;

    class TeamPlayer : public Player, public Singleton<TeamPlayer> {
    public:
        TeamPlayer();

        virtual ~TeamPlayer();

        /** initalization the player */
        virtual bool init();


    protected:
        /** the paly-on mode, mainly loop */
        virtual boost::shared_ptr<action::Action> playPlayOn();

        /** before kick off */
        virtual boost::shared_ptr<action::Action> playBeforeKickOff();

        /** kick off */
        virtual boost::shared_ptr<action::Action> playOurKickOff();

        virtual boost::shared_ptr<action::Action> playOppKickOff();

        /** kick in */
        virtual boost::shared_ptr<action::Action> playOurKickIn();

        virtual boost::shared_ptr<action::Action> playOppKickIn();

        /** corner kick */
        virtual boost::shared_ptr<action::Action> playOurCornerKick();

        virtual boost::shared_ptr<action::Action> playOppCornerKick();

        /** goal kick */
        virtual boost::shared_ptr<action::Action> playOurGoalKick();

        virtual boost::shared_ptr<action::Action> playOppGoalKick();

        /** offside */
        virtual boost::shared_ptr<action::Action> playOurOffSide();

        virtual boost::shared_ptr<action::Action> playOppOffSide();

        /** game over */
        virtual boost::shared_ptr<action::Action> playGameOver();

        /** Gooooooooooooooooal */
        virtual boost::shared_ptr<action::Action> playOurGoal();

        virtual boost::shared_ptr<action::Action> playOppGoal();

        /** free kick */
        virtual boost::shared_ptr<action::Action> playOurFreeKick();

        virtual boost::shared_ptr<action::Action> playOppFreeKick();

    protected:

        boost::shared_ptr<action::Action> defaultBehaviour();

        boost::shared_ptr<action::Action> attackerCentralBehaviour();//ac, 10

        boost::shared_ptr<action::Action> attackerRightWingBehaviour();// ar,11

        boost::shared_ptr<action::Action> attackerLeftWingBehaviour();// al, 9

	boost::shared_ptr<action::Action> middleFielderLeftBehaviour();//ml, 6
	
	boost::shared_ptr<action::Action> middleFielderCenterBehaviour();//mc,7
	
	boost::shared_ptr<action::Action> middleFielderRightBehaviour();//mr, 8

        boost::shared_ptr<action::Action> defenderRightWingBehaviour();//dr, 5
	
	boost::shared_ptr<action::Action> defenderRightBehaviour();//dl, 4

        boost::shared_ptr<action::Action> defenderLeftWingBehaviour();//dl, 2
	
	boost::shared_ptr<action::Action> defenderLeftBehaviour();//dl, 3

        boost::shared_ptr<action::Action> goalKeeperBehaviour();//gk, 1

        boost::shared_ptr<action::Action> playOurDeadBall();

        boost::shared_ptr<action::Action> playOppDeadBall();

        boost::shared_ptr<action::Action> shoot();
	
	bool canInterceptBall(int& interceptType);
	
	boost::shared_ptr<action::Action> ltyshoot();
	
	boost::shared_ptr<action::Action> ltydribble(seumath::Vector2f aim=Vector2f(12.0f,0.0f));

	boost::shared_ptr<action::Action> ltyclearball();
	
	//dpf 's go to
	boost::shared_ptr<action::Action> dpfGoTo(seumath::Vector2f dest);
	
	boost::shared_ptr<action::Action> dpfGoTo2(seumath::Vector2f dest);
	
	//should the keeper get the ball
	bool isMyBallKeeper();
	
	//should the other players except the keeper get the ball
	bool isMyBallOther();

	//update the place i should go to, ie mMyGoodPos;
	bool updateMyGoodPos();
	
	bool updateMyGoodPos(configuration::Formation::PlayerType playerType);//by type
	
	// get the pos i should go to ,ie get mMyGoodPos
	seumath::Vector2f getMyGoodPos(){
	  return mMyGoodPos;
	}
	
	// is other team's player is near me
	bool isOppNearMe();
	
	bool shouldIShoot();
	///for test
	bool shouldIBlock();
	///for test 
	// is it good to use quick kick
	bool isCloseToOpp();
	
	seumath::Vector2f updateAttackerCentralGoodPos();//11

        seumath::Vector2f updateAttackerRightWingGoodPos();//10

        seumath::Vector2f updateAttackerLeftWingGoodPos();//9
	
	seumath::Vector2f updateMiddleFielderRightGoodPos();//8

        seumath::Vector2f updateMiddleFielderCenterGoodPos();//7

        seumath::Vector2f updateMiddleFielderLeftGoodPos();//6

        seumath::Vector2f updateDefenderRightWingGoodPos();//5
	
	seumath::Vector2f updateDefenderRightGoodPos();//4
	
	seumath::Vector2f updateDefenderLeftGoodPos();//3

        seumath::Vector2f updateDefenderLeftWingGoodPos();//2

        seumath::Vector2f updateGoalKeeperGoodPos();//1

    private:
	seumath::Vector2f mMyGoodPos;
	bool mKickLock;
	unsigned int mLastFastestPlayer;
	Vector2f lastBlockPoint;
	float calBlockDis(Vector2f oppPos, Vector2f oppVector);
	
    };

#define AGENT soccer::TeamPlayer::GetSingleton()

} // namespace soccer

#endif // SOCCER_TEAM_PLAYER_H
