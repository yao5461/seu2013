/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: TeamPlayer.cpp 2755 2009-04-02 08:00:08Z zyj $
 *
 ****************************************************************************/

#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include "action/BeamAction.h"
#include "action/Actions.h"
#include "controller/FixedAngleTrace.h"
#include "TeamPlayer.h"
#include "task/KeepBalance.h"
#include "perception/Vision.h"
#include "task/CameraMotion.h"
#include "core/PassModel.h"

namespace soccer
{

using namespace std;
using namespace boost;
using namespace serversetting;
using namespace action;
using namespace task;
using namespace seumath;

TeamPlayer::TeamPlayer()
{
}

TeamPlayer::~TeamPlayer()
{
}

bool TeamPlayer::init()
{	if ( !Player::init() ) return false;

	// get the game state information from the server
	// such as team index, unum, etc
	while ( true )
	{	boost::shared_ptr<perception::Perception> p = sense();
		if ( 0 == p.get() ) break;
		if ( !WM.update ( p ) ) break;
		if ( WM.getMyUnum() > 0 )
		{	shared_ptr<Action> act = beamAndInit ( FM.getMy().beforeKickOffBeam/*+Vector3f(-0.3f,0.0f,0.0f)*/ );
			perform ( act );

			return true;
		}
	}
	return false;
}


/** the paly-on mode, mainly loop */
shared_ptr<Action> TeamPlayer::playPlayOn()
{	/*************************************
	 * EXPERIMENTS:
	 * We do some experiments here
	 *************************************/
	shared_ptr<Action> act;
	//first update my good pos
	//cout<<"fast"<< WM.getOurFastestToBallNum()<<endl;
	//if(FM.getMy().type!=configuration::Formation::PT_ATTACKER_CENTRAL){
	//return FAT.controlPreferThan("squat", "*");
	//}

	updateMyGoodPos();
	//  if (WM.isGameStateChanged())
	//    mTask.clear();
	//cout<<"playon"<<mTask.getSubTaskListSize()<<endl;
	if ( !mTask.isSubDone() )
	{	//cout<<"@"<<  WM.getGameTime()<<"isSubDone()" <<endl;;
		return mTask.perform();
	}
	//cout<<"@"<<  WM.getGameTime()<<endl;
	switch ( FM.getMy().type )
	{	case configuration::Formation::PT_GOALKEEPER: //No.1,gk

			act = goalKeeperBehaviour();
			break;
		case configuration::Formation::PT_ATTACKER_RIGHT://11,ar

			act = attackerRightWingBehaviour();
			break;
		case configuration::Formation::PT_ATTACKER_CENTRAL://10,ac

			act = attackerCentralBehaviour();
			break;
		case  configuration::Formation::PT_ATTACKER_LEFT: //No.9,al

			act = attackerLeftWingBehaviour();
			break;
		case configuration::Formation::PT_MIDFIELDER_LEFT://no.6,ml

			act = middleFielderLeftBehaviour();
			break;
		case configuration::Formation::PT_MIDFIELDER_CENTER://no.7,mc

			act = middleFielderCenterBehaviour();
			break;
		case configuration::Formation::PT_MIDFIELDER_RIGHT://no.8,mr

			act = middleFielderRightBehaviour();
			break;
		case configuration::Formation::PT_DEFENDER_RIGHT_WING://no.5,drw

			act = defenderRightWingBehaviour();
			break;
		case configuration::Formation::PT_DEFENDER_RIGHT://no.4,dr

			act = defenderRightBehaviour();
			break;
		case configuration::Formation::PT_DEFENDER_LEFT://no.3,dl

			act = defenderLeftBehaviour();
			break;
		case configuration::Formation::PT_DEFENDER_LEFT_WING://no.2,dlw

			act = defenderLeftWingBehaviour();
			break;
		default:

			act = defaultBehaviour();
			break;

	}
	if(WM.getMyUnum()==11)
	{
	    cout<<"block"<<WM.calOurBlockNumber() <<"--------------"<<"dribble"<<WM.getOurFastestToBallNum()<<endl;
	}
	/*
	Vector3f pos = WM.getOppBodyInformation(1,Vision::L_FOOT);
	int i = WM.isOppFall(10);
	if(i==-1)
	  cout<<"Can't See"<<endl;
	if(i==0)
	  cout<<"Steady"<<endl;
	if(i==1)
	  cout<<"Fallen"<<endl;
	if(i==2)
	  cout<<"Middle Condition"<<endl;
	*/
	/*for(int i=0;i<=19;i++)
	{
	  cout<<"Time"<<i<<":    "<<WM.GetmOppWalkPos(10,i,Vision::L_FOOT)<<"  "<<WM.GetmOppWalkPos(10,i,Vision::R_FOOT)<<endl;
	}
	cout<<"Speed="<<WM.GetMyWalkVec()<<"   "<<WM.GetOppWalkVec(10)<<endl;
	cout<<"Vector="<<WM.GetOppWalkVector(10)<<endl;*/
	return act;
}

/** before kick off */
shared_ptr<Action> TeamPlayer::playBeforeKickOff()   ///terrymimi
{

	return beamAndInit ( FM.getMy().beforeKickOffBeam );

	///modify in 2013/04/02
//       cout<<"Deg:"<<WM.getOurPlayerNumber()<<endl;
	if ( WM.getMyUnum() == 11 && WM.getOurPlayerNumber() <= 2 )
	{	//cout<<"11"<<endl;
		return beamAndInit ( FM.getMy().ourGoalBeam );
	}
	else
	{	//cout<<"22"<<endl;
		return beamAndInit ( FM.getMy().beforeKickOffBeam );
	}
}

/** kick off */
shared_ptr<Action> TeamPlayer::playOurKickOff()
{	shared_ptr<Action> act;
	Vector2f target = Vector2f ( half_field_length*0.4f,-half_field_width );

	/*a new kickoff strategy by liuyao*/
	switch ( FM.getMy().type )
	{	case configuration::Formation::PT_GOALKEEPER:
			if ( WM.amIFastestToBallOfOurTeam() )
			{	act =  kickTo ( target );
			}
			else
			{	act =  FAT.controlPreferThan ( "init", "*" );
			}
			break;

		default:
			if ( WM.amIFastestToBallOfOurTeam() )
			{

				if ( WM.getOurPlayerNumber() <= 2 )   /// a error in size of map for store our teammates
				{	act = dribble();
				}
				else
				{

// 		  if( FM.getMy().type == configuration::Formation::PT_ATTACKER_RIGHT ) {
// 		    if((WM.getMyGlobalPos2D()-WM.getBallGlobalPos2D()).length() < 1 && FM.getMy().beforeKickOffBeam.z() == -80.0 ) {
// 		      act = FAT.controlPreferThan("BTT_t1", "*");
// 		    } else {
// 		      act =  kickTo(target);
// 		    }
// 		  } else {
					act =  kickTo ( target );
// 		  }

				}

			}
			else
			{	act =  FAT.controlPreferThan ( "squat", "*" );
			}
			break;
	}

	return act;
}

shared_ptr<Action> TeamPlayer::playOppKickOff()
{	shared_ptr<Action> act;
	act = mBalance.perform();
	if ( NULL != act.get() )
	{	mTask.clear();
		return act;
	}
	else
	{	return FAT.controlPreferThan ( "squat", "*" );
	}
}

/** kick in */
shared_ptr<Action> TeamPlayer::playOurKickIn()
{	return playOurDeadBall();
	// return playPlayOn();
}

shared_ptr<Action> TeamPlayer::playOppKickIn()
{	updateMyGoodPos();
	return dpfGoTo ( mMyGoodPos );
}

/** corner kick */
shared_ptr<Action> TeamPlayer::playOurCornerKick()
{	return playPlayOn();
}

shared_ptr<Action> TeamPlayer::playOppCornerKick()
{	updateMyGoodPos();
	return dpfGoTo ( mMyGoodPos );
}

/** goal kick */
shared_ptr<Action> TeamPlayer::playOurGoalKick()
{	Vector2f target ( -5, -half_field_width );

	switch ( FM.getMy().type )
	{	case configuration::Formation::PT_GOALKEEPER:
		{
//                 const Vector2f& myPos = WM.getMyGlobalPos2D();
//                 const Vector2f& ballPos = WM.getBallGlobalPos2D();
//                 Vector2f Destination(-2, myPos.y() > 0 ? -4 : 4);
//                 Vector2f ourGoal(-half_field_length, 0);
//
//                 if (myPos.x() < ballPos.x()) {
//                     return kickTo(Destination, true);
//                 } else {
//                     if(abs(target.y()<0.1f)){
//                         if(target.y() >0.0f){
//                             target.y()=0.1f;
// 			}else{
//                             target.y()=-0.1;
// 			}
//                     }
//                     return goTo(target,0);
//                 }
			return kickTo ( target );
			break;
		}
		case configuration::Formation::PT_DEFENDER_LEFT:
		{	mMyGoodPos =Vector2f ( -half_field_length+3 -0.6f,3 + 0.6f );
			return goTo ( mMyGoodPos,-45 );
		}
		case configuration::Formation::PT_DEFENDER_RIGHT:
		{	mMyGoodPos =Vector2f ( -half_field_length+3 -0.6,-3 - 0.6f );
			return goTo ( mMyGoodPos,45 );
		}
		default:
			//updateMyGoodPos();
			if ( mMyGoodPos.x() <-8.5f )
			{	mMyGoodPos.x() = -8.5f;
			}

			return goTo ( mMyGoodPos,0 );
			//return FAT.controlPreferThan("init", "*");
			break;
	}
}

shared_ptr<Action> TeamPlayer::playOppGoalKick()
{	updateMyGoodPos();
	return dpfGoTo ( mMyGoodPos );
}

/** offside */
shared_ptr<Action> TeamPlayer::playOurOffSide()
{	return playPlayOn();
}

shared_ptr<Action> TeamPlayer::playOppOffSide()
{	return goTo ( getMyGoodPos(),0 );
}

/** game over */
shared_ptr<Action> TeamPlayer::playGameOver()
{	cerr << "[ERROR] Player can not hanle playGameOver\n";
	return FAT.controlPreferThan ( "squat", "*" );
}

/** Gooooooooooooooooal */
shared_ptr<Action> TeamPlayer::playOurGoal()
{	return beamAndInit ( FM.getMy().ourGoalBeam );
}

shared_ptr<Action> TeamPlayer::playOppGoal()
{	return beamAndInit ( FM.getMy().oppGoalBeam );
}

/** free kick */
shared_ptr<Action> TeamPlayer::playOurFreeKick()
{	return playOurDeadBall();
}

shared_ptr<Action> TeamPlayer::playOppFreeKick()
{	updateMyGoodPos();
	return dpfGoTo ( mMyGoodPos );
}

shared_ptr<Action> TeamPlayer::defaultBehaviour()
{	//return goTo(getMyGoodPos(),0);
	/**
	 * the simplest decision:
	 * If I am the fastes, shoot the ball,
	 * otherwise run to the strategic position
	 */
	/*
	///test cout by dpf
	cout<<"gyro info\t"<<WM.getMyGyroRate()*0.02<<endl;
	cout<<"acc info \t"<<WM.getMyAcc()<<endl;
	cout<<"bodyang\t"<<WM.getMyBodyAng()<<endl;
	 */
	///
	///only for test defend strategies
	///visen
	if ( WM.calOurBlockNumber() == WM.getMyUnum() )
	{	int oppNum = WM.getOppDribbleNum();
		Vector2f oppPos = oppNum==0?Vector2f ( 100000,100000 ) :WM.getOppGlobalPos2D ( oppNum );
		Vector2f myPos=WM.getMyGlobalPos2D();
		Vector2f ballPos=WM.getBallGlobalPos2D();
		float dis1= ( myPos-ballPos ).length();
		float dis2= ( oppPos-ballPos ).length();
		if ( oppNum==0 )
			return dribble();
		if ( dis1<dis2 )
			return dribble();
	///	if ( WM.IsInBlockRange ( oppNum,WM.getMyUnum(),0.1 ) ==true && WM.IsOnTheBlock ( oppNum,WM.getMyUnum() ) ==false )
		if ( WM.IsInBlockRange ( oppNum,WM.getMyUnum(),0.3 ) ==true && WM.IsOnTheBlock ( oppNum,WM.getMyUnum() ) ==false )
		{	if ( WM.isPlayerInField ( oppPos,1 ) ==true )
			{	Vector2f oppVector=WM.GetOppWalkVector ( oppNum );
				float blockDis=calBlockDis ( oppPos,oppVector );
				oppVector.normalize();
				Vector2f blockPos ( oppPos.x() +oppVector.x() *blockDis,oppPos.y() +oppVector.y() *blockDis );
				lastBlockPoint=blockPos;
				return dpfGoTo ( blockPos );
			}
			else
			{	if ( lastBlockPoint!=Vector2f ( 100000,100000 ) )
					return dpfGoTo ( lastBlockPoint );
			}
		}
		else
			lastBlockPoint=Vector2f ( 100000,100000 );

		return dribble();
	}
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	/* Vector2f ballPos=WM.getBallGlobalPos2D();
		 Vector2f opptoballpos(WM.getOppFastestToBallPos().x()-WM.getBallGlobalPos2D().x(),WM.getOppFastestToBallPos().y()-WM.getBallGlobalPos2D().y());
		 Vector2f mytoballpos (WM.getMyGlobalPos2D().x()-WM.getBallGlobalPos2D().x(),WM.getMyGlobalPos2D().y()-WM.getBallGlobalPos2D().y());

		 if(ballPos.x()<=-13.5&&abs(opptoballpos.length()-mytoballpos.length()<=0.8f))
		   return dribble();
		 else*/
		   
		return dpfGoTo ( mMyGoodPos );
	}


	///
	///
	Vector2f oppGoal ( half_field_length, 0 );
	Vector2f myDesiredPos = getMyGoodPos();
	//cout<<"g"<<endl;
	//cout<<myDesiredPos.getX();
	//cout<<'\t'<<me.getX()<<endl;
	/*if(WM.getGameTime()>300.0f){
	    oppGoal.y() =-oppGoal.y();
	    myDesiredPos.y() =-myDesiredPos.y();
	}*/
	float turnAng = ( myDesiredPos - WM.getMyGlobalPos2D() ).angle();
	//Vector2f test(15,2);
	if ( WM.amIFastestToBallOfOurTeam() )
	{	//return goToRel(Vector2f(-1,0),0);
		//return FAT.controlPreferThan("init", "*");
		//return goTo(test,0,true);//
		///return kickTo(oppGoal);/////original value
		return dribble();
	}
	else
	{	return goTo ( myDesiredPos,0 );
	}

	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}




}

bool TeamPlayer::isMyBallKeeper()
{	const Vector2f ourGoal = Vector2f ( -half_field_length, 0 );
	Vector2f ballGlobalPos = WM.getBallGlobalPos2D();
	if ( WM.amIFastestToBallOfOurTeam() )
	{	return true;
	}
	else
	{	return false;
	}
}

bool TeamPlayer::isMyBallOther()
{

	if ( WM.amIFastestToBallOfOurTeam() )
	{	if ( mLastFastestPlayer != WM.getMyUnum() )	//first fastest period
		{	mKickLock =false;
		}
		mLastFastestPlayer =WM.getOurFastestToBallNum();
		return true;
	}
	else
	{	mLastFastestPlayer =WM.getOurFastestToBallNum();
		return false;
	}
}

boost::shared_ptr<action::Action> TeamPlayer::ltyclearball()
{	if ( /*!isOppNearMe()*/1 )
		return ltyshoot();
	return dribble();//ltydribble();
}


bool TeamPlayer::updateMyGoodPos()
{

	return updateMyGoodPos ( FM.getMy().type );
}

bool TeamPlayer::updateMyGoodPos ( configuration::Formation::PlayerType playerType )
{
//         const seumath::Vector2f myGoal = Vector2f(-half_field_length, 0);
//         const seumath::Vector2f oppGoal = Vector2f(half_field_length, 0);

// 	cout<<WM.IsJoin()<<endl;
	if ( playerType  != configuration::Formation::PT_GOALKEEPER && WM.IsJoin() )
	{	mMyGoodPos =PM.findMyGoodPos ( WM.getHearGoodPosID() );
	}
	else
	{
// 	  mMyGoodPos =Vector2f(15,20);

		switch ( playerType )
		{	case configuration::Formation::PT_GOALKEEPER: //No.1
			{	mMyGoodPos = updateGoalKeeperGoodPos();
			}
			break;
			case configuration::Formation::PT_ATTACKER_CENTRAL://10,attacker center
			{
//		  mMyGoodPos = updateAttackerCentralGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_ATTACKER_RIGHT://11, attacker right
			{
//		  mMyGoodPos = updateAttackerRightWingGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_ATTACKER_LEFT://9, attacker left
			{
//		  mMyGoodPos = updateAttackerLeftWingGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_MIDFIELDER_LEFT://6, middler left
			{
//		  mMyGoodPos = updateMiddleFielderLeftGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_MIDFIELDER_CENTER://7, middler center
			{
//		  mMyGoodPos = updateMiddleFielderCenterGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_MIDFIELDER_RIGHT://8, middler right
			{	//mMyGoodPos = updateMiddleFielderRightGoodPos();
				mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_DEFENDER_RIGHT://4, defender right
			{	mMyGoodPos = updateDefenderRightGoodPos();
			}
			break;
			case configuration::Formation::PT_DEFENDER_RIGHT_WING://5, defender right wing
			{	mMyGoodPos = updateDefenderRightWingGoodPos();
			}
			break;
			case configuration::Formation::PT_DEFENDER_LEFT://3, defender left
			{	mMyGoodPos = updateDefenderLeftGoodPos();
			}
			break;
			case configuration::Formation::PT_DEFENDER_LEFT_WING://2, defender left wing
			{	mMyGoodPos = updateDefenderLeftWingGoodPos();
			}
			break;
			default:
			{	mMyGoodPos = * ( const Vector2f* ) ( FM.getMy().homePos ).get();
			}
			break;
		}

		if ( WM.getMyUnum() >7 )
		{	Vector2f tempVec =mMyGoodPos -Vector2f ( -half_field_length,0 );
			if ( tempVec.length() <6.0f )
			{	tempVec/=tempVec.length();
				tempVec *=6.0f;
				mMyGoodPos =Vector2f ( -half_field_length,0 ) +tempVec;
			}
		}

		if ( mMyGoodPos.y() >half_field_width )
		{	mMyGoodPos.y() = half_field_width;
		}
		if ( mMyGoodPos.y() <-half_field_width )
		{	mMyGoodPos.y() = -half_field_width;
		}
		if ( mMyGoodPos.x() >half_field_length )
		{	mMyGoodPos.x() = half_field_length;
		}
		if ( mMyGoodPos.x() <-half_field_length )
		{	mMyGoodPos.x() = -half_field_length;
		}
	}
	return true;
}

Vector2f TeamPlayer::updateAttackerCentralGoodPos()  //10
{

	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret;

	if ( ballGlobalPos.x() < -5 )
	{	ret.x() = ballGlobalPos.x() - 1.5f;
		if ( fabs ( ballGlobalPos.y() ) > 5 )
		{	ret.y() = ballGlobalPos.y() + ( ballGlobalPos.y() > 5 ? 1.5f : 2.0f );
		}
		else
		{	ret.y() = ballGlobalPos.y() + 1.0f;
		}
	}
	else
	{	ret.x() = ballGlobalPos.x() - 2.0f;
		if ( fabs ( ballGlobalPos.y() ) > 5 )
		{	ret.y() = ballGlobalPos.y() + ( ballGlobalPos.y() > 5 ? 0.5f : 2.0f );
		}
		else
		{	ret.y() = ballGlobalPos.y() + ( ballGlobalPos.y() > 0 ? 1.5f : 2.0f );
		}
	}
	return ret;
}

Vector2f TeamPlayer::updateAttackerLeftWingGoodPos()  //9
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret;

	if ( ballGlobalPos.x() < -5 )
	{	ret.x() = ballGlobalPos.x() - 1.5f;
		if ( fabs ( ballGlobalPos.y() ) > 5 )
		{	ret.y() = ballGlobalPos.y() - ( ballGlobalPos.y() > 5 ? 2.0f : 1.5f );
		}
		else
		{	ret.y() = ballGlobalPos.y() - 1.0f;
		}
	}
	else
	{	ret.x() = ballGlobalPos.x() - 2.0f;
		if ( fabs ( ballGlobalPos.y() ) > 5 )
		{	ret.y() = ballGlobalPos.y() - ( ballGlobalPos.y() < -5 ? 0.5f : 2.0f );
		}
		else
		{	ret.y() = ballGlobalPos.y() - ( ballGlobalPos.y() > 0 ? 2.0f : 1.5f );
		}
	}
	return ret;
}

Vector2f TeamPlayer::updateAttackerRightWingGoodPos()  //11
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret;
	if ( ballGlobalPos.x() > 13 )
	{	ret.x() = ballGlobalPos.x() - 0.5;
	}
	else if ( ballGlobalPos.x() > 5 )
	{	ret.x() = 13;
	}
	else
	{	ret.x() = ballGlobalPos.x() + 5;
	}
	ret.y() = ballGlobalPos.y() + 2 * ( ballGlobalPos.y() > 0 ? 1: -1 ) * ( fabs ( ballGlobalPos.y() ) < 5 ) ? 1 : -1;
	ret.y() = clamp ( ret.y(), -5.0f, 5.0f );

//         ret = ballGlobalPos+Vector2f(5f,0.25f);
//
// 	if(ballGlobalPos.x()<-7.0f)
// 	  ret=Vector2f(-6.0f,-0.4f);
// 	if(ballGlobalPos.x()>6.5f)
// 	  ret=Vector2f(10.0f,-0.4f);
	return ret;
}

Vector2f TeamPlayer::updateMiddleFielderCenterGoodPos()  //7
{	unsigned int id = WM.getOurFastestToBallNum();

	if ( id == 10 )
	{	return updateAttackerCentralGoodPos();  //10
	}
	else if ( id == 9 )
	{	return updateAttackerLeftWingGoodPos();  //9
	}
	else
	{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
		Vector2f ret;
		if ( ballGlobalPos.x() > -5 )
		{	ret.x() = ballGlobalPos.x() - 5.0f;
			ret.y() = ballGlobalPos.y();
			ret.y() = clamp ( ret.y(), -5.0f, 5.0f );
		}
		else
		{	Vector2f myGoal ( -half_field_length, 0 );
			float dist = ( myGoal-ballGlobalPos ).length();
			if ( fabs ( ballGlobalPos.y() ) > 5 )
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *4 + ballGlobalPos;
			}
			else
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *2 + ballGlobalPos;
			}
		}
		return ret;
	}
}

Vector2f TeamPlayer::updateMiddleFielderLeftGoodPos()  //6
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f myGoal ( -half_field_length, 0 );
	Vector2f ret;
	/*if(WM.getOurFastestToBallNum() == 11) {
	  return updateAttackerRightWingGoodPos();  //11
	} else*/
	if ( WM.getOurFastestToBallNum() == 9 )   //7
	{	if ( ballGlobalPos.x() > -5 )
		{	ret.x() = ballGlobalPos.x() - 5.0f;
			ret.y() = ballGlobalPos.y();
			ret.y() = clamp ( ret.y(), -5.0f, 5.0f );
		}
		else
		{	float dist = ( myGoal-ballGlobalPos ).length();
			if ( fabs ( ballGlobalPos.y() ) > 5 )
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *4 + ballGlobalPos;
			}
			else
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *2 + ballGlobalPos;
			}
		}
	}
	else
	{	if ( ballGlobalPos.x() > -5 )
		{	ret.x() = ballGlobalPos.x() - 7.0f;
			ret.y() = ballGlobalPos.y() - 4.0f;
			ret.y() = clamp ( ret.y() , -6.0f, 0.0f );
		}
		else
		{	if ( fabs ( ballGlobalPos.y() ) > 5 )
			{	Vector2f point ( -10, -5 );
				float dist = ( myGoal-point ).length();
				ret = ( ( myGoal-point ) /dist ) *3 + point;
			}
			else
			{	ret.x() = -12.0f;
				ret.y() = ballGlobalPos.y()-4.0f;
			}
		}
	}
	return ret;
}

Vector2f TeamPlayer::updateMiddleFielderRightGoodPos()  //8
{	unsigned int id = WM.getOurFastestToBallNum();
	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f myGoal ( -half_field_length, 0 );
	Vector2f ret;

	if ( id == 7 || id == 10 )  //7
	{	if ( ballGlobalPos.x() > -5 )
		{	ret.x() = ballGlobalPos.x() - 5.0f;
			ret.y() = ballGlobalPos.y();
			ret.y() = clamp ( ret.y(), -5.0f, 5.0f );
		}
		else
		{	float dist = ( myGoal-ballGlobalPos ).length();
			if ( fabs ( ballGlobalPos.y() ) > 5 )
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *4 + ballGlobalPos;
			}
			else
			{	ret = ( ( myGoal-ballGlobalPos ) /dist ) *2 + ballGlobalPos;
			}
		}
	}
	else
	{	if ( ballGlobalPos.x() > -5 )
		{	ret.x() = ballGlobalPos.x() - 7.0f;
			ret.y() = ballGlobalPos.y() + 4.0f;
			ret.y() = clamp ( ret.y() , 0.0f, 6.0f );
		}
		else
		{	if ( fabs ( ballGlobalPos.y() ) > 5 )
			{	Vector2f point ( -10, 5 );
				float dist = ( myGoal-point ).length();
				ret = ( ( myGoal-point ) /dist ) *3 + point;
			}
			else
			{	ret.x() = -12.0f;
				ret.y() = ballGlobalPos.y() +4.0f;
			}
		}
	}
	return ret;
}

Vector2f TeamPlayer::updateDefenderRightWingGoodPos()  //5
{	Vector2f ballPos = WM.getBallGlobalPos2D();
	Vector2f myGoal ( -half_field_length, 0 );
	Vector2f ret;

	ret = ( ballPos - myGoal ) / 2 + myGoal;

	return ret;

// 	unsigned int id = WM.getOurFastestToBallNum();
// 	Vector2f ballGlobalPos = WM.getBallGlobalPos2D();
// 	Vector2f myGoal(-half_field_length, 0);
// 	Vector2f ret;
// 	if(id == 10 || id == 7 || id == 8) {   //8
// 	  if(ballGlobalPos.x() > -5) {
// 	    ret.x() = ballGlobalPos.x() - 7.0f;
// 	    ret.y() = ballGlobalPos.y() + 4.0f;
// 	    ret.y() = clamp(ret.y() , 0.0f, 6.0f);
// 	  } else {
// 	    if(fabs(ballGlobalPos.y()) > 5) {
// 	      Vector2f point(-10, 5);
// 	      float dist = (myGoal-point).length();
// 	      ret = ((myGoal-point)/dist)*3 + point;
// 	    } else {
// 	      ret.x() = -12.0f;
// 	      ret.y() = ballGlobalPos.y()+4.0f;
// 	    }
// 	  }
// 	} else if(id == 9 || id == 6) {  //6
// 	  if(ballGlobalPos.x() > -5) {
// 	    ret.x() = ballGlobalPos.x() - 7.0f;
// 	    ret.y() = ballGlobalPos.y() - 4.0f;
// 	    ret.y() = clamp(ret.y() , -6.0f, 0.0f);
// 	  } else {
// 	    if(fabs(ballGlobalPos.y()) > 5) {
// 	      Vector2f point(-10, -5);
// 	      float dist = (myGoal-point).length();
// 	      ret = ((myGoal-point)/dist)*3 + point;
// 	    } else {
// 	      ret.x() = -12.0f;
// 	      ret.y() = ballGlobalPos.y()-4.0f;
// 	    }
// 	  }
// 	} else {
// 	  Vector2f myGoal = Vector2f(-half_field_length, 0);
// 	  ret.x() = ballGlobalPos.x() - 13.0f;
// 	  if(ret.x() < -12) {
// 	      ret.x() = -12;
// 	  }
// 	  if( ballGlobalPos.x() > -5 || fabs(ballGlobalPos.y()) < 5 ) {
// 	    ret.y() = ballGlobalPos.y() + (ballGlobalPos.y() > 0 ? -0.5 : 0.5);
// 	  } else {
// 	    ret.y() = 0;
// 	  }
// 	}
// 	return ret;
}


Vector2f TeamPlayer::updateDefenderLeftWingGoodPos()  //2
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret;
	if ( ballGlobalPos.x() > -5 )
	{	ret.x() = -12;
		ret.y() = half_penalty_width;
	}
	else
	{	const Vector2f myGoal = Vector2f ( -half_field_length, 0 );
		float ballDistToOurGoal = ( ballGlobalPos - myGoal ).length();

		if ( ballGlobalPos.y() > 5 )
		{	ret = ( myGoal + ( ballGlobalPos - myGoal ) / ballDistToOurGoal * 2.0f );
		}
		else if ( ballGlobalPos.y() > 0 )
		{	ret.x() = penalty_length - half_field_length;
			ret.y() = ballGlobalPos.y();
			ret.y() = clamp ( ret.y(), 0.0f, half_penalty_width );
		}
		else
		{	ret.x() = -13.5f;
			ret.y() = half_penalty_width + 1.0f;
		}
	}
	return ret;
}

Vector2f TeamPlayer::updateDefenderLeftGoodPos()  //3
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret;
	if ( ballGlobalPos.x() > -5 )
	{	ret.x() = -12;
		ret.y() = -half_penalty_width;
	}
	else
	{	const Vector2f myGoal = Vector2f ( -half_field_length, 0 );
		float ballDistToOurGoal = ( ballGlobalPos - myGoal ).length();

		if ( ballGlobalPos.y() < -5 )
		{	ret = ( myGoal + ( ballGlobalPos - myGoal ) / ballDistToOurGoal * 2.0f );
		}
		else if ( ballGlobalPos.y() < 0 )
		{	ret.x() = penalty_length - half_field_length;
			ret.y() = ballGlobalPos.y();
			ret.y() = clamp ( ret.y(), -half_penalty_width, 0.0f );
		}
		else
		{	ret.x() = -13.5f;
			ret.y() = -half_penalty_width - 1.0f;
		}
	}

	return ret;
}

Vector2f TeamPlayer::updateDefenderRightGoodPos()  //4
{	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	Vector2f ret ( ( half_penalty_width-half_field_length ), 0 );

// 	if( fabs(ballGlobalPos.y()) < 5 ) {
// 	  const Vector2f myGoal = Vector2f(-half_field_length, 0);
// 	  float ballDistToOurGoal = (ballGlobalPos - myGoal).length();
// 	  ret =(myGoal + (ballGlobalPos - myGoal) / ballDistToOurGoal * 2.0f);
// 	  ret.y() = clamp(ret.y(), -half_penalty_width, half_penalty_width);
// 	} else {
// 	  ret = Vector2f((half_penalty_width-half_field_length), 0);
// 	}

	return ret;
}

Vector2f TeamPlayer::updateGoalKeeperGoodPos()       //1
{	const Vector2f myGoal = Vector2f ( -half_field_length, 0 );
	Vector2f ret;
//         Vector2f ballGlobalPos = WM.getBallGlobalPos2D();
	Vector2f ballGlobalPos = WM.getInterceptBallGlobalPos2D();
	float ballDistToOurGoal = ( ballGlobalPos - myGoal ).length();
	//if (ret.x()<-14.7f)ret.setX(-14.7f);
	//std::cout<<ret<<std::endl;
	//modify by liuyao
	ret = myGoal + ( ( ballGlobalPos - myGoal ) / ballDistToOurGoal ); // * 0.8;
	ret.x() += 0.2;
// 	if(ballGlobalPos.x() > -7) {
// 	  ret.x() += 0.2;
// 	}
	cout<<"x="<<ret.x()<<"y="<<ret.y()<<endl;
	return ret;
//        return ret = (myGoal + (ballGlobalPos - myGoal) / ballDistToOurGoal );
}

shared_ptr<Action> TeamPlayer::dpfGoTo ( Vector2f dest )
{	Vector2f myGlobalPos = WM.getMyGlobalPos2D();
	return goTo ( dest, ( WM.getBallGlobalPos2D() - myGlobalPos ).angle() );

	/*
	//avoid get the same dest with other players
	map<unsigned int, seumath::Vector3f>ourPlayerMap=WM.getOurGlobalPos();
	Vector2f tmpPos;
	bool isSameDestWithOthers=false;
	map<unsigned int, seumath::Vector3f>::iterator It=ourPlayerMap.begin();
	for(;It!=ourPlayerMap.end();It++){
	  if(It->first==WM.getMyUnum())continue;
	  tmpPos=*(Vector2f*)(It->second).get();
	  if((tmpPos-dest).length()<0.5f){
	    isSameDestWithOthers=true;
	    break;
	  }
	}
	if(isSameDestWithOthers){//avoid the same dest with others
	    dest-=Vector2f(0,0.5);
	    if((tmpPos-dest).length()<0.5f)dest+=Vector2f(0,1.0f);
	}
	Vector2f myGlobalPos=WM.getMyGlobalPos2D();
	if((dest-myGlobalPos).length()>100.0f){//not good, don't use it, so set 100
	  //far enough
	  return goTo(dest,(dest-myGlobalPos).angle());
	}
	else{
	  //near the target
	  return goTo(dest,(WM.getBallGlobalPos2D()-myGlobalPos).angle());
	}
	 */
}

shared_ptr<Action> TeamPlayer::dpfGoTo2 ( Vector2f dest )
{	Vector2f myGlobalPos = WM.getMyGlobalPos2D();
	// Vector2f destRelPos=WM.transGlobalPosToRelPos(dest);
	//  float turnAng = normalizeAngle((WM.getBallGlobalPos2D()-myGlobalPos).angle() - WM.getMyBodyDirection());
	return goTo ( dest, ( WM.getBallGlobalPos2D() - myGlobalPos ).angle() );
	//return goToRel(destRelPos,0);
}

shared_ptr<Action> TeamPlayer::shoot()
{	const Vector2f& ballPos = WM.getBallGlobalPos2D();
	Vector2f goal ( half_field_length, 0 );
	Vector2f goalLeft ( half_field_length, 0 );
	Vector2f goalRight ( half_field_length, 0 );
	if ( ( ballPos - goal ).length() > half_field_length *0.7f || ( ballPos.x() > half_field_length-3.0f && ( ballPos - goal ).length() > 2.0f ) )
	{	return dribble();//ltydribble();//dribbleToOppGoal();.
	}
	else
	{	return kickTo ( ( ballPos.y() > 0 ) ? goalRight : goalLeft, true );
	}
}

bool TeamPlayer::isOppNearMe()
{	seumath::Vector2f ballGlobalPos = WM.getBallGlobalPos2D();
	float nearestTime = WM.getOppFastestToBallTime();
//  	cout<<nearestTime<<endl;
	return nearestTime < 1.0f; //easy to dribble
}

bool TeamPlayer::isCloseToOpp()
{	seumath::Vector2f goal ( Vector2f ( half_field_length,0 ) );
	seumath::Vector2f ball ( WM.getBallGlobalPos2D() );

	seumath::AngDeg goalDir = ( goal-ball ).angle();
	float goalLen = ( goal-ball ).length();

	cout<<goalDir <<'\t'<<goalLen<<endl;
	if ( fabs ( goalDir ) < 50.0f && goalLen <3.0f )
		return true;
	else
		return false;
}
bool TeamPlayer::shouldIBlock()
 {
	if(!WM.IsJoin())
	
	return false;
}

bool TeamPlayer::shouldIShoot()
{
      Vector2f ballGlobalPos = WM.getBallGlobalPos2D(),Len=Vector2f ( half_field_length,0.0f )-ballGlobalPos;

	if ( mKickLock )				//lock kick mode   -- ghd
	///{	if ( ! ( Len.length() >3.0f && Len.length() <9.0f ) )
	{	if ( ! (( Len.length() >3.0f ||Len.length()>=2.5f)&& (Len.length() <9.0f||Len.length()<8.6f )) )
	/// above code to avoid border value but in fact there always be some border
			mKickLock =false;
		else if ( WM.isFall() )
			mKickLock =false;
		else if ( fabs ( ballGlobalPos.y() +2.0f ) <0.5f )
			mKickLock =false;
		///else if ( fabs ( ballGlobalPos.y()  ) <2.5f )
		///    mKickLock =false;	--------------------------
	}

	if ( mKickLock )
	{	return true;
	}
	else
	{
		///if ( fabs ( ballGlobalPos.y() +2.0f ) <0.5f)
		if ( fabs ( ballGlobalPos.y() +2.0f ) <0.5f||fabs(ballGlobalPos.y()+2.0f)<0.4f )			///bug point   --ghd
		///if( fabs ( ballGlobalPos.y() ) <2.5f )
			return false;///------------------------------
		if ( !isOppNearMe() && ( Len.length() >3.0f && Len.length() <9.5f ) )/// for test Len.length() <9.0f
		{	mKickLock =true;
			return true;
		}
		else
		{	return false;
		}
	}
}

shared_ptr<Action> TeamPlayer::goalKeeperBehaviour() //No.1
{	/// Vector2f pos(15.0,0);
	///   return dpfGoTo(pos);
	int choose = 0;
	int numOfOpp = WM.getOppPlayerNumber();

	if ( canInterceptBall ( choose ) )
	{	if ( choose != 0 )
		{	//std::cout<<"fall time: "<<WM.getSimTime()<<std::endl;
			return fallToGetBall ( choose );
		}
		else if ( choose == -1 )
		{	if ( numOfOpp > 1 )
			{	return dribble();
			}
			else
			{	return kickTo ( Vector2f ( half_field_length, 0 ) );
			}
		}
		else
		{	if ( WM.amIFastestToBallOfOurTeam() && numOfOpp > 1 )
			{	return dribble();
			}
			else
			{	return dpfGoTo ( mMyGoodPos );
			}
		}
	}
	else
	{
		Vector2f ballPos = WM.getBallGlobalPos2D();
		Vector2f goalCenter ( -half_field_length, 0 );

		if ( WM.amIFastestToBallOfOurTeam() && numOfOpp > 1 )
		{	return dribble();
		}
		else if ( ( ballPos-goalCenter ).length() < half_goal_width )
		{	if ( numOfOpp > 1 )
			{	return dribble();
			}
			else
			{	return kickTo ( Vector2f ( half_field_length, 0 ) );
			}
		}
		else
		{	return dpfGoTo ( mMyGoodPos );
		}
	}
}

shared_ptr<Action> TeamPlayer::ltyshoot()
{	const Vector2f Len=WM.getBallGlobalPos2D()-Vector2f ( half_field_length,0 );
	const Vector2f leftMidPoint = Vector2f ( half_field_length, 0.4f );
	const Vector2f rightMidPoint = Vector2f ( half_field_length, -0.4f );
	if ( WM.getBallGlobalPos2D().x() < 12.0f && Len.length() >2.5f )
	{	if ( WM.getBallGlobalPos2D().y() < 0.0f )
		{	return kickTo ( leftMidPoint );
		}
		else
		{	return kickTo ( rightMidPoint );
		}
	}
	else return dribble();//return ltydribble();

}

shared_ptr<Action> TeamPlayer::ltydribble ( Vector2f aim )
{	Vector2f mypos=WM.getMyGlobalPos2D(),ballpos=WM.getBallGlobalPos2D();
	Vector2f Len=mypos-ballpos,dir=ballpos-aim;
	float L=dir.length();
	if ( ballpos.x() >11.0f || Len.x() >0.07 ) return dribble();
	if ( ( Len ).length() >0.2f ) return goTo ( ballpos+dir/L*0.1f,0 );
	else  return goTo ( ballpos-dir/L*0.25f,0,0 );
}

shared_ptr<Action> TeamPlayer::attackerCentralBehaviour() //10
{	return defaultBehaviour();
	//return dpfGoTo(WM.getBallGlobalPos2D() + Vector2f(-1.0f,0));
// 	return dpfGoTo(mMyGoodPos);
// 	return goTo(Vector2f(half_field_length ,0 ),0,0);
// 	return goToRel(Vector2f(0,1),0);
// 	return dynamicKick(Vector2f(half_field_length,0));
	if ( isMyBallOther() )
	{
//	  return dpfGoTo(WM.getBallGlobalPos2D() + Vector2f(-1.0f,0));
		if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{
	 /// if()
	  return dpfGoTo ( mMyGoodPos );
	}
}

shared_ptr<Action> TeamPlayer::attackerLeftWingBehaviour() //9
{	return defaultBehaviour();
// 	return dribble();
// 	return kickTo(Vector2f(half_field_length,0));
// 	return goTo(Vector2f(half_field_length ,3 ),0,0);
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}
}

shared_ptr<Action> TeamPlayer::attackerRightWingBehaviour() //11
{
// 	return kickTo(Vector2f(half_field_length,0));
//   	return dpfGoTo(WM.getBallGlobalPos2D() + Vector2f(-1.0f,0));
// 	return goTo(Vector2f(half_field_length ,6 ),0,0);
// 	return goToRel(Vector2f(0,1),0);
/// for test  
///	return shoot();
///for test 
	if ( WM.getSimTime()  < 5 )
	{	return dpfGoTo ( Vector2f ( WM.getMyGlobalPos2D().x() - 3, 0 ) );
	}
	else
	{	return defaultBehaviour();
		if ( isMyBallOther() )
		{
//		return dpfGoTo(WM.getBallGlobalPos2D() + Vector2f(-1.0f,0));
			if ( shouldIShoot() )
			{	return shoot();
			}
// 		  else if(isCloseToOpp())
// 		  {
// 		      return dynamicKick(Vector2f(half_field_length,0));
// 		  }
			else
			{	return dribble();
			}
		}
		else
		{	return dpfGoTo ( mMyGoodPos );
		}
	}
}

shared_ptr<Action> TeamPlayer::middleFielderCenterBehaviour() //7
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::middleFielderLeftBehaviour() //6
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::defenderLeftWingBehaviour() //2
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	// 	return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::middleFielderRightBehaviour() //8
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::defenderRightWingBehaviour() //5
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::defenderRightBehaviour() //4
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::defenderLeftBehaviour() //3
{	return defaultBehaviour();
	if ( isMyBallOther() )
	{	if ( shouldIShoot() )
		{	return shoot();
		}
		else
		{	return dribble();
		}
	}
	else
	{	return dpfGoTo ( mMyGoodPos );
	}

	//return defaultBehaviour();
}

shared_ptr<Action> TeamPlayer::playOurDeadBall()
{	shared_ptr<Action> act;
	switch ( FM.getMy().type )
	{	case configuration::Formation::PT_GOALKEEPER:
			updateMyGoodPos();
			//act = goTo(getMyGoodPos(),0);
			act = dpfGoTo ( mMyGoodPos );
			break;

		default:
			updateMyGoodPos();
			if ( WM.amIFastestToBallOfOurTeam() )
			{	act = dribble();
			}
			else
			{	//act = goTo(getMyGoodPos(),0);
				act = dpfGoTo ( mMyGoodPos );
			}

			break;
	}

	return act;
}

shared_ptr<Action> TeamPlayer::playOppDeadBall()
{	shared_ptr<Action> act;
	switch ( FM.getMy().type )
	{	case configuration::Formation::PT_GOALKEEPER:
		{
//                Vector2f goalLeft(-half_field_length, half_goal_width);
//                Vector2f goalRight(-half_field_length, -half_goal_width);
//                const Vector2f& posBall = WM.getBallGlobalPos2D();
//                float distL = (goalLeft - posBall).length();
//                float distR = (goalRight - posBall).length();
//                float dist = distL > distR ? distR : distL;
//                AngDeg angL = (goalLeft - posBall).angle();
//                AngDeg angR = (goalRight - posBall).angle();
//                AngDeg ang = calClipAng(angL, angR)*0.5f;
//                AngDeg dir = calBisectorTwoAngles(angL, angR);
//                dist *= cosDeg(ang);
//                dist -= 0.5f;
//                Vector2f p = posBall + pol2xyz(Vector2f(dist, dir));
//                p.y() = clamp(p.y(), -half_goal_width + 0.055f, half_goal_width - 0.055f);
//                act = goTo(p, posBall);
			updateMyGoodPos();
			act = goTo ( getMyGoodPos(),0 );
			break;
		}
		default:
		{	if ( isMyBallOther() )
			{	const Vector2f& posBall = WM.getBallGlobalPos2D();
				const Vector2f goal ( -half_field_length, 0 );
				AngDeg ang = ( goal - posBall ).angle();
				Vector2f p = pol2xyz ( Vector2f ( free_kick_distance + 0.5f, ang ) ) + posBall;
				act = goTo ( getMyGoodPos(),0 );
			}
			else
			{	updateMyGoodPos();
				act = goTo ( getMyGoodPos(),0 );
			}
			break;
		}
	}
	return act;
}

bool TeamPlayer::canInterceptBall ( int& interceptType )
{	Vector2f ballVec = * ( seumath::Vector2f* ) WM.getBallGlobalVel().get();
	float ballVel = ballVec.length();

	//cout<<"vel: "<<ballVel<<"\ttime: "<<WM.getSimTime()<<endl;

	if ( WM.isFall() )
	{	// std::cout<<"1"<<std::endl;
		return false;
	}
	else if ( ballVel < 3.0 )
	{	// std::cout<<"2-1"<<std::endl;
		interceptType = 0;
		return false;
	}
	else
	{	Vector2f ballStart = WM.getInterceptBallGlobalPos2D();
		Vector2f myPos = WM.getMyGlobalPos2D();
		//add more predict length
// 	  if(ballVel > 11) {
// 	    ballStop += ballVec * 0.5;
// 	  } else if(ballVel > 8) {
// 	    ballStop += ballVec * 0.3;
// 	  } else if(ballVel > 5) {
// 	    ballStop += ballVec * 0.1;
// 	  }
		///replace predict above by a functiond below
		//function1: 0.0037*x^2 + 0.0054*x -0.0035
		//float weight = (0.0037 * ballVel + 0.0054) * ballVel;
		//function2: -0.00053*x^3 + 0.0127*x^2 - 0.0304*x -6.2450*10^17
//	  float weight = (-0.00053*ballVel*ballVel + 0.0127*ballVel - 0.0304) * ballVel;
		float weight = ( -0.00053 * pow2 ( ballVel ) + 0.0127*ballVel - 0.0304 ) * ballVel;
		Vector2f ballStop;
		ballStop = ballStart + ballVec * weight;
		// cout<<"ball stop: "<<ballStop<<std::endl;
		// cout<<"ball pos: "<<WM.getBallGlobalPos2D()<<std::endl;

		///a new predict
		float predictY = ballStart.y() + ( ballStop.y()-ballStart.y() ) * ( myPos.x()-ballStart.x() ) / ( ballStop.x() - ballStart.x() );
		//cout<<"y: "<<predictY<<endl;
		//if( ballStop.x() < -13 && fabs(ballStop.y()) < half_goal_width*1.5 ) {
		if ( ballStop.x() < -13 && fabs ( predictY ) < half_goal_width*1.5 )
		{
// 	    if(ballVel < 2.0) {
// 	      std::cout<<"2-1"<<std::endl;
// 	      interceptType = 0;
// 	      return true;
// 	    }

			if ( ballStop.x() > ballStart.x() )
			{	return false;
			}

			if ( ballStart.x() < WM.getMyGlobalPos2D().x() )
			{	return false;
			}

			if ( ( ballStart-WM.getMyGlobalPos2D() ).length() < WM.getOppFastestToBallDistance() )
			{	// std::cout<<"2-1"<<std::endl;
				interceptType = -1;
				return true;
			}
// 	    if(myPos.x()> -13.5 && fabs(myPos.y()) > half_goal_width) {
// 	      std::cout<<"2-3"<<std::endl;
// 	      //interceptType = 0;
// 	      return false;
// 	    }

			//Vector2f ballStopRel = WM.transGlobalPosToRelPos(ballStop);
			//float posX = ballStopRel.x();
			///replace judgment above(rel) by below(global)
			float posY = predictY - WM.getMyGlobalPos2D().y();
			if ( posY > 1 )
			{	interceptType = 0;
			}
			else if ( posY > 0.6f )
			{	interceptType = -2;
			}
			else if ( posY > 0.0f )
			{	interceptType = -1;
			}
			else if ( posY > -0.6f )
			{	interceptType = 1;
			}
			else if ( posY > -1 )
			{	interceptType = 2;
			} /*else if( posY > 0.0f ) {
		interceptType = -1;
	    } else if( posY > -1 ) {
		interceptType = 1;
	    }*/ else
			{	interceptType = 0;
			}
			if ( interceptType != 0 )
			{	return true;
			}
			else
			{	return false;
			}
		}
		else
		{	// std::cout<<"4"<<std::endl;
			return false;
		}
	}
}
float TeamPlayer::calBlockDis ( Vector2f oppPos, Vector2f oppVector )
{	Vector2f myPos=WM.getMyGlobalPos2D();
	Vector2f oppToMe ( myPos.x()-oppPos.x(),myPos.y()-oppPos.y() );
	float deltaAngle=fabs ( oppVector.angle()-oppToMe.angle() );
	if ( deltaAngle>180 )
		deltaAngle=360-deltaAngle;
	float dis= ( myPos-oppPos ).length();
	float xDis=dis*cos ( deltaAngle*3.141592653/180 );
	if ( xDis>=5 )
		return 4.9f;
	else if ( xDis>=4 )
		return 3.9f;
	else if ( xDis>=3 )
		return 2.9f;
	else if ( xDis>=2 )
		return 1.9f;
	else
	{	return 0.8f*xDis;
	}
	return 0;
}

} // namespace soccer
