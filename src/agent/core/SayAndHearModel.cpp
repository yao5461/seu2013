/*
 * File:   SayAndHearModel.cpp
 * Author: robocup
 *
 * Created on 7/1/2011
 */

#include "SayAndHearModel.h"
#include <stdlib.h>

#include <cmath>
#include <string>
#include <sstream>
#include <vector>
namespace core
{
using namespace std;
using namespace boost;

void SayAndHearModel::update()
{	calcCanSay();
	makeMsg();

	const vector<shared_ptr<perception::Hear> >& hear = WM.lastPerception().hear();
	if ( hear.empty() )
	{	mIsHeard = false;
		return;
	}
	mIsHeard = false;

	FOR_EACH ( iter, hear )
	{	mHearString = ( ( *iter )->message() );
		// ;
		mIsHeard = resolveMsg();
		break;
	}

}

void SayAndHearModel::calcCanSay()
{	int time = std::floor ( WM.getSimTime() *100 );
	int myUnum = WM.getMyUnum();
	const int teamNum = 11;
	int mod = time % 220;    ///180;
	int type = ( mod / 4 ) % teamNum;
	int numMates = WM.getOurPlayerNumber();

	if ( myUnum - 1 == type )
	{	mCanSay = true;
	}
	else
	{	mCanSay = false;
	}

	//mCanSay = false;

}


void SayAndHearModel::calcOurGoodPos ( char* send,std::vector<int> pos )	//add by ghd
{	unsigned int tempInt;
	std::vector<int>::iterator posIter =pos.begin();
	tempInt = ( *posIter ) *10 + ( * ( posIter+1 ) );
	send[0] =char ( tempInt );
	posIter+=2;
	tempInt = ( *posIter ) *10 + ( * ( posIter+1 ) );
	send[1] =char ( tempInt );
	posIter+=2;
	tempInt = ( *posIter ) *10 + ( * ( posIter+1 ) );
	send[2] =char ( tempInt );
}

void SayAndHearModel::calcOurGoodPosYao ( char* send, std::vector< int > pos )
{	int sum = 0;

	//get the temp number consist of player pos id
	for ( vector<int>::iterator iter = pos.begin(); iter != pos.end(); iter++ )
	{	sum *= 10;
		sum += ( ( *iter ) - 2 );
	}

//	cout<<"!!!"<<sum<<endl;

	send[2] = ( char ) ( ( sum % 80 ) + 42 ); //plus '*'
	sum /= 80;
	send[1] = ( char ) ( ( sum % 80 ) + 42 ); //plus  '*'
	sum /= 80;
	send[0] = ( char ) ( sum + 42 ); //plus '*'

}


unsigned int SayAndHearModel::resolveOurGoodPos ( char* recive )	//ghd
{	unsigned int tempInt;
	unsigned int x[3];
	unsigned int myID =WM.getMyUnum();
	for ( int i =0; i<3; i++ )
	{	x[i] =int ( recive[i] );
	}

	//type
	tempInt =x[2]%10;
	mHearCastType =tempInt -2;

	//tempInt: 2 ~ 11	5 player
	//byte 1
	tempInt =x[0]%10;
	if ( tempInt <2 ) tempInt +=10;
	if ( tempInt ==myID ) return 1;
	else
	{	x[0] -=tempInt;
		x[0] /=10;
		if ( x[0] ==myID ) return 0;
	}
	//byte 2
	tempInt =x[1]%10;
	if ( tempInt <2 ) tempInt +=10;
	if ( tempInt ==myID ) return 3;
	else
	{	x[1] -=tempInt;
		x[1] /=10;
		if ( x[1] ==myID ) return 2;
	}
	//byte 3
	tempInt =x[2]%10;
	x[2] -=tempInt;
	x[2] /=10;
	if ( x[2] ==myID ) return 4;

	return dynamic_no_me;

}


unsigned int SayAndHearModel::resolveOurGoodPosAndStore ( char* recive )
{	unsigned int tempInt;
	unsigned int x[3];
	unsigned int myID =WM.getMyUnum();
	unsigned int result = dynamic_no_me;

	for ( int i =0; i<3; i++ )
	{	x[i] =int ( recive[i] );
	}

	//type
	tempInt =x[2]%10;
	mHearCastType =tempInt -2;

	//tempInt: 2 ~ 11	5 player
	//clear buffer for heard cast
	this->mHearOurCast.clear();

	///////  byte 1
	tempInt =x[0]%10;
	if ( tempInt < 2 )
	{	tempInt +=10;
	}
	if ( tempInt == myID )
	{	result = 1;
	}

	x[0] -=tempInt;
	x[0] /=10;
	if ( x[0] == myID )
	{	result = 0;
	}
	//insert 0
	if ( x[0] > 1 && x[0] < 12 )
	{	this->mHearOurCast.push_back ( x[0] );
	}
	//insert 1
	if ( tempInt > 1 && tempInt < 12 )
	{	this->mHearOurCast.push_back ( tempInt );
	}

	//////  byte 2
	tempInt =x[1]%10;
	if ( tempInt < 2 )
	{	tempInt +=10;
	}
	if ( tempInt == myID )
	{	result = 3;
	}

	x[1] -=tempInt;
	x[1] /=10;
	if ( x[1] == myID )
	{	result = 2;
	}
	//insert 2
	if ( x[0] > 1 && x[0] < 12 )
	{	this->mHearOurCast.push_back ( x[1] );
	}
	//insert 3
	if ( tempInt > 1 && tempInt < 12 )
	{	this->mHearOurCast.push_back ( tempInt );
	}

	//byte 3
	tempInt =x[2]%10;
	x[2] -=tempInt;
	x[2] /=10;

	if ( x[2] == myID )
	{	result = 4;
	}
	//insert 4
	if ( x[2] > 1 && x[2] < 12 )
	{	this->mHearOurCast.push_back ( x[2] );
	}

	//for test output
// 	cout<<"h: ";
// 	for(vector<int>::iterator iter = this->mHearOurCast.begin(); iter != this->mHearOurCast.end(); iter++) {
// 	  cout<<(*iter)<<"\t";
// 	}
// 	cout<<endl;

	return result;
}


unsigned int SayAndHearModel::resolveOurGoodPosAndStoreYao ( char* recive )
{	unsigned int tempInt;
	unsigned int myID =WM.getMyUnum();
	unsigned int result = dynamic_no_me;

	//raw num consist of number
	int sum = 0;
	sum = sum + ( int ) ( recive[0] - 42 ) * 6400 ;
	//sum *= 80;
	sum = sum + ( int ) ( recive[1] - 42 ) * 80;
	//sum *= 80;
	sum = sum + ( int ) ( recive[2] - 42 );

//	cout<<"!!"<<sum<<endl;

	//tempInt: 2 ~ 11	5 player
	//clear buffer for heard cast
	this->mHearOurCast.clear();


	for ( int i = 0; i < 5; i++ )
	{	//get player id
		tempInt = ( sum % 10 ) + 2;
		sum /= 10;

		//store
		if ( tempInt > 1 && tempInt < 12 )
		{	this->mHearOurCast.insert ( mHearOurCast.begin(), tempInt );
		}

		//if it is me
		if ( tempInt == myID )
		{	result = 4 - i;
		}
	}

	//for test output
// 	cout<<"h: ";
// 	for(vector<int>::iterator iter = this->mHearOurCast.begin(); iter != this->mHearOurCast.end(); iter++) {
// 	  cout<<(*iter)<<"\t";
// 	}
// 	cout<<endl;

	return result;
}


void SayAndHearModel::calcOurGoodPosMinor ( char* send, std::vector<int> pos )
{	unsigned int tempInt;
	std::vector<int>::iterator posIter =pos.begin();
	//first sendID
	tempInt = *posIter;
	send[0] = char ( tempInt );
	posIter++;
	//second sendID
	tempInt = *posIter;
	send[1] = char ( tempInt );
	posIter++;
	//third sendID
	tempInt = *posIter;
	send[2] = char ( tempInt );
	posIter++;
	//send castType
}

unsigned int SayAndHearModel::resolveOurGoodPosMinor ( char* recieve )
{	unsigned int tempInt;
	unsigned int receivePos[4];
	unsigned int myID =WM.getMyUnum();

	for ( int i=0; i<4; i++ )
	{	receivePos[i] = ( int ) ( recieve[i] );
	}

	tempInt = receivePos[3];
	mHearCastType = tempInt-2;

	tempInt = receivePos[0];
	if ( tempInt == myID )
		return 0;

	tempInt = receivePos[1];
	if ( tempInt == myID )
		return 1;

	tempInt = receivePos[2];
	if ( tempInt == myID )
		return 2;


}

void SayAndHearModel::calcTimeToBall()
{	const float speed = 0.4f;
	mTimeToBall = 0.0f;
	float dist;
	seumath::Vector2f ballRelPos = WM.getBallRelPos2D();
	dist = ballRelPos .length();
	mTimeToBall += dist / speed;
//         if (dist.length()<0.5f){	//ghd
// 	  mTimeToBall=0.0f;
// 	  return;
//
// 	}

	if ( WM.isFall() )
	{
///	  mTimeToBall += 3.5f; ////add time for get up
	  mTimeToBall +=4.1f;
	}
	//下面原是根据角度增加到球时间,现在不用
	float balldir = ( seumath::Vector2f ( serversetting::half_field_length,0 ) - WM.getBallGlobalPos2D() ).angle(); // modify by ghd
	//float oppdir =(Vector2f(half_field_length,0) - mHearGlobalPos).angle();
	//cout<< abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f <<endl;
	if ( WM.getBallGlobalPos2D().x() >0.0f )
		mTimeToBall += abs ( seumath::normalizeAngle ( balldir - WM.getMyBodyDirection() ) ) / 180.0f * 5.0f;
	else mTimeToBall += abs ( seumath::normalizeAngle ( balldir - WM.getMyBodyDirection() ) ) / 180.0f * 3.0f;


//下面部分是增加在球前方的球员的时间惩罚
	if ( WM.getBallGlobalPos2D().x() < WM.getMyGlobalPos2D().x() )
	{	mTimeToBall += 4.0f * ( ( WM.getMyGlobalPos2D().x() - WM.getBallGlobalPos2D().x() ) / speed );
	}
	else if ( WM.getBallGlobalPos2D().x() < WM.getMyGlobalPos2D().x() )
	{	mTimeToBall += 4.0f * ( ( WM.getMyGlobalPos2D().x()-WM.getBallGlobalPos2D().x() ) / speed );
	}
}

void SayAndHearModel::makeMsg()
{	mSayString = "";
	std::stringstream ss;
	int tempInt = 0;
	char tempH;
	char tempL;
	//playerID
	// tempInt = formatInt(WM.getMyUnum());
	//  DecToEightyFifth(tempInt, tempH, tempL);
	ss << ( char ) ( WM.getMyUnum() + '*' );

	//modify by ghd
	calcTimeToBall();

	tempInt = std::floor ( mTimeToBall * 10 );
	tempInt += ( mBase * mBase / 2 );
	DecToEightyFifth ( tempInt,tempH,tempL );
	ss<<tempH<<tempL;
	//player pos x,y
	/* tempInt = formatFloat(WM.getMyGlobalPos().x());
	 DecToEightyFifth(tempInt, tempH, tempL);
	 ss << tempH << tempL;

	 tempInt = formatFloat(WM.getMyGlobalPos().y());
	 DecToEightyFifth(tempInt, tempH, tempL);
	 ss << tempH << tempL;
	 //player bodydir
	 tempInt = formatFloat(WM.getMyBodyDirection() / 10.0f);
	 DecToEightyFifth(tempInt, tempH, tempL);
	 ss << tempH << tempL;
	 //player is fallen
	 if (WM.isFall()) {
	     ss << '1';
	 } else {
	     ss << '0';
	 }*/
	//now time
	tempInt = formatFloat ( WM.getBallGlobalPos().x() );
	DecToEightyFifth ( tempInt, tempH, tempL );
	ss << tempH << tempL;
	tempInt = formatFloat ( WM.getBallGlobalPos().y() );
	DecToEightyFifth ( tempInt, tempH, tempL );
	ss << tempH << tempL;

	float time = WM.getSimTime() - std::floor ( WM.getSimTime() / 10 ) *10;
	tempInt = formatFloat ( time );
	DecToEightyFifth ( tempInt, tempH, tempL );
	ss << tempH << tempL;

	/*this->statusToTurnOnDynamicCast = WM.getTheStatusToTurnOnDynamicCast();
	*/
	if ( PM.mIsDriving )
	{	unsigned int castType =WM.getOurState();		//add by ghd ,choose state
		std::vector<int> mGoodPosID ( PM.getOurCast() );
		//mGoodPosID.push_back(castType+2);

//  	    char send[4];		//add by ghd ,to send pos
		char send[3];
		//calcOurGoodPos(send,mGoodPosID);
		calcOurGoodPosYao ( send, mGoodPosID );

		//calcOurGoodPosMinor(send,mGoodPosID);
		//cout<<int(send[0])<<'\t'<<int(send[1])<<'\t'<<int(send[2])<<endl;
//  	    ss << send[0] << send[1] << send[2] <<send[3];


		//modify by yao 2013/06/28  in Netherlands
		bool flag = true;

		for ( int i = 0; i < 3; i++ )
		{	if ( send[i] < 42 || send[i] > 126 )
			{	//send[i] = 43;
				flag = false;
			}
		}

//	    cout<<"cast: "<<(int)send[0]<<'\t'<<(int)send[1]<<'\t'<<(int)send[2]<<endl;
		if ( flag )
		{	ss<<send[0]<<send[1]<<send[2];
		}
	}
	else if ( WM.getMyUnum() ==1 )
	{	tempH=WM.calOurBlockNumber() +42;
		ss<<tempH;
	}

	//the end
	ss<< '~';
	mSayString = ss.str();
}

bool SayAndHearModel::resolveMsg()
{	const int offset = 2;
	if ( mHearString.size() < offset + 6 )
	{	return false;
	}
	int tempInt = 0;
	char tempH;
	char tempL;
	tempL = mHearString[offset + 0];

	mHearPlayerID = tempL - '*';

	tempH =mHearString[offset +1];
	tempL =mHearString[offset +2];
	EightyFifthToDec ( tempH, tempL, tempInt );
	mHearPlayerDist = tempInt - ( mBase * mBase / 2 );
	mHearPlayerDist /= 10.0f;

// 	cout<<mHearPlayerID<<'\t'<<mHearPlayerDist<<endl;
	/*
	        tempH = mHearString[offset + 1];
	        tempL = mHearString[offset + 2];
	        EightyFifthToDec(tempH, tempL, tempInt);
	        mHearPlayerPos.x() = resolveFloat(tempInt);

	        tempH = mHearString[offset + 3];
	        tempL = mHearString[offset + 4];
	        EightyFifthToDec(tempH, tempL, tempInt);
	        mHearPlayerPos.y() = resolveFloat(tempInt);

	        tempH = mHearString[offset + 5];
	        tempL = mHearString[offset + 6];
	        EightyFifthToDec(tempH, tempL, tempInt);
	        mHearPlayerBodyDirection = resolveFloat(tempInt)*10.0f;
	        ;

	        tempH = mHearString[offset + 7];
	        if (tempH == '1') {
	            mHearIsPlayerFallen = true;
	        } else {
	            mHearIsPlayerFallen = false;
	        }*/
	tempH = mHearString[offset + 3];
	tempL = mHearString[offset + 4];
	EightyFifthToDec ( tempH, tempL, tempInt );
	mHearBallPos.x() = resolveFloat ( tempInt );

	tempH = mHearString[offset + 5];
	tempL = mHearString[offset + 6];
	EightyFifthToDec ( tempH, tempL, tempInt );
	mHearBallPos.y() = resolveFloat ( tempInt );

	tempH = mHearString[offset + 7];
	tempL = mHearString[offset + 8];
	EightyFifthToDec ( tempH, tempL, tempInt );
	mHearSendTime = resolveFloat ( tempInt );

	if ( mHearPlayerID !=WM.getMyUnum() && mHearString[offset +9]!='~' && WM.getMyUnum() >=1 )
	{	if ( mHearString[offset+10]=='~' )
		{	mHearBlockNum=mHearString[offset+9]-42;
		}
		else
		{	char recive[3];
// 	    char recive[3];
			recive[0] = mHearString[offset +9];
			recive[1] = mHearString[offset +10];
			recive[2] = mHearString[offset +11];
			//mHearGoodPosID =resolveOurGoodPos(recive);
			//mHearGoodPosID = resolveOurGoodPosAndStore(recive);
			mHearGoodPosID = resolveOurGoodPosAndStoreYao ( recive );

			if ( mHearGoodPosID != dynamic_no_me )
				mJoin =true;
			else
				mJoin =false;
			//cout<<mHearCastType<<endl;
			// cout<<mHearGoodPosID<<endl;
		}
	}


	return true;

}

void SayAndHearModel::printsay()
{

//            cout << "mSayString" << mSayString << endl;
//            cout << "mSayPlayerID" << WM.getMyUnum() << endl;
//            cout << "mSayPlayerPos" << WM.getMyGlobalPos2D() << endl;
//            cout << "mSayPlayerBodyDirection" << WM.getMyBodyDirection() << endl;
//            cout << "mSayIsPlayerFallen" << WM.isFall() << endl;
//            cout << "mSayBallPos" << WM.getBallGlobalPos() << endl;
//            cout << "mSaySendTime" << WM.getSimTime() << " " << WM.getSimTime() - std::floor(WM.getSimTime() / 10)*10 << endl;
}

void SayAndHearModel::emptyHeardOurCast()
{	mHearOurCast.clear();
}

int SayAndHearModel::addConditionAndBlockNum()
{	//int a = WM.calCondition();//0~6
	//int b=WM.calOurBlockNumber();//0~11
	int a=0;
	int b=0;
	int c=b+12*a;
	c+='*';
	return c;
}
int SayAndHearModel::addDribbleOrderAndBlockNnum()
{	//int a = WM.calDribbleNum();//0~6
	int a=0;
	int b = WM.calOurBlockNumber();//0~11
	int c = b+12*a;
	c+='*';
	return c;

}

int SayAndHearModel::addUNumAndTwoBool ( bool a, bool b )
{	unsigned int UNum=WM.getMyUnum()-1;

	int temp=0;
	temp+=UNum;
	//return temp+42;
	temp+= ( ( a==true?1:0 ) *11 );
	temp+= ( ( b==true?1:0 ) *2*11 );
	temp+=42;
	return temp;
}

void SayAndHearModel::solveUNumAndTwoBool ( unsigned int& UNum, bool& a, bool& b, int res )
{	res-=42;
	UNum=res%11;
	res-=UNum;
	res/=11;
	a=res%2;
	res-=a;
	res/=2;
	b=res;
	UNum+=1;
}

}
