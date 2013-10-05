/* 
 * File:   SayAndHearModel.h
 * Author: robocup
 *
 * Created on 7/1/2011 14:13
 */

#ifndef SAYANDHEARMODEL_H
#define	SAYANDHEARMODEL_H

#include "core/WorldModel.h"
#include "core/PassModel.h"
#include "math/Math.hpp"
#include "Singleton.hpp"
#include <string>
#include<iostream>

namespace core {
  
#define dynamic_no_me		100	//ghd
  
    using namespace std;
using namespace core;
    class SayAndHearModel : public Singleton<SayAndHearModel> {
    public:

        SayAndHearModel() : mCycles(0), mCanSay(false), mIsHeard(false), mBase(80) {
            mSayString = "";
        };

        virtual ~SayAndHearModel() {
        };

        //ascii　range 42~126 '*'~'~' 共85个
	
// 	unsigned int mHearCastType;
	
        void DecToEightyFifth(const int orig, char& high, char& low) {
            //   cout << orig <<endl;
            high = (char) (orig / mBase) + '*';
            //  cout <<"dech"<< (int)high<<endl;
            low = (char) (orig % mBase) + '*';
            // cout <<"decl"<< (int)low<<endl;
        }

        int formatFloat(float orig) {
            int res = std::floor(orig * 100);
            res += (mBase * mBase / 2);
            return res;
        }

        float resolveFloat(int orig) {
            float res = orig - (mBase * mBase / 2);
            res = res / 100.0f;
            return res;
        }

        int formatInt(int orig) {
            return orig + (mBase * mBase / 2);
        }

        char transIntToChar(int i)		//add by ghd
	{
	    char temp =char(i);
	    return temp;
	}
	
	int transCharToInt(char c)		//add by ghd
	{
	    return int(c);
	}
        
        void calcOurGoodPos(char* send,std::vector<int> pos);	//ghd
	
	//yao 2013/06/28
	//a new encode way to make message to send cast
	void calcOurGoodPosYao(char* send,std::vector<int> pos); //yao 2013/06/28
	
	unsigned int resolveOurGoodPos(char * recive);//ghd
	
	unsigned int resolveOurGoodPosAndStore(char *recive);  //test by yao 2013/06/25
	
	//a new parse way to get out cast
	unsigned int resolveOurGoodPosAndStoreYao(char *recive);
	
	void calcOurGoodPosMinor(char* send, std::vector<int> pos);
	
	unsigned int resolveOurGoodPosMinor(char* receieve);
	
	int addConditionAndBlockNum();///wh

	int addDribbleOrderAndBlockNnum();///wh
	
	int addUNumAndTwoBool(bool a,bool b);///wh
	
	
	
	void solveConditionAndBlockNum(unsigned int &condition,unsigned int  &blockNum,int res)///wh
	{	blockNum=(res-'*')%12;
		condition=std::floor((res-'*')/12);
	}
	
	void solveDribbleOrderAndBlockNum(unsigned int &dribbleNum, unsigned int &blockNum,int res)///wh
	{	blockNum=(res-'*')%12;
		dribbleNum=std::floor((res-'*')/12);
	}

	void solveUNumAndTwoBool(unsigned int &UNum, bool &a, bool &b, int res);
        
        int resolveInt(int orig) {

            return orig - (mBase * mBase / 2);
        }

        void EightyFifthToDec(const char origHigh, const char origLow, int & res) {

            res = (origHigh - '*') * mBase + (origLow - '*');
        }
        void update();

        float getHearSendTime() const {
            return mHearSendTime;
        }

//         seumath::Vector2f getHearBallPos() const {
//             return mHearBallPos;
//         }
// 
//         bool IsHearPlayerFallen() const {
//             return mHearIsPlayerFallen;
//         }
// 
//         seumath::AngDeg getHearPlayerBodyDirection() const {
//             return mHearPlayerBodyDirection;
//         }
// 
//         seumath::Vector2f getHearPlayerPos() const {
//             return mHearPlayerPos;
//         }

        unsigned int getHearPlayerID() const {
            return mHearPlayerID;
        }

        float getHearTime() const {
            return mHearTime;
        }

        std::string getSayString() const {
            return mSayString;
        }
        
        //-------------------ghd
        unsigned int getHearCastType() const 
	{
	    return mHearCastType;
	}
	
	float getHearPlayerDist() const {
	    return mHearPlayerDist;
	}
	
	//esh
	void setHearCastType(int _mHearCastType)
	{
	   mHearCastType = _mHearCastType;
	}

	unsigned int getHearGoodPosID() const
	{
	    return mHearGoodPosID;
	}

        bool IsCanSay() const {
            return mCanSay;
        }

        bool IsHeard() const {
            return mIsHeard;
        }
        
        bool IsJoin() const {
	    return mJoin;
	}
	
	//add by yao 2013/06/25  test
	vector<int> getHeardOurCast() const {
	    return mHearOurCast;
	}
	
	unsigned int getHearBlockNumber() const
	{	return mHearBlockNum;
	}
	
	//add by yao 2013/06/25 test
	void emptyHeardOurCast();

        void printhear() {

            cout << "mHearString" << mHearString << endl;
            cout << "mHearPlayerID" << mHearPlayerID << endl;
//             cout << "mHearPlayerPos" << mHearPlayerPos << endl;
//             cout << "mHearPlayerBodyDirection" << mHearPlayerBodyDirection << endl;
//             cout << "mHearIsPlayerFallen" << mHearIsPlayerFallen << endl;
//             cout << "mHearBallPos" << mHearBallPos << endl;
            cout << "mHearSendTime" << mHearSendTime << endl;
        }

        void printsay() ;

    private:
        void calcCanSay();
	void calcTimeToBall();
        void makeMsg();
        bool resolveMsg();
        const int mBase;

	int statusToTurnOnDynamicCast;
        //say

        int mCycles;
        bool mCanSay;
        std::string mSayString;


        //hear
        bool mIsHeard;
        float mHearTime;
        std::string mHearString;
	
	//time to ball
	float mTimeToBall;
	
	//dynamic cast
	bool mJoin;

        unsigned int mHearPlayerID; //1
//         seumath::Vector2f mHearPlayerPos; //2-3 4-5
//         seumath::AngDeg mHearPlayerBodyDirection; //6-7
//         bool mHearIsPlayerFallen; //8
        seumath::Vector2f mHearBallPos; //9-10 11-12
        float mHearSendTime; //2-3
        unsigned int mHearCastType; //15
	
        unsigned int mHearGoodPosID;	//16-18
        
        //modify by ghd
        float mHearPlayerDist;	//4-5
        //     int mState;

        //add by yao 2013/06/25  -- test
        //for storing our cast resl-time 
	vector<int> mHearOurCast;
	
	
	//wh
	unsigned int mHearCondition;
	unsigned int mHearBlockNum;
	unsigned int mHearDribbleNum;
	bool mHearDribbleCondition;
	vector<bool> mHearAllDribbleCondition;
    };
}
#define SHM core::SayAndHearModel::GetSingleton()
#endif	/* SAYANDHEARMODEL_H */

