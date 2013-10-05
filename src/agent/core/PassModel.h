/* 
 * File:   PassModel.h
 * Author: robocup
 *
 * Created on 6/11/2011
 */

#ifndef PASSMODEL_H
#define	PASSMODEL_H
#include "core/WorldModel.h"
#include "core/SayAndHearModel.h"
#include"math/Math.hpp"
namespace core {
#define HISTORY 10

#define dynamic_player_num	5	//ghd

    class VisionRobot {
        friend class PassModel;
    public:

        VisionRobot() :
        mLastSeeTime(-1), mHistoryFull(false), mGlobalOffset(0), mHaveSeen(false), mUnum(100), mIsOur(false),mHaveHeard(false)
        {
            for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
                mIsSeen[i] = false;
            }
        };
        //与全局坐标系X轴夹角
        //   seumath::AngDeg calcGlobalDirection();
        //   bool calcDirection();
        void calcInfo(bool Isopp =false); // update if seen
        const static seumath::Vector3f mErrPol;

        bool getIsFallen() {
            return mIsFallen;
        }

        bool getHaveSeen() {
            return mHaveSeen;
        }
        
        seumath::Vector2f getHearGlobalPos()
	{
	    return mHearGlobalPos;
	}
	
	float getLastHearTime() 
	{
	    return mlastHearTime;
	}
        //每次视觉周期更新
        bool calcGlobalVel();
       //update every vision

          seumath::Vector2f getMGlobalPosAll2D() const {
            return mGlobalPosAll;
        }
          seumath::Vector3f getRelPos() const {
              return mRelPos;
          }
    private:

        seumath::Vector3f rotationPoints(const seumath::Vector3f& p);
        bool calcRotationMat(seumath::AngDeg roll, seumath::AngDeg pitch, seumath::AngDeg yaw);
        bool calcHeight();
        //不一定准确
        void calcIsFacedToMe();
        void calcIsFacedToBall();
        void calcIsWalkedToBall();
        ///////////////////////////////////
        void calcWalkDirection();
        void calcPos();
        void calcTimeToBall();
        seumath::Matrix3x3f mRotationMat;
        seumath::Vector3f mGlobalPos;
        seumath::Vector3f mHistoryGlobalPos[HISTORY];
        bool mHistoryFull;
        bool mHaveSeen;
        int mGlobalOffset;
        seumath::AngDeg mGlobalWalkDirection;

        seumath::Vector3f mRelPos;
        seumath::Vector3f mPolPos;
        seumath::Vector3f mGlobalVel;
        float mHeight;
        bool mIsFallen;

        bool mIsOur;
        bool mIsFacedToMe;
        bool mIsFacedToBall;
        bool mIsWalkedToBall;

        unsigned int mUnum;

        float mTimeToBall;
        seumath::Vector3f mBodyPartPol[perception::Vision::PID_NULL];
        seumath::Vector3f mBodyPartRel[perception::Vision::PID_NULL];
        seumath::Vector3f mBodyPartGlobal[perception::Vision::PID_NULL];
        seumath::Vector3f mBodyPartVisionRel[perception::Vision::PID_NULL];
        bool mIsSeen[perception::Vision::PID_NULL];
        float mLastSeeTime;
        //hear
        seumath::Vector2f mHearGlobalPos;
        seumath::Vector2f mHearBallPos;
        seumath::AngDeg mHearBodyDirection;
        float mlastHearTime;
        bool mHaveHeard;
        bool mHearFallen;
        //both
        seumath::Vector2f mGlobalPosAll;

    };

    class PassModel : public Singleton<PassModel> {
        
    public:
        PassModel();
        //    PassModel(const PassModel& orig);
        ~PassModel();
        void update();
        void updateByListen();
typedef std::map<unsigned int, VisionRobot> TSeenPlayer;
        TSeenPlayer& getOurTeammates() {
            return mOurTeammates;
        }

        TSeenPlayer& getOppTeammates() {
            return mOppTeammates;
        }

        unsigned int getOurFastestID() {
            return mOurFastestID;
        }

        float getOurMinTimeToBall() {
            return mOurMinTimeToBall;
        }

        unsigned int getOppFastestID() {
            return mOppFastestID;
        }

        float getOppMinTimeToBall() {
            return mOppMinTimeToBall;
        }

//         bool choosePass();
// bool canPass();
        unsigned int getBestPassID() const {
            return mBestPassID;
        }
        bool isCanPass() const {
            return mCanPass;
        }

        std::vector<int> getOurCastID()		//ghd
	{
	    return mOurCastPlayerID;
	}
	
	std::vector<int> getOurCast()		//ghd
	{
	    return mOurCast;
	}	
	
	void updateOurCast();			//add by ghd
	
	seumath::Vector2f findMyGoodPos(int posID);			//ghd
	
	void sortPlayer(std::vector<unsigned int> readyPlayer);   //test function by liuyao
	
	bool mIsDriving;	//ghd
    private:
        void updateTeammates(const perception::Vision::TTeamPolMap& ourPol, TSeenPlayer& teammates, bool isOur);
        void calcOurFastestToBall();
        void calcOppFastestToBall();

	void updateOurCastPlayerID();		//add by ghd
	void updateOurTargetPos();		//add by ghd
        
	std::vector<int> calPosition(std::vector<int>& targetIDs, std::vector<int>& agentIDs, const int num);	//add by yzz
 	void genPreference(std::vector<std::vector<int> > &targetPre,std::vector<std::vector<int> > &agentPre,
 					std::vector<int> &targetIDs, std::vector<int>& agentIDs, const int num);
 	void stableMatch(std::vector<int>& targetResult,std::vector<std::vector<int> >& targetPre, std::vector<std::vector<int> >& agentPre,
 					std::vector<int>& agentIDs,const int num);
        float calLen(int targetID,int agentID);
	seumath::Vector2f getAgentPosByID(int id);		//add by yzz
//         bool  isBlocked(unsigned int ourTeammateID);


        std::set<unsigned int> mPassSet;
	
	std::vector<seumath::Vector2f> mTargetPos;	//ghd
	std::vector<int> mTargetID;		//add by ghd
	std::vector<int> mInitCast;		//add by ghd
	std::vector<int> mOurCast;		//add by ghd
	std::vector<int> mOurCastPlayerID;	//add by ghd
	
	unsigned int mOurLastFastestID;        	//add by ghd
	unsigned int mOurNearstID;           	//add by ghd
	
        TSeenPlayer mOurTeammates;
        TSeenPlayer mOppTeammates;

        unsigned int mOurFastestID;
        float mOurMinTimeToBall;
        unsigned int mOppFastestID;
        float mOppMinTimeToBall;

        bool mCanPass;
        float mCanPassTime;
        unsigned int mBestPassID;

    };
}

#define PM core::PassModel::GetSingleton()

#endif	/* PASSMODEL_H */

