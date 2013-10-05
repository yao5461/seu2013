/* 
 * File:   PassModel.cpp
 * Author: robocup
 * test source code for passing ball, not very good to use now
 * Created on 6/11/2011
 */

#include <algorithm>
#include <set>
#include <vector>
#include <queue>
#include "PassModel.h"
#include "core/WorldModel.h"

namespace core {
    using namespace perception;
    using namespace boost;
    using namespace std;
    using namespace seumath;
    const Vector3f VisionRobot::mErrPol(-1, -1, -1);

    void VisionRobot::calcInfo(bool isOpp) {
        mHaveSeen = true;
        const TransMatrixf& eyeMat = WM.getVisionTrans();

        seumath::AngDeg pitch = -WM.lastPerception().joints()[1].angle();
        calcRotationMat(0, pitch, 0);
        for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
            if (mIsSeen[i]) {

                mBodyPartGlobal[i] = eyeMat.transform(Vision::calLocalRelPos(mBodyPartPol[i]));
                // mBodyPartGlobal[i] = PM.rotationPoints(pol2xyz(mBodyPartPol[i]));
                mBodyPartRel[i] = WM.calObjRelPos2D(mBodyPartPol[i]);
                mBodyPartVisionRel[i] = rotationPoints(pol2xyz(mBodyPartPol[i]));
            } else {
                mBodyPartGlobal[i] = mErrPol;
                mBodyPartRel[i] = mErrPol;
                mBodyPartVisionRel[i] = mErrPol;
            }
        }
        if (WM.getMyUnum() == mUnum) {
            mIsSeen[Vision::HEAD] = true;
            mBodyPartGlobal[Vision::HEAD] = WM.getMyGlobalPos();
            mBodyPartRel[Vision::HEAD].x() = 0.0f;
            mBodyPartRel[Vision::HEAD].y() = 0.0f;
            mBodyPartRel[Vision::HEAD].z() = 0.0f;
            mBodyPartVisionRel[Vision::HEAD] = mBodyPartRel[Vision::HEAD];
        }
        
        if(isOpp)
	{
	  calcHeight();
	  calcPos();
	  calcIsFacedToMe();
	  calcIsFacedToBall();
	  calcTimeToBall();
	}
        //        
        //                               Vector3f h = Vision::calLocalRelPos(iterOur->second.find(Vision::HEAD)->second);
        //                        mOurTeammates[iterOur->first].mPolPos = iterOur->second.find(Vision::HEAD)->second;
        //                       mOurTeammates[iterOur->first].mGlobalPos = eyeMat.transform(h);
        //                       mOurTeammates[iterOur->first].mRelPos = WM.calObjRelPos2D(mOurTeammates[iterOur->first].mPolPos);
        //         eyeMat.transform(p)calObjRelPos2D
    }

    bool VisionRobot::calcHeight() {
        const float defaultHeight = 0.51f;
        float buttom = 0.0f;
        mHeight = defaultHeight;
        if (WM.getMyUnum() == mUnum) {
            mHeight = defaultHeight;
            mIsFallen = false;
            return true;
        }

        if (mIsSeen[Vision::L_FOOT] || mIsSeen[Vision::R_FOOT]) {
            if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
                buttom = std::min(mBodyPartVisionRel[Vision::L_FOOT].z(), mBodyPartVisionRel[Vision::R_FOOT].z());
            } else if (mIsSeen[Vision::L_FOOT]) {
                buttom = mBodyPartVisionRel[Vision::L_FOOT].z();
            } else {
                buttom = mBodyPartVisionRel[Vision::R_FOOT].z();
            }

        }
        if (mIsSeen[Vision::HEAD]) {
            mHeight = mBodyPartVisionRel[Vision::HEAD].z() - buttom;
        } else if (mIsSeen[Vision::L_HAND] || mIsSeen[Vision::R_HAND]) {
            float seeHeight = 0;
            if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
                seeHeight = std::min(mBodyPartVisionRel[Vision::L_HAND].z(), mBodyPartVisionRel[Vision::R_HAND].z()) - buttom;
            } else if (mIsSeen[Vision::L_HAND]) {
                seeHeight = mBodyPartVisionRel[Vision::L_HAND].z() - buttom;
            } else {
                seeHeight = mBodyPartVisionRel[Vision::R_HAND].z() - buttom;
            }
            mHeight += seeHeight;
            mHeight = mHeight * 1.5f;
        } else {
            mHeight = defaultHeight;
            return false;
        }
        if (mIsOur && mHaveHeard) {

            if (WM.getSimTime() - mlastHearTime < 0.5f) {
                mIsFallen = mHearFallen;
             return true;
            }

        }
//不考虑视觉了
        if (mHeight < 0.3) {
            mIsFallen = false;
        } else {
            mIsFallen = false;
        }
        return true;
    }

    void VisionRobot::calcPos() {
        if (WM.getMyUnum() == mUnum) {

            mGlobalPos = WM.getMyGlobalPos();
            mGlobalPosAll =  WM.getMyGlobalPos2D();
            mRelPos.x() = 0.0f;
            mRelPos.y() = 0.0f;
            mRelPos.z() = 0.0f;
            mGlobalPos.z() = mHeight / 2;
            mRelPos.z() = mHeight / 2;
            return;

        }
        if (mIsSeen[Vision::HEAD]) {
            mGlobalPos = mBodyPartGlobal[Vision::HEAD];
            mRelPos = mBodyPartRel[Vision::HEAD];
        } else if (mIsSeen[Vision::L_FOOT] || mIsSeen[Vision::R_FOOT]) {
            if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
                mGlobalPos = (mBodyPartGlobal[Vision::L_FOOT] + mBodyPartGlobal[Vision::R_FOOT]) / 2;
                mRelPos = (mBodyPartRel[Vision::L_FOOT] + mBodyPartRel[Vision::R_FOOT]) / 2;
            } else if (mIsSeen[Vision::L_FOOT]) {
                mGlobalPos = mBodyPartGlobal[Vision::L_FOOT];
                mRelPos = mBodyPartGlobal[Vision::L_FOOT];
            } else {
                mGlobalPos = mBodyPartGlobal[Vision::R_FOOT];
                mRelPos = mBodyPartGlobal[Vision::R_FOOT];
            }

        } else if (mIsSeen[Vision::L_HAND] || mIsSeen[Vision::R_HAND]) {
            if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
                mGlobalPos = (mBodyPartGlobal[Vision::L_HAND] + mBodyPartGlobal[Vision::R_HAND]) / 2;
                mRelPos = (mBodyPartRel[Vision::L_HAND] + mBodyPartRel[Vision::R_HAND]) / 2;
            } else if (mIsSeen[Vision::L_HAND]) {
                mGlobalPos = mBodyPartGlobal[Vision::L_HAND];
                mRelPos = mBodyPartGlobal[Vision::L_HAND];
            } else {
                mGlobalPos = mBodyPartGlobal[Vision::R_HAND];
                mRelPos = mBodyPartGlobal[Vision::R_HAND];
            }
        }
        if(!mHaveHeard&&WM.getSimTime() - mlastHearTime > 1.0f){
            mGlobalPosAll.set(mGlobalPos.x(),mGlobalPos.y());
        }
        mGlobalPos.z() = mHeight / 2;
        mRelPos.z() = mHeight / 2;
    }

    void VisionRobot::calcIsFacedToMe() {
        mIsFacedToMe = false;
        if (WM.getMyUnum() == mUnum) {
            return;
        }
        if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_FOOT].y() < mBodyPartPol[Vision::R_FOOT].y());
        } else if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_FOOT].y() < mBodyPartPol[Vision::R_HAND].y());
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_HAND].y() < mBodyPartPol[Vision::R_HAND].y());
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_HAND].y() < mBodyPartPol[Vision::R_FOOT].y());
        }
    }

    void VisionRobot::calcIsFacedToBall() {
        mIsFacedToBall = false;
        Vector2f ballRel = WM.getBallRelPos2D();
        if (WM.getMyUnum() == mUnum) {
            float ToBallDir = WM.getBallRelPos2D().angle();
            if (ToBallDir > 0) {
                mIsFacedToBall = true;
            } else {
                mIsFacedToBall = false;
            }
            return;
        }
        Vector2f lfRel(mBodyPartRel[Vision::L_FOOT].x(), mBodyPartRel[Vision::L_FOOT].y());
        lfRel = lfRel - ballRel;
        Vector2f rfRel(mBodyPartRel[Vision::R_FOOT].x(), mBodyPartRel[Vision::R_FOOT].y());
        rfRel = rfRel - ballRel;
        Vector2f lhRel(mBodyPartRel[Vision::L_HAND].x(), mBodyPartRel[Vision::L_HAND].y());
        lhRel = lhRel - ballRel;
        Vector2f rhRel(mBodyPartRel[Vision::R_HAND].x(), mBodyPartRel[Vision::R_HAND].y());
        rhRel = rhRel - ballRel;
        if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToBall = (calClipAng(lfRel, rfRel) < 0);
        } else if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToBall = (calClipAng(lfRel, rhRel) < 0);
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToBall = (calClipAng(lhRel, rhRel) < 0);
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToBall = (calClipAng(lhRel, rfRel) < 0);
        }
    }

    void VisionRobot::calcIsWalkedToBall() {
        seumath::Vector2f globalPos2D(mGlobalPos.x(), mGlobalPos.y());
        seumath::Vector2f globalVel2D(mGlobalVel.x(), mGlobalVel.y());
        seumath::AngDeg toBallDir = (WM.getBallGlobalPos2D() - globalPos2D).angle();
        seumath::AngDeg velDir = globalVel2D.angle();
        if (abs(toBallDir - velDir) < 40) {
            mIsWalkedToBall = true;
        } else {
            mIsWalkedToBall = false;
        }
    }

    bool VisionRobot::calcGlobalVel() {
        mGlobalOffset = mGlobalOffset % HISTORY;
        mHistoryGlobalPos[mGlobalOffset] = mGlobalPos;

        if (mHistoryFull) {
            mGlobalVel = (mGlobalPos - mHistoryGlobalPos[(mGlobalOffset + 1) % HISTORY]) / (HISTORY * 0.02);
        } else {
            if (mGlobalOffset > 0) {
                mGlobalVel = (mGlobalPos - mHistoryGlobalPos[0]) / (mGlobalOffset * 0.02);
            } else {
                mGlobalVel = (mGlobalPos - mHistoryGlobalPos[0]);
            }
            mHistoryFull = true;
        }
        mGlobalOffset++;
        calcWalkDirection();
        calcIsWalkedToBall();

    }

    void VisionRobot::calcWalkDirection() {
        mGlobalWalkDirection = mGlobalVel.angle();
    }

    void VisionRobot::calcTimeToBall() {
        const float speed = 0.4f;
        mTimeToBall = 0.0f;
        seumath::Vector2f dist;
        if (mHaveHeard && WM.getSimTime() - mlastHearTime < 0.5f) {
            dist = (WM.getBallGlobalPos2D() - mHearGlobalPos);
            mTimeToBall += dist.length() / speed;
        }//        if(mHaveSeen&&WM.getSimTime() - mLastSeeTime<0.1f){
            //        seumath::Vector2f ballRelPos = WM.getBallRelPos2D();
            //        seumath::Vector2f relPos(mRelPos.x(), mRelPos.y());
            //         dist = ballRelPos - relPos;
            //        mTimeToBall += dist.length() / speed;
            //        }
            //        else
        else {
            seumath::Vector2f ballRelPos = WM.getBallRelPos2D();
            seumath::Vector2f relPos(mRelPos.x(), mRelPos.y());
            dist = ballRelPos - relPos;
            mTimeToBall += dist.length() / speed;
        }
//         if (dist.length()<0.5f){	//ghd
// 	  mTimeToBall=0.0f;
// 	  return;
// 	  
// 	}

        if (mIsFallen) {
            mTimeToBall += 3.5f; //由2.7改到0.9,没有什么依据
          
        }
        //下面原是根据角度增加到球时间,现在不用
        if (mIsOur == true) {
            /*if (mHaveHeard) {
                if (mlastHearTime - mLastSeeTime>-1.0f) {
                    float balldir = (Vector2f(half_field_length,0) - WM.getBallGlobalPos2D()).angle();  // modify by ghd
		    //float oppdir =(Vector2f(half_field_length,0) - mHearGlobalPos).angle();
		    cout<< abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f <<endl;
                    mTimeToBall += abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f;
                }
            } else {
                if (!mIsFacedToBall) {
                    mTimeToBall += 0.6f;
                }
            }*/
	    float balldir = (Vector2f(serversetting::half_field_length,0) - WM.getBallGlobalPos2D()).angle();  // modify by ghd
	    //float oppdir =(Vector2f(half_field_length,0) - mHearGlobalPos).angle();
	    //cout<< abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f <<endl;
	    if(WM.getBallGlobalPos2D().x() >0.0f)
		mTimeToBall += abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 5.0f;
	    else mTimeToBall += abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f;
        } else {
            if (!mIsFacedToBall) {
                mTimeToBall += 0.6f;
            }
        }


//下面部分是增加在球前方的球员的时间惩罚
        if (mIsOur == true) {
           
                if (mHaveHeard&&(mlastHearTime - mLastSeeTime>0.06f)) {
                    if (WM.getBallGlobalPos2D().x() < mHearGlobalPos.x()) {
                        mTimeToBall += 2.0f * ((mHearGlobalPos.x() - WM.getBallGlobalPos2D().x()) / speed);
                    }           
            } else if (WM.getBallGlobalPos2D().x() < mGlobalPos.x()) {
                mTimeToBall += 2.0f * ((mGlobalPos.x()-WM.getBallGlobalPos2D().x()) / speed);
            }
        }
    }

    bool VisionRobot::calcRotationMat(seumath::AngDeg roll, seumath::AngDeg pitch, seumath::AngDeg yaw) {

        float c1 = cosDeg(roll);
        float s1 = sinDeg(roll);
        float c2 = cosDeg(pitch);
        float s2 = sinDeg(pitch);
        float c3 = cosDeg(yaw);
        float s3 = sinDeg(yaw);

        mRotationMat[0].set(
                c3*c2, -s3 * c1 + c3 * s2 * s1, s3 * s1 + c3 * s2 * c1);
        mRotationMat[1].set(
                s3*c2, c3 * c1 + s3 * s2 * s1, -c3 * s1 + s3 * s2 * c1);
        mRotationMat[2].set(
                -s2, c2 * s1, c2 * c1);
        return true;
    }

    Vector3f VisionRobot::rotationPoints(const Vector3f& p) {

        TMatrix<float, 3, 1 > objXYZEye;

        objXYZEye[0][0] = p.x();
        objXYZEye[1][0] = p.y();
        objXYZEye[2][0] = p.z();
        TMatrix<float, 3, 1 > objXYZ = mRotationMat*objXYZEye;
        return Vector3f(objXYZ[0][0], objXYZ[1][0], objXYZ[2][0]);
    }

    PassModel::PassModel() {
	mOurLastFastestID =0;
	mInitCast.clear();
	mTargetID.clear();
// 	for(int i =0;i<7;i++)
	
	for(int i=0; i<dynamic_player_num; i++)
	{
	    mInitCast.push_back(i);
	    mTargetID.push_back(i);
	}
	mOurCast =mInitCast;
    }

    //PassModel::PassModel(const PassModel& orig) {
    //}

    PassModel::~PassModel() {
    }

    void PassModel::calcOurFastestToBall() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOur;
        mOurFastestID = 0;
        mOurMinTimeToBall = 1000.0f;
	Vector2f ballPos =WM.getBallGlobalPos2D();
        map<float, unsigned int> timeOfmates; //利用map的key自动排序
        for (iterOur = mOurTeammates.begin();
                iterOur != mOurTeammates.end();
                ++iterOur) {
            if (timeNow - iterOur->second.mLastSeeTime < 2.0f || timeNow - iterOur->second.mlastHearTime < 2.0f  )
	    {
                timeOfmates[iterOur->second.mTimeToBall] = iterOur->second.mUnum;
            }
        }
        map<float, unsigned int>::const_iterator itermap;
        mOurMinTimeToBall = timeOfmates.begin()->first;
        int max_uid = timeOfmates.begin()->second;
	if(max_uid !=mOurLastFastestID)
	{
	    for (itermap = timeOfmates.begin();
		    itermap != timeOfmates.end();
		    ++itermap) {
		if (itermap->first - mOurMinTimeToBall < 0.5f) {
//       		    if(itermap->second ==mOurLastFastestID)
//       		    {
//       			max_uid =mOurLastFastestID;
//       			break;
//       		    }
		    if (max_uid < itermap->second) 
		    {
			max_uid = itermap->second;
		    }
		} 
	    }
	}
        mOurFastestID = max_uid;
 	mOurLastFastestID =mOurFastestID;
        //更新时间,虽然可能不是最短的
        mOurMinTimeToBall = mOurTeammates[mOurFastestID].mTimeToBall;
    }

    void PassModel::calcOppFastestToBall() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOur;
        mOppFastestID = 0;
        mOppMinTimeToBall = 1000.0f;

        for (iterOur = mOppTeammates.begin();
                iterOur != mOppTeammates.end();
                ++iterOur) {
            if (timeNow - iterOur->second.mLastSeeTime < 3.0f) {
                if (mOppMinTimeToBall > iterOur->second.mTimeToBall) {
                    mOppMinTimeToBall = iterOur->second.mTimeToBall;
                    mOppFastestID = iterOur->first;
                }
            }
        }
    }

    void PassModel::update() {
  	mIsDriving =false;
      
        updateByListen();
        shared_ptr<const Vision> vp = WM.lastPerception().vision();

        if (NULL == vp.get()) return;

        const Vision::TTeamPolMap& ourPol = vp->ourPolMap();
        updateTeammates(ourPol, mOurTeammates, true);
        unsigned int myUnum = WM.getMyUnum();
        if (myUnum != 0) {
            mOurTeammates[myUnum].mUnum = myUnum;
            mOurTeammates[myUnum].mIsOur = true;
            mOurTeammates[myUnum].mLastSeeTime = WM.getSimTime();
            mOurTeammates[myUnum].calcInfo();
            mOurTeammates[myUnum].calcGlobalVel();
        }
        const Vision::TTeamPolMap& oppPol = vp->oppPolMap();
        updateTeammates(oppPol, mOppTeammates, false);
        calcOurFastestToBall();
        calcOppFastestToBall();
	
	//ghd -- dynamic cast
	updateOurTargetPos();
	if(mOurFastestID ==myUnum && WM.getOurPlayerNumber()>=1)
	{
	  mIsDriving =true;
	  updateOurCast();
	}
    }

    void PassModel::updateTeammates(const Vision::TTeamPolMap& teamPol, TSeenPlayer& teammates, bool isOur) {
        float timeNow = WM.getSimTime();
        Vector3f errPol(VisionRobot::mErrPol);
        Vision::TTeamPolMap::const_iterator iterTeammatesPol;
        for (iterTeammatesPol = teamPol.begin();
                iterTeammatesPol != teamPol.end();
                ++iterTeammatesPol) {
            teammates[iterTeammatesPol->first].mLastSeeTime = timeNow;
            teammates[iterTeammatesPol->first].mUnum = iterTeammatesPol->first;
            teammates[iterTeammatesPol->first].mIsOur = isOur;
            //filter  objects which could not be seen
            for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
                if (iterTeammatesPol->second.find((perception::Vision::PID)i) != iterTeammatesPol->second.end()) {
                    teammates[iterTeammatesPol->first].mBodyPartPol[i] = (iterTeammatesPol->second.find((perception::Vision::PID)i)->second);
                    teammates[iterTeammatesPol->first].mIsSeen[i] = true;
                } else {
                    teammates[iterTeammatesPol->first].mBodyPartPol[i] = errPol;
                    teammates[iterTeammatesPol->first].mIsSeen[i] = false;
                }
            }
            if(!isOur)
	      teammates[iterTeammatesPol->first].calcInfo(true);

        }

        TSeenPlayer::iterator iterAllTeammates;
        for (iterAllTeammates = teammates.begin();
                iterAllTeammates != teammates.end();
                ++iterAllTeammates) {
            if (iterAllTeammates->second.getHaveSeen()) {
                iterAllTeammates->second.calcGlobalVel();
            }
        }

    }

    void PassModel::updateByListen() {
        if (!SHM.IsHeard()) {
            return;
        }

        unsigned int unum = SHM.getHearPlayerID();
	float timeNow = WM.getSimTime();

	mOurTeammates[unum].mUnum =unum;
        mOurTeammates[unum].mHaveHeard = true;
	mOurTeammates[unum].mlastHearTime = timeNow;
        //mOurTeammates[unum].mHearGlobalPos = SHM.getHearPlayerPos();
        //mOurTeammates[unum].mHearBodyDirection = SHM.getHearPlayerBodyDirection();
        //mOurTeammates[unum].mHearBallPos = SHM.getHearBallPos();
        
        //mOurTeammates[unum].mIsFallen = SHM.IsHearPlayerFallen();
        //mOurTeammates[unum].mHearFallen = SHM.IsHearPlayerFallen();
        mOurTeammates[unum].mIsOur = true;
	
	mOurTeammates[unum].mTimeToBall = SHM.getHearPlayerDist();
        //mOurTeammates[unum].calcTimeToBall();
        //mOurTeammates[unum].mGlobalPosAll.set(SHM.getHearPlayerPos().x(), SHM.getHearPlayerPos().y());

    }
    
    void PassModel::sortPlayer(vector<unsigned int> readyPlayer) {
	  bool sign[5] = {0};
	  vector<int>::iterator iter;
	  
	  for(iter = mOurCastPlayerID.begin(); iter != mOurCastPlayerID.end(); iter++) {
	    unsigned int temp = (*iter);
	    bool flag = false;
	    
	    vector<unsigned int>::iterator iter2;
	    for(iter2 = readyPlayer.begin(); iter2 != readyPlayer.end(); iter2++) {
	      if(temp == (*iter2)) {
		flag = true;
		readyPlayer.erase(iter2);
		break;
	      }
	    }
	    
	    sign[iter-mOurCastPlayerID.begin()] = flag;
	  }
	  
	  if(readyPlayer.size() == 0) {   //no change compare with before
	    
	    return ;
	  }
	  
	  int index = 0;
	  
	  for(iter = mOurCastPlayerID.begin(); iter != mOurCastPlayerID.end();iter++) {
	    if(!sign[iter-mOurCastPlayerID.begin()] && index < readyPlayer.size() ) {
	    
	      iter = mOurCastPlayerID.erase(iter);
	      mOurCastPlayerID.insert(iter, readyPlayer[index++]);
	     /* if(index < readyPlayer.size() - 1) {
		index++;
	      }*/
	    } /*else {
	      iter++; 
	    }*/
	  }
	  
/*	  vector<unsigned int>::iterator iter2;
	  for(iter2 = readyPlayer.begin(); iter2 != readyPlayer.end(); iter2++) {
	     mOurCastPlayerID.push_back((*iter2));
	  }
*/	  
    }

    void PassModel::updateOurCastPlayerID()			//ghd
    {
	vector<int> lastHeardCast = SHM.getHeardOurCast();
	if( !lastHeardCast.empty() ) {   //copy
	  //cout<<"Come in!!! use heard once!!"<<endl;
	  if( lastHeardCast.size() == 5 ) {
	   // cout<<"Copy!!"<<endl;
	    mOurCastPlayerID = lastHeardCast;
	    SHM.emptyHeardOurCast();
	    return ;
	  } else {
	   // cout<<"Not Copy"<<endl;
	    SHM.emptyHeardOurCast();
	  }
	}
      
	TSeenPlayer::iterator iterOur;
        multimap<float, unsigned int> timeOfmates; //利用map的key自动排序
	//cout<<" ++ "<<mOurTeammates.size()<<endl;
	for (iterOur = mOurTeammates.begin();
                iterOur != mOurTeammates.end();
                iterOur++) {  

	  if(iterOur ->second.mUnum !=configuration::Formation::PT_GOALKEEPER
	    && iterOur ->second.mUnum !=WM.getMyUnum() &&iterOur ->second.mUnum !=100) {
	    timeOfmates.insert(multimap<float, unsigned>::value_type(iterOur->second.mTimeToBall, iterOur->second.mUnum));
	      //timeOfmates[iterOur->second.mTimeToBall] = iterOur->second.mUnum;
	  }
         
        }

//         cout<<timeOfmates.size()<<endl;

// 	vector<int>::iterator iter;
// 	cout<<"b:\t";
// 	for(iter = mOurCastPlayerID.begin(); iter != mOurCastPlayerID.end(); iter++) {
// 	  cout<<(*iter)<<'\t';
// 	}
// 	cout<<endl;

/*        
        //choose player
	if(timeOfmates.size() >=5 )
	{
	  map<float ,unsigned int  >::iterator mapIter;
	  mapIter =timeOfmates.begin();
	  
	  mOurCastPlayerID.clear();
	  
	  unsigned int i;
	  for(i =0;i<5;i++)
	  {
	    mOurCastPlayerID.push_back(mapIter ->second);
	    
    	  cout<<mapIter ->second <<'\t';
	    
	    mapIter ++;
	  }
	}
 	cout<<endl;
	sort(mOurCastPlayerID.begin(),mOurCastPlayerID.end());
*/	
	
	///replace function bottom for test
	if(timeOfmates.size() >= 5) {
	  vector<unsigned int> readyPlayer;
	  map<float ,unsigned int  >::iterator mapIter;
	  
	  mapIter =timeOfmates.begin();
	  readyPlayer.clear();
	  	  
	  sort(readyPlayer.begin(),readyPlayer.end());
	  
// 	  cout<<"a:\t";
	  for(unsigned int i = 0; i < 5; i++) {
	    readyPlayer.push_back(mapIter ->second);	    
//	    cout<<mapIter->second<<'\t';	    
	    mapIter ++;
	  }
// 	  cout<<'\n'<<endl;
	  
	  sortPlayer(readyPlayer);
	}
	
	if(mOurCastPlayerID.empty() && timeOfmates.size() >= 5) {
// 	  cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	  map<float, unsigned int>::iterator mapIter;
	  mapIter =timeOfmates.begin();
	  
	  mOurCastPlayerID.clear();
	  
	  for(unsigned int i = 0; i < 5; i++) {
	    mOurCastPlayerID.push_back(mapIter ->second);  
	    mapIter ++;
	  }
	  
	  sort(mOurCastPlayerID.begin(),mOurCastPlayerID.end());
	}
	
	
// 	unsigned int i;
// 	mOurCastPlayerID.clear();
// 	for(i =6;i<=11;i++)
// 	{
// 	  if(i !=WM.getMyUnum())
// 	    mOurCastPlayerID.push_back(i);
// 	}
	
	
//       unsigned int ourFastest =getOurFastestID();
// 	mOurCastPlayerID.clear();
// 	
// 	vector<Vector2f> ourTeamPoses;
// 	vector<Vector2f>::iterator ourTeamPosesIter;
// 	
// 	vector<float> disFromTeamToFast;
// 	vector<float>::iterator disFromTeamToFastIter;
// 	
// 	map<float, int> mourTeamDisMap;
// 	map<float, int>::iterator mourTeamDisMapIter;
// 	
// 	float fastestDis[3];
// 	
// // 	vector<int> mFinalSelectAgentID;
// 	
// 	vector<int> mAgentIDBesidesFast;
// 	vector<int>::iterator mAgentIDBesidesFastIter;
// 	
// 	//save the IDs and poses beside fastest to the ball 
// 	for(int i=1; i<=11; i++) 
// 	{
// 	  if(i != ourFastest){
// 	    mAgentIDBesidesFast.push_back(i);
// 	    ourTeamPoses.push_back(WM.getOurGlobalPos2D(i));
// 	  }
// 	}
// 	
// 	ourTeamPosesIter = ourTeamPoses.begin();
// 	mAgentIDBesidesFastIter = mAgentIDBesidesFast.begin();
// 	for(int i=0; i<10; i++)
// 	{
// 	    disFromTeamToFast.push_back((*ourTeamPosesIter-WM.getOurGlobalPos2D(ourFastest)).length());
// 	    mourTeamDisMap.insert(pair<float, int>((*ourTeamPosesIter-WM.getOurGlobalPos2D(ourFastest)).length(), *mAgentIDBesidesFastIter));
// 	    ourTeamPosesIter++;
// 	    mAgentIDBesidesFastIter++;
// 	}
// 	
// 	sort(disFromTeamToFast.begin(), disFromTeamToFast.end());
// 	disFromTeamToFastIter = disFromTeamToFast.begin();
// 	for(int i=0; i<3; i++)
// 	{
// 	  fastestDis[i] = *disFromTeamToFastIter;
// 	  disFromTeamToFastIter++;
// 	}
// 	
// 	for(mourTeamDisMapIter = mourTeamDisMap.begin(); mourTeamDisMapIter != mourTeamDisMap.end(); mourTeamDisMapIter++){
// 
// // 	  if(mourTeamDisMapIter->first == fastestDis[0] || mourTeamDisMapIter->first == fastestDis[1] || mourTeamDisMapIter->first == fastestDis[2]){
//   	  if(mourTeamDisMapIter->first == fastestDis[0] || mourTeamDisMapIter->first == fastestDis[1] || mourTeamDisMapIter->first == fastestDis[2]){
// 
// // 	    mFinalSelectAgentID.push_back(mourTeamDisMapIter->second);
// 	    mOurCastPlayerID.push_back(mourTeamDisMapIter->second);
// 	    
// 	  }
// #if 0
// 	unsigned int ourFastest =getOurFastestID();
// 	mOurCastPlayerID.clear();
// 	if(ourFastest>3)
// 	{
//  	    for(int i =4;i<12;i++)
// 	    for(int i =4; i<7; i++)
// 	    {
// 		if(i != ourFastest)
// 		{
// 		    mOurCastPlayerID.push_back(i);
// 		}
// 	    }
// 	}
// 	else 
// 	{
//  	    for(int i =4;i<11;i++)
// 	    for(int i= 4; i<7; i++)
// 	    {
// 		mOurCastPlayerID.push_back(i);
// 	    }
// 	}
// #endif
//    // }
//     }
    }
    
    void PassModel::updateOurTargetPos()
    {
	unsigned int ourCastType =WM.getHearCastType();
	//follow the fastest guy
	//Vector2f ourPos =Vector2f(WM.getOurFastestToBallPos().x(),WM.getOurFastestToBallPos().y());
	//follow the ball
	Vector2f ourPos1 = WM.getBallGlobalPos2D();
	mTargetPos.clear();
	
///	float R = 2.0f;
///	float r1 = 1.0f;
///	float r2 = 2.5f;
///	float l1 = 2.4f;
///	float l2=  2.0f;
///  by yao
	
///by visen  for test 
	float R = 2.0f;
	float r1 = 1.0f;
	float r2 = 1.5f;
	float l1 = 1.4f;
	float l2=  1.3f;
	
/// by visen end of the test	
	
	///yao test
	/*if( ourPos1.x() < -7 ) {
	    int offset;
	  
	    if( ourPos1.y() > 2 ) {
	      offset = -2;
	    } else if ( ourPos1.y() < -2 ) {
	      offset = 2;
	    } else {
	      offset = 0;
	    }
	  
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0+offset));
	    
	    if( ourPos1.y() > 2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R,offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2+offset));
	    } else if( ourPos1.y() < -2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2+offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R,offset));
	    } else {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2+offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2+offset));
	    }
	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1+offset));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1+offset));
	  */
	  ///    by yao   2013 6
	    float RR = 2.0f;
	    float r11 = 1.0f;
	    float r22 = 2.5f;
	    float l11 = 2.4f;
	    float l22=  1.0f;
	    /// by visen for defend 
	  if( ourPos1.x() < -8 ) {
	    int offset;
	  
	    if( ourPos1.y() > 2 ) {
	      offset = -2;
	    } else if ( ourPos1.y() < -2 ) {
	      offset = 2;
	    } else {
	      offset = 0;
	    }
	  
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*RR,0+offset));
	    
	    if( ourPos1.y() > 2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*RR,offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r22,l22+offset));
	    } else if( ourPos1.y() < -2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r22,-1*l22+offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R,offset));
	    } else {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r22,-1*l22+offset));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r22,l22+offset));
	    }
	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r11,-1*l11+offset));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r11,l11+offset));	    
	    
	    
	    /// to avoid being out of ground 
/*	} else if( fabs(ourPos1.y()) < 2) {
	  
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1.5*r1,l1));
*/	    
/*
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));	   
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
*/	    
 /*	} else {
	  
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));
	    
	    if( ourPos1.y() > 2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R, 0));
	    } else {
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R, 0));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));
	    }
	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1));
	*/
 
 ///   for test  
 /// like applo
 	} else if( fabs(ourPos1.y()) < 4) {///changed.......
	  
	  if(ourPos1.x()>=10)
	  {	
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));
	    mTargetPos.push_back(ourPos1 +Vector2f(-0.5*r1,-1*l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-0.5*r1,l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(0.85*r1,-1*l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(0.85*r1,l1));
	    
	 
	  }
	  else
	  {
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1));
	  }
/*
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));	   
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
*/	    
 	} else {
	  
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*R,0));
	    
	    if( ourPos1.y() > 2 ) {
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,-1*l2));
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R, 0));
	    } else {
	      mTargetPos.push_back(ourPos1 +Vector2f(-2*R, 0));
	      mTargetPos.push_back(ourPos1 +Vector2f(-1*r2,l2));
	    }
	    
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,-1*l1));
	    mTargetPos.push_back(ourPos1 +Vector2f(-1*r1,l1));
 /// end  test
// 	    //test by liuyao;   in side
// 	    int flag = (ourPos1.y() > 0) ? -1 : 1;
// 	    
// 	    float x = ourPos1.x() + 15;
// 	    
// 	    if(x > 23) {
// 		x = 23;
// 	    }
// 	    if(x < 3) {
// 		x = 3;
// 	    }
// 	    
// 	    AngRad ang = deg2Rad( -3 * x + 99 );      // can modify angle
// 	    
// 	    Vector2f V0,VT,V;//V1L,V1R,V2L,V2R;
// 	    V0.x() = -1 * R * cos(ang);
// 	    V0.y() = flag * R * sin(ang);
// 
// 	    mTargetPos.push_back(ourPos1 + V0);
// 	    
// 	    V0 /= V0.length();
// 	    VT.x() = -1 * sin(ang);
// 	    VT.y() = -1 * flag * cos(ang);
// 	    
// 	    V = V0*r2 + VT*l2;
// 	    mTargetPos.push_back(ourPos1 + V);
// 	    V = V0*r2 - VT*l2;
// 	    mTargetPos.push_back(ourPos1 + V);
// 	    V = V0*r1 + VT*l1;
// 	    mTargetPos.push_back(ourPos1 + V);
// 	    V = V0*r1 - VT*l1;
// 	    mTargetPos.push_back(ourPos1 + V);
	       
	    //VT *= -1;
// 	    V1L = V0*r1 - VT*l1;
// 	    V1R = V0*r1 + VT*l1;
// 	    V2R = V0*r2 - VT*l2;
// 	    V2L = V0*r2 + VT*l2;
// 	    
// 	    cout<<"v2R:"<<V2R<<endl;
// 	    cout<<"v2L:"<<V2L<<endl;	 
// 	    cout<<"v1R:"<<V1R<<endl;	 
// 	    cout<<"v1L:"<<V1L<<endl;	 
// 	    
// 	    mTargetPos.push_back(ourPos1 + V2R);
// 	    mTargetPos.push_back(ourPos1 + V2L);
// 	    mTargetPos.push_back(ourPos1 + V1R);
// 	    mTargetPos.push_back(ourPos1 + V1L);
 	}
	
	/*if(ourCastType == 1)
	{
	    mTargetPos.push_back(ourPos1+Vector2f(0,-2.5f));
	    mTargetPos.push_back(ourPos1+Vector2f(-2.0f,-1.5f));
	    mTargetPos.push_back(ourPos1+Vector2f(2.0f,-1.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(0,-2.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(-2.0f,-1.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(-2.5f,0));
// 	    mTargetPos.push_back(ourPos+Vector2f(-1.5f,1.5f));
	}
	else if(ourCastType == 2)
	{
	    mTargetPos.push_back(ourPos1+Vector2f(0,2.5f));
	    mTargetPos.push_back(ourPos1+Vector2f(-2.0f,1.5f));
	    mTargetPos.push_back(ourPos1+Vector2f(2.0f,1.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(0,-2.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(-2.0f,-1.5f));
// 	    mTargetPos.push_back(ourPos+Vector2f(-2.5f,0));
// 	    mTargetPos.push_back(ourPos+Vector2f(-1.5f,1.5f));
	}else
	{
	    mTargetPos.push_back(ourPos+Vector2f(0,2.5f));
	    mTargetPos.push_back(ourPos+Vector2f(-2.0f,1.5f));
	    mTargetPos.push_back(ourPos+Vector2f(2.0f,1.5f));

	}*/
    }
    
    //it will kill our player(^_^),don't use now    	--ghd test

//     void PassModel::adjustOurCast()		//ghd
//     {
// 	int temp;
//  	for(unsigned int i =0;i!=5;i++)
//  	{
//  	    if(isTwoLineCross(mTargetPos[i],getAgentPosByID(mOurCast[i]),mTargetPos[i+1],getAgentPosByID(mOurCast[i+1])))
// 	    {
// 		temp =mOurCast[i];
// 		mOurCast[i] =mOurCast[i+1];
// 		mOurCast[i+1] =temp;
// 	    }
//  	}
//     }
//     
//     bool PassModel::isTwoLineCross(Vector2f p1, Vector2f p2, Vector2f q1, Vector2f q2)	//ghd
//     {
//  	Vector2f v1 =p1 -q1;
//  	Vector2f v2 =q2 -q1;
//  	Vector2f v3 =p2 -q1;
// 	Vector2f v4 =q1 -p1;
//  	Vector2f v5 =p2 -p1;
//  	Vector2f v6 =q2 -p1;
// 	int x1 =crossOfVecter2f(v1,v2);
// 	int x2 =crossOfVecter2f(v2,v3);
// 	int x3 =crossOfVecter2f(v4,v5);
// 	int x4 =crossOfVecter2f(v5,v6);
// 	bool result =(( x1*x2  >=0 )&&( x3 *x4 >=0));
//  	return result;
//     }
//     
//     int PassModel::crossOfVecter2f(Vector2f v1,Vector2f v2)		//ghd
//     {
// 	return sign(v2.angle() -v1.angle());
//     }

    void PassModel::updateOurCast()
    {
	updateOurCastPlayerID();
 	/*if(WM.getPlayMode() ==serversetting::PM_PLAY_ON)
	{
// 	    mOurCast =calPosition(mTargetID,mOurCastPlayerID,7);
	    mOurCast = calPosition(mTargetID, mOurCastPlayerID, 5);
	}
 	else*/ mOurCast =mOurCastPlayerID;
    }
    
    Vector2f PassModel::findMyGoodPos(int posID)
    {
	return mTargetPos[posID];
    }
    
//     bool PassModel::canPass() {
//         float timeNow = WM.getSimTime();
//         TSeenPlayer::const_iterator iterOpp;
//         bool resCanPass = true;
//         for (iterOpp = mOppTeammates.begin();
//                 iterOpp != mOppTeammates.end();
//                 ++iterOpp) {
//             if (timeNow - iterOpp->second.mLastSeeTime < 3.0f) {
//                 if (iterOpp->second.mTimeToBall < 1.5f) {
//                     resCanPass = false;
//                     break;
//                 }
//             }
//         }
//         if (resCanPass) {
//             mCanPass = true;
//             mCanPassTime = timeNow;
//             return true;
//         } else {
//             if (mCanPass && (timeNow - mCanPassTime < 1.5f)) {
//                 return true;
//             } else {
//                 mCanPass = false;
//                 return false;
//             }
//         }
//         //    return true;
//     }
// 
//     bool PassModel::choosePass() {
//         float timeNow = WM.getSimTime();
//         TSeenPlayer::const_iterator iterOur;
//         mPassSet.clear();
//         for (iterOur = mOurTeammates.begin();
//                 iterOur != mOurTeammates.end();
//                 ++iterOur) {
//             if (iterOur->first == WM.getMyUnum()) {
//                 continue;
//             }
//             if (iterOur->second.mGlobalPosAll.x() > WM.getBallGlobalPos().x()) {
//                 if (!isBlocked(iterOur->first)) {
//                     mPassSet.insert(iterOur->first);
//                 }
//             } else if (WM.getBallGlobalPos().x() > half_field_length - penalty_length * 0.5
//                     &&
//                     abs(WM.getBallGlobalPos().y()) > half_penalty_width) {
//                 if (iterOur->second.mGlobalPosAll.x() > half_field_length - penalty_length * 1.2
//                         &&
//                         abs(iterOur->second.mGlobalPosAll.y()) < half_penalty_width * 1.2) {
//                     if (!isBlocked(iterOur->first)) {
//                         mPassSet.insert(iterOur->first);
//                     }
//                 }
//             }
//         }
//         if (mPassSet.empty()) {
//             mBestPassID = 0;
//             return false;
//         } else {
//             std::set<unsigned int>::iterator setIter;
//             mBestPassID = *(mPassSet.begin());
//             seumath::AngDeg minDeg = 1000;
//             for (setIter = mPassSet.begin();
//                     setIter != mPassSet.end();
//                     setIter++) {
//                 Vector2f dist = mOurTeammates[*setIter].mGlobalPosAll - WM.getMyGlobalPos2D();
//                 float deg = abs(normalizeAngle(dist.angle() - 90));
//                 if (minDeg > deg) {
//                     mBestPassID = *setIter;
//                     minDeg = deg;
//                 }
//             }
//             return true;
//         }
//     }
// 
//     bool PassModel::isBlocked(unsigned int ourTeammateID) {
//         Vector2f our(mOurTeammates[ourTeammateID].mGlobalPos.x(), mOurTeammates[ourTeammateID].mGlobalPos.y());
//         Vector2f ballPos = WM.getBallGlobalPos2D();
//         Vector2f dist = (ballPos - our);
//         TSeenPlayer::const_iterator iterOpp;
// 
//         if (dist.length() > 8.0f) {
//             return true;
//         }
//         for (iterOpp = mOppTeammates.begin();
//                 iterOpp != mOppTeammates.end();
//                 ++iterOpp) {
// 
//             Vector2f opp(iterOpp->second.mGlobalPos.x(), iterOpp->second.mGlobalPos.y());
// 
//             Vector2f dd = ballPos - opp;
// 
//             if (abs(dd.angle() - dist.angle()) < 20) {
//                 if (dd.length() < dist.length()) {
//                     return false;
//                 }
//             }
//         }
//         return false;
//     }
    
    //calc shortest 		--yzz
    float PassModel::calLen(int targetID, int agentID)
    {
	return (findMyGoodPos(targetID)-getAgentPosByID(agentID)).length();
    }

    vector< int > PassModel::calPosition(vector< int >& targetIDs, vector< int >& agentIDs, const int num)
    {
	vector<vector<int> > agentPre;
	vector<vector<int> > targetPre;
	genPreference(targetPre,agentPre,targetIDs,agentIDs,num);

	//ŽË¶ÎÖ»ÎªÁËÊä³ö¿ŽÒ»ÏÂ

/*	for(int i=0;i!=num;++i){

		cout<<"agent["<<i<<"] pre:";

		for(int j=0;j!=num+1;++j){

			cout<<agentPre[i][j]<<" ";

		}

		cout<<"\n";

	}

	cout<<"\n";

	for(int i=0;i!=num;++i){

		cout<<"target["<<i<<"] agents' rank:";

		for(int j=0;j!=num;++j){

			cout<<targetPre[i][j]<<" ";

		}

		cout<<"\n";

	}*/
	vector<int> result(num,-1);
	stableMatch(result,targetPre,agentPre,agentIDs,num);
	return result;
    }
    
    void PassModel::genPreference(vector< vector< int > >& targetPre, vector< vector< int > >& agentPre, vector< int >& targetIDs, vector< int >& agentIDs, const int num)
    {
 	for(int i=0;i!=num;++i){
		vector<int> targetTemp(num,-1);
		vector<float> targetHelp(num,0);
		vector<int> rank(num);
		vector<int> agentTemp(num+1,1); //µÚÒ»žöÔªËØ·Å1,ÓÃÀŽŽæÒª·ÃÎÊµÄtargetµÄrank

		vector<float> agentHelp(num,0);

		targetTemp[0]=0;
		targetHelp[0]=calLen(targetIDs[i],agentIDs[0]);

		agentTemp[1]=0;
		agentHelp[0]=calLen(targetIDs[0],agentIDs[i]);
		float tCurr=0.0f;
		float aCurr=0.0f;
		int k=0;
		int h=0;
		for(int j=1;j!=num;++j){
			tCurr = calLen(targetIDs[i],agentIDs[j]);
			k = j-1;
			while( (k>=0) && (tCurr < targetHelp[k]) ){
				targetHelp[k+1] = targetHelp[k];
				targetTemp[k+1] = targetTemp[k];
				k--;
			}
			targetHelp[k+1] = tCurr;
			targetTemp[k+1] = j;
		
			aCurr = calLen(targetIDs[j],agentIDs[i]);
			h = j-1;
			while( (h>=0) && (aCurr < agentHelp[h]) ){
				agentHelp[h+1] = agentHelp[h];
				agentTemp[h+2] = agentTemp[h+1];
				h--;
			}
			agentHelp[h+1] = aCurr;
			agentTemp[h+2] = j;
		}
		for(int index=0;index!=num;++index){
			rank[targetTemp[index]] = index;
		}
  		targetPre.push_back(rank);
  		agentPre.push_back(agentTemp);
	}
    }

    void PassModel::stableMatch(vector< int >& targetResult, vector< vector< int > >& targetPre, vector< vector< int > >& agentPre, vector< int >& agentIDs, const int num)
    {
	queue<int> agentQueue;
	for(int i=0; i!=num; ++i){
		agentQueue.push(i);
	}
	int aID = 0;
	int targetAsked = 0;
	while(!agentQueue.empty()){
		aID = agentQueue.front();   //²¢·ÇÕæÕýagent's id£¬ÕâÖ»ÊÇÎªÁËžúagentPreÅäºÏÊ¹ÓÃ

		agentQueue.pop();
		targetAsked = agentPre[aID][agentPre[aID][0]];
		if(targetResult[targetAsked]<0){
			targetResult[targetAsked] = aID;
		}else if(targetPre[targetAsked][aID] < targetPre[targetAsked][targetResult[targetAsked]]){
			agentQueue.push(targetResult[targetAsked]);//œ«±»Œ·ÏÂµÄagentÖØÐÂŒÓÈë¶ÓÁÐ
			agentPre[targetResult[targetAsked]][0] = (agentPre[targetResult[targetAsked]][0]+1)%(num+1);//ÉèÖÃÏÂÒ»žöÒª·ÃÎÊµÄtarget 

			targetResult[targetAsked] = aID;
		}else {
			//ÕâÖÖÇé¿öÊÇžÃagent±È²»¹ýÇ°Õß£¬Ö±œÓ±»Ë¢ÁË

			agentQueue.push(aID);
			agentPre[aID][0] = (agentPre[aID][0]+1)%(num+1);
		}
	}
	for(int i=0;i!=num;++i){
	  targetResult[i] = agentIDs[targetResult[i]];
	}
    }

    Vector2f PassModel::getAgentPosByID(int id)
    {
	Vector3f pos =mOurTeammates[id].mGlobalPos;
	return seumath::Vector2f(pos.x(),pos.y());
    }
}