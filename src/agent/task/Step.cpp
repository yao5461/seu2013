/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Step.cpp 2139 2008-07-23 09:06:33Z xy $
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "Step.h"
#include "SwingFoot.h"
#include "iostream"
namespace task {

    using namespace std;
    using namespace boost;
    using namespace seumath;
    using namespace robot::humanoid;
    using namespace action;
    
    #define HeteroMode0	0
    #define HeteroMode1	1
    #define HeteroMode2	2

    //Vector2f Step::mMaxSizeAcc(0.0135f, 0.045f);
    //Vector2f Step::mMinSize(-0.027, -0.09);
    //Vector2f Step::mMaxSize(0.027, 0.09);
    
    
    
    Vector2f Step::mMaxSize0 = Vector2f(0.02f, 0.118f);//before modify --0.055 lty 0.055f, 0.165f
    Vector2f Step::mMinSize0 = mMaxSize0*(-1.0f);
    Vector2f Step::mMaxSizeAcc0 = mMaxSize0 * 0.23f;
    Vector2f Step::mMaxSize2Acc0 = mMaxSize0 * 0.23f;
    AngDeg Step::mMinDir0 = 0.0f;
    AngDeg Step::mMaxDir0 = 50.0f;
    AngDeg Step::mMaxDirAcc0 = mMaxDir0 *1.0f;
    AngDeg Step::mMaxDir2Acc0 = mMaxDir0 *1.0f;
    float Step::mStep1Time0 = 0.02f;
    float Step::mStep2Time0 = 0.02f;
    float Step::mStep3Time0 = 0.03f;
///   float Step::mh10 =0.045f;  ghd
///    float Step::mh20 =0.055f;ghd
    float Step::mh10=0.040;  ////////visen
    float Step::mh20=0.045;  ////////visen
    
///    float Step::mh10=0.040; test
///   float Step::mh20=0.048;  test
 ///    int Step::mi10 =50;  ghd
///    int Step::mi20 =112;  ghd
     
    int Step::mi10 =40; //////////////  visen
    int Step::mi20 =120; ///////////// visen
 ///   int Step::mi10=42;
 ///   int Step::mi20=130;
    
    ///.................................
    Vector2f Step::mMaxSize1 = Vector2f(0.02f, 0.126f);//before modify --0.055 lty 0.055f, 0.165f
    Vector2f Step::mMinSize1 = mMaxSize1*(-1.0f);
    Vector2f Step::mMaxSizeAcc1 = mMaxSize1 * 0.23f;
    Vector2f Step::mMaxSize2Acc1 = mMaxSize1 * 0.23f;
    AngDeg Step::mMinDir1 = 0.0f;
    AngDeg Step::mMaxDir1 = 50.0f;
    AngDeg Step::mMaxDirAcc1 = mMaxDir1 *1.0f;
    AngDeg Step::mMaxDir2Acc1 = mMaxDir1 *1.0f;
    float Step::mStep1Time1 = 0.02f;
    float Step::mStep2Time1 = 0.02f;
    float Step::mStep3Time1 = 0.03f;
    float Step::mh11 =0.06f;
    float Step::mh21 =0.06f;
    int Step::mi11 =50;
    int Step::mi21 =130;
    
    Vector2f Step::mMaxSize2 = Vector2f(0.02f, 0.119f);//before modify --0.055 lty 0.055f, 0.165f
    Vector2f Step::mMinSize2 = mMaxSize2*(-1.0f);
    Vector2f Step::mMaxSizeAcc2 = mMaxSize2 * 0.5f;
    Vector2f Step::mMaxSize2Acc2 = mMaxSize2 * 0.5f;
    AngDeg Step::mMinDir2 = 0.0f;
    AngDeg Step::mMaxDir2 = 50.0f;
    AngDeg Step::mMaxDirAcc2 = mMaxDir2 *1.0f;
    AngDeg Step::mMaxDir2Acc2 = mMaxDir2 *1.0f;
    float Step::mStep1Time2 = 0.02f;
    float Step::mStep2Time2 = 0.02f;
    float Step::mStep3Time2 = 0.03f;
    float Step::mh12 =0.055f;
    float Step::mh22 =0.055f;
    int Step::mi12 =40;
    int Step::mi22 =95;

    Step::Step(bool isLeft,
            const Vector2f& size, AngDeg dir,
            shared_ptr<const Step> preStep,
            float bodyHeight,
	    bool goStop,  
	    Task* primary
            )
    : Task(-1, primary),
    mIsLeft(isLeft) {
	mHaveLast =false; 
        mPreSize =Vector2f(0.0f, 0.0f);
        mPreSizeAcc =Vector2f(0.0f, 0.0f);
        mPreDir = 0.0f;
	mPreDirAcc =0.0f;
	
	///last step		ghd
	seumath::Vector2f pre2Size;
	seumath::Vector2f pre2SizeAcc;
	seumath::AngDeg pre2Dir;
	
	int preWalkType =3;///       why set the pre walk type =3?????
        if (0 != preStep.get()) {
	    mHaveLast =true;
            mPreSize = preStep->size();
            mPreSizeAcc = preStep->sizeAcc();
            mPreDir = preStep->dir();
	    mPreDirAcc = preStep->dirAcc();
	    preWalkType =preStep->walkType();
	    
	    if(preStep ->mHaveLast)
	    {
	      pre2Size =preStep ->preSize();
	      pre2SizeAcc =preStep ->preSizeAcc();
	      pre2Dir =preStep ->predir();
	      
	    }
        }
        
	switch (FM.getMy().heteroType)
	{
	  case HeteroMode0: mMaxSizeAcc =mMaxSizeAcc0;
			    mMinSize =mMinSize0;
			    mMaxSize =mMaxSize0;
			    mMaxDirAcc =mMaxDirAcc0;
			    mMaxDir =mMaxDir0;
			    mMinDir =mMinDir0;
			    mStep1Time =mStep1Time0;
			    mStep2Time =mStep2Time0;
			    mStep3Time =mStep3Time0;
			    mMaxSize2Acc =mMaxSize2Acc0;
			    mMaxDir2Acc =mMaxDir2Acc0;
			    mh1 =mh10;
			    mh2 =mh20;
			    mi1 =mi10;
			    mi2 =mi20;
			    break;
			  
	  case HeteroMode1: mMaxSizeAcc =mMaxSizeAcc1;
			    mMinSize =mMinSize1;
			    mMaxSize =mMaxSize1;
			    mMaxDirAcc =mMaxDirAcc1;
			    mMaxDir =mMaxDir1;
			    mMinDir =mMinDir1;
			    mStep1Time =mStep1Time1;
			    mStep2Time =mStep2Time1;
			    mStep3Time =mStep3Time1;
			    mMaxSize2Acc =mMaxSize2Acc1;
			    mMaxDir2Acc =mMaxDir2Acc1;
			    mh1 =mh11;
			    mh2 =mh21;
			    mi1 =mi11;
			    mi2 =mi21;
			    break;
			  
	  case HeteroMode2:mMaxSizeAcc =mMaxSizeAcc2;
			    mMinSize =mMinSize2;
			    mMaxSize =mMaxSize2;
			    mMaxDirAcc =mMaxDirAcc2;
			    mMaxDir =mMaxDir2;
			    mMinDir =mMinDir2;
			    mStep1Time =mStep1Time2;
			    mStep2Time =mStep2Time2;
			    mStep3Time =mStep3Time2;
			    mMaxSize2Acc =mMaxSize2Acc2;
			    mMaxDir2Acc =mMaxDir2Acc2;
			    mh1 =mh12;
			    mh2 =mh22;
			    mi1 =mi12;
			    mi2 =mi22;
			    break;
			
	}
        
        mMaxSizeAcc.x() =mMaxSize.x();

        //if(isLeft){
        //      mPreDir = WM.getBoneLocalTrans(robot::humanoid::Humanoid::L_FOOT).rotatedAngZ();
        //}else{
        //mPreDir = WM.getBoneLocalTrans(robot::humanoid::Humanoid::R_FOOT).rotatedAngZ();
        //}
        mSize =  size;
        /////////////////////////////////////////////////////////////
        /// restrict the step size and direction

        //----------------------------------------------------------
        // restrict the turning angle
        AngDeg maxDirAcc = mMaxDirAcc;
        AngDeg maxDir = mIsLeft ? mMaxDir : -mMinDir;
        AngDeg minDir = mIsLeft ? mMinDir : -mMaxDir;

        AngDeg dirAcc = calClipAng(dir, mPreDir);
        dirAcc = clamp(dirAcc, -maxDirAcc, maxDirAcc);
        float maxsizelen = mMaxSize.length();
        float presizelen = mPreSize.length();
///        dirAcc *= max(0.5f, 1 - presizelen / maxsizelen);   ghd
	dirAcc *= max(0.55f, 1 - presizelen / maxsizelen);
        mDir = dirAcc + mPreDir;
	mDir = clamp(mDir, minDir, maxDir);
	
	//cout<<mDir<<endl;
	
        //calc the rotate vector
        Vector2f rotateMov;
        float halfFeetWidth = HUMANOID.getHalfFeetWidth();
	//float halfFeetLength = 0.05f;
        rotateMov.x() = (mIsLeft ? 1 : -1) * sign(mDir)
                * halfFeetWidth * (1 - cosDeg(mDir)) * 2;
        rotateMov.y() = (mIsLeft ? -1 : 1) * halfFeetWidth * sinDeg(mDir) * 2;
        ///test walk code by dpf, keep it for use of future!!! don't delete it!
        
        /*seumath::TransMatrixf tmp;

        //if i am left, walk Coordinate is right foot's trans
        seumath::TransMatrixf leftFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::L_FOOT);
        seumath::TransMatrixf rightFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::R_FOOT);
        seumath::TransMatrixf walkCoordinateToRelTrans=(mIsLeft?rightFootRelTrans:leftFootRelTrans);//trans from walk coordinate to rel coordinate
        seumath::TransMatrixf moveFootTrans=(mIsLeft?leftFootRelTrans:rightFootRelTrans);
         */
        /*
        //suppose the bodydir x is always -10 degree
        seumath::TransMatrixf tmp;
        seumath::TransMatrixf leftFootLocalTrans=WM.getBoneLocalTrans(robot::humanoid::Humanoid::L_FOOT);
        seumath::TransMatrixf rightFootLocalTrans=WM.getBoneLocalTrans(robot::humanoid::Humanoid::R_FOOT);
        seumath::TransMatrixf leanRelToRelTrans;
        leanRelToRelTrans.rotationX(-15);
        tmp=leanRelToRelTrans;
        seumath::TransMatrixf leftFootRelTrans=tmp.transfer(leftFootLocalTrans);
        tmp=leanRelToRelTrans;
        seumath::TransMatrixf rightFootRelTrans=tmp.transfer(rightFootLocalTrans);
        seumath::TransMatrixf walkCoordinateToRelTrans=(mIsLeft?rightFootRelTrans:leftFootRelTrans);//trans from walk coordinate to rel coordinate
        seumath::TransMatrixf moveFootTrans=(mIsLeft?leftFootRelTrans:rightFootRelTrans);
	
        if(mIsLeft){//right foot support, so the orgin should minuse a halfFeetWidth
          walkCoordinateToRelTrans.p().x()-=HUMANOID.getHalfFeetWidth();
        }
        else{
          walkCoordinateToRelTrans.p().x()+=HUMANOID.getHalfFeetWidth();
        }
        tmp=walkCoordinateToRelTrans;
        moveFootTrans=tmp.inverseTransfer(moveFootTrans);
        AngDeg footNowDir=moveFootTrans.rotatedAngZ();
        mSize=*(Vector2f*)(walkCoordinateToRelTrans.inverseTransform(Vector3f(mSize.x(),mSize.y(),0.0f)).get());
         */
        //----------------------------------------------------------
        // restirct the step size
        
        //cout<<mDir<<endl;
        
        Vector2f maxSize, minSize;
        maxSize.x() = mMaxSize.x(); //(mIsLeft?-mMinSize.x():mMaxSize.x());
        maxSize.y() = mMaxSize.y() * cosDeg(mDir);
        minSize.x() = mMinSize.x(); //(mIsLeft?-mMaxSize.x():mMinSize.y());
        minSize.y() = mMinSize.y() * cosDeg(mDir);
        if (mSize.squareLength() > 1) {
            // to avoid side walk by noise data
            mSize.normalize();
            mSize *= 2;
        }
     
        Vector2f maxSizeL(-0.02f,0),minSizeL(-0.12f,0),maxSizeR(0.12f,0),minSizeR(0.02f,0);
	//Vector2f maxSizeAccL(-0.07f,0.10f),minSizeAccL(-0.12f,-0.10f),maxSizeAccR(0.12f,0.10f),minSizeAccR(0.07f,-0.10f);
	//mMaxSizeAcc.x() +=isLeft ? -0.07f:0.07f;
	

        mSize += rotateMov;
        mSizeAcc = mSize - mPreSize;
	
	//cout<<mPreSize.x()<<'\t'<<mSizeAcc.x()<<endl;
	
	if(!goStop)
	    mMaxSizeAcc.y() *= 0.6f;
	
// 	cout<<mMaxSizeAcc<<endl;
	
        mSizeAcc.x() = clamp(mSizeAcc.x(),
                -mMaxSizeAcc.x(), mMaxSizeAcc.x());
        mSizeAcc.y() = clamp(mSizeAcc.y(),
                -mMaxSizeAcc.y(), mMaxSizeAcc.y());
        mSizeAcc.y() *= cosDeg(mDir);

	//mPreSize.x() -= isLeft ? -0.07f :0.07f;
	
	//mPreSize.x() -= isLeft ? 0.07f :-0.07f;
	
	//mSizeAcc.x() += isLeft ? -0.07f : 0.07f;
	
	
	// 2jie --added by ghd
	
	pre2SizeAcc = mSizeAcc -mPreSizeAcc;
	pre2SizeAcc.x() = clamp(pre2SizeAcc.x(),
                -mMaxSize2Acc.x(), mMaxSize2Acc.x());
        pre2SizeAcc.y() = clamp(pre2SizeAcc.y(),
                -mMaxSize2Acc.y(), mMaxSize2Acc.y());
	pre2SizeAcc *= cosDeg(mPreDir);
	
	//mSize *= 0.54f;
	
	//mSize.x() = clamp(mSize.x(), minSize.x(), maxSize.x());
	//mSize.y() = clamp(mSize.y(), minSize.y(), maxSize.y());
	//mSize *= 0.54f;
	
	
	
	
	//mSizeAcc.x() += isLeft ? -0.07f :0.07f;
	
	//cout<<isLeft<<'\t'<<mPreSize.x()<<'\t'<<mSizeAcc.x()<<endl;
	//mPreSize.x() =-mPreSize.x();
	//mSizeAcc.x() =-mSizeAcc.x();
	//pre2SizeAcc.x() =-pre2SizeAcc.x();
	//mSize.x() = 0.9f * 1
// 	cout<<goStop<<endl;
	

	//cout<<mSize<<endl;
	
		
// 	cout<<mSizeAcc<<endl;
	
	mSize.x() =mPreSize.x() + mSizeAcc.x();				//only modify y   --ghd
	mSize.y() =mPreSize.y() + mSizeAcc.y();
	
	///have bugs      --ghd test
// 	mSize.x()  =/*0.54f **/ mPreSize.x() + mSizeAcc.x();// +pre2SizeAcc.x();	//only modify y   --ghd
// 	if(goStop)
// 	    mSize.y() =/*0.92f * *//*0.75* */mPreSize.y() +/*0.5f **//*1.5f * */mSizeAcc.y() /*+1.5f * pre2SizeAcc.y()*/;//for more acc
// 	else
// 	    mSize.y() =/*0.92f * */1.12f* mPreSize.y() +0.5f *mSizeAcc.y();// +0.5f *pre2SizeAcc.y();	//for more stable
	//mPreSize.x() -= isLeft ? -0.07f :0.07f;
	//mPreSize.x() =-mPreSize.x();
	//mSizeAcc.x() =-mSizeAcc.x();
	//pre2SizeAcc.x() =-pre2SizeAcc.x();
	//cout<<mSize.x()<<'\t'<<mPreSize.x()<<'\t'<<mSizeAcc.x()<<'\t'<<pre2SizeAcc.x()<<endl;
	//cout<<mSize<<'\t';
	
        // restric walk side according to forward or backward
        float maxLen = maxSize.length();
        float len = mSize.length();
        bool slowDown = false;
        if (len > maxLen) {
            mSize *= (maxLen / len);
        } else if (len < 0.03f) {
            LOG_PRINTF("new", "slow down");
            slowDown = true;
        }
        /*
        float temp;
	temp =mDir */
// 	cout<<mDir<<endl;
        
        mTempSize =mSize;
	
	
	
	mSize.x() =clamp(mSize.x(),minSize.x(),maxSize.x());
	mSize.y() = clamp(mSize.y(), minSize.y(), maxSize.y());
// 	cout<<mSize<<endl;
	
        
        mSize.x() +=isLeft ? -0.06f : 0.06f ;
	
	//mSize.x() = -mSize.x();
        
        	//cout<<startPoint<<'\t'<<mSize<<endl;
	
//         if(mSize.length() >0.1f)
// 	{
// 	    cout<<"!!!"<<endl;
	mSize.x() =isLeft? clamp(mSize.x(), minSizeL.x(), maxSizeL.x()) : clamp(mSize.x(), minSizeR.x(), maxSizeR.x());
// 	}
// 	else
//  	    mSize.x() = clamp(mSize.x(), minSize.x(), maxSize.x());
        
// 	cout<<mSize<<endl;
	
        mSizeAcc = mSize - mPreSize;
	//cout<<mSize.y()<<endl;
       
        //cout<<'\t'<<mSize<<'\t'<<mPreSize<<'\t'<<pre2Size<<endl;
        
	//cout<<mSize<<endl;
	
// 	const float maxSizeLen =0.165f;
// 	AngDeg sIzeDir =mSize.angle();
// 	
// 	if(mSize.length() >maxSizeLen)
// 	{
// 	    mSize.x() =maxSizeLen*cosDeg<AngDeg>(sIzeDir);
// 	    mSize.y() =maxSizeLen*sinDeg<AngDeg>(sIzeDir);
// 	}

        // create sequences of one step
        //const float loft_foot_height = HUMANOID.getMinFootHeight();
        //float swingHeight = slowDown ? (loft_foot_height) : (loft_foot_height * 2.0f);

        //calc startPoint, see details in our documents about walk, rewritted dy dpf
	
	//mSize =isLeft ? Vector2f(-0.013f,0.105f) : Vector2f(0.07f,0.105f);

        TransMatrixf t0;
        t0.rotationZ(mPreDir);
        float feetx = (mIsLeft ? 1 : -1) * HUMANOID.getHalfFeetWidth();
        t0.p().x() = mPreSize.x();
        t0.p().y() = mPreSize.y();
        t0.p().x() += feetx;
        t0.transfer(Vector3f(-feetx, 0, 0));
        TransMatrixf t1;
        t1.identity();
        t1.p().x() = -feetx;
        TransMatrixf t2 = t0;
        t2.inverseTransfer(t1);

        Vector2f startPoint;
        startPoint.x() = t2.p().x() + feetx;
        startPoint.y() = t2.p().y();
        //Vector2f swingVec = mSize - startPoint;
	
	//mSize =Vector2f(0,0.16);
	
	seumath::TransMatrixf leftFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::L_FOOT);
	seumath::TransMatrixf rightFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::R_FOOT);
 	Vector2f leftFootRel2D=*(Vector2f*)(leftFootRelTrans.p().get());
        Vector2f rightFootRel2D=*(Vector2f*)(rightFootRelTrans.p().get());
	
	leftFootRel2D.x() += halfFeetWidth;
	rightFootRel2D.x() -= halfFeetWidth;
	
	//leftFootRel2D.y() +=isLeft ? -halfFeetLength:halfFeetLength;
	//rightFootRel2D.y() +=isLeft ? halfFeetLength:-halfFeetLength;
	
	//Vector2f test =isLeft ? leftFootRel2D - rightFootRel2D :rightFootRel2D - leftFootRel2D ;
	
	//cout<<isLeft<<endl;
	//cout<<leftFootRel2D<<endl;
	//cout<<rightFootRel2D<<endl;

	//startPoint =test;
        /*
        Vector2f startPoint(0.0f,0.0f);
        TransMatrixf leftFootLocalTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::L_FOOT);
        TransMatrixf rightFootLocalTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::R_FOOT);
        //dpf, rel pos is relative to torso, walk rel is the coordinate systems describled in our documents
        Vector2f leftFootRel2D=*(Vector2f*)(leftFootLocalTrans.p().get());
        Vector2f rightFootRel2D=*(Vector2f*)(rightFootLocalTrans.p().get());
        AngDeg leftFootAng=leftFootLocalTrans.rotatedAngZ();
        AngDeg rightFootAng=rightFootLocalTrans.rotatedAngZ();
        //walk rel coordinate's origin pos in rel pos
        //float halfFeetWidth=Humanoid.getHalfFeetWidth();
        Vector2f pos1=leftFootRel2D+Vector2f(halfFeetWidth*cosDeg(leftFootAng),halfFeetWidth*sinDeg(leftFootAng));
        Vector2f pos2=rightFootRel2D-Vector2f(halfFeetWidth*cosDeg(rightFootAng),halfFeetWidth*sinDeg(rightFootAng));
        //Vector2f walkRelToRel2D=mIsLeft?rightFootRel2D:leftFootRel2D;//walk rel coordinate's origin pos2D
        //float feetx = (mIsLeft?1:-1)*HUMANOID.getHalfFeetWidth();
        //startPoint in rel pos2D (ie. body rel pos2D)
        //Vector2f startPointRel2D=(mIsLeft?leftFootRel2D:rightFootRel2D) + Vector2f(feetx,0);
        startPoint=(pos1-pos2)*(mIsLeft?1.0:-1.0f);
         */
        /*
        //feedback test by dpf
        AngDeg leanAng=WM.getBoneTrans(robot::humanoid::Humanoid::TORSO).rotatedAngX();
        //danamic step size, test by dpf
        float addStepY=(-sinDeg(leanAng)*0.115-0.03)*0.25f;
        cout<<"addStepY: "<<addStepY<<endl;
        mSize.y()+=addStepY;
         */
        /// test src of dpf for walk, keep it !!!
        /*
        if(mIsLeft){
          mSize.x()+=-HUMANOID.getHalfFeetWidth();
        }
        else{
          mSize.x()+=HUMANOID.getHalfFeetWidth();
        }
        //rel pos startPoint
        */
        /*Vector2f startPoint=*(seumath::Vector2f*)(mIsLeft?leftFootRelTrans.p():rightFootRelTrans.p()).get();
        // from rel to walk rel
        startPoint=*(seumath::Vector2f*)(walkCoordinateToRelTrans.inverseTransform(Vector3f(startPoint.x(),startPoint.y(),0.0f))).get();
         */
        //startPoint=mPreSize*(-0.78);
        //test by g&q
        //cout<<"startPoint:"<<startPoint<<endl;
        
        //modify by g&q
        //cout<<mSize<<'\t'<<startPoint<<endl;
//         cout<<mDir<<endl;
        
        //cout<<mDir<<endl;
     
        //cout<<WM.getMyGyroRate()<<endl;
        
//         cout<<mDir<<endl;

// 	cout<<mSize.x()<<endl;
        
        Vector2f swingVec = mSize - startPoint;
	
	mSize =mTempSize;	//ghd
	//swingVec *= reduceStep;
	
	//cout<<swingVec<<endl;
        float totalTime = 0.1; // allen change from 0.2 to 0.15 June 18
        float stepTime = 1 * ceil(WM.getAverageStepTime() / serversetting::sim_step) * serversetting::sim_step;
        ///int n = 160; // = max(3, int(ceil(totalTime/stepTime)));
	int n=180;
        swingVec /= n;
        AngDeg swingAng = (mDir + mPreDir) / n;
        //float a = -(4 * swingHeight / n / n);
        //float b = 4 * swingHeight / n;
        AngDeg rotateFoot = 0;
        if (mSize.y() > 0.02) {
            if (stepTime < 0.05f) {
                rotateFoot = 12;
            } else {
               ///  the orogin by ghd  rotateFoot = 2.5f;
              rotateFoot=4.0f;
            }
        }
        
//         cout<<mMaxSize.y()<<mh1<<'\t'<<mh2<<'\t'<<'\t'<<mi1<<'\t'<<mi2<<endl;
        //new walk test
//         int i = 41;
// 	float h = 0.031;
// 	//h = 0.021f; //maybe it is a better one
// 	h = 0.025f;
// 	h *= reduceStep;
//         shared_ptr<Task> swingFoot1(new SwingFoot(mIsLeft, startPoint + swingVec*i,
//                 h,
//                 -mPreDir + swingAng*i,
//                 3,
//                 bodyHeight, 0.02, this));
//         mSubTaskList.push_back(swingFoot1);
// 
//         i = 86;
//         h = 0.028f;
// 	h = 0.032f;
// 	//h = 0.037; //maybe it is a better one
// 	h *= reduceStep;
//         shared_ptr<Task> swingFoot2(new SwingFoot(mIsLeft, startPoint + swingVec*i,
//                 h,
//                 -mPreDir + swingAng*i,
//                 3,
//                 bodyHeight, 0.02, this));
//         mSubTaskList.push_back(swingFoot2);
// 
//         i = 131;
//         h = 0.028;
// 	h = 0.028;
// 	//h = 0.032f;  //maybe it is a better one
// 	h *= reduceStep;
//         shared_ptr<Task> swingFoot3(new SwingFoot(mIsLeft, startPoint + swingVec*i,
//                 h,
//                 -mPreDir + swingAng*i,
//                 0,
//                 bodyHeight, 0.02, this));
//         mSubTaskList.push_back(swingFoot3);
// 
// 
//         i = 160;
//         h = 0.00;
// 	h *= reduceStep;
//         shared_ptr<Task> swingFoot4(new SwingFoot(mIsLeft, startPoint + swingVec*i,
//                 h,
//                 -mPreDir + swingAng*i,
//                 0,
//                 bodyHeight, 0.03, this));
//         mSubTaskList.push_back(swingFoot4);
        int i = mi1; 
	float h;// = 0.031;
	//h = 0.021f; //maybe it is a better one
	h = mh1;
	//h *= reduceStep;
        shared_ptr<Task> swingFoot1(new SwingFoot(mIsLeft, startPoint + swingVec*i,
                h,
                -mPreDir + swingAng*i,
                0,
                bodyHeight, mStep1Time, this));
        mSubTaskList.push_back(swingFoot1);

        i = mi2;
        //h = 0.028f;
	h = mh2;
	//h = 0.037; //maybe it is a better one
	//h *= reduceStep;
        shared_ptr<Task> swingFoot2(new SwingFoot(mIsLeft, startPoint + swingVec*i,
                h,
                -mPreDir + swingAng*i,
                0,
                bodyHeight, mStep2Time, this));
        mSubTaskList.push_back(swingFoot2);

//         i = 131;
//         h = 0.028;
// 	h = 0.028;
	
// 	//h = 0.032f;  //maybe it is a better one
// 	h *= reduceStep;
//         shared_ptr<Task> swingFoot3(new SwingFoot(mIsLeft, startPoint + swingVec*i,
//                 h,
//                 -mPreDir + swingAng*i,
//                 0,
//                 bodyHeight, 0.03, this));
//         mSubTaskList.push_back(swingFoot3);


        i = 160;
        h = 0.00;
	//h *= reduceStep;
        shared_ptr<Task> swingFoot4(new SwingFoot(mIsLeft, startPoint + swingVec*i,
                h,
                -mPreDir + swingAng*i,
                0,
                bodyHeight, mStep3Time, this));
        mSubTaskList.push_back(swingFoot4);

        /*
        for(int i=1; i<n+1; i++){
                float h = a*i*i + b*i;
                shared_ptr<Task> swingFoot
                        ( new SwingFoot(mIsLeft, startPoint + swingVec*i,
                                                        h,
                                                        -mPreDir + swingAng*i,
                                                        rotateFoot,
                                                        bodyHeight, stepTime, this ) );
                mSubTaskList.push_back(swingFoot);
        }
         */

        LOG_FLUSH;
    } //end of Step()

    bool Step::isDone() const {
        return Task::isDone();

    }

    shared_ptr<Action> Step::perform() {
        // try do sub task
        shared_ptr<Action> act = Task::perform();

        if (0 == act.get()) {
            // there is no sub task
            shared_ptr<JointAction> jact(new JointAction);
            jact->fill(0);
            act = shared_static_cast<Action > (jact);
        }

        return act;
    }

    bool Step::isTerminable() const {
        // if double support, the aciton can be breaked
        return mSubTaskList.size() < 2
                && WM.isTouch(/*FRID_LEFT_FOOT TODO*/0) && WM.isTouch(/*FRID_RIGHT_FOOT*/1);
    }


} //end of namespace task
