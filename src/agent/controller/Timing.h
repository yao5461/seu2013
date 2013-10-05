/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                       -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Timing.h,v 1.1 2007/03/13 07:39:26 xy Exp $
 *
 ****************************************************************************/

#ifndef CONTROLLER_TIMING_H
#define CONTROLLER_TIMING_H

#include <boost/shared_ptr.hpp>
#include "../action/JointAction.h"
#include "../perception/Perception.h"
#include <fstream>
#include <sstream>
#include "string"
namespace controller {

static std::list<std::map<unsigned int, seumath::AngDeg> > mJointAngleMap;
  
class Timing
{
public:

	~Timing(){}

	static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const perception::JointPerception& p1, float deltaTime,const std::string& task);

    static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const std::map<unsigned int, seumath::AngDeg>& p1, float deltaTime,const std::string& task);

	static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const perception::JointPerception& p1, float deltaTime,const std::string& task,int errAng);

    static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const std::map<unsigned int, seumath::AngDeg>& p1, float deltaTime,const std::string& task,int errAng);

    /**
     * The trajectory of each joint is expressed as a cubic polynomial,
     * to make the motion of robot smooth.
     *
     * @param p0 current joint angle and velocity
     * @param pf desired joint angle
     * @param tf the duration for reach desired time
     * @param pff the next desired joint angle,
     * used for determining veocity at tf
     * @param tff the duration between pf and pff
     *
     * @return the action of current state
     */
    static boost::shared_ptr<action::Action>
    control(const perception::JointPerception& p0,
            const perception::JointPerception& pf, float tf,
            const perception::JointPerception& pff, float tff);

    static boost::shared_ptr<action::Action>
    control(const perception::JointPerception& p0,
            const std::map<unsigned int, seumath::AngDeg>& pf, float tf,
            const std::map<unsigned int, seumath::AngDeg>& pff, float tff);

    static boost::shared_ptr<action::Action>
    dpfcontrol(const perception::JointPerception& p0,
            const std::map<unsigned int, seumath::AngDeg>& pf, float tf,
            const std::map<unsigned int, seumath::AngDeg>& pff, float tff);

    /**
     * let this jid to the angle
     */
	static void controlJointAngle(boost::shared_ptr<action::Action> act,
                                  unsigned int jid,
                                  seumath::AngDeg ang0,
                                  seumath::AngDeg ang1,
                                  float useTime=serversetting::sim_step);

    /**
     * 设置这个jid相对于自己坐标系y，也就是全局坐标系中x的夹角为angle
     */
    static void controlJointToAngleY(boost::shared_ptr<action::Action> act,
                                     unsigned int jid,
                                     const seumath::TransMatrixf& mat,
                                     seumath::AngDeg angle,
                                     float useTime=serversetting::sim_step);
    
  	//test by dpf, to storage the old and present and future caculate angle rates of joints
    

private:
	Timing(); // do not try to create an instance

	typedef std::map<unsigned int, perception::Joint> TJointMap;
};

} // namespace controller

#endif // CONTROLLER_TIMING_H
