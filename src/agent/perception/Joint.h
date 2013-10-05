/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/


#ifndef PERCEPTION_JOINT_H
#define PERCEPTION_JOINT_H

#include "math/Math.hpp"
#include "BasicPerception.h"
#include <iostream>

namespace perception {

class Joint: public BasicPerception
{
public:
    Joint(seumath::AngDeg a=0, seumath::AngDeg r=0):mAngle(a),mRate(r){};

    Joint(const sexp_t* sexp);
    
	Joint(const sexp_t* sexpAx, const sexp_t* sexpRt);
	
	~Joint(){};
	
	virtual bool update(const sexp_t* sexp);
    
    bool update(const sexp_t* sexpAx, const sexp_t* sexpRt);
		
	void setAngle(seumath::AngDeg ang) { mAngle = ang; }
	
	void setRate(seumath::AngDeg rate) { mRate = rate; }
	
	friend std::ostream& operator<<(std::ostream &stream, const Joint& j);

    seumath::AngDeg angle() const { return mAngle; }
	
    seumath::AngDeg rate() const { return mRate; }
	
	void angReverse() { mAngle = -mAngle; }
	
	Joint& operator+=(seumath::AngDeg ang);
	
private:
    seumath::AngDeg mAngle, mRate;
};

std::ostream& operator<<(std::ostream &stream, const Joint& j);

} // namespace perception

#endif // PERCEPTION_JOINT_H
