/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _TASK_SWINGFOOT_H_
#define _TASK_SWINGFOOT_H_

#include "LowerLimbsMotion.h"


#define 	NORMALSUPPORT	0
#define 	LEFTSUPPORT	1
#define		RIGHTSUPPORT	2

namespace task{

    class SwingFoot : public LowerLimbsMotion
    {
    public:
        SwingFoot(bool isLeft,
                  const seumath::Vector2f& p,
                  float footHeight,
                  seumath::AngDeg ang,
                  seumath::AngDeg rotateFoot,
                  float bodyHeight,
                  float duration,
                  Task* primary,
		  // 0 : normal
		  // 1 : Left
		  // 2 : Right
		  //add by ghd
		  int supportMode =0
 		);
    private:
      bool mIsLeft;
    };

} // namespace task

#endif /* _TASK_SWINGFOOT_H_ */
