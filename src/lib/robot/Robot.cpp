/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Robot.h"
#include "importer/RSGImporter.h"

namespace robot {

    Robot* Robot::mTheRobot;
    
    Robot::Robot(const string& name)
        /*:mName(name)*/
    {
      //added by yao below 2013/05/23
      if( '#' == name[0] ) {
	heteroType = name[1];
	mName = name.substr(2, name.length()-2);
      } else {
	heteroType = '0';
	mName = name;
      }
      
      assemble();
    }

    Robot::~Robot()
    {
    }

    void Robot::assemble()
    {
        // create the robot node
        shared_ptr<Device> root( new Device() );
        root->setName("/");
        mDevice = Node<Device>::create(root);
  

        // read rsg file
        importer::RSGImporter importer;
// 	if ( importer.import(mName,mDevice) ){
// 	      cout<<"[Robot] '"<<mName<<"' is assembled."<<endl;
// 	}
	
	//modify by yao 2013/05/23
	if(importer.importHetero(mName, heteroType, mDevice)) {
	  cout<<"[Robot] '"<<mName<<"' for hetero type "<<heteroType<<" is assembled."<<endl;
	}
        
       
        // update the global matrix of Transform
        TransMatrixf mat;
        mat.identity();
        updateTransformGlobalMat(mDevice,mat);
        
        //cout<<*mDevice<<endl;
    }
    
    void Robot::updateTransformGlobalMat(shared_ptr<Node<Device> > dev,
                                         TransMatrixf mat) const
    {
        // try to cast it to Transform
        shared_ptr<Transform> t = shared_dynamic_cast<Transform>(dev->data());
        if ( 0 != t.get() ){
            // it is a transform, apply the matrix
            mat.transfer(t->getLocalMat());
            t->setGlobalMat(mat);
        }
        FOR_EACH(iter, dev->children() ){
            updateTransformGlobalMat(*iter,mat);
        }
    }

    void Robot::destrut()
    {
        if ( NULL != mTheRobot ){
            cout<<"[Robot] destruct "<<mTheRobot->getName()<<endl;
            delete mTheRobot;
        }
    }
    
} /* namespace robot */
