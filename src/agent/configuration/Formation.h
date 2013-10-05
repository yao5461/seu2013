/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_FORMATION_H
#define SOCCER_FORMATION_H

#include "Template.hpp"
#include "math/Math.hpp"
#include "parser/SexpParser.hpp"
#include "ClassException.hpp"

namespace configuration {
 
class Formation
{
public: 
	Formation();

	~Formation();

	//# We have the following players types:
	//# -----------------------------------
	enum PlayerType
	{
		PT_GOALKEEPER = 0,
		PT_DEFENDER_RIGHT,
		PT_DEFENDER_RIGHT_WING,
		PT_DEFENDER_LEFT,
		PT_DEFENDER_LEFT_WING,
		PT_MIDFIELDER_RIGHT,
		PT_MIDFIELDER_CENTER,
		PT_MIDFIELDER_LEFT,
		PT_ATTACKER_CENTRAL,
		PT_ATTACKER_LEFT,
		PT_ATTACKER_RIGHT,
		PT_JOYSTICK,
        PT_NULL
    };
    //# We have the following Formation types:
	//# Allen-----------------------------------
    enum FormationType
    {
        FT_HOME=0,
        FT_ATTACK_MIDDLE,
        FT_ATTACK_RIGHT,
        FT_ATTACK_LEFT,
        FT_DEFEND_MIDDLE,
        FT_DEFEND_RIGHT,
        FT_DEFEND_LEFT,
        FT_NULL
    };
	/** data struct of one palyer's formation */
	struct FormationData
	{
        std::string robot;
        PlayerType type;
        seumath::Vector3f beforeKickOffBeam; // (x, y, o)
        seumath::Vector3f ourGoalBeam;
        seumath::Vector3f oppGoalBeam;
        seumath::Vector3f oppGoalKick;
        seumath::Vector3f homePos;
        seumath::Vector3f attackPos;
        seumath::Vector3f defendPos;
        seumath::Vector2f attraction;
        seumath::Vector2f xRange;
        seumath::Vector2f yRange;
	float	behindBall;
		//Vector2f _ourGoalKickPosLeft;// position when our goal kick to the left side
		//Vector2f _ourGoalKickPosRight;//position when our goal kick to the right side
		//Vector2f _ourGoalieCatchedPos;//position when our goalie catched the ball

	//add by yao in 2013/05/22
	//this attribute added for record the type of hetero_nao
	//the value: -1 -- not hetero;  0 -- standrad nao;    1 -- hetero_type1 longer leg   2 -- hetero_type2 faster foot
	//the value define before is from server with hetero
	int heteroType;
	};

    void loadFile(const std::string& filename) throw(ClassException<Formation>);

    /** set the my formation data pointer by the unum and the formation name */
    void setMyFormation(const std::string& formationName, unsigned int num)
        throw(ClassException<Formation>);

    const FormationData& getMy() const 
        { return *mMyDataPtr;}

    
    static seumath::Vector3f calStrategicPosition( const FormationData& fdata,
                                                const seumath::Vector3f& posBall );
    

    seumath::Vector3f calMyStrategicPos( const seumath::Vector3f& posBall ) const;
   
 private:
    void setupStrPlayerTypeMap();
    
    void parseOneFormation(const sexp_t* sexp)throw(ClassException<Formation>);
    
    FormationData parseFormationData(const sexp_t* sexp) const throw(ClassException<Formation>);

    PlayerType getPlayerTypeByStr(const std::string& str) const
        { return getSecondValueByFirstRef(mStrPlayerTypeMap,str); }
    
	/** formation data */
    typedef std::map<std::string, std::map<unsigned int, FormationData> >
    TFormationDataMap;
    TFormationDataMap mDatas;

    /**  mapping from string name to PlayerType */
    typedef std::map<std::string, PlayerType> TStrPlayerTypeMap;
    TStrPlayerTypeMap mStrPlayerTypeMap;

    const FormationData* mMyDataPtr;
 

};

} // namespace configuration

#endif // SOCCER_FORMATION_H
