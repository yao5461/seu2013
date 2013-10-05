/* 
 * File:   SegmentPol.h
 * Author: robocup
 *
 * Created on 2011年3月30日, 下午2:06
 */
#ifndef SEGMENTPOL_H
#define	SEGMENTPOL_H
#include "BasicPerception.h"
#include"math/Math.hpp"
namespace perception {

    class SegmentPol : public BasicPerception {
    public:
        //
        //        LinePol(seumath::Vector3f startP, seumath::Vector3f endP) {
        //        };
        // LinePol(const sexp_t* sexp);

        SegmentPol() {
            mLength = -1;
        };

        ~SegmentPol() {
        };


        virtual bool update(const sexp_t* sexp);

        friend std::ostream & operator<<(std::ostream &stream, const SegmentPol& l);

        const seumath::Vector3f& p0() const {
            return mP[0];
        }

        const seumath::Vector3f& p1() const {
            return mP[1];
        }

        float length() {
            if (mLength <= 0) {
                mLength = (pol2xyz(mP[0]) - pol2xyz(mP[1])).length();
                return mLength;
            } else {
                return mLength;
            }
        }

    private:
        seumath::Vector3f mP[2];
        float mLength;
    };

    std::ostream & operator<<(std::ostream &stream, SegmentPol& l);

} // namespace perception



#endif	/* SEGMENTPOL_H */

