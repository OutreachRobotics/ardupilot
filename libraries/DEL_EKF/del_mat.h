
// del_mat.h

#ifndef DEL_MAT_H
#define DEL_MAT_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <cstring>
#include "AP_Math/AP_Math.h"

/***************************************************************************
    Macro :
***************************************************************************/


/***************************************************************************
	Enumerations :
***************************************************************************/


/***************************************************************************
	Class :
***************************************************************************/

class Mat
{
public:
    Mat();
    Mat(uint8_t rowCnt, uint8_t colCnt);
    Mat(uint8_t rowCnt, uint8_t colCnt, double* lineMat);
    Mat(Mat const& other);
    ~Mat();
    float det();
    Mat inv();
    Mat t();
    Mat operator*(const Mat& other);
    Mat operator*(double other);
    Mat operator+(const Mat& other);
    Mat operator-(const Mat& other);
    Mat operator=(const Mat& other);
    double& operator[](uint16_t index);
    

private:
    uint8_t row;
    uint8_t col;
    double* content;

};

#endif