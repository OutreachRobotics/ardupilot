// del_mat.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_mat.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/


/***************************************************************************
	Function definition :
***************************************************************************/

Mat::Mat()
{
    row = 0;
    col = 0;
    content = 0;
}

Mat::Mat(uint8_t rowCnt, uint8_t colCnt)
{
    row = rowCnt;
    col = colCnt;
    content = (double*)malloc(row*col*sizeof(double));
    memset(content, 0.0, row*col*sizeof(double));
}

Mat::Mat(uint8_t rowCnt, uint8_t colCnt, double* lineMat)
{
    row = rowCnt;
    col = colCnt;
    content = (double*)malloc(row*col*sizeof(double));
    memcpy(content, lineMat, row*col*sizeof(double));
}

Mat::Mat(Mat const& other)
{
    row = other.row;
    col = other.col;
    content = (double*)malloc(row*col*sizeof(double));
    memcpy(content, other.content, row*col*sizeof(double));
}

Mat::~Mat()
{
    free(content);
}
float Mat::det()
{
    float ret = -1;

    if(row==2 && col == 2)
    {
        ret = content[0]*content[3] - content[1]*content[2];
    }
    else if (row==3 && col==3)
    {
        ret = content[0]*(content[4]*content[8]-content[5]*content[7]) -
            content[1]*(content[3]*content[8]-content[5]*content[6]) +
            content[2]*(content[3]*content[7]-content[4]*content[6]);
    }
    else if(row==4 && col==4)
    {
        double array1[] = {content[5],content[6],content[7],content[9],content[10],content[11],
                            content[13],content[14],content[15]};
        double array2[] = {content[4],content[6],content[7],content[8],content[10],content[11],
                            content[12],content[14],content[15]};
        double array3[] = {content[4],content[5],content[7],content[8],content[9],content[11],
                            content[12],content[13],content[15]};
        double array4[] = {content[4],content[5],content[6],content[8],content[9],content[10],
                            content[12],content[13],content[14]};
        Mat mat1 = Mat(3,3,array1);
        Mat mat2 = Mat(3,3,array2);
        Mat mat3 = Mat(3,3,array3);
        Mat mat4 = Mat(3,3,array4);
        ret = content[0]*mat1.det()-content[1]*mat2.det()+content[2]*mat3.det()-content[3]*mat4.det();        
    }
    return ret;
}
Mat Mat::inv()
{
    Mat inv = Mat(row,col);
    float det;

    if(row==2 && col==2)
    {
        det = content[0]*content[3]-content[1]*content[2];
        inv.content[0] = content[3];
        inv.content[1] = -content[1];
        inv.content[2] = -content[2];
        inv.content[3] = content[0];
        det = 1.0f / det;

        for (uint8_t i = 0; i < row*col; i++)
        {
            inv.content[i] = inv.content[i] * det;
        }
    }
    else if(row==3 && col==3)
    {
        det = this->det();
        inv.content[0] = (content[4] * content[8] - content[7] * content[5]) / det;
        inv.content[1] = (content[2] * content[7] - content[1] * content[8]) / det;
        inv.content[2] = (content[1] * content[5] - content[2] * content[4]) / det;
        inv.content[3] = (content[5] * content[6] - content[3] * content[8]) / det;
        inv.content[4] = (content[0] * content[8] - content[2] * content[6]) / det;
        inv.content[5] = (content[3] * content[2] - content[0] * content[5]) / det;
        inv.content[6] = (content[3] * content[7] - content[6] * content[4]) / det;
        inv.content[7] = (content[6] * content[1] - content[0] * content[7]) / det;
        inv.content[8] = (content[0] * content[4] - content[3] * content[1]) / det;
    }
    else if(row==4 && col==4)
    {
        inv.content[0] = content[5]  * content[10] * content[15] -
        content[5]  * content[11] * content[14] -
        content[9]  * content[6]  * content[15] +
        content[9]  * content[7]  * content[14] +
        content[13] * content[6]  * content[11] -
        content[13] * content[7]  * content[10];

        inv.content[4] = -content[4]  * content[10] * content[15] +
        content[4]  * content[11] * content[14] +
        content[8]  * content[6]  * content[15] -
        content[8]  * content[7]  * content[14] -
        content[12] * content[6]  * content[11] +
        content[12] * content[7]  * content[10];

        inv.content[8] = content[4]  * content[9] * content[15] -
        content[4]  * content[11] * content[13] -
        content[8]  * content[5] * content[15] +
        content[8]  * content[7] * content[13] +
        content[12] * content[5] * content[11] -
        content[12] * content[7] * content[9];

        inv.content[12] = -content[4]  * content[9] * content[14] +
        content[4]  * content[10] * content[13] +
        content[8]  * content[5] * content[14] -
        content[8]  * content[6] * content[13] -
        content[12] * content[5] * content[10] +
        content[12] * content[6] * content[9];

        inv.content[1] = -content[1]  * content[10] * content[15] +
        content[1]  * content[11] * content[14] +
        content[9]  * content[2] * content[15] -
        content[9]  * content[3] * content[14] -
        content[13] * content[2] * content[11] +
        content[13] * content[3] * content[10];

        inv.content[5] = content[0]  * content[10] * content[15] -
        content[0]  * content[11] * content[14] -
        content[8]  * content[2] * content[15] +
        content[8]  * content[3] * content[14] +
        content[12] * content[2] * content[11] -
        content[12] * content[3] * content[10];

        inv.content[9] = -content[0]  * content[9] * content[15] +
        content[0]  * content[11] * content[13] +
        content[8]  * content[1] * content[15] -
        content[8]  * content[3] * content[13] -
        content[12] * content[1] * content[11] +
        content[12] * content[3] * content[9];

        inv.content[13] = content[0]  * content[9] * content[14] -
        content[0]  * content[10] * content[13] -
        content[8]  * content[1] * content[14] +
        content[8]  * content[2] * content[13] +
        content[12] * content[1] * content[10] -
        content[12] * content[2] * content[9];

        inv.content[2] = content[1]  * content[6] * content[15] -
        content[1]  * content[7] * content[14] -
        content[5]  * content[2] * content[15] +
        content[5]  * content[3] * content[14] +
        content[13] * content[2] * content[7] -
        content[13] * content[3] * content[6];

        inv.content[6] = -content[0]  * content[6] * content[15] +
        content[0]  * content[7] * content[14] +
        content[4]  * content[2] * content[15] -
        content[4]  * content[3] * content[14] -
        content[12] * content[2] * content[7] +
        content[12] * content[3] * content[6];

        inv.content[10] = content[0]  * content[5] * content[15] -
        content[0]  * content[7] * content[13] -
        content[4]  * content[1] * content[15] +
        content[4]  * content[3] * content[13] +
        content[12] * content[1] * content[7] -
        content[12] * content[3] * content[5];

        inv.content[14] = -content[0]  * content[5] * content[14] +
        content[0]  * content[6] * content[13] +
        content[4]  * content[1] * content[14] -
        content[4]  * content[2] * content[13] -
        content[12] * content[1] * content[6] +
        content[12] * content[2] * content[5];

        inv.content[3] = -content[1] * content[6] * content[11] +
        content[1] * content[7] * content[10] +
        content[5] * content[2] * content[11] -
        content[5] * content[3] * content[10] -
        content[9] * content[2] * content[7] +
        content[9] * content[3] * content[6];

        inv.content[7] = content[0] * content[6] * content[11] -
        content[0] * content[7] * content[10] -
        content[4] * content[2] * content[11] +
        content[4] * content[3] * content[10] +
        content[8] * content[2] * content[7] -
        content[8] * content[3] * content[6];

        inv.content[11] = -content[0] * content[5] * content[11] +
        content[0] * content[7] * content[9] +
        content[4] * content[1] * content[11] -
        content[4] * content[3] * content[9] -
        content[8] * content[1] * content[7] +
        content[8] * content[3] * content[5];

        inv.content[15] = content[0] * content[5] * content[10] -
        content[0] * content[6] * content[9] -
        content[4] * content[1] * content[10] +
        content[4] * content[2] * content[9] +
        content[8] * content[1] * content[6] -
        content[8] * content[2] * content[5];

        det = content[0] * inv.content[0] + content[1] * inv.content[4] +
             content[2] * inv.content[8] + content[3] * inv.content[12];    
        
        det = 1.0f / det;

        for (uint8_t i = 0; i < row*col; i++)
        {
            inv.content[i] = inv.content[i] * det;
        }
    }
    return inv;
}
Mat Mat::t()
{
    Mat t(col,row);
    for(uint8_t i=0;i<col;i++)
    {
        for(uint8_t j=0;j<row;j++)
        {
            t.content[i*row+j] = content[i+j*col];
        }
    }
    return t;
}

Mat Mat::operator*(const Mat& other)
{
    Mat ret(row,other.col);
    for(uint8_t i=0;i<row;i++)
    {
        for(uint8_t j=0;j<other.col;j++)
        {
            for(uint8_t k=0;k<col;k++) 
            {
                ret.content[i*other.col+j] += content[col*i+k] * other.content[k*other.col+j];
            }
        }
    }
    return ret;
}

Mat Mat::operator*(double other)
{
    Mat temp = *this;
    for(uint8_t i=0;i<row;i++)
    {
        for(uint8_t j=0;j<col;j++)
        {
            temp.content[i*col+j] *= other;
        }
    }
    return temp;
}

Mat Mat::operator+(const Mat& other)
{
    Mat ret(row,col);
    for(uint8_t i=0;i<row;i++)
    {
        for(uint8_t j=0;j<col;j++)
        {
            ret.content[i+j*col] = content[i+j*col] + other.content[i+j*col];
        }
    }
    return ret;
}

Mat Mat::operator-(const Mat& other)
{
    Mat ret(row,col);
    for(uint8_t i=0;i<row;i++)
    {
        for(uint8_t j=0;j<col;j++)
        {
            ret.content[i+j*col] = content[i+j*col] - other.content[i+j*col];
        }
    }
    return ret;
}

Mat Mat::operator=(const Mat& other)
{
    if (this == &other)
    {
        return *this;
    }   
    row = other.row;
    col = other.col;
    free(content);
    content = (double*)malloc(row*col*sizeof(double));
    memcpy(content, other.content, row*col*sizeof(double));
    return *this;
}

double & Mat::operator[](uint16_t index)
{
    return content[index];
}
