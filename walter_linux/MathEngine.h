#pragma once
#include <math.h>

class MathEngine
{
    
public:
    struct CartesianVector
    {
        double x;
        double y;
        double z;
        CartesianVector(double paramX, double paramY, double paramZ)
        {
            x = paramX;
            y = paramY;
            z = paramZ;
        }
        CartesianVector()
        {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
        bool operator != (CartesianVector const& obj) 
        {
            bool res;
            bool isEqual =
                (fabs(this->x - obj.x) < 0.05) &&
                (fabs(this->y - obj.y) < 0.05) &&
                (fabs(this->z - obj.z) < 0.05);
            res = !isEqual;
            return res;
        }
    };

	static double FromDegToRad(double angleInDegree);
	static double FromRadToDeg(double angleInRadians);
};