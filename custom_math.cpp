
// This code and data is in the public domain.


#include "custom_math.h"


double custom_math::d(const double &a, const double &b)
{
	return fabs(a - b);
}

double custom_math::d_3(const vector_3 &a, const vector_3 &b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

double custom_math::d_4(const vector_4 &a, const vector_4 &b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z) + (a.w - b.w)*(a.w - b.w));
}

