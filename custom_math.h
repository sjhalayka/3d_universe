
#ifndef custom_math_h
#define custom_math_h


#include <limits>
using std::numeric_limits;

#include <vector>
using std::vector;

#include <iostream>
using std::cout;
using std::endl;

#include <cmath>
#include <cstdlib>
#include <GL/glut.h>


namespace custom_math
{
	class vector_3;
	class vector_4;

	class triangle;
	class tetrahedron;

	class indexed_line_segment;
	class indexed_triangle;

	const double pi = 3.14159265358979323846;
	const double pi_half = pi/2;
	const double pi_2 = 2*pi;
	const double epsilon = 1e-6;

	double d(const double &a, const double &b);
	double d_3(const vector_3 &a, const vector_3 &b);
	double d_4(const vector_4 &a, const vector_4 &b);
};



class custom_math::vector_3
{
public:
	double x, y, z;


	vector_3(const double& src_x, const double& src_y, const double& src_z)
	{
		x = src_x;
		y = src_y;
		z = src_z;
	}


	vector_3(void)
	{
		x = 0;
		y = 0;
		z = 0;
	}

	void zero(void)
	{
		x = y = z = 0;
	}

	void rotate_x(const double& radians)
	{
		double t_y = y;

		y = t_y * cos(radians) + z * sin(radians);
		z = t_y * -sin(radians) + z * cos(radians);
	}

	void rotate_y(const double& radians)
	{
		double t_x = x;

		x = t_x * cos(radians) + z * -sin(radians);
		z = t_x * sin(radians) + z * cos(radians);
	}

	vector_3 operator+(const vector_3& rhs)
	{
		return vector_3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	vector_3 operator-(const vector_3& rhs)
	{
		return vector_3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	vector_3 operator*(const vector_3& rhs)
	{
		return vector_3(x * rhs.x, y * rhs.y, z * rhs.z);
	}

	vector_3 operator*(const double& rhs)
	{
		return vector_3(x * rhs, y * rhs, z * rhs);
	}

	vector_3 operator/(const double& rhs)
	{
		return vector_3(x / rhs, y / rhs, z / rhs);
	}

	vector_3& operator=(const vector_3& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	vector_3& operator+=(const vector_3& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}

	vector_3& operator*=(const vector_3& rhs)
	{
		x *= rhs.x; y *= rhs.y; z *= rhs.z;
		return *this;
	}

	vector_3& operator*=(const double& rhs)
	{
		x *= rhs; y *= rhs; z *= rhs;
		return *this;
	}

	vector_3 operator-(void)
	{
		vector_3 temp;
		temp.x = -x;
		temp.y = -y;
		temp.z = -z;

		return temp;
	}

	double length(void) const
	{
		return sqrt(self_dot());
	}

	vector_3& normalize(void)
	{
		double len = length();

		if (len != 1)
		{
			x /= len;
			y /= len;
			z /= len;
		}

		return *this;
	}

	double dot(const vector_3& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	double self_dot(void) const
	{
		return x * x + y * y + z * z;
	}

	vector_3 cross(const vector_3& rhs) const
	{
		vector_3 cross;
		cross.x = y * rhs.z - rhs.y * z;
		cross.y = z * rhs.x - rhs.z * x;
		cross.z = x * rhs.y - rhs.x * y;

		return cross;
	}


};


class custom_math::vector_4
{
public:
	double x, y, z, w;

	inline bool operator==(const vector_4& right) const
	{
		if (right.x == x &&
			right.y == y &&
			right.z == z &&
			right.w == w)
		{
			return true;
		}

		return false;
	}

	vector_4(const double& src_x, const double& src_y, const double& src_z, const double& src_w)
	{
		x = src_x;
		y = src_y;
		z = src_z;
		w = src_w;
	}


	vector_4(void)
	{
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	}

	
	void zero(void)
	{
		x = y = z = w = 0;
	}

	vector_4 operator+(const vector_4& rhs)
	{
		return vector_4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	vector_4 operator-(const vector_4& rhs)
	{
		return vector_4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	vector_4 operator*(const vector_4& rhs)
	{
		return vector_4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
	}

	vector_4 operator*(const double& rhs)
	{
		return vector_4(x * rhs, y * rhs, z * rhs, w * rhs);
	}

	vector_4 operator/(const double& rhs)
	{
		return vector_4(x / rhs, y / rhs, z / rhs, w / rhs);
	}

	vector_4& operator=(const vector_4& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		return *this;
	}

	vector_4& operator+=(const vector_4& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
		return *this;
	}

	vector_4& operator*=(const vector_4& rhs)
	{
		x *= rhs.x; y *= rhs.y; z *= rhs.z; w *= rhs.w;
		return *this;
	}

	vector_4& operator*=(const double& rhs)
	{
		x *= rhs; y *= rhs; z *= rhs; w *= rhs;
		return *this;
	}

	vector_4 operator-(void)
	{
		vector_4 temp;
		temp.x = -x;
		temp.y = -y;
		temp.z = -z;
		temp.w = -w;

		return temp;
	}

	double length(void) const
	{
		return sqrt(self_dot());
	}

	vector_4& normalize(void)
	{
		double len = length();

		if (len != 1)
		{
			x /= len;
			y /= len;
			z /= len;
			w /= len;
		}

		return *this;
	}

	double dot(const vector_4& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
	}

	double self_dot(void) const
	{
		return x * x + y * y + z * z + w * w;
	}


	inline bool operator<(const vector_4 &right) const
	{
		if (right.x > x)
			return true;
		else if (right.x < x)
			return false;

		if (right.y > y)
			return true;
		else if (right.y < y)
			return false;

		if (right.z > z)
			return true;
		else if (right.z < z)
			return false;

		if (right.w > w)
			return true;
		else if (right.w < w)
			return false;

		return false;
	}
};


class indexed_vector_3 : public custom_math::vector_3
{
public:
	inline indexed_vector_3(void) { x = y = z = 0; index = 0; }
	inline indexed_vector_3(const float src_x, const float src_y, const float src_z, const size_t src_index) { x = src_x; y = src_y; z = src_z; index = src_index; }
	inline indexed_vector_3(const float src_x, const float src_y, const float src_z) { x = src_x; y = src_y; z = src_z; index = 0; }

	inline bool operator<(const vector_3& right) const
	{
		if (right.x > x)
			return true;
		else if (right.x < x)
			return false;

		if (right.y > y)
			return true;
		else if (right.y < y)
			return false;

		if (right.z > z)
			return true;
		else if (right.z < z)
			return false;

		return false;
	}

	inline bool operator>(const vector_3& right) const
	{
		if (right.x < x)
			return true;
		else if (right.x > x)
			return false;

		if (right.y < y)
			return true;
		else if (right.y > y)
			return false;

		if (right.z < z)
			return true;
		else if (right.z > z)
			return false;

		return false;
	}

	size_t index;
};





class custom_math::indexed_line_segment
{
public:
	size_t vertex_indices[2];

	inline bool operator<(const indexed_line_segment& right) const
	{
		if (right.vertex_indices[0] > vertex_indices[0])
			return true;
		else if (right.vertex_indices[0] < vertex_indices[0])
			return false;

		if (right.vertex_indices[1] > vertex_indices[1])
			return true;
		else if (right.vertex_indices[1] < vertex_indices[1])
			return false;

		return false;
	}


	inline bool operator==(const indexed_line_segment& right) const
	{
		if (right.vertex_indices[0] == vertex_indices[0] &&
			right.vertex_indices[1] == vertex_indices[1])
		{
			return true;
		}

		return false;
	}
};




class custom_math::indexed_triangle
{
public:
	size_t vertex_indices[3];

	inline bool operator<(const indexed_triangle& right) const
	{
		if (right.vertex_indices[0] > vertex_indices[0])
			return true;
		else if (right.vertex_indices[0] < vertex_indices[0])
			return false;

		if (right.vertex_indices[1] > vertex_indices[1])
			return true;
		else if (right.vertex_indices[1] < vertex_indices[1])
			return false;

		if (right.vertex_indices[2] > vertex_indices[2])
			return true;
		else if (right.vertex_indices[2] < vertex_indices[2])
			return false;

		return false;
	}


	inline bool operator==(const indexed_triangle& right) const
	{
		if (right.vertex_indices[0] == vertex_indices[0] &&
			right.vertex_indices[1] == vertex_indices[1] &&
			right.vertex_indices[2] == vertex_indices[2])
		{
			return true;
		}

		return false;
	}
};


#endif

