#ifndef _VECTOR4_H
#define _VECTOR4_H
#include <vector>
#include <cmath>
using namespace std;

class Vector4
{
private:
	vector<double> m_Vec;
    void initialize();

public:
	Vector4();
	virtual ~Vector4();
	Vector4 (double v1, double v2, double v3, double v4);
	Vector4 (Vector4 & vector4);
	void set (double v1, double v2, double v3, double v4);
	void set (Vector4 & vector);
	double getElement (int i);
	void setElement (int i, double value);
	Vector4 & operator=(const Vector4 &rhs);
	bool operator==(const Vector4 &rhs);
	vector<double>& operator[] (int i);

};

#endif
