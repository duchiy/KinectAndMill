#ifndef _MATRIX4x4_H
#define _MATRIX4x4_H
#include <vector>
#include <cmath>
using namespace std;
#include "Vector4.h"
//template <typename T>
class Matrix4x4
{
 
public:
	Matrix4x4();
	virtual ~Matrix4x4();
	Matrix4x4 (vector<double> m);

	Matrix4x4 (Matrix4x4 & matrix);
	Matrix4x4 (double m00, double m01, double m02, double m03,
						double m10, double m11, double m12, double m13,
						double m20, double m21, double m22, double m23,
						double m30, double m31, double m32, double m33);
	void setIdentity();

	void set (Matrix4x4 & matrix);
	void set (vector<double> m);
	void set (double m00, double m01, double m02, double m03,
					   double m10, double m11, double m12, double m13,
					   double m20, double m21, double m22, double m23,
					   double m30, double m31, double m32, double m33);

	vector<double> get();
	Matrix4x4 & operator=(const Matrix4x4 &rhs);
	vector<double>& operator[] (int i);

	bool operator==(const Matrix4x4 &rhs);

	double getElement (int i, int j);
	void setElement (int i, int j, double value);
	void add (Matrix4x4 matrix);
	Matrix4x4 & add (Matrix4x4 m1, Matrix4x4 m2);
	void multiply (Matrix4x4 matrix);
	Matrix4x4 & multiply (Matrix4x4 m1, Matrix4x4 m2);
	Vector4 multiply (Vector4 vector4);
	void Matrix4x4::multiply (Vector4 & product, Vector4 vector4);

	vector<double> transformPoint (vector<double> point);  
	void transformPoints (vector<double> & point);
	void transformXyPoints (vector<double> & point);
	void transformPoints (vector<int> & points);
	void transformXyPoints (vector<int> & points);
	void translate (double dx, double dy, double dz);
	void translate (double dx, double dy);
	void rotateX (double angle);
	void rotateY (double angle);
	void rotateZ (double angle);
	void rotate (double angle, vector<double> p0, vector<double> p1);

	void scale (double xScale, double yScale, double zScale);
	void scale (double xScale, double yScale, double zScale,
						 vector<double> fixedPoint);

	void invert();
	Matrix4x4 inverse (Matrix4x4 matrix);
	Vector4 solve (Vector4 vector);
	void setWorld2DeviceTransform (vector<double> w0, vector<double> w1, vector<double> w2,
								   int x0, int y0, int width, int height);
/*
	String toString();
*/
private:

	vector<double> m_Mat;
	void initialize();

};
#endif
