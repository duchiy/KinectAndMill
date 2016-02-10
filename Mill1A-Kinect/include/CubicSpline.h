#ifndef _CUBICSPLINE_H
#define _CUBICSPLINE_H
#include "Spline.h"
class CubicSpline: public Spline
{
 
public:
  CubicSpline();
  virtual ~CubicSpline();
  CubicSpline (vector<double> controlPoints, int nParts);
  CubicSpline (int iMethod, vector<double> controlPoints, int nParts);
  void initialize (vector<double> controlPoints, int nParts);
  int GetNumberOfPts();
  vector<double> generate();
  int GetPoint(double t, vector<double> & vPoint);
  int GetPointA(double t, vector<double> & vPoint);
  void Pts  (int i, double t, vector<double> ctlPts, vector<double> & spline, int SplIndex);
private:
  double virtual blend (int i, double t);

};
#endif
