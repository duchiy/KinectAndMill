#ifndef _BEZIERSPLINE_H
#define _BEZIERSPLINE_H
#include "Spline.h"
class BezierSpline: public Spline
{
 
public:
  BezierSpline();
  virtual ~BezierSpline();
  BezierSpline (vector<double> controlPoints, int nParts);
  void initialize (vector<double> controlPoints, int nParts);
  vector<double> generate();
  int GetNumberOfPts();
  int GetPoint(double t, vector<double> & vPoint);
private:
  int Pts  (int i, double t, vector<double> ctlPts, vector<double> & spline, int SplIndex);
  double blend (int i, double t);

};
#endif
