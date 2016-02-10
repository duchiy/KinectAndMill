#ifndef _CATMULLROMSPLINE_H
#define _CATMULLROMSPLINE_H
#include "CubicSpline.h"

class CatmullRomSpline : public CubicSpline
{
public:
  CatmullRomSpline ();
  virtual ~CatmullRomSpline ();
  CatmullRomSpline  (vector<double> controlPoints, int nParts);
  void initialize (vector<double> controlPoints, int nParts);
  int GetPoint(double t, vector<double> & vPoint);

private:
//  void Pts  (int i, double t, vector<double> ctlPts, vector<double> spline, int SplIndex);
  double virtual blend (int i, double t);

};
#endif
