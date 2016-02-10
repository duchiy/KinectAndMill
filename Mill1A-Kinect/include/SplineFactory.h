//class SplineFactory
//{
  /**
   * Create a Bezier spline based on the given control points.
   * The generated curve starts in the first control point and ends
   * in the last control point.
   * 
   * @param controlPoints  Control points of spline (x0,y0,z0,x1,y1,z1,...).
   * @param nParts         Number of parts to divide each leg into.
   * @return               Spline (x0,y0,z0,x1,y1,z1,...).
   */
#ifndef _SPLINEFACTORY_H
#define _SPLINEFACTORY_H

#include "Spline.h"
#include "BezierSpline.h"
#include "CubicSpline.h"
#include "CatmullRomSpline.h"
class SplineFactory
{
private:
	BezierSpline		m_Bezier;
	CubicSpline			m_Cubic;
	CatmullRomSpline	m_CatmullRom;
	enSplineType		m_SplineType;
	bool				m_bInitialized;
	int					m_nSegments;

public:
	SplineFactory();
	bool IsInitialized();
	int uninitialize();
	int initialize	( enSplineType SplineType, vector<double> controlPoints, vector<double> Length, int nParts);
	int SetLength	( vector<double> & vLengths );
	int SetType		( enSplineType	SplineType );
	int GetPoint	( double dLength, vector<double> & vPoint );
	int GetControlPts (vector<double> & CtrlPts);
	int NumberOfSegments();

	vector<double> createBezier		(vector<double> controlPoints, int nParts);
	vector<double> createCubic		(vector<double> controlPoints, int nParts);
	vector<double> createCatmullRom (vector<double> controlPoints, int nParts);
	vector<double> createSpline		(vector<double> controlPoints, int nParts);
};

#endif