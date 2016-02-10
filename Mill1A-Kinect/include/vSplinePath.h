#ifndef __VCSPLINE_H__
#define __VCSPLINE_H__
#include "stdafx.h"
//--------------------------------------------------------------------------------
//	Class Name:  Arc
//	
//	Description:
//	This class is derived from the Segemnt class.  This class is used
//  in a link list to describe a motion path. 
//	
//
//
//	
//	History:
//
//
//
//
//
//
//
//
//--------------------------------------------------------------------------------

#include <vPathSegment.h>
#include <EnumTypes.h>

//namespace vSplinePath
//{
	class PATHLIB_API vSpline: public vPathSegment
	{
	public:
		vSpline();
		vSpline(vector<double> CtrlPts, int iParts);
		~vSpline();
		int	SetControlPts	 (vector<double> CtrlPts);
		int SetParts(int iParts);
		int SetLength( vector<double> SplinePts );
		vSpline &	operator=(vSpline & rSpline);
		vSpline &	operator=(vSpline * pSpline);
		vSpline 	operator+(vSpline & rSpline);
		vSpline 	operator+(vSpline * pSpline);
	
	private:
		double mag (vector<double> Pts);

	public:
		vector<double>  m_CtrlPts;		 //Control Points of Spline
		vector<double>  m_Length;		 //Length of each Spline in Control Points
		int				m_Parts;		//Number of Control Points between each control point
		enSplineType	m_Type;
		friend ofstream & operator << (ofstream & out, const vSpline & path);
		friend ifstream & operator >> (ifstream & in, vSpline & path);

	};
//};
#endif
