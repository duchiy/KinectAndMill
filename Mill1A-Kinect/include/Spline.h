#include "stdafx.h"
#include <vector>
#include <EnumTypes.h>
using namespace std;

#ifndef _SPLINE_H
#define _SPLINE_H


class Spline
{
public:
	Spline(): m_nParts(0), m_nSegments(0), m_iPoints(0),
			  m_nCurrentPart(0), m_nCurrentSegment(0), m_bLastPoint(0), m_CurrentLength(0)
	{};
	int SetLength(vector<double> & vLengths)
	{
		m_SegmentLength=vLengths;
		return 0;
	};		

public:
	vector<double>	m_controlPoints;

    int				m_nParts;
    int				m_nSegments; //  Number of Segments in the spline
	enSplineType	m_nType;
	int				m_iPoints;
public:
	bool			m_bLastPoint;	// This flag is set true after the last point has been retrieved

	int				m_nCurrentPart;
    int				m_nCurrentSegment;			//  The current Segment in the spline
    int				m_controlPointsPerSegment;  // The number of Control Points in a Bezier Spline
	vector<double>	m_SegmentLength;			// The lengths of the segments in a spline
	double			m_CurrentLength;			// This is the cummulative sum of the m_SegmentLength

	virtual int		GetNumberOfPts()
	{
		return 0;
	};
	virtual vector<double> generate()
	{
		return vector<double>(0.0);
	};
};

#endif

