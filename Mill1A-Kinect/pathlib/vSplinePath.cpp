#include "stdafx.h"
#include <math.h>
#include<vPathSegment.h>
#include<vSplinePath.h>
//using namespace vSplinePath;

vSpline::vSpline(): vPathSegment(), m_CtrlPts(), m_Length(), m_Parts(0)
{
	m_segType = enSpline;

};
vSpline::vSpline(vector<double> CtrlPts, int iParts)
{
	
	m_segType = enSpline;
	for (int i=0; i <= CtrlPts.size()-1; i++)
	{
		m_CtrlPts.push_back(CtrlPts[i]);
	}
	m_Parts=iParts;
};

vSpline::~vSpline()
{

};
int	vSpline::SetControlPts	 (vector<double> CtrlPts)
{
	int i=0;
	int iCtrlPts=0;
	iCtrlPts=CtrlPts.size();
	m_CtrlPts.clear();
	for(i=0 ; i <= iCtrlPts-1 ; i++)
	{
		m_CtrlPts.push_back(CtrlPts[i]);
	}
	return 0;	
};
int	vSpline::SetParts(int iParts)
{
	m_Parts = iParts;
	return 0;
};
double vSpline::mag (vector<double> Pt)
{
	double dmag=0.0;
//	if ( Pt.size() == 0)

	for (int i=0 ; i <= Pt.size()-1 ; i++)
		dmag = dmag + Pt[i]*Pt[i];

	dmag = sqrt(dmag);
	return dmag;
};

int	vSpline::SetLength( vector<double> Pts )
{
	if (!(Pts.size() >= 8))
		return -1;

	vector<double> Pt1(3);
	double	dLength	=0.0;
	int		iSize	=0;
	int		iParts	=0;

	while (iSize <= Pts.size()-4 )
	{
		Pt1[0]=Pts[iSize+3]-Pts[iSize];
		Pt1[1]=Pts[iSize+4]-Pts[iSize+1];
		Pt1[2]=Pts[iSize+5]-Pts[iSize+2];
		iSize=iSize+3;
		iParts++;
		dLength=dLength+mag(Pt1);
		if (iParts == m_Parts)
		{
			m_Length.push_back(dLength);	
			iParts=0;
			dLength=0.0;
		}
	}	

	for (iSize=0 ; iSize <= m_Length.size()-1; iSize++ )
	{
		m_fLength = m_fLength + m_Length[iSize];
	}
	
//	for (iSize=0 ; iSize <= m_Length.size()-1; iSize++ )
//	{
//		m_Length[iSize]=m_fLength/m_Length.size();
//	}


	return 0;
};
vSpline & vSpline::operator=(vSpline & rSpline)
{
	this->m_segType	=rSpline.m_segType;
	this->m_End		=rSpline.m_End;
	this->m_Start	=rSpline.m_Start;
	this->m_Length	=rSpline.m_Length;
	this->m_fLength	=rSpline.m_fLength;
	this->m_Parts	=rSpline.m_Parts;
	this->m_CtrlPts	=rSpline.m_CtrlPts;
	this->m_Type	=rSpline.m_Type;

	return *this;
};
vSpline & vSpline::operator=(vSpline * pSpline)
{
	this->m_segType	=pSpline->m_segType;
	this->m_End		=pSpline->m_End;
	this->m_Start	=pSpline->m_Start;
	this->m_Length	=pSpline->m_Length;
	this->m_fLength	=pSpline->m_fLength;
	this->m_Parts	=pSpline->m_Parts;
	this->m_CtrlPts	=pSpline->m_CtrlPts;
	this->m_Type	=pSpline->m_Type;

	return *this;
};
vSpline vSpline::operator+(vSpline & rSpline)
{
	vSpline resultSpline = *this;
	for (int i=0; i <= rSpline.m_CtrlPts.size(); i++)
	{
		resultSpline.m_CtrlPts[i]=resultSpline.m_CtrlPts[i]+rSpline.m_CtrlPts[i];
	}
	return resultSpline;

};
vSpline vSpline::operator+(vSpline * pSpline)
{
	
	vSpline resultSpline = *this;
	resultSpline=resultSpline + *pSpline;
	return resultSpline;
	
};

ofstream & operator << (ofstream & out, const vSpline & path)
{
	int nPoints;
	nPoints=path.m_CtrlPts.size();
	out << path.m_segType  << endl;
	out << path.m_Start[0] << '\t' << path.m_Start[1] << '\t' << path.m_Start[2] << endl;
	out << path.m_End[0] << '\t' << path.m_End[1] << '\t' << path.m_End[2] << endl;
	out << path.m_Parts << endl;
	out << path.m_Type << endl;
	out << nPoints << endl;
	for (int iPoints=0 ; iPoints <= nPoints-1; iPoints+=3)
	{
		out << path.m_CtrlPts[iPoints] << '\t' << path.m_CtrlPts[iPoints+1] << '\t' << path.m_CtrlPts[iPoints+2] << endl;
	}
	return out;
};
ifstream & operator >> (ifstream & in, vSpline & path)
{
	int nPoints;
	int segment_type;
	int spline_type;
	in >> segment_type;
	path.m_segType = (enType)segment_type;
	in >> path.m_Start[0] >> path.m_Start[1] >> path.m_Start[2] ;
	in >> path.m_End[0] >> path.m_End[1] >> path.m_End[2] ;
	in >> path.m_Parts;
	in >> spline_type;
	path.m_Type =(enSplineType)spline_type;
	in >> nPoints;
	int x, y, z;
	for (int iPoints=0 ; iPoints <= nPoints-1; iPoints+=3)
	{
//		in >> path.m_CtrlPts[iPoints] >> path.m_CtrlPts[iPoints+1] >> path.m_CtrlPts[iPoints+2] ;
		in >> x >> y >> z ;
		path.m_CtrlPts.push_back(x);path.m_CtrlPts.push_back(y);path.m_CtrlPts.push_back(z);
	}
	return in;
};
