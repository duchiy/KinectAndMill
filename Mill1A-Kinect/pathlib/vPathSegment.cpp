#include "stdafx.h"
#include "vPathSegment.h"

vPathSegment::vPathSegment() : m_segType(enOrigin), m_fLength(0.0), m_Start(3), m_End(3)
{
	
};
vPathSegment & vPathSegment::operator=(vPathSegment & rPathSeg)
{
	this->m_segType=rPathSeg.m_segType;
	this->m_End[0]=rPathSeg.m_End[0];
	this->m_End[1]=rPathSeg.m_End[1];
	this->m_End[2]=rPathSeg.m_End[2];
	this->m_Start[0]=rPathSeg.m_Start[0];
	this->m_Start[1]=rPathSeg.m_Start[1];
	this->m_Start[2]=rPathSeg.m_Start[2];
	this->m_fLength=rPathSeg.m_fLength;
	return *this;	
};
vPathSegment & vPathSegment::operator=(vPathSegment * pPathSeg)
{
	this->m_segType=pPathSeg->m_segType;
	this->m_End[0]=pPathSeg->m_End[0];
	this->m_End[1]=pPathSeg->m_End[1];
	this->m_End[2]=pPathSeg->m_End[2];
	this->m_Start[0]=pPathSeg->m_Start[0];
	this->m_Start[1]=pPathSeg->m_Start[1];
	this->m_Start[2]=pPathSeg->m_Start[2];
	this->m_fLength=pPathSeg->m_fLength;
	return *this;	
	
	
};

vPathSegment::~vPathSegment()
{


};
int vPathSegment::SetStart(float fX, float fY, float fZ)
{
	m_Start[0]=fX;
	m_Start[1]=fY;
	m_Start[2]=fZ;
	return 0;
};
int vPathSegment::GetStart(float & fX, float & fY, float & fZ)
{
	 fX =m_Start[0];
	 fY =m_Start[1];
	 fZ =m_Start[2];
	return 0;
};
int vPathSegment::SetEnd(float fX, float fY, float fZ)
{
	
	m_End[0]=fX;
	m_End[1]=fY;
	m_End[2]=fZ;
	return 0;
};
int vPathSegment::SetEnd(fPt fEnd)
{
	
	m_End[0]=fEnd[0];
	m_End[1]=fEnd[1];
	m_End[2]=fEnd[2];

	return 0;	
};
int vPathSegment::SetEnd(Point End)
{
	m_End[0]=End.X;
	m_End[1]=End.Y;
	m_End[2]=End.Z;
	return 0;	
};
int vPathSegment::SetEnd	(vector<double> End)
{
	
	m_End[0]=End[0];
	m_End[1]=End[1];
	m_End[2]=End[2];
	return 0;	
};

int vPathSegment::GetEnd(float & fX, float & fY, float & fZ)
{
	
	fX=m_End[0];
	fY=m_End[1];
	fZ=m_End[2];
	return 0;
};

int vPathSegment::SetLength		(float fLength)
{
	
	m_fLength=fLength;
	return 0;
};

ofstream & operator << (ofstream & out, const vPathSegment & path)
{
	out << path.m_segType  << endl;
	out << path.m_Start[0] << '\t' << path.m_Start[1] << '\t' << path.m_Start[2] << endl;
	return out;
};
ifstream & operator >> (ifstream & in, vPathSegment & path)
{
	int shapetype;
	in >> shapetype;
	path.m_segType = (enType)shapetype;
	in >> path.m_Start[0] >> path.m_Start[1] >> path.m_Start[2] ;
	return in;
};



