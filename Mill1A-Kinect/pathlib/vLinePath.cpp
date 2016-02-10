#include "stdafx.h"
#include<vLinePath.h>

using namespace std;
//using namespace vLinePath;

vLine::vLine() : vPathSegment()
{
	m_segType = enLine;

};
vLine & vLine::operator=(vLine & rLine)
{
	this->m_segType=rLine.m_segType;
	this->m_End=rLine.m_End;

	this->m_Start=rLine.m_Start;
	this->m_fLength=rLine.m_fLength;

	return *this;
};
vLine& vLine::operator=(vLine * rLine)
{

	this->m_segType=rLine->m_segType;
	this->m_End=rLine->m_End;
	this->m_Start=rLine->m_Start;
	this->m_fLength=rLine->m_fLength;
	return *this;	
};
vLine & vLine::operator+(vLine & rLine)
{
	vLine resultLine=*this;
	resultLine=resultLine+rLine;
	return resultLine;
};
vLine & vLine::operator+(vLine * pLine)
{
	vLine resultLine=*this;
	resultLine=resultLine + *pLine;
	return resultLine;
};
void vLine::Write (ostream & out)
{
	out << this->m_segType  << endl;
	out << this->m_Start[0] << '\t' << this->m_Start[1] << '\t' << this->m_Start[2] << endl;
	out << this->m_End[0] << '\t' << this->m_End[1] << '\t' << this->m_End[2] << endl;

}
void vLine::Read  (istream & in)
{
	int shapetype;
	in >> shapetype;
	this->m_segType = (enType)shapetype;
	in >> this->m_Start[0] >> this->m_Start[1] >> this->m_Start[2] ;
}

ofstream & operator << (ofstream & out, vLine & path)
{
	out << path.m_segType  << endl;
	out << path.m_Start[0] << '\t' << path.m_Start[1] << '\t' << path.m_Start[2] << endl;
	out << path.m_End[0] << '\t' << path.m_End[1] << '\t' << path.m_End[2] << endl;
	return out;	
};
ifstream & operator >> (ifstream & in, vLine & path)
{
	int nType;
	in >> nType;
	path.m_segType = (enType)nType;
	in >> path.m_Start[0] >> path.m_Start[1] >> path.m_Start[2];
	in >> path.m_End[0] >>  path.m_End[1] >> path.m_End[2];
	return in;
};

vLine::~vLine()
{

};
