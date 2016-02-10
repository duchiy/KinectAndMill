#include "stdafx.h"
#include<vPathSegment.h>
#include<vArcPath.h>
//using namespace vArcPath;

vArc::vArc(): vPathSegment(), m_Center(3), m_Normal(3), m_fRadius(0.0)
{
	m_segType = enArc;

};
vArc::~vArc()
{

};
vArc & vArc::operator=(vArc & rArc)
{
	this->m_segType	=rArc.m_segType;
	this->m_End		=rArc.m_End;
	this->m_Center	=rArc.m_Center;
	this->m_Normal	=rArc.m_Normal;
	this->m_Start	=rArc.m_Start;
	this->m_fLength	=rArc.m_fLength;
	this->m_fRadius	=rArc.m_fRadius;

	return *this;
};
vArc & vArc::operator=(vArc * pArc)
{
	this->m_segType	=pArc->m_segType;
	this->m_End		=pArc->m_End;
	this->m_Center	=pArc->m_Center;
	this->m_Normal	=pArc->m_Normal;
	this->m_Start	=pArc->m_Start;
	this->m_fLength	=pArc->m_fLength;
	this->m_fRadius	=pArc->m_fRadius;

	return *this;
};
vArc vArc::operator+(vArc & rArcPath)
{
	vArc resultArc = *this;
	resultArc.m_Center[0]=resultArc.m_Center[0]+rArcPath.m_Center[0];
	resultArc.m_Center[1]=resultArc.m_Center[1]+rArcPath.m_Center[1];
	resultArc.m_Center[2]=resultArc.m_Center[2]+rArcPath.m_Center[2];

	resultArc.m_Normal[0]=resultArc.m_Normal[0]+rArcPath.m_Normal[0];
	resultArc.m_Normal[1]=resultArc.m_Normal[1]+rArcPath.m_Normal[1];
	resultArc.m_Normal[2]=resultArc.m_Normal[2]+rArcPath.m_Normal[2];

	resultArc.m_Start[0]=resultArc.m_Start[0]+rArcPath.m_Start[0];
	resultArc.m_Start[1]=resultArc.m_Start[1]+rArcPath.m_Start[1];
	resultArc.m_Start[2]=resultArc.m_Start[2]+rArcPath.m_Start[2];

	resultArc.m_End[0]=resultArc.m_End[0]+rArcPath.m_End[0];
	resultArc.m_End[1]=resultArc.m_End[1]+rArcPath.m_End[1];
	resultArc.m_End[2]=resultArc.m_End[2]+rArcPath.m_End[2];

	resultArc.m_fRadius =resultArc.m_fRadius+rArcPath.m_fRadius;

	return resultArc;

};
vArc vArc::operator+(vArc * pArc)
{
	
	vArc resultArc = *this;
	resultArc=resultArc + *pArc;
	return resultArc;
	
};

int vArc::SetCenter	(float fX, float fY, float fZ)
{
	m_Center[0]=fX;
	m_Center[1]=fY;
	m_Center[2]=fZ;

	return 0;
};
int vArc::SetNormal		(float fI, float fJ, float fK)
{
	m_Normal[0]=fI;
	m_Normal[1]=fJ;
	m_Normal[2]=fK;
	return 0;	
};
int vArc::SetRadius		(float p_Radius)
{
	m_fRadius=p_Radius;
	return 0;	
};
ofstream & operator << (ofstream & out, vArc & path)
{
	out << (int)path.m_segType  << endl;
	out << path.m_Start[0] << '\t' << path.m_Start[1] << '\t' << path.m_Start[2] << endl;
	out << path.m_End[0] << '\t' << path.m_End[1] << '\t' << path.m_End[2] << endl;
	out << path.m_Center[0] << '\t' << path.m_Center[1] << '\t' << path.m_Center[2] << endl;
	out << path.m_Normal[0] << '\t' << path.m_Normal[1] << '\t' << path.m_Normal[2] << endl;
	return out;	
};
ifstream & operator >> (ifstream & in, vArc & path)
{
	int nType;
	in >> nType;
	path.m_segType = (enType)nType;
	in >> path.m_Start[0] >> path.m_Start[1] >> path.m_Start[2];
	in >> path.m_End[0] >>  path.m_End[1] >> path.m_End[2];
	in >> path.m_Center[0] >>  path.m_Center[1] >> path.m_Center[2];
	in >> path.m_Normal[0] >>  path.m_Normal[1] >> path.m_Normal[2];
	return in;
};
