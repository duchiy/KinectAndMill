#ifndef __VARC_H__
#define __VARC_H__
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

//namespace vArcPath
//{
	class PATHLIB_API vArc: public vPathSegment
	{
	public:
		vArc();
		~vArc();
		int	SetCenter	 (float fX, float fY, float fZ);
		int	SetNormal		 (float fI, float fJ, float fK);
		int	SetRadius		 (float fRadius);
		vArc &	operator=(vArc & rArcPath);
		vArc &	operator=(vArc * pArc);
		vArc 	operator+(vArc & rArcPath);
		vArc 	operator+(vArc * pArc);

	public:
		float	m_fRadius;		 //Radius (arcs only)
		vector<double>  m_Center;		 //Center point (arcs only)
		vector<double>  m_Normal;		 //Normal vector (arcs only)
		friend ofstream & operator << (ofstream & out, vArc & path);
		friend ifstream & operator >> (ifstream & in, vArc & path);

	};
//};
#endif
