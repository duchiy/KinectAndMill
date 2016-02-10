#ifndef __POINT1
#define __POINT1
//=============================================================================
//      
//      
//
//	File	:	NMCSERVO.h
//	Name	:	Don Uchiyama
//	Date	:	6/8/2001
//
//	Desc	: 	
//			
//			.
//
//	Defines	:	
//
//=============================================================================


class PathPoint
{
public:
	PathPoint():X(0),Y(0),Z(0)
	{
	}
public:
	long int X;
	long int Y;
	long int Z;
};
#endif
