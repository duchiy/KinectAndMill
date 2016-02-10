#ifndef __SEGMENT
#define __SEGMENT

typedef float fPt[3];     //floating point 3x1 vector
enum enType {enOrigin, enLine, enArc};
class Point
{
public:
	Point();
	virtual ~Point();

	float X;
	float Y;
	float Z;
	
};

class Segment
{
public:
	Segment();
	virtual ~Segment();
	enType m_segType;
	int SetStartPoint	(float fX, float fY, float fZ);
	int GetStartPoint	(float & fX, float & fY, float & fZ);
	int SetEndPoint		(float fX, float fY, float fZ);
	int SetEndPoint		(fPt p_fEnd);
	int SetEndPoint		(Point p_fEnd);
	int GetEndPoint		(float & fX, float & fY, float & fZ);
	int SetLength		(float p_fLength);
	Segment & operator=(Segment & p_Segment);
	Segment & operator=(Segment * p_Segment);
public:
    float	m_fLength;		 //Segment length

	Point   m_Start;
	Point   m_End;
};


class LineSegment: public Segment
{
public:
	LineSegment();
	LineSegment & operator=(LineSegment & p_LineSegment);
	LineSegment & operator=(LineSegment * p_LineSegment);
	~LineSegment();

private:

};
class ArcSegment:public Segment
{
public:
	ArcSegment();
	~ArcSegment();
	int SetCenterPoint	(float fX, float fY, float fZ);
	int SetNormal		(float fI, float fJ, float fK);
	int SetRadius		(float fRadiusp);
	ArcSegment & operator=(ArcSegment & p_ArcSegment);
	ArcSegment & operator=(ArcSegment * p_ArcSegment);

public:
	fPt		m_fCenter;         //Center point (arcs only)
	fPt		m_fNormal;           //Normal vector (arcs only)
	float	m_fRadius;		 //Radius (arcs only)

	Point   m_Center;
	Point   m_Normal;

};
#endif
