#ifndef __VLINE_H__
#define __VLINE_H__
#include "stdafx.h"
//--------------------------------------------------------------------------------
//	Class Name:  Line
//	
//	Description:
//	This class is derived from the segment class.  This class is used
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

//namespace vLinePath
//{
	class PATHLIB_API vLine: public vPathSegment
	{
	public:
		vLine();
		vLine & operator=(vLine & rLine);
		vLine & operator=(vLine * pLine);
		vLine & operator+(vLine & rLine);
		vLine & operator+(vLine * pLine);
		void Write  (ostream & out);
		void Read  (istream & in);
		friend ofstream & operator << (ofstream & out, vLine & path);
		friend ifstream & operator >> (ifstream & in, vLine & path);

		~vLine();
		void Draw() {}
	private:

	};
//}
#endif
