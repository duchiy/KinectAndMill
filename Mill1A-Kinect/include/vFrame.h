#ifndef __VFRAME_H__
#define __VFRAME_H__
#include "stdafx.h"
#include <vector>
using namespace std;

class vFrame
{
public:
	vFrame();
	~vFrame();
	vector<double> PtX;
	vector<double> PtY;
	vector<double> PtZ;
	vector<double> PtP;
};
#endif
