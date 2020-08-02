#ifndef __CONTROL_H__
#define __CONTROL_H__
#include <algorithm>
#include <vector>
#ifndef __CV_STD_NAMESPACE__
#define __CV_STD_NAMESPACE__
using namespace std;
#endif
#endif

int checkGoStraight(vector<double> &pnt, vector<double> &lr);
int checkCorner(vector<double> &fntPnt);
int firstGetSteering(double a, double e);
int secondGetSteering(double b, double d);
int thirdGetSteering(double fntPnt0, double fntPnt4);



