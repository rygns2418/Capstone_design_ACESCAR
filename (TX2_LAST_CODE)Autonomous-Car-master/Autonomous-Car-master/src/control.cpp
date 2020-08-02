#include "control.h"

int checkGoStraight(vector<double> &pnt, vector<double> &lr)
{
	if(pnt[2] == 3000 || pnt[2] >3) 
	{
		if((pnt[0] == 1000) && (pnt[1] == 2000) && (lr[0] == 100) && (pnt[2] == 3000) && (lr[1] < 1) && (pnt[4] <= 2))
			return 2;
		return 0; //need to go straight
	}
	return 1; // do not go straight
}
int checkCorner(vector<double> &fntPnt)
{
	double dif = abs(fntPnt[0] - fntPnt[4]);
	if(dif > 0.5)
		return 2;// need to go straight but not do that
	else
		return 4;// coner situation, need to turn left

}
int firstGetSteering(double a, double e)
{
	if((a > 2.5) || (e > 2.5))
		return 87;
	else
		return (90 + 20*(e-a)); // return steeringInfo using depth difference
	
}
int secondGetSteering(double b, double d)
{
	if((b > 3) || (d > 3))
		return 87;
	else
		return (90 + 20*(d-b)); // return steeringInfo using depth difference
}
int thirdGetSteering(double fntPnt0, double fntPnt4)
{
	return (90 + 30*(fntPnt4-fntPnt0)); // return steeringInfo using depth difference
}
