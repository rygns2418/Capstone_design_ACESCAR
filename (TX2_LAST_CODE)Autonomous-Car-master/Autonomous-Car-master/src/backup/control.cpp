#include "control.h"

int checkGoStraight(double c)
{
	if(c == 3000 || c >2.5) 
		return 0; //need to go straight
	return 1; // do not go straight
}
int checkCorner(vector<double> &fntPnt, vector<double> &lr)
{
	double dif = abs(fntPnt[0] - fntPnt[4]);
	if(dif > 0.5)
		return 2;// need to go straight but not do that
	else if((lr[0] < 2) || (lr[1] < 2))
		return 5;
	else
		return 4;// coner situation, need to turn left

}
int firstGetSteering(double a, double e)
{
	if((a > 3) || (e > 3))
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
