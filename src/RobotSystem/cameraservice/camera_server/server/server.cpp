#include <stdio.h>
#include <stdlib.h>
#include "ShmMotorServer.h"
 
int main(int argc,char* argv[])
{
	ShmMotorServer motorServer;
	int i= 0;
	while (1)
	{

		motorServer.GetMotorStatus()->roadfollowingSwitch = i;
		motorServer.GetMotorStatus()->collisionSwitch = i;
		motorServer.GetMotorStatus()->objectDetectionSwitch = i;
		motorServer.GetMotorStatus()->roadobjectDetectionSwitch = i;
		motorServer.GetMotorStatus()->roadfollowingChange = i;
		motorServer.GetMotorStatus()->collisionChange = i;
		motorServer.GetMotorStatus()->objectDetectionChange = i;
		motorServer.GetMotorStatus()->roadObjectDetectionChange = i;
		motorServer.GetMotorStatus()->angle = i;
		motorServer.GetMotorStatus()->x = i;
		motorServer.GetMotorStatus()->y = i;
		motorServer.GetMotorStatus()->collisionStatus = i;
		motorServer.GetMotorStatus()->motorAngle = i;
		motorServer.GetMotorStatus()->motorRunStatus = i++;
		motorServer.GetMotorStatus()->objNum =2;
		motorServer.GetMotorStatus()->object[0].x =i;
		motorServer.GetMotorStatus()->object[0].y =i;
		motorServer.GetMotorStatus()->object[0].width =i;
		motorServer.GetMotorStatus()->object[0].height =i;
		motorServer.GetMotorStatus()->object[0].lable =i;
		motorServer.GetMotorStatus()->object[0].confidence =3;

		motorServer.GetMotorStatus()->object[1].x =i;
		motorServer.GetMotorStatus()->object[1].y =i;
		motorServer.GetMotorStatus()->object[1].width =i;
		motorServer.GetMotorStatus()->object[1].height =i;
		motorServer.GetMotorStatus()->object[1].lable =i;
		motorServer.GetMotorStatus()->object[1].confidence =3;
		sleep(1);
	}
	


}
