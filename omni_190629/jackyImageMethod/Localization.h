/*
  Localization.h V1.0
  Features:
  1.getLocation: To get location result.including rotation information.Globally or locally..
  2.getRotation(int* whiteLineData):Just for test Never use this!
  3. getRotation(int BlueOne,int YellowOne :Get rotation from two goal 
  4.updataGreenFieldBound: To find the green field boudary.
  5.setupFunctions : setup LUTs ..
  
  NOTICE:
  1.Face to blue goal is the inital 0 rotation.
  test for 
  Jacky Guo 2014.4
  */

#pragma once
#include<math.h>
#include<stdio.h>
#define FORPANDA true
//true for Panda
using namespace std;
struct LocalizeDataBaseFormat
{
	unsigned char count;
	unsigned int data[4];
};

namespace ImageCenter{
struct localizeDataFormat{
	int x;
	int y;
	int FstMetLineData[360];
};

struct possibleLocation {
	int rotation;
	int xInField;
	int yInField;
	double totalError;
	int validPoint;
};

class Localization
{

public:
	//Rotation
	int rotation;
	//Location
	int xInField,yInField;
	//Triangle needed data
	//int blueGoalAngle,yellowGoalAngle;
	//White line optimalize
	LocalizeDataBaseFormat RotatedlocalizeData[75][55][360];

	possibleLocation bestMatch;
	possibleLocation* LocationHistory;

	//search range
	int xInFieldRange,yInFieldRange,rotationRange;

	//gloabal search threshold
	int globalSearchThreshold;

	//serialport

	//probability:

	//Pixel to CM LUT
	int *pixelToCM;

	//kevin: encoder data
	int LastEncoderX;
	int LastEncoderY;
	int CurrentEncoderX;
	int CurrentEncoderY;

	Localization(void)
	{
		
		//search range
		xInFieldRange = 20;
		yInFieldRange = 20;
		rotationRange = 0;

		//gloabal search threshold
		globalSearchThreshold = 20;
		LocationHistory = new possibleLocation[5];


		rotation = 90;
		xInField = 300;//Initial value for global search.
		yInField = 300;
		bestMatch.xInField = 0;
		bestMatch.yInField =0;
		bestMatch.rotation = 0;
		bestMatch.totalError = 999;//initialize the error maximum at beginning
		this->readDataBaseToMemory("C:\\LocalizeDataBase.bin",RotatedlocalizeData);// read data from database
		
		pixelToCM = new int[401];
		setUpPixelToCMLUT();

		//kevin: encoder data
		LastEncoderX=300;
		LastEncoderY=300;
		CurrentEncoderX=300;
		CurrentEncoderY=300;
	}

	~Localization(void)
	{
		
	}

	
	//First:
	//Triangle localize
	//Need two goals' location&angles and the form's location to reduce the noise localization...
	//updata! using digital compass rather than the triangle 

	//Second:
	//set up a mapping White line ,Using grid,Minimum error localization...
//	void getLocation(int* whiteLineData,int* whiteLineDataLayerTwo,int bluePhase,int yellowPhase)
//	{
//		//Default : Not use gloabalSearch 
//		bool globalSearch = false;
//		// need updata
//		int realX,realY,rotation;// = getRotation(bluePhase,yellowPhase);
//		//check rotation 
//		//....Need fill the rotation from arduino to HERE.
//		rotation = this->bestMatch.rotation;//= arduino's rotation.
//
//		int angle = 0,searchDataAngle = 0;
//		int nowError = 0;
//		int nowErrorLayerTwo = 0;
//
//		double minTotalError = 999;//minimum's totalError in one processing,So given a max value at beginning
//		double totalError;//errorLayerOne RMS errorLayerTwo
//
//		int validAngle = 0;
//		int validAngleLayerTwo = 0;
//
//		//search , and compare with the data in the pre-builded location database
//		//get the error value
//		int searchXMin = xInField-xInFieldRange, searchXMax = xInField+xInFieldRange ;
//		int searchYMin = yInField-yInFieldRange, searchYMax = yInField+yInFieldRange;
//		int rotationMin = rotation-rotationRange, rotationMax = rotation+rotationRange ;
//
//		if(bestMatch.totalError>globalSearchThreshold)
//		{//deal with the error is locked by near searcher
//			globalSearch = true;
//			searchXMin = 0 , searchXMax = 740 ;
//			searchYMin = 0,  searchYMax = 540 ;
//			rotationMin = rotation-rotationRange, rotationMax = rotation+rotationRange ;
//			//rotation = getrotation(bluephase,yellowphase);
//			//if(rotation==-1) // cant decide the rotation by dual-goal phase
//			//{
//			//	rotationmin = 0, rotationmax = 359 ;
//			//}
//			//else//if it can decide the rotation by dual-goal. search the rotation between +- 10 degrees...
//			//{
//			//	rotationmin = rotation-10;
//			//	rotationmax = rotation+10;
//			//}
//
//		}
//		//the constants above are given by me,that's can be relative to the feedback from motors.
//		//means the max movement change is determine from motor feedback
//
//#pragma region prevent the overflow value	
//		if(searchXMin<0) searchXMin = 0;
//		else if(searchXMax>750) searchXMax = 740;
//		if(searchYMin<0) searchYMin = 0;
//		else if(searchYMax>550) searchYMax = 540;
//
//		if(rotationMin<0) rotationMin = rotationMin+360;
//		else if(rotationMax>=360) rotationMax = rotationMax - 360;
//#pragma endregion 
//
//		if(globalSearch==true)
//		{
//		#pragma region search globally
//		for(realX=searchXMin;realX <= searchXMax;realX= realX+20)
//		{//loop of X
//			for(realY=searchYMin;realY <= searchYMax;realY = realY+20)
//			{//loop of Y
//				if(rotationMin>rotationMax)
//				{// if rotation max is smaller than the min rotation
//#pragma region range: rotation min ---> 359 degree
//					//----------------------------rotation min ---> 359 degree --------------------
//					for(rotation = rotationMin;rotation<360;rotation=rotation+4)
//					{
//						nowError = 0;//reset for nowError
//						nowErrorLayerTwo = 0;
//
//						validAngle = 0;//reset for validAngle
//						validAngleLayerTwo = 0;
//
//						for(angle = 0;angle<360;angle = angle+10) //compare with database
//						{
//							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
//							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//    
//							if(whiteLineData[searchDataAngle]!= -1)//-1 means that there's no white in this angle 
//							{
//								validAngle++;
//								nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//							}
//
//							if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//							{
//								validAngleLayerTwo++;
//								nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//							}
//						}
//
//						if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//						nowError = nowError/validAngle;//error of this rotation's 360 scan line,Layer One.
//						if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//						nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//
//						//Define Error.
//						totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//						
//						// make the decision
//						if( totalError < minTotalError )						
//						{
//							minTotalError = totalError;
//							//bestMatch.validLine = validAngle;
//							bestMatch.totalError = totalError;
//							bestMatch.rotation = rotation;
//							bestMatch.xInField = realX;
//							bestMatch.yInField = realY;
//							bestMatch.ErrorOfLayerOne = nowError;
//							bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//							bestMatch.validNumberOfLayerOne = validAngle;
//							bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//						}
//					}
//					//----------------------------rotation min ---> 359 degree --------------------
//#pragma endregion 
//
//#pragma region range: 0 degree---> rotation max
//					//Below ----------------------------0 degree---> rotation max --------------------
//					for(rotation = 0;rotation <= rotationMax;rotation=rotation+4)
//					{
//						nowError = 0;//reset for nowError
//						nowErrorLayerTwo = 0;
//
//						validAngle = 0;//reset for validAngle
//						validAngleLayerTwo = 0;
//
//						for(angle = 0;angle<360;angle = angle+10)
//						{
//							searchDataAngle = angle + rotation;
//							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//
//							if(whiteLineData[searchDataAngle]!= -1)//-1 meaning no white this angle 
//							{
//								validAngle++;
//							    nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//							}
//
//							if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//							{
//								validAngleLayerTwo++;
//								nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//							}
//						}
//						if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//						nowError = nowError/validAngle;//error of this rotation's 360 scan line
//						if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//						nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//
//						totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//						
//						if( totalError < minTotalError )
//						{
//							minTotalError = totalError;
//							//bestMatch.validLine = validAngle;
//							bestMatch.totalError = totalError;
//							bestMatch.rotation = rotation;
//							bestMatch.xInField = realX;
//							bestMatch.yInField = realY;
//							bestMatch.ErrorOfLayerOne = nowError;
//							bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//							bestMatch.validNumberOfLayerOne = validAngle;
//							bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//						}
//
//					}
//					//Above----------------------------0 degree---> rotation max --------------------
//#pragma endregion 
//				} // if rotation max is smaller than the min rotation
//
//				else //if rotation max is greater than the min rotation
//				{
//#pragma region range: rotation min ---> rotation max
//					for(rotation = rotationMin;rotation <= rotationMax;rotation = rotation + 4)
//					{
//						nowError = 0;//reset for nowError
//						nowErrorLayerTwo = 0;
//
//						validAngle = 0;//reset for validAngle
//						validAngleLayerTwo = 0;
//
//						for(angle = 0;angle<360;angle = angle+10)
//						{
//
//							searchDataAngle = angle + rotation;
//							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//
//							if(whiteLineData[searchDataAngle]!= -1) 
//							{
//								validAngle++;
//								nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//							}
//
//							if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//							{
//								validAngleLayerTwo++;
//								nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//							}
//
//						}
//						if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//						nowError = nowError/validAngle;//error of this rotation's 360 scan line
//						if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//						nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//						//define the error value
//						totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//
//						if( totalError < minTotalError )						
//						{
//							minTotalError = totalError;
//							//bestMatch.validLine = validAngle;
//							bestMatch.totalError = totalError;
//							bestMatch.rotation = rotation;
//							bestMatch.xInField = realX;
//							bestMatch.yInField = realY;
//							bestMatch.ErrorOfLayerOne = nowError;
//							bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//							bestMatch.validNumberOfLayerOne = validAngle;
//							bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//						}
//					}//rotation for
//#pragma endregion 
//				}// if the max rotation is greater than min 
//			}// for Y
//		}//for x
//		} // if globalsearch
//
//		else //search near field
//		{
//	//	#pragma region search in near fieled
//			for(realX=searchXMin;realX<searchXMax;realX=realX+10)
//			{
//				for(realY=searchYMin;realY<searchYMax;realY = realY+10)
//				{
//					if(rotationMin>rotationMax)
//					{
//#pragma region -------- rotationMin --- three-six-zeor degree-----------
//						for(rotation = rotationMin;rotation<360;rotation=rotation+1)
//						{
//							nowError = 0;//reset for nowError
//							nowErrorLayerTwo = 0;
//
//							validAngle = 0;//reset for validAngle
//							validAngleLayerTwo = 0;
//
//							for(angle = 0;angle<360;angle = angle+1)
//							{
//								searchDataAngle = angle + rotation;
//								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//    
//								if(whiteLineData[searchDataAngle]!= -1)//-1 means that there's no white in this angle 
//								{
//									validAngle++;
//									nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//								}
//
//								if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//								{
//									validAngleLayerTwo++;
//									nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//								}
//							}
//
//							if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//							nowError = nowError/validAngle;//error of this rotation's 360 scan line
//							if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//							nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//
//
//						totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//
//							if( totalError < minTotalError )						
//						    {
//								validNumberOfLayerOne = validAngle;
//								validNumberOfLayerTwo = validAngleLayerTwo;
//
//								ErrorOfLayerOne = nowError;
//								ErrorOfLayerTwo = nowErrorLayerTwo;
//
//								minTotalError = totalError;
//								//bestMatch.validLine = validAngle;
//								bestMatch.totalError = totalError;
//								bestMatch.rotation = rotation;
//								bestMatch.xInField = realX;
//								bestMatch.yInField = realY;
//								bestMatch.ErrorOfLayerOne = nowError;
//								bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//								bestMatch.validNumberOfLayerOne = validAngle;
//								bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//
//							}
//						}
//#pragma endregion 
//
//#pragma region ------ degree -- rotationMax------
//						for(rotation = 0;rotation<rotationMax;rotation=rotation+1)
//						{
//							nowError = 0;//reset for nowError
//							nowErrorLayerTwo = 0;
//
//							validAngle = 0;//reset for validAngle
//							validAngleLayerTwo = 0;
//
//							for(angle = 0;angle<360;angle = angle+1)
//							{
//
//								searchDataAngle = angle + rotation;
//								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//
//								if(whiteLineData[searchDataAngle]!= -1)//-1 meaning no white this angle 
//								{
//									validAngle++;
//									nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//
//								}
//
//								if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//								{
//									validAngleLayerTwo++;
//									nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//								}
//							}
//							if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//							nowError = nowError/validAngle;//error of this rotation's 360 scan line
//							if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//							nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//
//						totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//
//							if( totalError < minTotalError )						
//						    {
//								validNumberOfLayerOne = validAngle;
//								validNumberOfLayerTwo = validAngleLayerTwo;
//
//								ErrorOfLayerOne = nowError;
//								ErrorOfLayerTwo = nowErrorLayerTwo;
//
//								minTotalError = totalError;
//								//bestMatch.validLine = validAngle;
//								bestMatch.totalError = totalError;
//								bestMatch.rotation = rotation;
//								bestMatch.xInField = realX;
//								bestMatch.yInField = realY;
//								bestMatch.ErrorOfLayerOne = nowError;
//								bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//								bestMatch.validNumberOfLayerOne = validAngle;
//								bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//							}
//
//						}
//#pragma endregion
//						
//					}//if rotationMin>rotationMax
//
//					else
//					{
//#pragma region --- rotationMin->rotationMax ---
//						for(rotation = rotationMin;rotation<rotationMax;rotation=rotation+1)
//						{
//							nowError = 0;//reset for nowError
//							nowErrorLayerTwo = 0;
//
//							validAngle = 0;//reset for validAngle
//							validAngleLayerTwo = 0;
//
//							for(angle = 0;angle<360;angle = angle+1)
//							{
//
//								searchDataAngle = angle + rotation;
//								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
//
//								if(whiteLineData[searchDataAngle]!= -1) 
//								{
//									validAngle++;
//									nowError += abs(locationDataBase[realX/10][realY/10][angle] - whiteLineData[searchDataAngle]);
//								}
//								if(whiteLineDataLayerTwo[searchDataAngle]!= -1)
//								{
//									validAngleLayerTwo++;
//									nowErrorLayerTwo += abs(locationDataBaseSecondLayer[realX/10][realY/10][angle] - whiteLineDataLayerTwo[searchDataAngle]);
//								}
//							}
//
//							if(validAngle==0) validAngle =1;//prevent '/0' divided by 0;
//							nowError = nowError/validAngle;//error of this rotation's 360 scan line
//							if(validAngleLayerTwo==0) validAngleLayerTwo =1;
//							nowErrorLayerTwo = nowErrorLayerTwo/validAngleLayerTwo;
//
//						    totalError = sqrt((double)(nowError*nowError + nowErrorLayerTwo*nowErrorLayerTwo));
//
//							if( totalError < minTotalError )						
//						    {
//								validNumberOfLayerOne = validAngle;
//								validNumberOfLayerTwo = validAngleLayerTwo;
//
//								ErrorOfLayerOne = nowError;
//								ErrorOfLayerTwo = nowErrorLayerTwo;
//
//								minTotalError = totalError;
//								bestMatch.totalError = totalError;
//								bestMatch.rotation = rotation;
//								bestMatch.xInField = realX;
//								bestMatch.yInField = realY;
//								bestMatch.ErrorOfLayerOne = nowError;
//								bestMatch.ErrorOfLayerTwo = nowErrorLayerTwo;
//								bestMatch.validNumberOfLayerOne = validAngle;
//								bestMatch.validNumberOfLayerTwo = validAngleLayerTwo;
//							}
//						}//for rotation...
//#pragma endregion
//					}//else
//				}
//			}
//		}//else..
//		//finish!
//		this->rotation = bestMatch.rotation;
//		this->xInField = bestMatch.xInField;
//		this->yInField = bestMatch.yInField;
//		this->ErrorOfLayerOne = bestMatch.ErrorOfLayerOne;
//		this->ErrorOfLayerTwo = bestMatch.ErrorOfLayerTwo;
//		this->validNumberOfLayerOne = bestMatch.validNumberOfLayerOne;
//		this->validNumberOfLayerTwo = bestMatch.validNumberOfLayerTwo;
//	}
#pragma region kevin add: triangle localization
	void getLocationUsingTriangleLocalizeMethod(int yellowGoalLeftPhase,int yellowGoalRightPhase
		,int blueGoalLeftPhase,int blueGoalRightPhase
		,unsigned char scheme)
	{
		//setting goal position
		//goal1 means yellow goal, goal2 means blue goal
		static double goal1lx=75;
		static double goal1ly=275-80;
		static double goal1rx=75;
		static double goal1ry=275+80;
		static double goal2lx=750-75;
		static double goal2ly=goal1ry;
		static double goal2rx=750-75;
		static double goal2ry=goal1ly;

		//declare phi angles
		static double phi01;
		static double phi02;
		static double phi03;
		
		//declare known point
		static double point1x;
		static double point1y;
		static double point2x;
		static double point2y;
		static double point3x;
		static double point3y;

		if(scheme==0)//scheme 1
		{
		phi01=blueGoalLeftPhase;
		phi02=blueGoalRightPhase;
		phi03=yellowGoalLeftPhase;
		//scheme 1
		point1x=goal2lx;
		point1y=goal2ly;
		point2x=goal2rx;
		point2y=goal2ry;
		point3x=goal1lx;
		point3y=goal1ly;
		}
		else if(scheme==1)//scheme 2
		{
		phi01=blueGoalRightPhase;
		phi02=yellowGoalLeftPhase;
		phi03=yellowGoalRightPhase;
		//scheme 1
		point1x=goal2rx;
		point1y=goal2ry;
		point2x=goal1lx;
		point2y=goal1ly;
		point3x=goal1rx;
		point3y=goal1ry;
		}
		else if(scheme==2)//scheme 3
		{
		phi01=yellowGoalLeftPhase;
		phi02=yellowGoalRightPhase;
		phi03=blueGoalLeftPhase;
		//scheme 1
		point1x=goal1lx;
		point1y=goal1ly;
		point2x=goal1rx;
		point2y=goal1ry;
		point3x=goal2lx;
		point3y=goal2ly;
		}
		else if(scheme==3)//scheme 4
		{
		phi01=yellowGoalRightPhase;
		phi02=blueGoalLeftPhase;
		phi03=blueGoalRightPhase;
		//scheme 1
		point1x=goal1rx;
		point1y=goal1ry;
		point2x=goal2lx;
		point2y=goal2ly;
		point3x=goal2rx;
		point3y=goal2ry;
		}
		else
		{
			return;//scheme input error!
		}

		//main calculation
		double distance21=sqrt((point1x-point2x)*(point1x-point2x)+(point1y-point2y)*(point1y-point2y));
		double distance23=sqrt((point3x-point2x)*(point3x-point2x)+(point3y-point2y)*(point3y-point2y));
		double phi21=atan2((point1y-point2y),(point1x-point2x))/3.1415926*180.0;
		double phi23=atan2((point3y-point2y),(point3x-point2x))/3.1415926*180.0;
		double alpha102=phi02-phi01;
		//alpha102=abs(alpha102);
		double alpha203=phi03-phi02;
		//alpha203=abs(alpha203);
		double alpha321=phi21-phi23;
		//alpha321=abs(alpha321);
		double beta=360-alpha102-alpha203-alpha321;
		////normalize beta
		//if(beta<0)
		//{
		// beta+=360;
		//}
		//else if(beta>=360)
		//{
		// beta-=360;
		//}

		double mPart1=(distance21*sin(alpha203/180.0*3.1415926))/(distance23*sin(alpha102/180.0*3.1415926));
		double alpha210=atan2(sin(beta/180.0*3.1415926),(mPart1+cos(beta/180.0*3.1415926)))/3.1415926*180.0;
		if(alpha102<0)
			alpha102+=360;
		else if(alpha102>=360)
			alpha102-=360;
		if(alpha203<0)
			alpha203+=360;
		else if(alpha203>=360)
			alpha203-=360;
		if(alpha321<0)
			alpha321+=360;
		else if(alpha321>=360)
			alpha321-=360;
		if(beta<0)
			beta+=360;
		else if(beta>=360)
			beta-=360;
		//if(alpha210<0)
		// alpha210+=360;
		//else if(alpha210>=360)
		// alpha210-=360;

		static double mPart2=(distance21*sin((180-alpha102-alpha210)/180.0*3.1415926))/(sin(alpha102/180.0*3.1415926));
		static double triangleX=point1x+mPart2*cos((180+phi21+alpha210)/180.0*3.1415926);
		static double triangleY=point1y+mPart2*sin((180+phi21+alpha210)/180.0*3.1415926);
		static double phi0=phi21+alpha210-phi01;

		//output result to ui
		this->rotation=phi0;
		this->xInField=triangleX;
		this->yInField=triangleY;
	}
#pragma endregion

	void getLocationUsingNewMethod(LocalizeDataBaseFormat* observedWhiteData,int bluePhase,int yellowPhase,int rotation,
		int EncoderLocationX,int EncoderLocationY,bool EnableFullGlobalSearch)
	{		
		//Default : Not use gloabalSearch 
		bool globalSearch = false;
		// need updata
		int realX,realY;// = getRotation(bluePhase,yellowPhase);
		bestMatch.rotation = rotation;
		//serach gap...
		int gap =2;//120 validangles

		int delta0 = 999;

		int angle = 0,searchDataAngle = 0;
		int nowError = 0;

		double minTotalError = 999;//minimum's totalError in one processing,So given a max value at beginning
		double totalError;//errorLaye;rOne RMS errorLayerTwo

		int validPoint = 0;
		//search , and compare with the data in the pre-builded location database
		//get the error value
		int searchXMin = xInField-xInFieldRange, searchXMax = xInField+xInFieldRange ;
		int searchYMin = yInField-yInFieldRange, searchYMax = yInField+yInFieldRange;
		int rotationMin = rotation-rotationRange, rotationMax = rotation+rotationRange ;
		
		if(bestMatch.totalError>globalSearchThreshold&&EnableFullGlobalSearch==true)//request full search
		{//deal with the error is locked by near searcher
			globalSearch = true;
			searchXMin = 0 , searchXMax = 740 ;
			searchYMin = 0 ,  searchYMax = 540 ;
		}
		if(bestMatch.totalError>globalSearchThreshold&&EnableFullGlobalSearch==false)//only search the range that encoder given
		{
			globalSearch = true;
			searchXMin = EncoderLocationX-xInFieldRange , searchXMax = EncoderLocationX+xInFieldRange ;
			searchYMin = EncoderLocationY-yInFieldRange ,  searchYMax = EncoderLocationY+yInFieldRange ;
		}
		//the constants above are given by me,that's can be relative to the feedback from motors.
		//means the max movement change is determine from motor feedback

		if(EnableFullGlobalSearch==false)//to check this parameter is because when encoder data used, this parameter will set to false
		{//and only when this parameter equal to false, means incoming encoder data or local encoder data is valid.
			//------------------------------------------------------------
			//kevin: compare last and current encoder data
			//this section is use to prevent location jumping when robot stop

			//put current to last
			LastEncoderX=CurrentEncoderX;
			LastEncoderY=CurrentEncoderY;

			//load current to current
			CurrentEncoderX=EncoderLocationX;
			CurrentEncoderY=EncoderLocationY;

			//compare
			if(
				CurrentEncoderX==LastEncoderX
				&&
				CurrentEncoderY==LastEncoderY
				)
			{
				return;//break the further localize data base search, remain the current location
			}
			//------------------------------------------------------------
		}

#pragma region prevent the overflow value	
		if(searchXMin<=0) searchXMin = 0;
		 if(searchXMax>=740) searchXMax = 740;
		if(searchYMin<=0) searchYMin = 0;
		 if(searchYMax>=540) searchYMax = 540;

		if(rotationMin<0) rotationMin = rotationMin+360;
		else if(rotationMax>=360) rotationMax = rotationMax - 360;

#pragma endregion 

	if(globalSearch==true)
	{
//#pragma region search globally
		for(realX=searchXMin;realX <= searchXMax;realX= realX+10)
		{//loop of X
			for(realY=searchYMin;realY <= searchYMax;realY = realY+10)
			{//loop of Y
				if(rotationMin>rotationMax)
				{// if rotation max is smaller than the min rotation
#pragma region range: rotation min ---> 359 degree
					//----------------------------rotation min ---> 359 degree --------------------
					for(rotation = rotationMin;rotation<360;rotation=rotation+1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validAngleg

						 //delta0 = 999,delta1 = 999,delta2 = 999,delta3 = 999;
						 
						for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}

						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;

						//Define Error.
						totalError = sqrt((double)(nowError*nowError ));
						
						// make the decision
						if( totalError < minTotalError )						
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}
					}
					//----------------------------rotation min ---> 359 degree --------------------
#pragma endregion 

#pragma region range: 0 degree---> rotation max
					//Below ----------------------------0 degree---> rotation max --------------------
					for(rotation = 0;rotation <= rotationMax;rotation=rotation+1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validPoint
						for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}

						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;//error of this rotation's 360 scan line

						totalError = sqrt((double)(nowError*nowError ));
						
						if( totalError < minTotalError )
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}

					}
					//Above----------------------------0 degree---> rotation max --------------------
#pragma endregion 
				} // if rotation max is smaller than the min rotation

				else //if rotation max is greater than the min rotation
				{
#pragma region range: rotation min ---> rotation max
					for(rotation = rotationMin;rotation <= rotationMax;rotation = rotation + 1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validPoint

					for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}
						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;//error of this rotation's 360 scan line
						//define the error value
						totalError = sqrt((double)(nowError*nowError ));

						if( totalError < minTotalError )
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}
					}//rotation for
#pragma endregion 
				}// if the max rotation is greater than min 
			}// for Y
		}//for x
	} // if globalsearch
//#pragma endregion 

		else //search near field
		{
	//	#pragma region search in near fieled
			for(realX=searchXMin;realX<searchXMax;realX=realX+10)
			{
				for(realY=searchYMin;realY<searchYMax;realY = realY+10)
				{
					if(rotationMin>rotationMax)
					{
#pragma region -------- rotationMin --- three-six-zeor degree-----------
						for(rotation = rotationMin;rotation < 360; rotation = rotation + 1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
							if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion 

#pragma region ------0 degree -- rotationMax------
						for(rotation = 0;rotation <= rotationMax; rotation = rotation + 1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
									int testCount = observedWhiteData[searchDataAngle].count-1;
									validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
							if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion
						
					}//if rotationMin>rotationMax

					else
					{
#pragma region --- rotationMin->rotationMax ---
						for(rotation = rotationMin;rotation<=rotationMax;rotation=rotation+1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
									int testCount = observedWhiteData[searchDataAngle].count-1;
									validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
						    if(validPoint==0) 
							{
								validPoint =1;
							}//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion
					}//else
				}
			}
		}//else..
		//finish!
		this->rotation = bestMatch.rotation;
		this->xInField = bestMatch.xInField;
		this->yInField = bestMatch.yInField;

	}

	
	void getLocationUsingNewMethod(LocalizeDataBaseFormat* observedWhiteData,int bluePhase,int yellowPhase,int rotation)
	{		
		//Default : Not use gloabalSearch 
		bool globalSearch = false;
		// need updata
		int realX,realY;// = getRotation(bluePhase,yellowPhase);
		bestMatch.rotation = rotation;
		//serach gap...
		int gap =2;//120 validangles

		int delta0 = 999;

		int angle = 0,searchDataAngle = 0;
		int nowError = 0;

		double minTotalError = 999;//minimum's totalError in one processing,So given a max value at beginning
		double totalError;//errorLaye;rOne RMS errorLayerTwo

		int validPoint = 0;
		//search , and compare with the data in the pre-builded location database
		//get the error value
		int searchXMin = xInField-xInFieldRange, searchXMax = xInField+xInFieldRange ;
		int searchYMin = yInField-yInFieldRange, searchYMax = yInField+yInFieldRange;
		int rotationMin = rotation-rotationRange, rotationMax = rotation+rotationRange ;
		
		if(bestMatch.totalError>globalSearchThreshold)
		{//deal with the error is locked by near searcher
			globalSearch = true;
			searchXMin = 0 , searchXMax = 740 ;
			searchYMin = 0 ,  searchYMax = 540 ;
		}
		//the constants above are given by me,that's can be relative to the feedback from motors.
		//means the max movement change is determine from motor feedback

#pragma region prevent the overflow value	
		if(searchXMin<=0) searchXMin = 0;
		 if(searchXMax>=740) searchXMax = 740;
		if(searchYMin<=0) searchYMin = 0;
		 if(searchYMax>=540) searchYMax = 540;

		if(rotationMin<0) rotationMin = rotationMin+360;
		else if(rotationMax>=360) rotationMax = rotationMax - 360;

#pragma endregion 

	if(globalSearch==true)
	{
//#pragma region search globally
		for(realX=searchXMin;realX <= searchXMax;realX= realX+10)
		{//loop of X
			for(realY=searchYMin;realY <= searchYMax;realY = realY+10)
			{//loop of Y
				if(rotationMin>rotationMax)
				{// if rotation max is smaller than the min rotation
#pragma region range: rotation min ---> 359 degree
					//----------------------------rotation min ---> 359 degree --------------------
					for(rotation = rotationMin;rotation<360;rotation=rotation+1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validAngleg

						 //delta0 = 999,delta1 = 999,delta2 = 999,delta3 = 999;
						 
						for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}

						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;

						//Define Error.
						totalError = sqrt((double)(nowError*nowError ));
						
						// make the decision
						if( totalError < minTotalError )						
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}
					}
					//----------------------------rotation min ---> 359 degree --------------------
#pragma endregion 

#pragma region range: 0 degree---> rotation max
					//Below ----------------------------0 degree---> rotation max --------------------
					for(rotation = 0;rotation <= rotationMax;rotation=rotation+1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validPoint
						for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}

						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;//error of this rotation's 360 scan line

						totalError = sqrt((double)(nowError*nowError ));
						
						if( totalError < minTotalError )
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}

					}
					//Above----------------------------0 degree---> rotation max --------------------
#pragma endregion 
				} // if rotation max is smaller than the min rotation

				else //if rotation max is greater than the min rotation
				{
#pragma region range: rotation min ---> rotation max
					for(rotation = rotationMin;rotation <= rotationMax;rotation = rotation + 1)
					{
						nowError = 0;//reset for nowError

						validPoint = 0;//reset for validPoint

					for(angle = 0;angle<360;angle = angle+gap) //compare with database
						{
							searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
							if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
							if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
							{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
								for(;testCount>=0;testCount--)
								{
									int minDelta = 999;
									for(int i = 0;i<4 ;i++)
									{
										delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
										if(delta0 < minDelta) 
											minDelta = delta0;
									}
									nowError = nowError + minDelta;
								}
							}
						}
						if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
						nowError = nowError/validPoint;//error of this rotation's 360 scan line
						//define the error value
						totalError = sqrt((double)(nowError*nowError ));

						if( totalError < minTotalError )
						{
							minTotalError = totalError;
							//bestMatch.validLine = validAngle;
							bestMatch.totalError = totalError;
							bestMatch.rotation = rotation;
							bestMatch.xInField = realX;
							bestMatch.yInField = realY;
							bestMatch.validPoint = validPoint;
						}
					}//rotation for
#pragma endregion 
				}// if the max rotation is greater than min 
			}// for Y
		}//for x
	} // if globalsearch
//#pragma endregion 

		else //search near field
		{
	//	#pragma region search in near fieled
			for(realX=searchXMin;realX<searchXMax;realX=realX+10)
			{
				for(realY=searchYMin;realY<searchYMax;realY = realY+10)
				{
					if(rotationMin>rotationMax)
					{
#pragma region -------- rotationMin --- three-six-zeor degree-----------
						for(rotation = rotationMin;rotation < 360; rotation = rotation + 1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
								int testCount = observedWhiteData[searchDataAngle].count-1;
								validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
							if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion 

#pragma region ------0 degree -- rotationMax------
						for(rotation = 0;rotation <= rotationMax; rotation = rotation + 1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
									int testCount = observedWhiteData[searchDataAngle].count-1;
									validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
							if(validPoint==0) validPoint =1;//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion
						
					}//if rotationMin>rotationMax

					else
					{
#pragma region --- rotationMin->rotationMax ---
						for(rotation = rotationMin;rotation<=rotationMax;rotation=rotation+1)
						{
							nowError = 0;//reset for nowError

							validPoint = 0;//reset for validPoint

							for(angle = 0;angle<360;angle = angle+gap) //compare with database
							{
								searchDataAngle = angle + rotation;//array index  = angle + rotaiton ... database[45] = array[5]  means the robot's rotation is 40 degree
								if(searchDataAngle>=360)	 searchDataAngle = searchDataAngle-360;
							
								if(observedWhiteData[searchDataAngle].count != 0)// && RotatedlocalizeData[realX/10][realY/10][searchDataAngle].count != 0)//0 means that there's no white in this angle 
								{
									int testCount = observedWhiteData[searchDataAngle].count-1;
									validPoint = observedWhiteData[searchDataAngle].count + validPoint;
								
									for(;testCount>=0;testCount--)
									{
										int minDelta = 999;
										for(int i = 0;i<4 ;i++)
										{
											//if(RotatedlocalizeData[realX/10][realY/10][rotation].data[i] != 0) 
											delta0 = abs((pixelToCM[observedWhiteData[searchDataAngle].data[testCount]] - (int)RotatedlocalizeData[realX/10][realY/10][angle].data[i]));
											if(delta0 < minDelta) 
												minDelta = delta0;
										}
										nowError = nowError + minDelta;
									}
								}
							}
						    if(validPoint==0) 
							{
								validPoint =1;
							}//prevent '/0' divided by 0;
							nowError = nowError/validPoint;//error of this rotation's 360 scan line
							//define the error value
							totalError = sqrt((double)(nowError*nowError ));

							if( totalError < minTotalError )
							{
								minTotalError = totalError;
								//bestMatch.validLine = validAngle;
								bestMatch.totalError = totalError;
								bestMatch.rotation = rotation;
								bestMatch.xInField = realX;
								bestMatch.yInField = realY;
								bestMatch.validPoint = validPoint;
							}
						}
#pragma endregion
					}//else
				}
			}
		}//else..
		//finish!
		this->rotation = bestMatch.rotation;
		this->xInField = bestMatch.xInField;
		this->yInField = bestMatch.yInField;

	}
	
	void getLocationUsingGoal(int bluePhase,int yellowPhase,int rotation,int blueDistance, int yellowDistance,int selector)// selector 0-> defend blue. 1-> defned yellow
	{
		if(selector == 0)//defend blue
		{
			int belta = bluePhase + rotation;
			if(belta>=360) belta = belta - 360;

			int xInF = 730 - blueDistance * cos((double)belta);
			int yInF = 275 - blueDistance * sin((double)belta);
			
			this->rotation = rotation;
			this->xInField = xInF;
			this->yInField = yInF;
		}
		else //defend yellow
		{
			int belta = yellowPhase + rotation;
			if(belta>=360) belta = belta - 360;

			int xInF = 20 + yellowDistance * sin((double)belta);
			int yInF = 275 - blueDistance * cos((double)belta);

			this->rotation = rotation;
			this->xInField = xInF;
			this->yInField = yInF;
		}
		
	}

	int getRotation(int BlueOne,int YellowOne ){
		int rotation;
		int blueAngle = BlueOne-180;
		if(blueAngle<=0) blueAngle = blueAngle+360;

		if(blueAngle-YellowOne < 10)
		{
			rotation = BlueOne - 270;
			if(rotation<0) rotation = rotation+360;
			return rotation;
		}
		else 
			return -1;
	}

	//read the database from file to memory,souce is the dirctory of the .bin
	void readDataBaseToMemory(char* source,LocalizeDataBaseFormat RotatedlocalizeData[75][55][360] )
	{
		FILE* databasefile = fopen(source,"rb");//20150607, i change the code, that can dynamic load any given file
		//FILE* databasefile = fopen("d:\\LocalizeDataBase.bin","rb");//this is the original method that jacky load the database file
		int i,j;
		for(i=0;i<75;i++)
		for(j=0;j<55;j++)
		{
			fread(&i,sizeof(int),1,databasefile);
			fread(&j,sizeof(int),1,databasefile);
			fread(RotatedlocalizeData[i][j],sizeof(LocalizeDataBaseFormat),360,databasefile);
		}
		fclose(databasefile);
	}
	
	//void setupDataBaseByCamera(int *layerOneDataBase,int *layerTwoDataBase,int xInField,int yInField)
	//{
	//// the rotation is 0 degree
	//	int i;
	//	for(i=0;i<360;i++)
	//	{
	//		this->locationDataBase[xInField/10][yInField/10][i] = layerOneDataBase[i];
	//		this->locationDataBaseSecondLayer[xInField/10][yInField/10][i] = layerTwoDataBase[i];
	//	}
	//}

	//Third:
	//Combinate the feedback system
	//Do one time vision localization and three times feedback localization


	int getXInfield()
	{
		return this->xInField;
	}

	int getYInfield()
	{
		return this->yInField;
	}

	int getRotation()
	{
		return this->rotation;
	}

	void setUpPixelToCMLUT()
	{
		if(FORPANDA)
		{
		//Panda 690mm
#pragma region lut for panda
		pixelToCM[0]=	28	;
		pixelToCM[1]=	28	;
		pixelToCM[2]=	28	;
		pixelToCM[3]=	28	;
		pixelToCM[4]=	28	;
		pixelToCM[5]=	28	;
		pixelToCM[6]=	28	;
		pixelToCM[7]=	28	;
		pixelToCM[8]=	28	;
		pixelToCM[9]=	28	;
		pixelToCM[10]=	28	;
		pixelToCM[11]=	28	;
		pixelToCM[12]=	28	;
		pixelToCM[13]=	28	;
		pixelToCM[14]=	28	;
		pixelToCM[15]=	28	;
		pixelToCM[16]=	28	;
		pixelToCM[17]=	28	;
		pixelToCM[18]=	28	;
		pixelToCM[19]=	28	;
		pixelToCM[20]=	28	;
		pixelToCM[21]=	28	;
		pixelToCM[22]=	28	;
		pixelToCM[23]=	28	;
		pixelToCM[24]=	28	;
		pixelToCM[25]=	28	;
		pixelToCM[26]=	28	;
		pixelToCM[27]=	28	;
		pixelToCM[28]=	28	;
		pixelToCM[29]=	28	;
		pixelToCM[30]=	28	;
		pixelToCM[31]=	28	;
		pixelToCM[32]=	28	;
		pixelToCM[33]=	28	;
		pixelToCM[34]=	28	;
		pixelToCM[35]=	28	;
		pixelToCM[36]=	28	;
		pixelToCM[37]=	28	;
		pixelToCM[38]=	28	;
		pixelToCM[39]=	28	;
		pixelToCM[40]=	28	;
		pixelToCM[41]=	28	;
		pixelToCM[42]=	28	;
		pixelToCM[43]=	28	;
		pixelToCM[44]=	28	;
		pixelToCM[45]=	28	;
		pixelToCM[46]=	28	;
		pixelToCM[47]=	28	;
		pixelToCM[48]=	28	;
		pixelToCM[49]=	28	;
		pixelToCM[50]=	28	;
		pixelToCM[51]=	28	;
		pixelToCM[52]=	28	;
		pixelToCM[53]=	28	;
		pixelToCM[54]=	28	;
		pixelToCM[55]=	28	;
		pixelToCM[56]=	29	;
		pixelToCM[57]=	29	;
		pixelToCM[58]=	29	;
		pixelToCM[59]=	29	;
		pixelToCM[60]=	30	;
		pixelToCM[61]=	30	;
		pixelToCM[62]=	31	;
		pixelToCM[63]=	31	;
		pixelToCM[64]=	32	;
		pixelToCM[65]=	32	;
		pixelToCM[66]=	33	;
		pixelToCM[67]=	34	;
		pixelToCM[68]=	34	;
		pixelToCM[69]=	34	;
		pixelToCM[70]=	35	;
		pixelToCM[71]=	35	;
		pixelToCM[72]=	36	;
		pixelToCM[73]=	37	;
		pixelToCM[74]=	37	;
		pixelToCM[75]=	38	;
		pixelToCM[76]=	38	;
		pixelToCM[77]=	39	;
		pixelToCM[78]=	39	;
		pixelToCM[79]=	40	;
		pixelToCM[80]=	41	;
		pixelToCM[81]=	41	;
		pixelToCM[82]=	42	;
		pixelToCM[83]=	42	;
		pixelToCM[84]=	43	;
		pixelToCM[85]=	44	;
		pixelToCM[86]=	44	;
		pixelToCM[87]=	45	;
		pixelToCM[88]=	45	;
		pixelToCM[89]=	46	;
		pixelToCM[90]=	47	;
		pixelToCM[91]=	47	;
		pixelToCM[92]=	48	;
		pixelToCM[93]=	49	;
		pixelToCM[94]=	49	;
		pixelToCM[95]=	49	;
		pixelToCM[96]=	51	;
		pixelToCM[97]=	52	;
		pixelToCM[98]=	53	;
		pixelToCM[99]=	54	;
		pixelToCM[100]=	54	;
		pixelToCM[101]=	55	;
		pixelToCM[102]=	55	;
		pixelToCM[103]=	56	;
		pixelToCM[104]=	56	;
		pixelToCM[105]=	57	;
		pixelToCM[106]=	58	;
		pixelToCM[107]=	59	;
		pixelToCM[108]=	59	;
		pixelToCM[109]=	60	;
		pixelToCM[110]=	61	;
		pixelToCM[111]=	61	;
		pixelToCM[112]=	62	;
		pixelToCM[113]=	63	;
		pixelToCM[114]=	64	;
		pixelToCM[115]=	65	;
		pixelToCM[116]=	65	;
		pixelToCM[117]=	66	;
		pixelToCM[118]=	67	;
		pixelToCM[119]=	68	;
		pixelToCM[120]=	69	;
		pixelToCM[121]=	70	;
		pixelToCM[122]=	70	;
		pixelToCM[123]=	71	;
		pixelToCM[124]=	72	;
		pixelToCM[125]=	73	;
		pixelToCM[126]=	74	;
		pixelToCM[127]=	75	;
		pixelToCM[128]=	76	;
		pixelToCM[129]=	77	;
		pixelToCM[130]=	78	;
		pixelToCM[131]=	78	;
		pixelToCM[132]=	79	;
		pixelToCM[133]=	81	;
		pixelToCM[134]=	82	;
		pixelToCM[135]=	82	;
		pixelToCM[136]=	83	;
		pixelToCM[137]=	85	;
		pixelToCM[138]=	86	;
		pixelToCM[139]=	87	;
		pixelToCM[140]=	88	;
		pixelToCM[141]=	89	;
		pixelToCM[142]=	90	;
		pixelToCM[143]=	90	;
		pixelToCM[144]=	92	;
		pixelToCM[145]=	93	;
		pixelToCM[146]=	94	;
		pixelToCM[147]=	95	;
		pixelToCM[148]=	96	;
		pixelToCM[149]=	97	;
		pixelToCM[150]=	98	;
		pixelToCM[151]=	99	;
		pixelToCM[152]=	101	;
		pixelToCM[153]=	102	;
		pixelToCM[154]=	103	;
		pixelToCM[155]=	104	;
		pixelToCM[156]=	105	;
		pixelToCM[157]=	107	;
		pixelToCM[158]=	108	;
		pixelToCM[159]=	109	;
		pixelToCM[160]=	111	;
		pixelToCM[161]=	112	;
		pixelToCM[162]=	113	;
		pixelToCM[163]=	115	;
		pixelToCM[164]=	116	;
		pixelToCM[165]=	117	;
		pixelToCM[166]=	118	;
		pixelToCM[167]=	120	;
		pixelToCM[168]=	121	;
		pixelToCM[169]=	123	;
		pixelToCM[170]=	124	;
		pixelToCM[171]=	126	;
		pixelToCM[172]=	128	;
		pixelToCM[173]=	129	;
		pixelToCM[174]=	131	;
		pixelToCM[175]=	132	;
		pixelToCM[176]=	134	;
		pixelToCM[177]=	136	;
		pixelToCM[178]=	137	;
		pixelToCM[179]=	139	;
		pixelToCM[180]=	141	;
		pixelToCM[181]=	143	;
		pixelToCM[182]=	145	;
		pixelToCM[183]=	147	;
		pixelToCM[184]=	148	;
		pixelToCM[185]=	150	;
		pixelToCM[186]=	152	;
		pixelToCM[187]=	154	;
		pixelToCM[188]=	157	;
		pixelToCM[189]=	159	;
		pixelToCM[190]=	161	;
		pixelToCM[191]=	163	;
		pixelToCM[192]=	166	;
		pixelToCM[193]=	169	;
		pixelToCM[194]=	171	;
		pixelToCM[195]=	173	;
		pixelToCM[196]=	175	;
		pixelToCM[197]=	177	;
		pixelToCM[198]=	180	;
		pixelToCM[199]=	183	;
		pixelToCM[200]=	185	;
		pixelToCM[201]=	188	;
		pixelToCM[202]=	191	;
		pixelToCM[203]=	193	;
		pixelToCM[204]=	197	;
		pixelToCM[205]=	200	;
		pixelToCM[206]=	203	;
		pixelToCM[207]=	205	;
		pixelToCM[208]=	209	;
		pixelToCM[209]=	212	;
		pixelToCM[210]=	215	;
		pixelToCM[211]=	219	;
		pixelToCM[212]=	222	;
		pixelToCM[213]=	226	;
		pixelToCM[214]=	229	;
		pixelToCM[215]=	233	;
		pixelToCM[216]=	237	;
		pixelToCM[217]=	242	;
		pixelToCM[218]=	245	;
		pixelToCM[219]=	248	;
		pixelToCM[220]=	253	;
		pixelToCM[221]=	258	;
		pixelToCM[222]=	263	;
		pixelToCM[223]=	266	;
		pixelToCM[224]=	268	;
		pixelToCM[225]=	272	;
		pixelToCM[226]=	278	;
		pixelToCM[227]=	286	;
		pixelToCM[228]=	293	;
		pixelToCM[229]=	300	;
		pixelToCM[230]=	300	;
		pixelToCM[231]=	300	;
		pixelToCM[232]=	300	;
		pixelToCM[233]=	300	;
		pixelToCM[234]=	300	;
		pixelToCM[235]=	300	;
		pixelToCM[236]=	300	;
		pixelToCM[237]=	300	;
		pixelToCM[238]=	300	;
		pixelToCM[239]=	300	;
		pixelToCM[240]=	300	;
		pixelToCM[241]=	300	;
		pixelToCM[242]=	300	;
		pixelToCM[243]=	300	;
		pixelToCM[244]=	300	;
		pixelToCM[245]=	300	;
		pixelToCM[246]=	300	;
		pixelToCM[247]=	300	;
		pixelToCM[248]=	300	;
		pixelToCM[249]=	300	;
		pixelToCM[250]=	300	;
		pixelToCM[251]=	300	;
		pixelToCM[252]=	300	;
		pixelToCM[253]=	300	;
		pixelToCM[254]=	300	;
		pixelToCM[255]=	300	;
		pixelToCM[256]=	300	;
		pixelToCM[257]=	300	;
		pixelToCM[258]=	300	;
		pixelToCM[259]=	300	;
		pixelToCM[260]=	300	;
		pixelToCM[261]=	300	;
		pixelToCM[262]=	300	;
		pixelToCM[263]=	300	;
		pixelToCM[264]=	300	;
		pixelToCM[265]=	300	;
		pixelToCM[266]=	300	;
		pixelToCM[267]=	300	;
		pixelToCM[268]=	300	;
		pixelToCM[269]=	300	;
		pixelToCM[270]=	300	;
		pixelToCM[271]=	300	;
		pixelToCM[272]=	300	;
		pixelToCM[273]=	300	;
		pixelToCM[274]=	300	;
		pixelToCM[275]=	300	;
		pixelToCM[276]=	300	;
		pixelToCM[277]=	300	;
		pixelToCM[278]=	300	;
		pixelToCM[279]=	300	;
		pixelToCM[280]=	300	;
		pixelToCM[281]=	300	;
		pixelToCM[282]=	300	;
		pixelToCM[283]=	300	;
		pixelToCM[284]=	300	;
		pixelToCM[285]=	300	;
		pixelToCM[286]=	300	;
		pixelToCM[287]=	300	;
		pixelToCM[288]=	300	;
		pixelToCM[289]=	300	;
		pixelToCM[290]=	300	;
		pixelToCM[291]=	300	;
		pixelToCM[292]=	300	;
		pixelToCM[293]=	300	;
		pixelToCM[294]=	300	;
		pixelToCM[295]=	300	;
		pixelToCM[296]=	300	;
		pixelToCM[297]=	300	;
		pixelToCM[298]=	300	;
		pixelToCM[299]=	300	;
		pixelToCM[300]=	300	;
		pixelToCM[301]=	300	;
		pixelToCM[302]=	300	;
		pixelToCM[303]=	300	;
		pixelToCM[304]=	300	;
		pixelToCM[305]=	300	;
		pixelToCM[306]=	300	;
		pixelToCM[307]=	300	;
		pixelToCM[308]=	300	;
		pixelToCM[309]=	300	;
		pixelToCM[310]=	300	;
		pixelToCM[311]=	300	;
		pixelToCM[312]=	300	;
		pixelToCM[313]=	300	;
		pixelToCM[314]=	300	;
		pixelToCM[315]=	300	;
		pixelToCM[316]=	300	;
		pixelToCM[317]=	300	;
		pixelToCM[318]=	300	;
		pixelToCM[319]=	300	;
		pixelToCM[320]=	300	;
		pixelToCM[321]=	300	;
		pixelToCM[322]=	300	;
		pixelToCM[323]=	300	;
		pixelToCM[324]=	300	;
		pixelToCM[325]=	300	;
		pixelToCM[326]=	300	;
		pixelToCM[327]=	300	;
		pixelToCM[328]=	300	;
		pixelToCM[329]=	300	;
		pixelToCM[330]=	300	;
		pixelToCM[331]=	300	;
		pixelToCM[332]=	300	;
		pixelToCM[333]=	300	;
		pixelToCM[334]=	300	;
		pixelToCM[335]=	300	;
		pixelToCM[336]=	300	;
		pixelToCM[337]=	300	;
		pixelToCM[338]=	300	;
		pixelToCM[339]=	300	;
		pixelToCM[340]=	300	;
		pixelToCM[341]=	300	;
		pixelToCM[342]=	300	;
		pixelToCM[343]=	300	;
		pixelToCM[344]=	300	;
		pixelToCM[345]=	300	;
		pixelToCM[346]=	300	;
		pixelToCM[347]=	300	;
		pixelToCM[348]=	300	;
		pixelToCM[349]=	300	;
		pixelToCM[350]=	300	;
		pixelToCM[351]=	300	;
		pixelToCM[352]=	300	;
		pixelToCM[353]=	300	;
		pixelToCM[354]=	300	;
		pixelToCM[355]=	300	;
		pixelToCM[356]=	300	;
		pixelToCM[357]=	300	;
		pixelToCM[358]=	300	;
		pixelToCM[359]=	300	;
		pixelToCM[360]=	300	;
		pixelToCM[361]=	300	;
		pixelToCM[362]=	300	;
		pixelToCM[363]=	300	;
		pixelToCM[364]=	300	;
		pixelToCM[365]=	300	;
		pixelToCM[366]=	300	;
		pixelToCM[367]=	300	;
		pixelToCM[368]=	300	;
		pixelToCM[369]=	300	;
		pixelToCM[370]=	300	;
		pixelToCM[371]=	300	;
		pixelToCM[372]=	300	;
		pixelToCM[373]=	300	;
		pixelToCM[374]=	300	;
		pixelToCM[375]=	300	;
		pixelToCM[376]=	300	;
		pixelToCM[377]=	300	;
		pixelToCM[378]=	300	;
		pixelToCM[379]=	300	;
		pixelToCM[380]=	300	;
		pixelToCM[381]=	300	;
		pixelToCM[382]=	300	;
		pixelToCM[383]=	300	;
		pixelToCM[384]=	300	;
		pixelToCM[385]=	300	;
		pixelToCM[386]=	300	;
		pixelToCM[387]=	300	;
		pixelToCM[388]=	300	;
		pixelToCM[389]=	300	;
		pixelToCM[390]=	300	;
		pixelToCM[391]=	300	;
		pixelToCM[392]=	300	;
		pixelToCM[393]=	300	;
		pixelToCM[394]=	300	;
		pixelToCM[395]=	300	;
		pixelToCM[396]=	300	;
		pixelToCM[397]=	300	;
		pixelToCM[398]=	300	;
		pixelToCM[399]=	300	;
		pixelToCM[400]=	300	;
		}
#pragma endregion
		else {
		//TKU
#pragma region lut for TKU
		pixelToCM[0]=	23	;
		pixelToCM[1]=	23	;
		pixelToCM[2]=	23	;
		pixelToCM[3]=	23	;
		pixelToCM[4]=	23	;
		pixelToCM[5]=	23	;
		pixelToCM[6]=	23	;
		pixelToCM[7]=	23	;
		pixelToCM[8]=	23	;
		pixelToCM[9]=	23	;
		pixelToCM[10]=	23	;
		pixelToCM[11]=	23	;
		pixelToCM[12]=	23	;
		pixelToCM[13]=	23	;
		pixelToCM[14]=	23	;
		pixelToCM[15]=	23	;
		pixelToCM[16]=	23	;
		pixelToCM[17]=	23	;
		pixelToCM[18]=	23	;
		pixelToCM[19]=	23	;
		pixelToCM[20]=	23	;
		pixelToCM[21]=	23	;
		pixelToCM[22]=	23	;
		pixelToCM[23]=	23	;
		pixelToCM[24]=	23	;
		pixelToCM[25]=	23	;
		pixelToCM[26]=	23	;
		pixelToCM[27]=	23	;
		pixelToCM[28]=	23	;
		pixelToCM[29]=	23	;
		pixelToCM[30]=	23	;
		pixelToCM[31]=	23	;
		pixelToCM[32]=	23	;
		pixelToCM[33]=	23	;
		pixelToCM[34]=	23	;
		pixelToCM[35]=	23	;
		pixelToCM[36]=	23	;
		pixelToCM[37]=	23	;
		pixelToCM[38]=	23	;
		pixelToCM[39]=	23	;
		pixelToCM[40]=	23	;
		pixelToCM[41]=	23	;
		pixelToCM[42]=	23	;
		pixelToCM[43]=	23	;
		pixelToCM[44]=	23	;
		pixelToCM[45]=	23	;
		pixelToCM[46]=	23	;
		pixelToCM[47]=	23	;
		pixelToCM[48]=	23	;
		pixelToCM[49]=	23	;
		pixelToCM[50]=	23	;
		pixelToCM[51]=	23	;
		pixelToCM[52]=	23	;
		pixelToCM[53]=	23	;
		pixelToCM[54]=	23	;
		pixelToCM[55]=	23	;
		pixelToCM[56]=	23	;
		pixelToCM[57]=	23	;
		pixelToCM[58]=	23	;
		pixelToCM[59]=	23	;
		pixelToCM[60]=	23	;
		pixelToCM[61]=	23	;
		pixelToCM[62]=	23	;
		pixelToCM[63]=	23	;
		pixelToCM[64]=	23	;
		pixelToCM[65]=	24	;
		pixelToCM[66]=	24	;
		pixelToCM[67]=	25	;
		pixelToCM[68]=	25	;
		pixelToCM[69]=	26	;
		pixelToCM[70]=	26	;
		pixelToCM[71]=	26	;
		pixelToCM[72]=	27	;
		pixelToCM[73]=	27	;
		pixelToCM[74]=	28	;
		pixelToCM[75]=	28	;
		pixelToCM[76]=	29	;
		pixelToCM[77]=	29	;
		pixelToCM[78]=	30	;
		pixelToCM[79]=	30	;
		pixelToCM[80]=	30	;
		pixelToCM[81]=	31	;
		pixelToCM[82]=	31	;
		pixelToCM[83]=	32	;
		pixelToCM[84]=	32	;
		pixelToCM[85]=	32	;
		pixelToCM[86]=	33	;
		pixelToCM[87]=	33	;
		pixelToCM[88]=	34	;
		pixelToCM[89]=	34	;
		pixelToCM[90]=	34	;
		pixelToCM[91]=	35	;
		pixelToCM[92]=	35	;
		pixelToCM[93]=	36	;
		pixelToCM[94]=	36	;
		pixelToCM[95]=	37	;
		pixelToCM[96]=	37	;
		pixelToCM[97]=	38	;
		pixelToCM[98]=	39	;
		pixelToCM[99]=	39	;
		pixelToCM[100]=	39	;
		pixelToCM[101]=	40	;
		pixelToCM[102]=	41	;
		pixelToCM[103]=	41	;
		pixelToCM[104]=	41	;
		pixelToCM[105]=	42	;
		pixelToCM[106]=	43	;
		pixelToCM[107]=	43	;
		pixelToCM[108]=	44	;
		pixelToCM[109]=	44	;
		pixelToCM[110]=	45	;
		pixelToCM[111]=	45	;
		pixelToCM[112]=	46	;
		pixelToCM[113]=	47	;
		pixelToCM[114]=	47	;
		pixelToCM[115]=	47	;
		pixelToCM[116]=	48	;
		pixelToCM[117]=	49	;
		pixelToCM[118]=	49	;
		pixelToCM[119]=	50	;
		pixelToCM[120]=	50	;
		pixelToCM[121]=	51	;
		pixelToCM[122]=	52	;
		pixelToCM[123]=	53	;
		pixelToCM[124]=	53	;
		pixelToCM[125]=	54	;
		pixelToCM[126]=	54	;
		pixelToCM[127]=	55	;
		pixelToCM[128]=	55	;
		pixelToCM[129]=	56	;
		pixelToCM[130]=	57	;
		pixelToCM[131]=	58	;
		pixelToCM[132]=	58	;
		pixelToCM[133]=	59	;
		pixelToCM[134]=	59	;
		pixelToCM[135]=	60	;
		pixelToCM[136]=	61	;
		pixelToCM[137]=	62	;
		pixelToCM[138]=	63	;
		pixelToCM[139]=	63	;
		pixelToCM[140]=	64	;
		pixelToCM[141]=	65	;
		pixelToCM[142]=	66	;
		pixelToCM[143]=	67	;
		pixelToCM[144]=	68	;
		pixelToCM[145]=	69	;
		pixelToCM[146]=	69	;
		pixelToCM[147]=	70	;
		pixelToCM[148]=	71	;
		pixelToCM[149]=	72	;
		pixelToCM[150]=	72	;
		pixelToCM[151]=	73	;
		pixelToCM[152]=	74	;
		pixelToCM[153]=	75	;
		pixelToCM[154]=	76	;
		pixelToCM[155]=	77	;
		pixelToCM[156]=	77	;
		pixelToCM[157]=	78	;
		pixelToCM[158]=	79	;
		pixelToCM[159]=	80	;
		pixelToCM[160]=	81	;
		pixelToCM[161]=	82	;
		pixelToCM[162]=	83	;
		pixelToCM[163]=	84	;
		pixelToCM[164]=	85	;
		pixelToCM[165]=	86	;
		pixelToCM[166]=	87	;
		pixelToCM[167]=	88	;
		pixelToCM[168]=	89	;
		pixelToCM[169]=	90	;
		pixelToCM[170]=	91	;
		pixelToCM[171]=	92	;
		pixelToCM[172]=	93	;
		pixelToCM[173]=	94	;
		pixelToCM[174]=	95	;
		pixelToCM[175]=	96	;
		pixelToCM[176]=	97	;
		pixelToCM[177]=	98	;
		pixelToCM[178]=	100	;
		pixelToCM[179]=	100	;
		pixelToCM[180]=	101	;
		pixelToCM[181]=	102	;
		pixelToCM[182]=	103	;
		pixelToCM[183]=	104	;
		pixelToCM[184]=	105	;
		pixelToCM[185]=	106	;
		pixelToCM[186]=	108	;
		pixelToCM[187]=	109	;
		pixelToCM[188]=	110	;
		pixelToCM[189]=	111	;
		pixelToCM[190]=	113	;
		pixelToCM[191]=	114	;
		pixelToCM[192]=	116	;
		pixelToCM[193]=	117	;
		pixelToCM[194]=	118	;
		pixelToCM[195]=	120	;
		pixelToCM[196]=	122	;
		pixelToCM[197]=	123	;
		pixelToCM[198]=	125	;
		pixelToCM[199]=	127	;
		pixelToCM[200]=	129	;
		pixelToCM[201]=	130	;
		pixelToCM[202]=	132	;
		pixelToCM[203]=	134	;
		pixelToCM[204]=	135	;
		pixelToCM[205]=	137	;
		pixelToCM[206]=	139	;
		pixelToCM[207]=	141	;
		pixelToCM[208]=	143	;
		pixelToCM[209]=	144	;
		pixelToCM[210]=	146	;
		pixelToCM[211]=	149	;
		pixelToCM[212]=	151	;
		pixelToCM[213]=	153	;
		pixelToCM[214]=	155	;
		pixelToCM[215]=	157	;
		pixelToCM[216]=	159	;
		pixelToCM[217]=	161	;
		pixelToCM[218]=	164	;
		pixelToCM[219]=	166	;
		pixelToCM[220]=	168	;
		pixelToCM[221]=	170	;
		pixelToCM[222]=	174	;
		pixelToCM[223]=	176	;
		pixelToCM[224]=	179	;
		pixelToCM[225]=	182	;
		pixelToCM[226]=	184	;
		pixelToCM[227]=	186	;
		pixelToCM[228]=	189	;
		pixelToCM[229]=	193	;
		pixelToCM[230]=	195	;
		pixelToCM[231]=	197	;
		pixelToCM[232]=	200	;
		pixelToCM[233]=	203	;
		pixelToCM[234]=	209	;
		pixelToCM[235]=	212	;
		pixelToCM[236]=	215	;
		pixelToCM[237]=	221	;
		pixelToCM[238]=	224	;
		pixelToCM[239]=	227	;
		pixelToCM[240]=	230	;
		pixelToCM[241]=	235	;
		pixelToCM[242]=	239	;
		pixelToCM[243]=	244	;
		pixelToCM[244]=	250	;
		pixelToCM[245]=	254	;
		pixelToCM[246]=	257	;
		pixelToCM[247]=	262	;
		pixelToCM[248]=	267	;
		pixelToCM[249]=	274	;
		pixelToCM[250]=	279	;
		pixelToCM[251]=	287	;
		pixelToCM[252]=	293	;
		pixelToCM[253]=	297	;
		pixelToCM[254]=	302	;
		pixelToCM[255]=	302	;
		pixelToCM[256]=	302	;
		pixelToCM[257]=	302	;
		pixelToCM[258]=	302	;
		pixelToCM[259]=	302	;
		pixelToCM[260]=	302	;
		pixelToCM[261]=	302	;
		pixelToCM[262]=	302	;
		pixelToCM[263]=	302	;
		pixelToCM[264]=	302	;
		pixelToCM[265]=	302	;
		pixelToCM[266]=	302	;
		pixelToCM[267]=	302	;
		pixelToCM[268]=	302	;
		pixelToCM[269]=	302	;
		pixelToCM[270]=	302	;
		pixelToCM[271]=	302	;
		pixelToCM[272]=	302	;
		pixelToCM[273]=	302	;
		pixelToCM[274]=	302	;
		pixelToCM[275]=	302	;
		pixelToCM[276]=	302	;
		pixelToCM[277]=	302	;
		pixelToCM[278]=	302	;
		pixelToCM[279]=	302	;
		pixelToCM[280]=	302	;
		pixelToCM[281]=	302	;
		pixelToCM[282]=	302	;
		pixelToCM[283]=	302	;
		pixelToCM[284]=	302	;
		pixelToCM[285]=	302	;
		pixelToCM[286]=	302	;
		pixelToCM[287]=	302	;
		pixelToCM[288]=	302	;
		pixelToCM[289]=	302	;
		pixelToCM[290]=	302	;
		pixelToCM[291]=	302	;
		pixelToCM[292]=	302	;
		pixelToCM[293]=	302	;
		pixelToCM[294]=	302	;
		pixelToCM[295]=	302	;
		pixelToCM[296]=	302	;
		pixelToCM[297]=	302	;
		pixelToCM[298]=	302	;
		pixelToCM[299]=	302	;
		pixelToCM[300]=	302	;
		pixelToCM[301]=	302	;
		pixelToCM[302]=	302	;
		pixelToCM[303]=	302	;
		pixelToCM[304]=	302	;
		pixelToCM[305]=	302	;
		pixelToCM[306]=	302	;
		pixelToCM[307]=	302	;
		pixelToCM[308]=	302	;
		pixelToCM[309]=	302	;
		pixelToCM[310]=	302	;
		pixelToCM[311]=	302	;
		pixelToCM[312]=	302	;
		pixelToCM[313]=	302	;
		pixelToCM[314]=	302	;
		pixelToCM[315]=	302	;
		pixelToCM[316]=	302	;
		pixelToCM[317]=	302	;
		pixelToCM[318]=	302	;
		pixelToCM[319]=	302	;
		pixelToCM[320]=	302	;
		pixelToCM[321]=	302	;
		pixelToCM[322]=	302	;
		pixelToCM[323]=	302	;
		pixelToCM[324]=	302	;
		pixelToCM[325]=	302	;
		pixelToCM[326]=	302	;
		pixelToCM[327]=	302	;
		pixelToCM[328]=	302	;
		pixelToCM[329]=	302	;
		pixelToCM[330]=	302	;
		pixelToCM[331]=	302	;
		pixelToCM[332]=	302	;
		pixelToCM[333]=	302	;
		pixelToCM[334]=	302	;
		pixelToCM[335]=	302	;
		pixelToCM[336]=	302	;
		pixelToCM[337]=	302	;
		pixelToCM[338]=	302	;
		pixelToCM[339]=	302	;
		pixelToCM[340]=	302	;
		pixelToCM[341]=	302	;
		pixelToCM[342]=	302	;
		pixelToCM[343]=	302	;
		pixelToCM[344]=	302	;
		pixelToCM[345]=	302	;
		pixelToCM[346]=	302	;
		pixelToCM[347]=	302	;
		pixelToCM[348]=	302	;
		pixelToCM[349]=	302	;
		pixelToCM[350]=	302	;
		pixelToCM[351]=	302	;
		pixelToCM[352]=	302	;
		pixelToCM[353]=	302	;
		pixelToCM[354]=	302	;
		pixelToCM[355]=	302	;
		pixelToCM[356]=	302	;
		pixelToCM[357]=	302	;
		pixelToCM[358]=	302	;
		pixelToCM[359]=	302	;
		pixelToCM[360]=	302	;
		pixelToCM[361]=	302	;
		pixelToCM[362]=	302	;
		pixelToCM[363]=	302	;
		pixelToCM[364]=	302	;
		pixelToCM[365]=	302	;
		pixelToCM[366]=	302	;
		pixelToCM[367]=	302	;
		pixelToCM[368]=	302	;
		pixelToCM[369]=	302	;
		pixelToCM[370]=	302	;
		pixelToCM[371]=	302	;
		pixelToCM[372]=	302	;
		pixelToCM[373]=	302	;
		pixelToCM[374]=	302	;
		pixelToCM[375]=	302	;
		pixelToCM[376]=	302	;
		pixelToCM[377]=	302	;
		pixelToCM[378]=	302	;
		pixelToCM[379]=	302	;
		pixelToCM[380]=	302	;
		pixelToCM[381]=	302	;
		pixelToCM[382]=	302	;
		pixelToCM[383]=	302	;
		pixelToCM[384]=	302	;
		pixelToCM[385]=	302	;
		pixelToCM[386]=	302	;
		pixelToCM[387]=	302	;
		pixelToCM[388]=	302	;
		pixelToCM[389]=	302	;
		pixelToCM[390]=	302	;
		pixelToCM[391]=	302	;
		pixelToCM[392]=	302	;
		pixelToCM[393]=	302	;
		pixelToCM[394]=	302	;
		pixelToCM[395]=	302	;
		pixelToCM[396]=	302	;
		pixelToCM[397]=	302	;
		pixelToCM[398]=	302	;
		pixelToCM[399]=	302	;
		pixelToCM[400]=	302	;
#pragma endregion
		}

	}

};


}
