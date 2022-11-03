/*
  ImageMethod.h V1.0
  Features:
  1.thoughVeryFastHSV: For runtime color model in competition .
  2.thoughAdjustHSV: For runtime color model setup you can get the result of color model 
  3.thoughObjectTrace:To find the object, extract object from meaningMap constrcuting from color model
  4.updataGreenFieldBound: To find the green field boudary.
  5.setupFunctions : setup LUTs ..
  (construting...)6.get the 2 layers' white line>
  Jacky Guo 2014.4
  */

#pragma once
//This is the image process header file for panda by jacky 2013@ MCU-RRC , Taiwan
//I and my friends really enjoy this game!



#include"../cameraHeader2_7_3_17/FlyCapture2.h"
#include<stdio.h>
#include<stdlib.h>
#include<windows.h>
#include"Pixel.h"
#include"HsvEdge.h"
#include<cmath>
#include"ObjectInField.h"
#include"Localization.h"

using namespace FlyCapture2;
using namespace std;

/**************************
kevin: result plot feature
**************************/
#define PlotMagnitudeLineForWhiteLine
#ifdef PlotMagnitudeLineForWhiteLine
static int PlotMagnitudeLineIgnoreAngle=6;
#endif // PlotMagnitudeLineForWhiteLine

namespace ImageCenter{
	//class template for sync signal among image process,localization,serialport rx/tx 
	class threadSignal{
	 public://dont forget public!!
		bool printResultReady;
		bool localizationReady;
		bool externalShareReady;
		bool printLocalizeTimeReady;

		threadSignal()
		{
			 printResultReady = true;
			 localizationReady = true;
			 externalShareReady = true;
			 printLocalizeTimeReady = true;
		}
	};


	struct CenterPixel
	{
	  int x;
	  int y;
	};

	enum outPutMode{
	    outputHSV,
		outputFastHSV,
		outputRaw,
		outputVeryFastHSV,
		outputNormalHSV,
		outputAdjustHSV,
		outputObjectTrace
	};


class ImageMethod
{
public:
	//constructor
	ImageMethod(void)
	{
		CamDisconnected = true;//just for opendialog 
		testPixel = new Pixel();
		FloorImage = new unsigned char[600*600*3];
		FloorToRadiualTable = new int[600*600];
		radialPixelToFloorDistanceTable = new int [400]; // 
		//whiteDetection = new unsigned char[800*800*3];
		for( initialI = 0;initialI<360;initialI++)//initial a 360*400's array store the color meanning 1->ball(XXX) 2->XXX 3->XXX easy to use
		{
			colorMeaningMap[initialI] = new int[400];//stand for every pixel's meaning in polar coodinate
		}
		

		greenFieldBound = new int[360]; //store the green field in any angle;
		robotSelfRadial =40;// descibe the robot self radial from omni camera frame
		maxMagnitude=330;// max Magnitude of searching range
		whiteDetection  = new unsigned char [800*800*3]; // for show white
		blackObstacle = new int [360];
		outputGreenFieldLineDrawing = true;

		outputFinalLineInRealMagnitude=false;//kevin add, to control whether plot the white line in real magnitude or not

		/*enum ColorMeaning
		{
			Ball, = 0
			White, = 1
			Blue, = 2
			Yellow, = 3
			Green, = 4
			other = 5
		};*/
		
		ballInfield = new ObjectInField(radialPixelToFloorDistanceTable);
		blueGoalInField = new ObjectInField(radialPixelToFloorDistanceTable);
		yellowGoadInField = new ObjectInField(radialPixelToFloorDistanceTable);
		
		//localize!
		//MyLocalization = new Localization();//20150607, kevin comment this, since this instance is not used in here.
		ObservedWhiteLine = new LocalizeDataBaseFormat[360];

			for(int angle=0;angle <360 ;angle++)
			{
				//reset
				ObservedWhiteLine[angle].count = 0;
				for(int j = 0;j<4; j++)
				ObservedWhiteLine[angle].data[j] = 0;
			}

		radiualCenter.x = 400;//robot center in the pixel
		radiualCenter.y = 400;//robot center in the pixel 
		floorCenter.x = 300;// recovery image's center pixel
		floorCenter.y = 300;//same as above


		whiteEdge.setEdge(165,150,240,220,240,210); //hsv
		greenEdge.setEdge(160,98,255,171,179,27);
		yellowEdge.setEdge(85,70,255,230,255,210);
		blueEdge.setEdge(234,180,255,208,91,31);
		ballEdge.setEdge(40,0,255,170,255,180);

#pragma region kevin add, for speed up the image parameter and some table build time
		readData();//this function will read the data from the default location, later will build it if load successful
#pragma endregion

		setUpFastHSVTable();//bool fast hsv
		setUpVeryFastHSVTable();// determine very fast from bool hsv
		setUpFloorToRadialTable();//determine Floor image to radiual image
		setUpPolarToCartesian();//set up polarToCartesian mapping
		setradialPixelToFloorDistanceTable();//set up the RadialPixelToFloorDistance ..int [i] = xx meter
		
		
		blackMin = 10;// the black's min,if V<= blackmin ,then it's black
		ballRange = 15;//the ball hs range
		yellowGoalRange = 10;// yellow goal hs range
		blueGoalRange = 30;// blue goal hs range
		whiteMin = 210;// white v min,if V>=whiteMin then it's white

		//look up table;
		pixelToCM = new int[401];
		setUpPixelToCMLUT();
	}

	//distorier
	~ImageMethod(void)
	{
	}


//-------------------------variable-------------------
	
    Error error;
	BusManager busMgr;
    unsigned int numCameras;
	FlyCapture2::Image rawImage;//raw RGB24 format    
	FlyCapture2::Image convertedImage;//convert to BGR format
	//FlyCapture2::Image outImage;
	unsigned char* pRawConvertImage;
	Camera cam;//cam ptr
	PGRGuid guid;// Cam GUID
	bool CamDisconnected; // true->cam discnt false->cam connected!
	
//---------------------image process varaible-----------------
	Pixel* testPixel ;// Pixel data ()
	HsvEdge whiteEdge,greenEdge,yellowEdge,blueEdge,ballEdge;//color model setup
	HsvEdge* adjustEdge;// a pointer using store which edge should be change. adjustEdge = &ballEdge....
	unsigned char* FloorImage; //store the floor view image transform from the radiual image
	int* FloorToRadiualTable;//FloorToRadiualTable[xy] = which pixel in radiual image
	int* RadiualToFloorTable;// RadiualToFloorTable[radial distance] = which pixel in floor image
	CenterPixel radiualCenter,floorCenter;//center pixel for radiual image and floor image
	int* colorMeaningMap[360];//stand for every pixel's meaning in polar coodinate
	int* polarToCartesian[360];//polarToCartesian...sry I forget..maybe : polarToCartesian[angle][distance] = a squence i@(800*800)
	//,but I'am sure that it's not the pixel squence IT'S A COODINATE SQUENCE.
	int* greenFieldBound;//store 360 degree's green field  boundary
	int robotSelfRadial;// robot self Radial in Radial distance
	int initialI;// initialI
	int maxMagnitude;// the max search magnitude 
	int* blackObstacle; // - value 0 means no obstacle in that angle... the max distance is 3 meters
//--------------------Object Trace variable--------------------
	int * radialPixelToFloorDistanceTable;//  int* [radial] = XX meters , radialToFloorDistance
	ObjectInField* ballInfield,*blueGoalInField,*yellowGoadInField;
	int blackMin,ballRange,yellowGoalRange,blueGoalRange,whiteMin;
	unsigned char* whiteDetection ;// store for the white line detection
//---------------------control output-----------
	bool outputGreenFieldLineDrawing;

	bool outputFinalLineInRealMagnitude;
	
//---------------------Localize-----------------
	//Localization* MyLocalization;//20150607, kevin comment this, since this instance is not used in here.
	LocalizeDataBaseFormat *ObservedWhiteLine;
//--------------------Pixel to CM LUT
	int* pixelToCM;

//---------------------Camera method-------------------//---------------------Camera method-------------------
//---------------------Camera method-------------------//---------------------Camera method-------------------
//-----------------------------------------------------
	int CamConnect()
	{
		if(CamDisconnected==true)
		{
			error = busMgr.GetNumOfCameras(&numCameras);
			 for (unsigned int i=0; i < numCameras; i++)
			 {
				error = busMgr.GetCameraFromIndex(i, &guid);
				error = cam.Connect(&guid);
	   		 }
		}
	return 0;
	}
//-----------------------------------------------------
//-----------------------------------------------------
	int  ConnectAndCapture(PGRGuid guid)
	{
		if(CamDisconnected == true)
			error = cam.Connect(&guid);

		error = cam.StartCapture();
		CamDisconnected = false;
		return 0;
	}
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
	unsigned char* getRawImage()
	{
		error = this->cam.RetrieveBuffer( &(this->rawImage));
		rawImage.Convert(PIXEL_FORMAT_BGR,&convertedImage);
		return convertedImage.GetData();
	}
// Image process// Image process// Image process// Image process// Image process// Image process// Image process// Image process
// Image process// Image process// Image process// Image process// Image process// Image process// Image process// Image process
//-----------------------------------------------------
	unsigned char* thoughVeryFastHSV(unsigned char* BGRData)//with field boundary. will not change the BGRData, build a meanning map for object trace and line detect
	{
	 updataGreenFieldBound(BGRData);// show hsv mode in range of the green field
	 // first green ,second ball,then blue yellow goal,final black
	 int b,g,r,h,s,v;
	 int i;//pixel squence in the cartesian coordiante
	 int angle = 0,magnitude=0;
	 unsigned char determinator;
	 bool theLastPixelIsGreen = false;
	 //initial the colormeaing m
	  for(angle=0;angle <360 ;angle++)
			for(magnitude=robotSelfRadial;magnitude<=maxMagnitude;magnitude++)
				colorMeaningMap[angle][magnitude] = 1000;//1000 -> no meaning, just for initial, cant equal to any of enum{XXX}...

	 //assume all the color in the field is green
	 for(angle=0;angle <360 ;angle++)
		for(magnitude=robotSelfRadial;magnitude<= greenFieldBound[angle] ;magnitude++)
			colorMeaningMap[angle][magnitude]=ColorMeaning::Green;	 

	 int nowDifference;//hs range(No V!,Only the shadow will make difference )
	 int whiteThreshold = this->whiteMin;



#pragma region ball finder
	// int ballRange = this->ballRange;//hs range(No V!,Only the shadow will make difference )
	 for(angle=0;angle <360 ;angle++)
			for(magnitude=robotSelfRadial;magnitude<=maxMagnitude;magnitude++)
			{
				i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
				b = BGRData[i] ;
				g = BGRData[i+1] ;
				r = BGRData[i+2] ;
			    determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
// set the ball meanning map in global area ,Not only in the field
				if(determinator ==  ColorMeaning::Ball && (magnitude-greenFieldBound[angle]<10))// the color is ball's color and its magnitude is in the range of the field,reduce the ball's color out of the field
				{
					colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;
					
					if(colorMeaningMap[angle][magnitude-1] != ColorMeaning::Ball)
					{//check the form pixel in the same angle...
						int tempi = i,tempMagnitude = magnitude;
						int nextH,nextS,nextV;
						while(1)//stop until find the pixel is out of ball range.
						{
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;

							h = testPixel->BGR_HSV_H[b][g][r];//current
							s = testPixel->BGR_HSV_S[b][g][r];//current
							//v = testPixel->BGR_HSV_V[b][g][r];//current
							//current  and next time the form pixel beacuse of tempMagnitude--;
							tempMagnitude--;
							if(tempMagnitude <= robotSelfRadial) break;
							tempi = (int(polarToCartesian[angle][tempMagnitude]/800))*800*3+(polarToCartesian[angle][tempMagnitude]%800)*3;
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							nextH = testPixel->BGR_HSV_H[b][g][r];//backward
							nextS = testPixel->BGR_HSV_S[b][g][r];//backward
							nextV = testPixel->BGR_HSV_V[b][g][r];//backward
							nowDifference = (int)sqrt((double)((nextH-h)*(nextH-h) + (nextS-s)*(nextS-s)));//+ 0.5*(nextV-v)*(nextV-v))); //+(nextV-v));
							
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							//if(nextV>=150)
							//	colorMeaningMap[angle][tempMagnitude] = ColorMeaning::Ball;

							//if(colorMeaningMap[angle][tempMagnitude] == ColorMeaning::Green)// if green out!
							//	break;
							//else 
							if(nowDifference<=this->ballRange || determinator == ColorMeaning::Ball)
								colorMeaningMap[angle][tempMagnitude] = ColorMeaning::Ball;
							else break;
						}
						//Above:get all pixels in the ball shadow 
						//Below:get all pixels in the top of the ball (the brightness is very high, cant cover by color model, must be processed
						//in this step,named 'part of bright top' filter  )
						// start with next pixel
					
						while (1)
						{
						//	now pixel information
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							h = testPixel->BGR_HSV_H[b][g][r];//current
							s = testPixel->BGR_HSV_S[b][g][r];//current
							v = testPixel->BGR_HSV_V[b][g][r];//current
							magnitude++;// the next pixel information
							if(magnitude>maxMagnitude) break;
							//if the hs >XXX and V.XXX then it's the ball
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							nextH = testPixel->BGR_HSV_H[b][g][r];//forward
							nextS = testPixel->BGR_HSV_S[b][g][r];//forward
							nextV = testPixel->BGR_HSV_V[b][g][r];//forward
							nowDifference = (int)sqrt((double)((nextH-h)*(nextH-h) + (nextS-s)*(nextS-s)));//+ 0.5*(nextV-v)*(nextV-v)));
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							//nowDifference = (int)sqrt((double)((nextV-v)*(nextV-v)));
							//where is the problem??!!
							//if(colorMeaningMap[angle][magnitude]== ColorMeaning::Green )// meet green or black
							//	break;
							//else 
							if(nowDifference<= this->ballRange || determinator == ColorMeaning::Ball)// if the pixel is ball's color , continue check next pixel 
							{
						    	colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;
							}
							else break;							 
						}

					}//if(colorMeaningMap[angle][magnitude-1] != ColorMeaning::Ball)

				}// if(determinator ==  ColorMeaning::Ball)
				//else dont care if not orange
				//{
				//	colorMeaningMap[angle][magnitude] = ColorMeaning::other;
				//}
			}// ball find global for loop end
// Above :  set the ball meanning map in global area,Not only in the field
#pragma endregion 

#pragma region  yellow goal finder
	 for(angle=0;angle <360 ;angle++)
		 for(magnitude=greenFieldBound[angle];magnitude<=maxMagnitude  ;magnitude++)//Search from boudary of field. So the ball will not influence the result.
		 {
			 i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
				b = BGRData[i] ;
				g = BGRData[i+1] ;
				r = BGRData[i+2] ;
			    determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
				if(determinator ==  ColorMeaning::Yellow && (magnitude-greenFieldBound[angle]<100) && (greenFieldBound[angle] > robotSelfRadial))// the color is goal's color and its magnitude is in the range of the field,reduce the goal's color out of the field
				{																				  // if the greenFieldBound is exsited.
					colorMeaningMap[angle][magnitude] = ColorMeaning::Yellow;
					int tempi = i,tempMagnitude = magnitude;
					int nextH,nextS;
					while (1)
						{
						//	now pixel information
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							h = testPixel->BGR_HSV_H[b][g][r];//current
							s = testPixel->BGR_HSV_S[b][g][r];//current
							v = testPixel->BGR_HSV_V[b][g][r];//current
							magnitude++;// the next pixel information
							if(magnitude>maxMagnitude) break;
							//if the hs >XXX and V.XXX then it's the ball
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							nextH = testPixel->BGR_HSV_H[b][g][r];//forward
							nextS = testPixel->BGR_HSV_S[b][g][r];//forward
							
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];

							nowDifference = (int)sqrt((double)((nextH-h)*(nextH-h) + (nextS-s)*(nextS-s)));
							//nowDifference = (int)sqrt((double)((nextV-v)*(nextV-v)));
							//where is the problem??!!
							if(nowDifference<= this->yellowGoalRange || determinator== ColorMeaning::Yellow)// if the pixel is ball's color , continue check next pixel 
							{
								colorMeaningMap[angle][magnitude] = ColorMeaning::Yellow;
							}
							else break;							 
						}

				}
		 }
	#pragma endregion   

#pragma region  blue goal finder
	//	 int goalRange = blueGoalRange;//hs range(No V!,Only the shadow will make difference )
		 for(angle=0;angle <360 ;angle++)
		 for(magnitude=greenFieldBound[angle];magnitude<=maxMagnitude ;magnitude++)
		 {
			 i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
				b = BGRData[i] ;
				g = BGRData[i+1] ;
				r = BGRData[i+2] ;
			    determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
				if(determinator ==  ColorMeaning::Blue && (magnitude-greenFieldBound[angle]<100) )// the color is ball's color and its magnitude is in the range of the field,reduce the ball's color out of the field
				{
					colorMeaningMap[angle][magnitude] = ColorMeaning::Blue;
					int tempi = i,tempMagnitude = magnitude;
					
					int nextH,nextS;

					while (1)
						{
						//	now pixel information
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							h = testPixel->BGR_HSV_H[b][g][r];//current
							s = testPixel->BGR_HSV_S[b][g][r];//current
							v = testPixel->BGR_HSV_V[b][g][r];//current
							magnitude++;// the next pixel information
							if(magnitude>maxMagnitude) break;
							//if the hs >XXX and V.XXX then it's the ball
							tempi = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;// the new i
							b = BGRData[tempi] ;
							g = BGRData[tempi+1] ;
							r = BGRData[tempi+2] ;
							nextH = testPixel->BGR_HSV_H[b][g][r];//forward
							nextS = testPixel->BGR_HSV_S[b][g][r];//forward
			
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];

							nowDifference = (int)sqrt((double)((nextH-h)*(nextH-h) + (nextS-s)*(nextS-s)));
							//nowDifference = (int)sqrt((double)((nextV-v)*(nextV-v)));
							//where is the problem??!!
							if(nowDifference<= this->blueGoalRange || determinator== ColorMeaning::Blue)// if the pixel is ball's color , continue check next pixel 
							{
								colorMeaningMap[angle][magnitude] = ColorMeaning::Blue;
							}
							else break;							 
						}

				}
		 }
#pragma endregion 

#pragma region  obstacle finder,large black finder
			for(angle=0;angle <360 ;angle++)
			{
				int blackCountThreshold = 10;
				int blackCount = 0;//if black continous in blackCountThreshold
				int blackMin = this->blackMin;
				// obstacle search range.
				for(magnitude=80;magnitude<=maxMagnitude;magnitude++)
				{
					i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
					b = BGRData[i] ;
					g = BGRData[i+1] ;
					r = BGRData[i+2] ; //find white and black in the field
					if(testPixel->BGR_HSV_V[b][g][r]<blackMin && colorMeaningMap[angle][magnitude]!= ColorMeaning::Ball )// if the pixel's brightness is high
					{
					//label the first met as black 
						blackCount ++;
						if(blackCount==blackCountThreshold)
						{
							colorMeaningMap[angle][magnitude-blackCountThreshold] = ColorMeaning::obstacle;
						 break;
						}
					}
					else 
					{
						blackCount = 0;//reset if one is not black
					}
				}
			}
			//Here check the "Other" pixel  obstacle? ball? white?
#pragma endregion

#pragma region rotary search
//search every angle at every radial.
		for(magnitude=robotSelfRadial;magnitude<=maxMagnitude;magnitude++)
		 for(angle=0;angle <360 ;angle++)
			{
				int metAngle = angle;
				switch (colorMeaningMap[angle][magnitude] )
				{
				case ColorMeaning::Ball :
					//backward
						while(1)//CCW search
						{
							angle--;
							if(angle < 0) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Ball) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];

							/*if(determinator == ColorMeaning::Ball )
								colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;*/

							//the high brigtness part on the ball is Labeled Yellow but not labeled yellow goal
							if(determinator == ColorMeaning::Ball || ( determinator== ColorMeaning::Yellow && colorMeaningMap[angle][magnitude] == ColorMeaning::White)  )
								colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;
							else break;

						}
					//forward
						angle = metAngle;
						while(1)// CW search
						{
							angle++;
							if(angle>360) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Ball) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							/*if(determinator == ColorMeaning::Ball)
								colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;20140816*/

							if(determinator == ColorMeaning::Ball || ( determinator== ColorMeaning::Yellow && colorMeaningMap[angle][magnitude] == ColorMeaning::White ))
								colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;
							else break;
						}

					break;

				case ColorMeaning::Blue :
					while(1)
						{
							angle--;
							if(angle < 0) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Blue) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							if(determinator == ColorMeaning::Blue)
								colorMeaningMap[angle][magnitude] = ColorMeaning::Blue;
							else break;

						}
					//forward
						angle = metAngle;
						while(1)
						{
							angle++;
							if(angle>360) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Blue) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							if(determinator == ColorMeaning::Blue)
								colorMeaningMap[angle][magnitude] = ColorMeaning::Blue;
							else break;
						}

					break;
				case ColorMeaning::Yellow :
					while(1)
						{
							angle--;
							if(angle < 0) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Yellow) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							if(determinator == ColorMeaning::Yellow)
								colorMeaningMap[angle][magnitude] = ColorMeaning::Yellow;
							else break;

						}
					//forward
						angle = metAngle;
						while(1)
						{
							angle++;
							if(angle>360) break;
							if(colorMeaningMap[angle][magnitude] == ColorMeaning::Yellow) continue;
							i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							determinator = testPixel->VeryFast_BGR_HSV[b][g][r];
							if(determinator == ColorMeaning::Yellow)
								colorMeaningMap[angle][magnitude] = ColorMeaning::Yellow;
							else break;
						}
					break;



					default:
					break;
				}//switch
			}//for
#pragma endregion

#pragma region For reduce the highlight part of the ball(radial) 
			int whiteStart,whiteEnd;
			bool faceBallTwice = false,faceWhiteOnce = false;
			int count = 0;

			for(angle=0;angle <360 ;angle++)
			{
				faceBallTwice = false;
				faceWhiteOnce = false;
				count = 0;
				for(magnitude=robotSelfRadial;magnitude<=maxMagnitude;magnitude++)
				{
					if(colorMeaningMap[angle][magnitude] == ColorMeaning::Ball) // first meet the ball
					{
						while(true)// search the white inside the ball
						{
						 magnitude++;
						 if(magnitude>maxMagnitude) break;

						    i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
							b = BGRData[i] ;
							g = BGRData[i+1] ;
							r = BGRData[i+2] ;
							// testPixel->BGR_HSV_V[b][g][r];

							if( testPixel->BGR_HSV_V[b][g][r]>whiteThreshold && colorMeaningMap[angle][magnitude]!= ColorMeaning::Ball && colorMeaningMap[angle][magnitude] != ColorMeaning::Yellow)//colorMeaningMap[angle][magnitude] == ColorMeaning::White  )
							{
								if(faceWhiteOnce== false)
								{
								 whiteStart = magnitude;
								 faceWhiteOnce = true;
								}
								count++;
							}

						 if(colorMeaningMap[angle][magnitude] == ColorMeaning::Ball && faceWhiteOnce == true )
							{
								whiteEnd = magnitude;
								faceBallTwice = true;
								break;
							}
						}//while
					}


					if(faceBallTwice == true && (whiteEnd - whiteStart < 35))
					{
						faceBallTwice = false;
						faceWhiteOnce = false;
						count = 0;
						for(magnitude = whiteStart;magnitude<=whiteEnd;magnitude++)
						 {
							 colorMeaningMap[angle][magnitude] = ColorMeaning::Ball;
						 }
					}

				}

				
			}
			
			
#pragma endregion

#pragma region white finder
		for(angle=0;angle <360 ;angle++)
			for(magnitude=robotSelfRadial;magnitude<= greenFieldBound[angle]-2 ;magnitude++)
			{
				i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
				b = BGRData[i] ;
				g = BGRData[i+1] ;
				r = BGRData[i+2] ; //find white and black in the field
				if(testPixel->BGR_HSV_V[b][g][r]>whiteThreshold && colorMeaningMap[angle][magnitude]!= ColorMeaning::Ball && colorMeaningMap[angle][magnitude] != ColorMeaning::Yellow)// if the pixel's brightness is high
				{
					while (true)// to reduce noise: if the white continous 2 pixel in the same angle then label there're white
					{
						i = (int(polarToCartesian[angle][magnitude+1]/800))*800*3+(polarToCartesian[angle][magnitude+1]%800)*3;
						b = BGRData[i] ;
						g = BGRData[i+1] ;
						r = BGRData[i+2] ; //find white and black in the field
						if(testPixel->BGR_HSV_V[b][g][r]>whiteThreshold && 
							colorMeaningMap[angle][magnitude]!= ColorMeaning::Ball && 
							colorMeaningMap[angle][magnitude] != ColorMeaning::Yellow &&
							magnitude<= 212)//pixel<=232 means that the real distance is less than 200cm.
						{//if the next v still bright,then label the form is white
							colorMeaningMap[angle][magnitude] = ColorMeaning::White;
							break;
						}
						else 
						{
							magnitude++;
							break;
						}
					}

				}// if white end ..
				 
				/*if(angle == 270 && colorMeaningMap[angle][magnitude] == ColorMeaning::White)
					int a = 0;*/
			} // white finder ,field for loop end
#pragma endregion 
		return BGRData;
	}
//-----------------------------------------------------
	unsigned char* thoughAdjustHSV(unsigned char* BGRData)
	{
		int b,g,r;
		int h;
		unsigned char s,v;
		int i ;
		for(i=0;i<800*800*3;i=i+3)
		{
			b = BGRData[i];
			g = BGRData[i+1];
			r = BGRData[i+2];

			if(ballEdge.H_up >= ballEdge.H_low )
			{
				if( testPixel->BGR_HSV_H[b][g][r] <= ballEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= ballEdge.H_low &&
					testPixel->BGR_HSV_S[b][g][r] <= ballEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= ballEdge.S_low &&
					testPixel->BGR_HSV_V[b][g][r] <= ballEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= ballEdge.V_low)
				{
					BGRData[i] = 128;
					BGRData[i+1] = 0;
					BGRData[i+2] =128;
				}

					else if(testPixel->BGR_HSV_H[b][g][r] <= blueEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= blueEdge.H_low &&
					testPixel->BGR_HSV_S[b][g][r] <= blueEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= blueEdge.S_low &&
					testPixel->BGR_HSV_V[b][g][r] <= blueEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= blueEdge.V_low)
				{
					BGRData[i] = 128;
					BGRData[i+1] =0;
					BGRData[i+2] =255;
				}
				else if(testPixel->BGR_HSV_H[b][g][r] <= yellowEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= yellowEdge.H_low &&
						testPixel->BGR_HSV_S[b][g][r] <= yellowEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= yellowEdge.S_low &&
						testPixel->BGR_HSV_V[b][g][r] <= yellowEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= yellowEdge.V_low)
				{
					BGRData[i] = 22;
					BGRData[i+1] = 22;
					BGRData[i+2] = 222;
				}
				else if(testPixel->BGR_HSV_H[b][g][r] <= greenEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= greenEdge.H_low &&
						testPixel->BGR_HSV_S[b][g][r] <= greenEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= greenEdge.S_low &&
						testPixel->BGR_HSV_V[b][g][r] <= greenEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= greenEdge.V_low)
				{
					BGRData[i] = 0;
					BGRData[i+1] =255;
					BGRData[i+2] =0;
				}
				else
				{
					/*BGRData[i] = 255;
					BGRData[i+1] =255;
					BGRData[i+2] =255;*/
				}
			}

			else // balledage.h_up < balledge.h_low
			{
				if( (testPixel->BGR_HSV_H[b][g][r] >= ballEdge.H_low || testPixel->BGR_HSV_H[b][g][r] <= ballEdge.H_up) 
					    &&
						testPixel->BGR_HSV_S[b][g][r] <= ballEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= ballEdge.S_low 
						&&
						testPixel->BGR_HSV_V[b][g][r] <= ballEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= ballEdge.V_low)
					{
						BGRData[i] = 128;
						BGRData[i+1] = 0;
						BGRData[i+2] =128;
					}

					else if(testPixel->BGR_HSV_H[b][g][r] <= blueEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= blueEdge.H_low &&
						testPixel->BGR_HSV_S[b][g][r] <= blueEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= blueEdge.S_low &&
						testPixel->BGR_HSV_V[b][g][r] <= blueEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= blueEdge.V_low)
					{
						BGRData[i] = 128;
						BGRData[i+1] =0;
						BGRData[i+2] =255;
					}
					else if(testPixel->BGR_HSV_H[b][g][r] <= yellowEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= yellowEdge.H_low &&
							testPixel->BGR_HSV_S[b][g][r] <= yellowEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= yellowEdge.S_low &&
							testPixel->BGR_HSV_V[b][g][r] <= yellowEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= yellowEdge.V_low)
					{
						BGRData[i] = 22;
						BGRData[i+1] = 22;
						BGRData[i+2] = 222;
					}
					else if(testPixel->BGR_HSV_H[b][g][r] <= greenEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= greenEdge.H_low &&
							testPixel->BGR_HSV_S[b][g][r] <= greenEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= greenEdge.S_low &&
							testPixel->BGR_HSV_V[b][g][r] <= greenEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= greenEdge.V_low)
					{
						BGRData[i] = 0;
						BGRData[i+1] =255;
						BGRData[i+2] =0;
					}
					else
					{
						/*BGRData[i] = 255;
						BGRData[i+1] =255;
						BGRData[i+2] =255;*/
					}
			}
	
		
		}
		return BGRData;
	}
//-----------------------------------------------------
	// transform the radius image to floor image ,output image is changable ,Not equal to original resolution
	unsigned char* thoughFloorTransform(unsigned char* BGRRadialImage)
	{
		int iOfFloorImg,iOfRadialImg;
		for(iOfFloorImg = 0;iOfFloorImg<=600*600;iOfFloorImg = iOfFloorImg + 1)
		{
			iOfRadialImg = FloorToRadiualTable[iOfFloorImg];
			FloorImage[iOfFloorImg*3] = BGRRadialImage[iOfRadialImg];
			FloorImage[iOfFloorImg*3+1] = BGRRadialImage[iOfRadialImg+1];
			FloorImage[iOfFloorImg*3+2] = BGRRadialImage[iOfRadialImg+2];
		}
		return FloorImage;
	}
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//----------------Object detect---------------------------	
unsigned char* thoughObjectTrace(unsigned char* BGRData)// trace for whtie yellow blue orange
{
		 //updataGreenFieldBound(BGRData); the very fast hsv has updated the green field boundary
	 int b,g,r;
	 int i;
	 int angle = 0,magnitude=0;
	 bool WhiteRecord = true;

	 int greenGap = 0;//green pixel between  first met line and second met line
	 ballInfield->reSet();
	 blueGoalInField->reSet();
	 yellowGoadInField->reSet();

		 delete whiteDetection;//only for show
		 whiteDetection = new unsigned char[800*800*3];
	     
		 
//white tracer local array, store the first meet white and give the white to"show white"
	 	for(angle=0;angle <360 ;angle++)
		{
			WhiteRecord = true;
			//reset
			ObservedWhiteLine[angle].count = 0;
			for(int j = 0;j<4; j++)
			ObservedWhiteLine[angle].data[j] = 0;

			for(magnitude=robotSelfRadial;magnitude<= greenFieldBound[angle] ;magnitude++)
			{
				if(ObservedWhiteLine[angle].count >=4) break;

			    i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
				unsigned char determinator = colorMeaningMap[angle][magnitude];

				//kevin add, for show the final line in real magnitude
				int finalLineInRealMagnitude;
				if(magnitude<0||magnitude>=400)//the constant here is according to pixelToCM length
					finalLineInRealMagnitude=0;
				else
				finalLineInRealMagnitude=pixelToCM[magnitude];
				int finalLineInRealMagnitudeIndex=(int(polarToCartesian[angle][finalLineInRealMagnitude]/800))*800*3+(polarToCartesian[angle][finalLineInRealMagnitude]%800)*3;
				
				if( determinator == ColorMeaning::White)
				{
					//20140816 
					BGRData[i] = 255;// may cause protect memory exception
					BGRData[i+1] = 255;// may cause protect memory exception
					BGRData[i+2] = 255;// may cause protect memory exception
					

					if(WhiteRecord == true)
					{
						ObservedWhiteLine[angle].count++; // record this!
						WhiteRecord = false;// this magnitude finished.

						ObservedWhiteLine[angle].data[ObservedWhiteLine[angle].count-1] = magnitude;
						

						BGRData[i] = 128;
						BGRData[i+1] = 255;
						BGRData[i+2] = 255;
						/*
						before kevin modified code, the original white line drawing

						whiteDetection[i] = 0;//for show
					    whiteDetection[i+1] = 0;//for show
					    whiteDetection[i+2] = 0;//for show
						*/
						//below is modified by kevin, original is in above section marked as kevin
						if(outputFinalLineInRealMagnitude)
						{
							whiteDetection[finalLineInRealMagnitudeIndex] = 0;//for show
							whiteDetection[finalLineInRealMagnitudeIndex+1] = 0;//for show
							whiteDetection[finalLineInRealMagnitudeIndex+2] = 0;//for show
						}
						else
						{
							whiteDetection[i] = 0;//for show
							whiteDetection[i+1] = 0;//for show
							whiteDetection[i+2] = 0;//for show
						}

						//above: for confirm the white detection
						greenGap = 0;//reset for continous count..
					}
					
				}//white line ...
			
				else if(determinator == ColorMeaning::obstacle && magnitude>100)//something shield the white give up the firstmetwhite in this angle
				{
					break;
				}
				else//meet green
				{
					if(WhiteRecord==false) greenGap++;

					if(greenGap > 5 && WhiteRecord==false ) 
					{
						greenGap = 0;
						WhiteRecord= true;
					}
				}
			}// magnitude -- for loop
		}//angle --  for loop

#ifdef PlotMagnitudeLineForWhiteLine
		for(int angle=0;angle<360;angle++)
		{
			//check whether to ignore this angle or not
			if((angle%PlotMagnitudeLineIgnoreAngle))
				continue;

			static int farestRadius=0;
			//get the farest radius
			if(ObservedWhiteLine[angle].count)//if not zero, white line found in this angle
			{
				if(outputFinalLineInRealMagnitude)
				{
					farestRadius=pixelToCM[ObservedWhiteLine[angle].data[ObservedWhiteLine[angle].count-1]];
				}
				else
				{
					farestRadius=ObservedWhiteLine[angle].data[ObservedWhiteLine[angle].count-1];
				}
			}
			else
			{
				//not white line found in this angle
				//comment below code if want to use last radius to plot on angle that no white line found
				//farestRadius=0;
			}

			//plot on the white detection buffer
			int startMagnitude;
			if(outputFinalLineInRealMagnitude)
			{
				startMagnitude=20;
			}
			else
			{
				startMagnitude=robotSelfRadial;
			}

			//main plot process
			for(int magnitude=startMagnitude;magnitude<farestRadius;magnitude++)
			{
				bool ignoreThisMagnitude=false;
				//check whether to ignore lower layer dot or not
				for(int i=0;i<(ObservedWhiteLine[angle].count-1);i++)
				{
					if(outputFinalLineInRealMagnitude)
					{
						if(magnitude==pixelToCM[ObservedWhiteLine[angle].data[i]])
						{
							ignoreThisMagnitude=true;
							break;
						}
					}
					else
					{
						if(magnitude==ObservedWhiteLine[angle].data[i])
						{
							ignoreThisMagnitude=true;
							break;
						}
					}
				}
				//check whether to plot or skip to next magnitude
				if(ignoreThisMagnitude)
				{
					continue;
				}
				else
				{
					int index=(int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;
					whiteDetection[index] = 0;//for show
					whiteDetection[index+1] = 128;//for show
					whiteDetection[index+2] = 255;//for show
				}
			}//radius
		}//angle
#endif // PlotMagnitudeLineForWhiteLine

		// obstacle blue yellow ball

		for(i = 0;i<360;i++)  blackObstacle[i] = 0;

		ballInfield->pixelCount = 0;
		blueGoalInField->pixelCount = 0;
		yellowGoadInField->pixelCount = 0;

		//for test
		/*for(angle = 0;angle<360;angle++)
		for(magnitude = 90;magnitude<=200;magnitude++)
		{
			colorMeaningMap[angle][magnitude] == ColorMeaning::Ball;
		}*/

		for(angle=0;angle <360 ;angle++)
		{
			for(magnitude=robotSelfRadial;magnitude<= maxMagnitude ;magnitude++)
			{
			    i = (int(polarToCartesian[angle][magnitude]/800))*800*3+(polarToCartesian[angle][magnitude]%800)*3;

				unsigned char determinator = colorMeaningMap[angle][magnitude];


			switch (determinator)// give the meanning color & finish find the object in radial phase and magnitude
			{
			case ColorMeaning::Ball:
					BGRData[i] = 255;
					BGRData[i+1] = 0;
					BGRData[i+2] = 0;
					ballInfield->gatherObjectInforamtion(angle,magnitude);
					ballInfield->pixelCount++;
					break;

			case ColorMeaning::Blue:
					BGRData[i] = 255;
					BGRData[i+1] = 128;
					BGRData[i+2] = 255;
					blueGoalInField->gatherObjectInforamtion(angle,magnitude);
					blueGoalInField->pixelCount++;
					break;

			case ColorMeaning::Yellow:
					BGRData[i] = 192;
					BGRData[i+1] = 128;
					BGRData[i+2] = 0;
					yellowGoadInField->gatherObjectInforamtion(angle,magnitude);
					yellowGoadInField->pixelCount++;
					break;

			case ColorMeaning::Green://green green
					BGRData[i] = 0;
					BGRData[i+1] = 0;
					BGRData[i+2] = 0;
					break;
			case ColorMeaning::obstacle:
					BGRData[i] = 255;
					BGRData[i+1] = 0;
					BGRData[i+2] = 255;
					blackObstacle[angle] = magnitude;
					//
			case ColorMeaning::other:
					break;

			default:
				break;
			}//switch loop
		}//magnitude for
	  }//angle for
		//Found signal Threshold.
		if(ballInfield->pixelCount>=2) ballInfield->hasFound = true;
		if(blueGoalInField->pixelCount >=5) blueGoalInField->hasFound = true;
		if(yellowGoadInField->pixelCount >=5) yellowGoadInField->hasFound = true;

		if(ballInfield->hasFound)
		ballInfield->findIt();

		if(blueGoalInField->hasFound)
		blueGoalInField->findIt();

		if(yellowGoadInField->hasFound)
		yellowGoadInField->findIt();

		// show the center circle
		for(angle=0;angle <360 ;angle++)
		{
			i = (int(polarToCartesian[angle][robotSelfRadial]/800))*800*3+(polarToCartesian[angle][robotSelfRadial]%800)*3;
			BGRData[i] = 255;
			BGRData[i+1] = 255;
			BGRData[i+2] =100;
		}	

		if(outputGreenFieldLineDrawing==true)
		return whiteDetection;//return whiteDetect PIC just for show whiteline detection result 
		else
		return BGRData;//return map show the result of the image processed
	
}
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------

	void setBlackMin(int value)
	{
		this->blackMin = value;
	}
//-----------------------------------------------------
	void setBallRange(int value)
	{
		this->ballRange = value;
	}
//-----------------------------------------------------
	void setYellowGoalRange(int value)
	{
		this->yellowGoalRange = value;
	}
//-----------------------------------------------------
	void setBlueGoalRange(int value){
		this->blueGoalRange = value;
	}
//-----------------------------------------------------
	void setWhiteMin(int value)
	{
		this->whiteMin = value;
	}
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//---------------------setup table initializer-------------
	//determine true or false(whether the BGR is Green... or not)
	void setUpFastHSVTable()
	{
		int b,g,r;
		for(b=0;b<=255;b++)
		for(g=0;g<=255;g++)
		for(r=0;r<=255;r++)	
		{
			//Green field color build up
			if( testPixel->BGR_HSV_H[b][g][r] <= greenEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= greenEdge.H_low && 
				testPixel->BGR_HSV_S[b][g][r] <= greenEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= greenEdge.S_low &&
				testPixel->BGR_HSV_V[b][g][r] <= greenEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= greenEdge.V_low)
			{
				testPixel->Fast_Green_BGR_HSV[b][g][r] = true;
			}
			else
			{
				testPixel->Fast_Green_BGR_HSV[b][g][r] = false;
			}

			//Yellow color build up
			if( testPixel->BGR_HSV_H[b][g][r] <= yellowEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= yellowEdge.H_low && 
				testPixel->BGR_HSV_S[b][g][r] <= yellowEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= yellowEdge.S_low &&
				testPixel->BGR_HSV_V[b][g][r] <= yellowEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= yellowEdge.V_low)
			{
				testPixel->Fast_Yellow_BGR_HSV[b][g][r] = true;
			}
			else
			{
				testPixel->Fast_Yellow_BGR_HSV[b][g][r] = false;
			}

			//Blue color build up
			if( testPixel->BGR_HSV_H[b][g][r] <= blueEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= blueEdge.H_low && 
				testPixel->BGR_HSV_S[b][g][r] <= blueEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= blueEdge.S_low &&
				testPixel->BGR_HSV_V[b][g][r] <= blueEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= blueEdge.V_low)
			{
				testPixel->Fast_Blue_BGR_HSV[b][g][r] = true;
			}
			else
			{
				testPixel->Fast_Blue_BGR_HSV[b][g][r] = false;
			}

			//ball color build up
			if(ballEdge.H_up >= ballEdge.H_low )
			{
				if( testPixel->BGR_HSV_H[b][g][r] <= ballEdge.H_up && testPixel->BGR_HSV_H[b][g][r] >= ballEdge.H_low && 
					testPixel->BGR_HSV_S[b][g][r] <= ballEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= ballEdge.S_low &&
					testPixel->BGR_HSV_V[b][g][r] <= ballEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= ballEdge.V_low)
				{
					testPixel->Fast_Ball_BGR_HSV[b][g][r] = true;
				}
				else
				{
					testPixel->Fast_Ball_BGR_HSV[b][g][r] = false;
				}
			}
			else //ballEdge.H_up < ballEdge.H_low
			{
			    if( (testPixel->BGR_HSV_H[b][g][r] <= ballEdge.H_up || testPixel->BGR_HSV_H[b][g][r] >= ballEdge.H_low) 
					&& 
					testPixel->BGR_HSV_S[b][g][r] <= ballEdge.S_up && testPixel->BGR_HSV_S[b][g][r] >= ballEdge.S_low 
					&&
					testPixel->BGR_HSV_V[b][g][r] <= ballEdge.V_up && testPixel->BGR_HSV_V[b][g][r] >= ballEdge.V_low)
				{
					testPixel->Fast_Ball_BGR_HSV[b][g][r] = true;
				}
				else
				{
					testPixel->Fast_Ball_BGR_HSV[b][g][r] = false;
				}
			}


		}
	}
//-----------------------------------------------------
	//determine whether the BGR is Green/.. or others
	void setUpVeryFastHSVTable()
	{
		int b,g,r;
		
		for(b=0; b<=255; b++)
		for(g=0; g<=255; g++)
		for(r=0; r<=255; r++)
		{
			if(testPixel->Fast_Ball_BGR_HSV[b][g][r] )
			{
				testPixel->VeryFast_BGR_HSV[b][g][r] = ColorMeaning::Ball;
			}
			else if(testPixel->Fast_Blue_BGR_HSV[b][g][r])
			{
				testPixel->VeryFast_BGR_HSV[b][g][r] = ColorMeaning::Blue;
			}
			else if(testPixel->Fast_Yellow_BGR_HSV[b][g][r])
			{
				testPixel->VeryFast_BGR_HSV[b][g][r] = ColorMeaning::Yellow;
			}
			else if(testPixel->Fast_Green_BGR_HSV[b][g][r])
			{
				testPixel->VeryFast_BGR_HSV[b][g][r] = ColorMeaning::Green;
			}
			else
			{
				testPixel->VeryFast_BGR_HSV[b][g][r] = 90;// not 0~5
			}

		}
	}
//-----------------------------------------------------
	//determine the Floor image pixel mapping to which pixel in the radiual image
	void setUpFloorToRadialTable()
	{
	 //FloorToRadiualTable
		int i;
		int radiusX,radiusY;
		int floorX,floorY;
		double floorPixelDistanceToCenter,radialPixelDistanceToCenter;
		double  angle;
		for(i=0;i<600*600*3;i=i+3)// Transform every pixel in floor view image to pixel in the radial image
		{
			floorX = i%(600*3)/3 - floorCenter.x;
			floorY = floorCenter.y - i/(600*3);
			angle = atan(floorY/(floorX+0.0001));//0.00001 avoid the X/0..-> X/0.0001
			if(floorY <= 0 && floorX<0) angle = angle+3.1415;
			else if(floorY >= 0 && floorX <0) angle = angle+3.1415;

			floorPixelDistanceToCenter = sqrt( double(floorX*floorX + floorY*floorY));
			radialPixelDistanceToCenter = realDistanceToRadialDistance(floorPixelDistanceToCenter);
			radiusY = radialPixelDistanceToCenter * sin(angle);
			radiusX = radialPixelDistanceToCenter * cos(angle);

			FloorToRadiualTable[i/3] = (radiualCenter.y - radiusY)*800*3 + 3*(radiusX + radiualCenter.x);//record 600*600 pixels start @ address of 800*800 image
		}
	}
//Need Finish!!!!!-------------------------------------------
	void setradialPixelToFloorDistanceTable()//need finish!!
	{
	  for(int i = 0;i<400;i++)
	  {
	    radialPixelToFloorDistanceTable[i] = RadialDistanceTorealDistance(i);
	  }
	}
//-----------------------------------------------------
	void setUpPolarToCartesian()
	{
		int angle=0;//(0-360)
		int distanceFromCenter=0;
		int tempX,tempY;
		for(angle = 0;angle<360;angle++)
		{
			polarToCartesian[angle] = new int [400];
			for(distanceFromCenter=0;distanceFromCenter<400;distanceFromCenter++)
			{
				if(angle!=0){tempX = radiualCenter.x + distanceFromCenter*cos(angle*3.1415926/180); tempY = radiualCenter.y - distanceFromCenter*sin(angle*3.1415926/180);}// mul first anf divide
				else { tempX =radiualCenter.x + distanceFromCenter ; tempY =radiualCenter.y;} 

				if(tempX<0) tempX = 0;
				if(tempX>=800) tempX =799;
				if(tempY<0) tempY = 0;
				if(tempY>=800) tempY = 799;

				polarToCartesian[angle][distanceFromCenter] = tempX + 800*tempY;
			}
		}

	}
//-----------------------------------------------------
	void setUpPixelToCMLUT()
	{
		if(FORPANDA)
		{
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
		else 
		{
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
		}

	}
//-----------------------------------------------------
	inline int realDistanceToRadialDistance(double x)//XXX(real distance) = radial magnitiude 
	{ // Piecewise function
		//this equation is determined by experiment  by jacky 2013
		// if (real distance>200cm ) return XXX;
		// else return XXX
		if(x<=220 && x>=40)
			return  (9.95246583481871e-06 *x*x*x -0.00759220913245679*x*x + 2.12040149393090*x +12.2912361116696 );
		else
			return  (3.90584683314728E-06 *x*x*x -0.004121974*x*x + 1.565461673*x +38.56611697 );
	}
//-----------------------------------------------------
	inline int RadialDistanceTorealDistance(double x)// XXX(radial magnitude) = real Distance 
	{
		//this equation is determined by experiment  by jacky 2013
		// Piecewise function
		// if (radial distance>216 pixel ) return XXX;
		// else return XXX
		if(x<=220 && x>=85)
			return (8.41848324248555e-05*x*x*x-0.0299654954976824*x*x +4.25678404199723*x -158.911243758519 );
		else
		return (9.73251074503088e-05*x*x*x -0.0349336835496916*x*x + 4.71691522637923*x - 166.829695436432 );
	}
//-----------------------------------------------------
	void updataGreenFieldBound(unsigned char *BGRData)//dont change the RGB content,Only updata the green field range in polar coordinate
	{
		int angle,magnitude;
		int transI;
		int x,y;
		int b,g,r;
		int greenCount=0;// if greenCount>4 continuous,that's the real green boundary OR that's the green noise
		for(angle = 0;angle <360;angle++)
		{
			for(magnitude=maxMagnitude;magnitude>robotSelfRadial;magnitude--)
			{
				greenFieldBound[angle] = robotSelfRadial;

				transI = polarToCartesian[angle][magnitude];
				x = transI %800;
				y = transI/800;
				b = BGRData[x*3+y*800*3];
				g = BGRData[x*3+y*800*3+1];
				r =  BGRData[x*3+y*800*3+2];
				
				if(testPixel->Fast_Green_BGR_HSV[b][g][r])
				{
					greenCount++;
					if(greenCount>=5)
					{
						greenFieldBound[angle] = magnitude+5;
						greenCount=0;
						break;
					}
				}
				else
					greenCount = 0;
			}
			if(magnitude ==robotSelfRadial) greenFieldBound[angle] = robotSelfRadial;
		}
	}
//-----------------------------------------------------
//-----------------------------------------------------
//variable controller
	void outputLine()
	{
		outputGreenFieldLineDrawing = true;
	}
//-----------------------------------------------------
	void outputImage()
	{
		outputGreenFieldLineDrawing = false;
	}
//-----------------------------------------------------
//-----------------------------------------------------
//image parameter control
	bool readData()//this function is copy from imageParameterForm by KevinWei.
	{
		FILE* databasefile = fopen("C:\\ImageCenter_Default_Parameter.bin","rb");

		if(databasefile==NULL)
		{
			return false;
		}

		//ball edge read 
		fread(&(this->ballEdge.H_low),sizeof(int),1,databasefile);
		fread(&(this->ballEdge.H_up),sizeof(int),1,databasefile);		
		fread(&(this->ballEdge.S_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->ballEdge.S_up),sizeof(unsigned char),1,databasefile);
		fread(&(this->ballEdge.V_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->ballEdge.V_up),sizeof(unsigned char),1,databasefile);
		//yellow edge read
		fread(&(this->yellowEdge.H_low),sizeof(int),1,databasefile);
		fread(&(this->yellowEdge.H_up),sizeof(int),1,databasefile);		
		fread(&(this->yellowEdge.S_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->yellowEdge.S_up),sizeof(unsigned char),1,databasefile);
		fread(&(this->yellowEdge.V_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->yellowEdge.V_up),sizeof(unsigned char),1,databasefile);
		//blue edge read
		fread(&(this->blueEdge.H_low),sizeof(int),1,databasefile);
		fread(&(this->blueEdge.H_up),sizeof(int),1,databasefile);		
		fread(&(this->blueEdge.S_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->blueEdge.S_up),sizeof(unsigned char),1,databasefile);
		fread(&(this->blueEdge.V_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->blueEdge.V_up),sizeof(unsigned char),1,databasefile);
		//green edge read
		fread(&(this->greenEdge.H_low),sizeof(int),1,databasefile);
		fread(&(this->greenEdge.H_up),sizeof(int),1,databasefile);		
		fread(&(this->greenEdge.S_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->greenEdge.S_up),sizeof(unsigned char),1,databasefile);
		fread(&(this->greenEdge.V_low),sizeof(unsigned char),1,databasefile);
		fread(&(this->greenEdge.V_up),sizeof(unsigned char),1,databasefile);

		//range read
		fread(&(this->blackMin),sizeof(int),1,databasefile);
		fread(&(this->whiteMin),sizeof(int),1,databasefile);
		fread(&(this->ballRange),sizeof(int),1,databasefile);
		fread(&(this->yellowGoalRange),sizeof(int),1,databasefile);
		fread(&(this->blueGoalRange),sizeof(int),1,databasefile);

		fclose(databasefile);
		return true;
	}
//-----------------------------------------------------
};

}