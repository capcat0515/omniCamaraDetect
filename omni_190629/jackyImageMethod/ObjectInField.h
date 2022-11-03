/*
  ObjectInField.h V1.0
  Features:
  1.findIt: Find the object location from meaningMap. it includes overlap detection.

  Jacky Guo 2014.4
  */
#pragma once
//objects detection algrithem

namespace ImageCenter{
	struct positionOfObjectFeature//which edge in which angle distance..
	{
		double theatBetweenZero;//Zero degree is along x-axis
		double radialDistance;//the feature's radial distance from robot center
		double realWorldDistance;//the feature's real world distance from robot center
		//double virtualPosition;//
	};

class ObjectInField//object information detected by omni camera
{
public:

	ObjectInField(int * radialToFloorMapping)
	{
		pixelCount= 0;
		hasFound = false;
		phase = 0;
		magnitude = 0;
		leftEdge.radialDistance= 0;
		rightEdge.radialDistance= 0;
		objectCenter.radialDistance = 0;
		anglePixelAccumulation = new int [360];
		anglePixelMinRadial = new int [360];// store the every angle 's min distance (pixel) from center
		radialDistanceMappingToFloorDistance = radialToFloorMapping;
		overlap = false;
		initialArray();
	}

	ObjectInField(){
		hasFound = false;
		phase = 0;
		magnitude = 360;
		leftEdge.radialDistance= 0;
		rightEdge.radialDistance= 0;
		objectCenter.radialDistance = 0;
	//	objectCenter.virtualPosition = 0;
		bool initialLastAngle = true;
		leftEdge.theatBetweenZero = 0;
		rightEdge.theatBetweenZero = 360;
		pixelCount = 0;
	}


	~ObjectInField(void)
	{
	}

	positionOfObjectFeature leftEdge,rightEdge,objectCenter;
	bool hasFound;
	int centerX,centerY;
	int phase,magnitude;
	int* radialDistanceMappingToFloorDistance;//center to center
	int lastAngle;//for reduce the suddenly noise
	bool initialLastAngle ; 
	int *anglePixelAccumulation;// angle accumulation
	int *anglePixelMinRadial;// @ XX angle. the minimua distance from center
	bool overlap;
	int pixelCount;
	void gatherObjectInforamtion(int angle,int magnitude)// fill the information then give the data to next procedure
	{
		anglePixelAccumulation[angle]++;
		if(anglePixelMinRadial[angle]>=magnitude)
			anglePixelMinRadial[angle] = magnitude;
	}

	void initialArray()
	{
	 for(int i=0;i<360;i++)
		 anglePixelMinRadial[i] = 400;
	 for(int i = 0;i<360;i++)
		 anglePixelAccumulation[i] = 0;
	}

	void findIt()
	{
	 int average = 0;
	 int breakPixel=0;
	 int continuousPixel = 0;
	 int leftside=0,rightside=0;
	 int maxLeftside=0,maxRightside=0;
	 int maxContinuous=0;
	 int minMagnitude = 400;
	 int occupiedAngle = 0;

	 for(int angle = 0;angle<360;angle++)
	 {
			 if(anglePixelAccumulation[angle]<=2 || angle== 359 ) // met the break point .if the pixel is less than 2 .
			 {
				 if((breakPixel>3 || angle== 359 ) && continuousPixel!=0 ) //continuousPixel!=0  break point til now ...
				 {
					 if(maxContinuous <(rightside-leftside+1) )//decision 
					 {
						
						 continuousPixel=0;
						 maxContinuous = rightside-leftside +1;
						 
						 maxLeftside = leftside;
						 maxRightside = rightside;
						 leftside = 0;
						 rightside = 0;
					 	 continue;
					 }
					 continuousPixel=0;
				 }
				 breakPixel++; 
			 }

			 else// if (anglePixelAccumulation[angle]>2)  no breakpoint
			 { 
				 if( continuousPixel==0)//start point
				 {
					 leftside = angle;
					 rightside = angle;
					 breakPixel = 0;
				     continuousPixel++;
				 }
				 else 
				 {
					 rightside = angle;
					 breakPixel = 0;
				     continuousPixel++;
				 }

				 

			 }//else //if(anglePixelAccumulation[angle]!=0)
	 }//for
	 
	 for(int i = maxLeftside;i<=maxRightside;i++)
	 {
		 if(anglePixelMinRadial[i]<minMagnitude)
			 minMagnitude = anglePixelMinRadial[i];
	 }

	 //------Above :the max in the scan without overlapping detection.
	 //------Below :the max int the scan with overlapping detection. 
	 bool overlapping = anglePixelAccumulation[1]!=0 && anglePixelAccumulation[359]!=0 ;//check the exist overlapping object  
	// bool overlap = overlapping;
	// debug this->overlap = overlapping;
	 bool overlappingWin = false;// for the compare with the non-overlapping object ... detetmine which one is more wide
	 if(overlapping)
	 {//determine the overlapping object...
		 int lpLeftSideAngle = 0 ,lpRightSideAngle;//left part -> lp
		 int rpLeftSideAngle,rpRightSideAngle = 359;
		 // find the left part...
		 for(int angle = 0;angle<360;angle++)
		 {
				 if(anglePixelAccumulation[angle]<=2) // met the break point 
				 { 
					 breakPixel++; 
					 if(breakPixel>3 && continuousPixel!=0) //continuousPixel!=0  break point til now ...
					 {
						breakPixel = 0;
						continuousPixel = 0;
						break;
					 }
					 
				 }
				 else// if (anglePixelAccumulation[angle]!=0)  no breakpoint
				 { 
					//the start point is 0;
					 lpRightSideAngle = angle;
					 continuousPixel++;
				 }//else //if(anglePixelAccumulation[angle]!=0)
		 }//for
		 //=------------------------------------------------------------------
		//find the right part
		 continuousPixel = 0;
		  for(int angle = 359;angle>0;angle--)
		 {
				 if(anglePixelAccumulation[angle]<=2) // met the break point 
				 { 
					 breakPixel++; 
					 if(breakPixel>3 && continuousPixel!=0) //continuousPixel!=0  break point til now ...
					 {
						 breakPixel = 0;
						 continuousPixel = 0;
						 break;
					 }
					 
				 }
				 else// if (anglePixelAccumulation[angle]!=0)  no breakpoint
				 { 
					//the start point is 0;
					 rpLeftSideAngle = angle;
					 continuousPixel++;

				 }//else //if(anglePixelAccumulation[angle]!=0)
		 }//for
		  //=------------------------------------------------------------------
		  //calculate the overlapping angle 
		  int overlappingAngleAccumlation = 360-rpLeftSideAngle + lpRightSideAngle;
		 
		  //=------------------------------------------------------------------
		  if (overlappingAngleAccumlation >= (maxRightside-maxLeftside))
		  {
			  overlappingWin = true;
			  overlap = true; //make sure the overlapping object is the most wide .
			  maxLeftside = rpLeftSideAngle;
			  maxRightside = lpRightSideAngle;

			  for(int i = 359;i >=maxLeftside;i--)
			 {
				 if(anglePixelMinRadial[i]<minMagnitude)
					 minMagnitude = anglePixelMinRadial[i];
			 }

			  for(int i = 0 ;i<=maxRightside ;i++)
			 {
				 if(anglePixelMinRadial[i]<minMagnitude)
					 minMagnitude = anglePixelMinRadial[i];
			 }
		  }
		  

	 }//if(overlapping)

	 if(overlap) //compare with the non-overlapping object
	 {
		 int phase = (maxLeftside-359 + maxRightside)/2;
		 if (phase<0)	this->phase = phase+360;
		 else			this->phase = phase;
	 }
	 else this->phase = maxLeftside/2 + maxRightside/2;

	 

	 this->leftEdge.theatBetweenZero = maxLeftside;
	 this->rightEdge.theatBetweenZero = maxRightside;
	// int test_magnitude = radialDistanceMappingToFloorDistance[minMagnitude];
	 
	 if( minMagnitude>=399) this->magnitude = 0;
	 else this->magnitude = minMagnitude;
		 //this->magnitude = anglePixelMinRadial[phase];

	}//find it




	void reSet(){
		hasFound = false;
		overlap = false;
		initialLastAngle = true;
		phase = 0;
		magnitude = 10000;
		leftEdge.theatBetweenZero = 0;
		rightEdge.theatBetweenZero = 360;
		initialArray();
	}

};

}