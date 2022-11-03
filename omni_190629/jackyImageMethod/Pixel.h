/*
  Pixel.h V1.0
  Features:
  1.enum ColorMeaning: Standard enum for color meanning
  2.set_BGR_HSV() : RGB converts to HSV
  3.setLUTOfRGB2HSV(): Same to above
  for test the global branch.
  Jacky Guo 2014.4
  jaccck

  */

#pragma once
//look up table and RGB->HSV algrithem
namespace ImageCenter{

enum ColorMeaning
	{
		Ball,
		White,/*dont delete"white" or will change the original 0-5*/
		Blue,
		Yellow,
		Green,
		other,
		obstacle
	};

class Pixel
{
public:

	unsigned char B,G,R;
	int H;
	unsigned char S,V;
	double DB_H,DB_S,DB_V;

	int BGR_HSV_H[256][256][256];
	unsigned char BGR_HSV_S[256][256][256];
	unsigned char BGR_HSV_V[256][256][256];
	ColorMeaning colorMean;
	//bool Fast_White_BGR_HSV[256][256][256];
	bool Fast_Yellow_BGR_HSV[256][256][256];
	bool Fast_Blue_BGR_HSV[256][256][256];
	bool Fast_Green_BGR_HSV[256][256][256];
	bool Fast_Ball_BGR_HSV[256][256][256];
	unsigned char VeryFast_BGR_HSV[256][256][256];


	Pixel(void)
	{
		this->B = 0;
		this->G = 0;
		this->R = 0;
		this->H = 0;
		this->S = 0;
		this->V = 0;
		setLUTOfRGB2HSV();
		
	}

	Pixel(unsigned char in_B,unsigned char in_G,unsigned char in_R){
	this->B = in_B;
	this->G = in_G;
	this->R = in_R;
	}

	~Pixel(void)
	{
	}

	void set_BGR_HSV(unsigned char B,unsigned char G,unsigned char R)
	{
		double min, max, delta;
 
			if(R<B) min=R; else min=B;
			if(G<min) min=G;
 
			if(R>G) max=R; else max=G;
			if(B>max) max=B;

			DB_V= max;							    	 // v: 0~255
			delta = max - min;                      // 0..255, < v

			if(delta==0)
				DB_H=0;
			else if(max==R && G>=B)
				DB_H=60*(G-B)/delta;
			else if(max==R && G<B)
				DB_H=60*(G-B)/delta+360;
			else if(max==G)
				DB_H=60*(B-R)/delta+120;
			else if(max==B)
				DB_H=60*(R-G)/delta+240;

			if(max==0) DB_S=0;
			else DB_S = 255*(1-min/max);

			DB_V = max;
			if(DB_H>359) DB_H = 359;
			else if(DB_H<0) DB_H = 0;
			if(DB_S>255) DB_H = 255;
			else if(DB_S<0) DB_H = 0;
			if(DB_V>255) DB_H = 255;
			else if(DB_V<0) DB_H = 0;


			this->H = DB_H;
			this->S = DB_S;
			this->V = DB_V;

	}

	void setLUTOfRGB2HSV()
	{
		int b,g,r;
		for(b=0;b<=255;b++)
		for(g=0;g<=255;g++)
		for(r=0;r<=255;r++)
		{
		 set_BGR_HSV(b,g,r);
		 BGR_HSV_H[b][g][r] = H;
		 BGR_HSV_S[b][g][r] = S;
		 BGR_HSV_V[b][g][r] = V;
		}
	}
};
}