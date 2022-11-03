/*
  HsvEdge.h V1.0
  For HSV edge setup
  Jacky Guo 2014.4

  */

#pragma once
namespace ImageCenter
{
class HsvEdge
{
public:

	HsvEdge(void)
	{
	}

	~HsvEdge(void)
	{
	}

	int H_up;
	int H_low;
	unsigned char S_up;
	unsigned char S_low;
	unsigned char V_up;
	unsigned char V_low;

	void setEdge(int HUp,int HLow,unsigned char SUp,unsigned char SLow,unsigned char VUp,unsigned char VLow)
	{
		this->H_up = HUp;
		this->H_low = HLow;
		this->S_low =SLow;
		this->S_up = SUp;
		this->V_low = VLow;
		this->V_up = VUp;
	}

	void setHLow(int low)
	{
		if(low>=0 && low <=360)
		this->H_low = low ;
	}

	void setHUp(int up)
	{
		if(up >=0 && up <=360)
		this->H_up = up;
	}

	void setSLow(unsigned char low)
	{
		if(low >= 0 && low <= 255)
		this->S_low = low;
	}

	void setSUp(unsigned char up)
	{
		if(up >= 0 && up <= 255)
		this->S_up =up ;
	}

	void setVLow(unsigned char low)
	{
		if(low >= 0 && low <= 255)
		this->V_low = low;
	}

	void setVUp(unsigned char up)
	{
		if(up >= 0 && up <=255)
		this->V_up = up;
	}


};
}
