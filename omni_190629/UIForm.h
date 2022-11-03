#pragma once
#pragma comment(lib, "Ws2_32.lib")
#include <WinSock2.h>
#include<cmath>
#include<iostream>
#include <Windows.h>
#include "jackyImageMethod\ImageMethod.h"
#include "jackyImageMethod\Localization.h"
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\core.hpp>
#include <opencv\highgui.h>
#include <opencv2\highgui\highgui.hpp>
#include "road\road0813.h"

#include <vector>
#include <queue>
#include <tchar.h>
#include <cstring>
#include "road\json.hpp"

namespace omni190629 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace std;
	using namespace System::Net;
	using namespace System::Net::Sockets;
	using namespace System::Text;
	using namespace System::Threading;
	
	using namespace ImageCenter;
	using namespace cv;
	using namespace road0813;
	

	using json =nlohmann::json;
	/// <summary>
	/// UIForm 的摘要
	/// </summary>
	public ref class UIForm : public System::Windows::Forms::Form
	{
		
	public:

		UIForm(void)
		{
			
			InitializeComponent();
			JackyConstructor();
			//ConnectSocketThread();
			ConnectMobileAndCar_CLR();
			//ConnectMobileAndCar();
			cout << "123345\n";
		}
		unsigned char * ShowPtr;//for show,it has its instance of 800*800
		unsigned char * WorkPtrtemp;
		unsigned char * WorkPtr;//process image buffer,only to hold the address of the process image
		unsigned char *Omni2DPtr;
		unsigned char *Omni3DPtr;
		double h, b, c, a, zoom, f;
		double point_len, cosTheta, sinTheta, cosAlpha, sinAlpha;
		double *NP_3D, *M_3D, *temp2D, NP_Distance;
		int px, py;
		int *address1, *address2, *address_temp;
		int delayCountMaximum, delayCount, th1;
		ImageMethod * imageMethod;
		threadSignal * threadSyncSignal;
		bool boolImageProcessHaltRequest;
	
		Mat *redimg, *element, *blueImg, *detectImg, *data_pts_copy, *sendphoneIMG;

		SocketType sockType;
		ProtocolType sockProtocol;
		System::Net::IPAddress^ localAddress;
		IPEndPoint^ localEndPoint, ^ senderAddress;
		Socket^ serverSocket, ^ clientSocket;
		EndPoint^ castSenderAddress;
		cli::array<Byte>^ sendBuffer;
		bool boool; // look socket cennect
		System::String^ ssss;
		bool camaraOpen = false; int camaraOpenN = 0, camaraNotOpenN = 0;
		json *j;
		///*cx -> 每層頂點x, *Cy -> 每層頂點y, *Tx -> 輸出點x, *Ty -> 輸出點y
		std::vector<double> *Cx, *Cy, *Tx, *Ty, *V, *framRangeX, *framRangeY; ///
		int *framX, *framY;
		int ssC;
		int countframe;
		//void formatBuffer()
		int formatBufferIndex;
		int *nowPosition, nowPoint, nowangle;
		int *nextPosition;
		//cv::Point *cntr;
		int *carG;
		double Carsangle;
		double carDisSum, carDisVarianceSum, carAngleSum, carAngleVarianceSum, realdistance, realangle;
		int framepage;
		queue<string> *queMessage;
		int portNumber;
		int clientNo;
		Thread^ phoneC;
		
		BackgroundWorker bgwServer;
		TcpClient socketClient;
		
	private: System::Windows::Forms::CheckBox^  checkBox1;
	private: System::Windows::Forms::Button^  openIMG;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::PictureBox^  pictureBox1;

	public: void JackyConstructor() {
		cout << "456789\n";
		imageMethod = new ImageMethod();
		imageMethod->CamConnect();
		imageMethod->ConnectAndCapture(imageMethod->guid);
		WorkPtrtemp = new unsigned char[800 * 800 * 3];
		ShowPtr = new unsigned char[800 * 800 * 3];
		Omni2DPtr = new unsigned char[800 * 800 * 3];
		Omni3DPtr = new unsigned char[800 * 800 * 3];
		address1 = new int[2];
		address_temp = new int[2];
		NP_3D = new double[3];
		M_3D = new double[3];
		address2 = new int[2];
		temp2D = new double[2];
		//h = 350, b = 68.5, c = 92, f = 9.5, zoom = 5; //mm 
		h = 1200, b = 68.5, c = 92, f = 9.5, zoom = 5; //mm h=360 min
		a = sqrt(c*c - b * b);
		boolImageProcessHaltRequest = false;
		th1 = 220;//300;
		redimg = new Mat(800, 800, CV_8UC1);
		detectImg = new Mat(800, 800, CV_8UC1);
		blueImg= new Mat(800, 800, CV_8UC1);
		element =new Mat(getStructuringElement(MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1)));
		sendphoneIMG=new Mat(800, 800, CV_8UC3);
		//Cx[10]; Cy[10]; Tx[10]; Ty[10]; V[10];
		ssC = 0;
		countframe=0;
		//framRangeX; framRangeY;
		nowPosition= new int[2];
		nowPosition[0] = 200; nowPosition[1] = 600; //x, y
		nowPoint = 0;
		//cntr = (0,0);
		carG = new int[2];
		nowangle = 90;
		nextPosition = new int[2];
		nextPosition[0] = 200; nextPosition[1] = 400;//x, y
		carDisSum = 0, carDisVarianceSum = 0, carAngleSum = 0, carAngleVarianceSum = 0, framepage = 0;
		
	}
	void FormatBuffer(cli::array<Byte>^ dataBuffer, System::String^ message)
	{
		cli::array<Byte>^ byteMessage = System::Text::Encoding::ASCII->GetBytes(message);
		formatBufferIndex = 0; 
		for (int j = 0; j < byteMessage->Length; j++)
		{
			dataBuffer[formatBufferIndex] = byteMessage[j];
			Console::Write(Convert::ToChar(dataBuffer[formatBufferIndex]));
			formatBufferIndex++;
			if (formatBufferIndex >= dataBuffer->Length){break;}
		}
	}
	void ConnectSocketThread() {
		//phoneC = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(this, phoneListeners));
		//phoneC->Start();
		//System::Threading::Thread carC = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(this, carListeners));
		//carC->Start();
	}
	void phoneListeners() {
		sockType = SocketType::Stream;
		sockProtocol = ProtocolType::Tcp;
		int localPort = 9000;
		localAddress = System::Net::IPAddress::Any;
		localAddress = System::Net::IPAddress::Parse("192.168.137.1");
		localEndPoint = gcnew IPEndPoint(localAddress, localPort);
		senderAddress = gcnew IPEndPoint(localAddress, 0);
		bool udpConnect = false;
		serverSocket = nullptr;
		serverSocket = gcnew Socket(localAddress->AddressFamily, sockType, sockProtocol);
		serverSocket->Bind(localEndPoint);
		if (sockProtocol == ProtocolType::Tcp) {
			serverSocket->Listen(5);
		}
		while (true) {
			if (sockProtocol == ProtocolType::Tcp && boool) {
				clientSocket = serverSocket->Accept();
				Console::WriteLine("Server: Accept() is OK...");
				Console::WriteLine("Server: Accepted connection from: {0}", clientSocket->RemoteEndPoint->ToString());
				boool = false;
				break;
			}
		}
	}

	void carListeners() {
		sockType = SocketType::Stream;
		sockProtocol = ProtocolType::Tcp;
		int localPort = 8040;
		localAddress = System::Net::IPAddress::Any;
		localAddress = System::Net::IPAddress::Parse("192.168.137.1");
		localEndPoint = gcnew IPEndPoint(localAddress, localPort);
		senderAddress = gcnew IPEndPoint(localAddress, 0);
		bool udpConnect = false;
		serverSocket = nullptr;
		serverSocket = gcnew Socket(localAddress->AddressFamily, sockType, sockProtocol);
		serverSocket->Bind(localEndPoint);

		if (sockProtocol == ProtocolType::Tcp) {
			serverSocket->Listen(5);
		}
		boool = true;
		sendBuffer = gcnew cli::array<Byte>(3); //------------
		cli::array<Byte>^ receiveBuffer = gcnew cli::array<Byte>(20);
		while (true) {
			receiveBuffer->Clear;
			sendBuffer->Clear;
			if (sockProtocol == ProtocolType::Tcp && boool) {
				clientSocket = serverSocket->Accept();
				Console::WriteLine("Server: Accept() is OK...");
				Console::WriteLine("Server: Accepted connection from: {0}", clientSocket->RemoteEndPoint->ToString());
				boool = false;
				break;
			}
			int timess = 20;
			System::String^ sff = "";
			Char scc;
			int de;
			Byte byteNum;
			while (timess > 0) {
				sendBuffer->Clear;
				sff = "";

				//ssss = Console::ReadLine();
				de = 1;
				byteNum = 0;
				ssss = "L1";
				for (int L = ssss->Length - 1; L >= 1; L--) {
					byteNum += ((Byte)ssss[L] - '0')*de;
					de *= 10;
				}
				//Console::WriteLine(ssss->Length);
				sff += ssss[0];
				sff += ((Char)(byteNum)).ToString();
				//Console::WriteLine("SSF "+ sff+ (Byte)byteNum);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);

				sendBuffer->Clear;
				sff = "";
				Sleep(1000);
				//ssss = Console::ReadLine();
				de = 1;
				byteNum = 0;
				ssss = "L1";
				for (int L = ssss->Length - 1; L >= 1; L--) {
					byteNum += ((Byte)ssss[L] - '0')*de;
					de *= 10;
				}
				//Console::WriteLine(ssss->Length);
				sff += ssss[0];
				sff += ((Char)(byteNum)).ToString();
				//Console::WriteLine("SSF "+ sff+ (Byte)byteNum);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
				timess--;
			}
		}
	}


	void ConnectMobileAndCar_CLR() {
		
		//cout << s1 << endl;
		sockType = SocketType::Stream;
		sockProtocol = ProtocolType::Tcp;
		int localPort = 8040;
		localAddress = System::Net::IPAddress::Any;
		localAddress = System::Net::IPAddress::Parse("192.168.137.1");
		localEndPoint = gcnew IPEndPoint(localAddress, localPort);
		senderAddress = gcnew IPEndPoint(localAddress, 0);

		bool udpConnect = false;
		serverSocket = nullptr;
		serverSocket = gcnew Socket(localAddress->AddressFamily, sockType, sockProtocol);
		serverSocket->Bind(localEndPoint);
	
		if (sockProtocol == ProtocolType::Tcp) {
			serverSocket->Listen(5);
		}
		boool = true;
		sendBuffer = gcnew cli::array<Byte>(20); //------------
		cli::array<Byte>^ receiveBuffer = gcnew cli::array<Byte>(5);
		
		while (true) {
			receiveBuffer->Clear;
			sendBuffer->Clear;
			if (sockProtocol == ProtocolType::Tcp && boool) {
				clientSocket = serverSocket->Accept();
				Console::WriteLine("Server: Accept() is OK...");
				Console::WriteLine("Server: Accepted connection from: {0}", clientSocket->RemoteEndPoint->ToString());
				//ssss = "ConnectOK";
				ssss = "";
				while (true) {
					ssss = Console::ReadLine();
					FormatBuffer(sendBuffer, ssss);
					clientSocket->Send(sendBuffer);
					//clientSocket->Receive(receiveBuffer);
					//for (int ii = 0; ii < receiveBuffer->Length; ii++) Console::Write(Convert::ToChar(receiveBuffer[ii]));
					//for(int ii=0; ii< sendBuffer->Length; ii++) Console::WriteLine(sendBuffer[ii]);
					//Console::Write("\n"+ receiveBuffer->Length.ToString()+"\n");
					receiveBuffer->Clear;
					sendBuffer->Clear;
				}
				boool = false;
				break;
			}
			int timess = 20;
			System::String^ sff = "";
			Char scc;
			int de;
			Byte byteNum;
			while (timess>0) {
				sendBuffer->Clear;
				sff = "";
				
				//ssss = Console::ReadLine();
				de = 1;
				byteNum = 0;
				ssss = "L1";
				for (int L = ssss->Length - 1; L >= 1; L--) {
					byteNum += ((Byte)ssss[L] - '0')*de;
					de *= 10;
				}
				//Console::WriteLine(ssss->Length);
				sff += ssss[0];
				sff += ((Char)(byteNum)).ToString();
				//Console::WriteLine("SSF "+ sff+ (Byte)byteNum);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);

				sendBuffer->Clear;
				sff = "";
				Sleep(1000);
				//ssss = Console::ReadLine();
				de = 1;
				byteNum = 0;
				ssss = "L1";
				for (int L = ssss->Length - 1; L >= 1; L--) {
					byteNum += ((Byte)ssss[L] - '0')*de;
					de *= 10;
				}
				//Console::WriteLine(ssss->Length);
				sff += ssss[0];
				sff += ((Char)(byteNum)).ToString();
				//Console::WriteLine("SSF "+ sff+ (Byte)byteNum);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
				timess--;
			}
		}
	}
	///socket setting 
	void ConnectMobileAndCar() 
	{
		char message[200];
		int r;
		WSAData wsaData;
		WORD DLLVSERION;
		//DLLVSERION = MAKEWORD(2, 2);//Winsocket-DLL 版本

		//用 WSAStartup 開始 Winsocket-DLL
		r = WSAStartup(MAKEWORD(2, 2), &wsaData);

		//宣告 socket 位址資訊(不同的通訊,有不同的位址資訊,所以會有不同的資料結構存放這些位址資訊)
		SOCKADDR_IN addr;
		int addrlen = sizeof(addr);

		//建立 socket
		SOCKET sListen; //listening for an incoming connection
		SOCKET sConnect; //operating if a connection was found

		//AF_INET：表示建立的 socket 屬於 internet family
		//SOCK_STREAM：表示建立的 socket 是 connection-oriented socket 
		//sConnect = socket(AF_INET, SOCK_STREAM, NULL);

		//設定位址資訊的資料
		addr.sin_addr.s_addr = inet_addr("192.168.137.1");//("127.0.0.1");
		addr.sin_family = AF_INET;
		addr.sin_port = htons(8040);

		//設定 Listen
		sListen = socket(AF_INET, SOCK_STREAM, NULL);
		bind(sListen, (SOCKADDR*)&addr, sizeof(addr));
		listen(sListen, SOMAXCONN);//SOMAXCONN: listening without any limit

		int socketIsConnet = 0;
		sConnect = accept(sListen, 0, 0);
		cout << sListen << endl;
		//等待連線
		SOCKADDR_IN clinetAddr;
		while (sConnect = accept(sListen, 0, 0))
		{
			if (sConnect == INVALID_SOCKET)
			{
				printf("invalid client socket", GetLastError());
				continue;
			}
			char sendbuf[3] = "F1";
			//string sendbuf1;// = "on";//"sending data test";
			//cout << "input on or fo:";
			//cin >> sendbuf;
			send(sConnect, sendbuf, (int)strlen(sendbuf), 0);
			cout << sendbuf << endl;
			//_beginthreadex(0, 0, ServClient, (void*)&client, 0, 0);

		}
		/*
		while (true)
		{
			cout << "waiting..." << endl;
			socketIsConnet = 0;

			
			while (!socketIsConnet) {
				//sConnect = accept(sListen, (SOCKADDR*)&clinetAddr, &addrlen);
				sConnect = accept(sListen, 0, 0);
				cout << sConnect << "  " << INVALID_SOCKET << endl;
				if (sConnect != INVALID_SOCKET) break;
			}
			if (sConnect != INVALID_SOCKET)
			{
				cout<<"44444\n";
				if (socketIsConnet == 0) {
					cout << "a connection was found" << endl;
					printf("server: got connection from %s\n", inet_ntoa(addr.sin_addr));
					socketIsConnet = 1;
					cout << "55555\n";
				}
			}
			
			if (socketIsConnet) {
				//傳送訊息給 client 端
				char sendbuf[3]="F1";
				//string sendbuf1;// = "on";//"sending data test";
				//cout << "input on or fo:";
				//cin >> sendbuf;
				send(sConnect, sendbuf, (int)strlen(sendbuf), 0);
				cout << sendbuf << endl;
			}
			
			if (socketIsConnet == true)
			{
				//while (message != "close")
				//{
				ZeroMemory(message, 200);
				r = recv(sConnect, message, sizeof(message), 0);
				cout << message << endl;
				//}
			}
			
		}*/
	}
	protected:
		~UIForm()
		{
			if (components)
			{
				delete components;
			}
		}
	protected:

	private:
		System::Windows::Forms::Button^  button1;
		System::ComponentModel::BackgroundWorker^  BW_imageProcessThread;
		System::ComponentModel::BackgroundWorker^  BW_socketThread;
		System::ComponentModel::Container ^components;
		#pragma region Windows Form Designer generated code

		void InitializeComponent(void)
		{
			this->button1 = (gcnew System::Windows::Forms::Button());

			this->BW_imageProcessThread = (gcnew System::ComponentModel::BackgroundWorker());
			this->BW_socketThread = (gcnew System::ComponentModel::BackgroundWorker());

			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->openIMG = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->button3 = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(12, 36);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(75, 23);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Go";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &UIForm::button1_Click);
			// 
			// BW_imageProcessThread 
			// 
			this->BW_imageProcessThread->WorkerReportsProgress = true;
			this->BW_imageProcessThread->WorkerSupportsCancellation = true;
			this->BW_imageProcessThread->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &UIForm::BW_imageProcessThread_DoWork);
			this->BW_imageProcessThread->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &UIForm::BW_imageProcessThread_ProgressChanged);
			this->BW_imageProcessThread->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &UIForm::BW_imageProcessThread_RunWorkerCompleted);
			// 
			// pictureBox1
			// 
			this->pictureBox1->Location = System::Drawing::Point(130, 12);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(800, 800);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox1->TabIndex = 1;
			this->pictureBox1->TabStop = false;
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Location = System::Drawing::Point(12, 80);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(77, 16);
			this->checkBox1->TabIndex = 2;
			this->checkBox1->Text = L"checkBox1";
			this->checkBox1->UseVisualStyleBackColor = true;
			// 
			// openIMG
			// 
			this->openIMG->Location = System::Drawing::Point(12, 185);
			this->openIMG->Name = L"openIMG";
			this->openIMG->Size = System::Drawing::Size(75, 23);
			this->openIMG->TabIndex = 3;
			this->openIMG->Text = L"openIMG";
			this->openIMG->UseVisualStyleBackColor = true;
			this->openIMG->Click += gcnew System::EventHandler(this, &UIForm::openIMG_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(12, 246);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(75, 23);
			this->button2->TabIndex = 4;
			this->button2->Text = L"car";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &UIForm::button2_Click);
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(12, 304);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(75, 23);
			this->button3->TabIndex = 5;
			this->button3->Text = L"road";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &UIForm::button3_Click);
			// 
			// UIForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(940, 822);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->openIMG);
			this->Controls->Add(this->checkBox1);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->button1);
			this->Name = L"UIForm";
			this->Text = L"UIForm";
			this->Load += gcnew System::EventHandler(this, &UIForm::UIForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
	#pragma endregion
	private: System::Void UIForm_Load(System::Object^  sender, System::EventArgs^  e) {
	}

	//open camara 
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
		//ConnectMobileAndCar();
		try
		{
			if (this->BW_imageProcessThread->IsBusy == false)
			{
				this->BW_imageProcessThread->RunWorkerAsync();
				this->button1->Text = "STOP";
			}

			else
			{
				this->BW_imageProcessThread->CancelAsync();
				this->button1->Text = "Go";
			}
		}
		catch (System::Exception^ e)
		{
			MessageBox::Show(e->ToString(), "From StartButton!!!", System::Windows::Forms::MessageBoxButtons::OK, System::Windows::Forms::MessageBoxIcon::Error);
		}
	}
	private: System::Void BW_imageProcessThread_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		try
		{
			int count_offset;
			///read onmicamara image 
			while (true)
			{
				///--------------------
				///get onmicamara image
				//for escape from worker
				if (BW_imageProcessThread->CancellationPending == true) {e->Cancel = true; break;}
				LARGE_INTEGER t1, t2, ts; //tick timer
				QueryPerformanceFrequency(&ts); //tick timer
				QueryPerformanceCounter(&t1); //tick timer
				bool ReadyToGetRealObjectData = false;
				
				if (this->checkBox1->Checked)
				{   // In the futher, checkBox1 will be delited, because it will be auto 
					try {
						imageMethod->outputImage();
						unsigned char* BGRdata = imageMethod->getRawImage();
						WorkPtr = BGRdata;
						WorkPtr = imageMethod->thoughVeryFastHSV(BGRdata); //get image
						camaraOpen = true;
					}
					catch(System::Exception^ e){}
				}
				
				if (camaraOpen == true && camaraOpenN == 0) {
					ssss = "CamaraStatus";
					//FormatBuffer(sendBuffer, ssss);
					//clientSocket->Send(sendBuffer);
					ssss = "ON";
					//FormatBuffer(sendBuffer, ssss);
					//clientSocket->Send(sendBuffer);
					camaraOpenN = 1;
				}
				else if (camaraOpen == false && camaraNotOpenN == 0) {
					ssss = "CamaraStatus";
					//FormatBuffer(sendBuffer, ssss);
					//clientSocket->Send(sendBuffer);
					ssss = "OFF";
					//FormatBuffer(sendBuffer, ssss);
					//clientSocket->Send(sendBuffer);
					camaraNotOpenN = 1;
					continue;
				}
				else if (camaraOpen == false) {
					continue;
				}
				
				QueryPerformanceCounter(&t2);//tick timer
				#pragma region to copy the data and send them to next stage.
				int workCX = 0, workCY = 0, WCRow = 0, WCCol = 0, WCCount=0;
				// WorkPtr can not show image, so put the image in Omni3DPtr

				int jj = 0, omni = 800 * 800 * 3, omniOffset = 800 * 13 * 3;
				for (; jj < omniOffset; jj++) {
					WorkPtrtemp[jj] = (byte)255;
				}
				for (; jj < omni- omniOffset; jj++) {
					WorkPtrtemp[jj] = WorkPtr[jj- omniOffset];
				}
				for (; jj < omni; jj++) {
					WorkPtrtemp[jj] = (byte)255;
				}
				for (int i = 0; i < omni; i++) {
					if (WorkPtr[i] < 30 && WorkPtr[i + 1] < 30 && WorkPtr[i + 2] < 30 && i % 3 == 0) {
						if (WCCol == 800) { WCCol = 0; WCRow++;}
						workCX += WCCol;
						workCY += WCRow;
						WCCol++;
						WCCount++;
					}
					Omni2DPtr[i] = WorkPtrtemp[i];
					Omni3DPtr[i] = (byte)255;
				}
			
				#pragma endregion
				///Image center regulate
				///Distorted image transfor to original image
				//workCX /= WCCount;
				//workCY /= WCCount;
				//cout << workCX << " " << workCY << endl;
				
				for (int i = 0; i < 800 * 800; i++) {
					oneD_To_twoD(i);
					pixel_to_mm_3D();
					P3D_to_P2D();
					mm_to_pixel_2D();
					makeImg();
				}
				
				if (camaraOpen == true && camaraOpenN == 1) {
					int ttemp=0;
					for (int i = 0; i < 800; i++) {
						for (int j = 0; j < 800; j++) {
							sendphoneIMG->at<Vec3b>(800 - 1 - i, j)[0] = Omni3DPtr[ttemp];
							sendphoneIMG->at<Vec3b>(800 - 1 - i, j)[1] = Omni3DPtr[ttemp+1];
							sendphoneIMG->at<Vec3b>(800 - 1 - i, j)[2] = Omni3DPtr[ttemp+2];
							ttemp+=3;
							detectImg->at<uchar>(i, j) = 0;
							blueImg->at<uchar>(i, j) = 0;
							redimg->at<uchar>(i, j) = 0;
						}
					}
				}
				///find the car location and car direction
				carLocation();
				carCenter();
				//correctTheCar();
				this->BW_imageProcessThread->ReportProgress(1 / ((t2.QuadPart - t1.QuadPart) / (double)(ts.QuadPart)), NULL);
				//delay a time , prevent from reading the protect memory... reduce the rate
				//_sleep(3);
			}//while
		}//try
		catch (System::Exception^ e)
		{
			MessageBox::Show(e->ToString(), "From ImageProcessThread_DoWorkEvent!!!", System::Windows::Forms::MessageBoxButtons::OK, System::Windows::Forms::MessageBoxIcon::Error);
		}
	}

	private: System::Void BW_imageProcessThread_ProgressChanged(System::Object^  sender, System::ComponentModel::ProgressChangedEventArgs^  e) {
	if (boolImageProcessHaltRequest)
	{
		button1_Click(nullptr, nullptr);//fake button event to stop the image process //BT_startImageProcess_Click
		boolImageProcessHaltRequest = false;//this request handled, reset the status
		return;
	}
	try
	{	
		delayCount++;
		//based on the radius button status to determine the delay count maximum value
		if (this->checkBox1-> Checked)//final result
		{  // In the futher, checkBox1 will be delited, because it will be auto
			delayCountMaximum = 10;//show every frame
		}
		
		LARGE_INTEGER t1, t2, ts; //tick count for the total time cost in show information on form...
		QueryPerformanceFrequency(&ts);
		QueryPerformanceCounter(&t1);

		if (this->checkBox1->Checked == true && delayCount >= delayCountMaximum)//every 10 frames -> show once
		{   //CB_showVision
            #pragma region Show Information
			delayCount = 0;
			//PCA pca_analysis(*data_pts_copy, cv::Mat(), PCA::DATA_AS_ROW, 0.95);
			//IntPtr pt1 = IntPtr(Omni3DPtr);
				//IntPtr pt1 = IntPtr(WorkPtrtemp);
			//ssss = "F1";
			//FormatBuffer(sendBuffer, ssss);
		    //clientSocket->Send(sendBuffer);
			//IntPtr pt1 = IntPtr(redimg);
			//IntPtr pt1 = IntPtr(WorkPtr);
			/*
				System::Drawing::Image^ showImage = gcnew Bitmap(imageMethod->rawImage.GetCols(),
					imageMethod->rawImage.GetRows(), imageMethod->rawImage.GetStride(),
					Drawing::Imaging::PixelFormat::Format24bppRgb, pt1
				);
			*/
				//showImage->RotateFlip(RotateFlipType::RotateNoneFlipY);//y-axis flipped
			//delete the last image for avoiding memory leak...
				//if (this-> pictureBox1->Image != nullptr)
					//delete this->pictureBox1->Image; //PB_visionShow
			//load the image to the picbox 
				//this->pictureBox1->Image = showImage; //PB_visionShow
            #pragma endregion	
			//cv::imshow("redimg", *redimg);
			//cv::imshow("blue", *blueImg);
			
			string strframe = "D:\\omniCamara\\19111003\\gray_" + to_string(countframe) + ".png";
			cv::imshow("detectImg", *detectImg);
			cv::imshow("sendphoneIMG", *sendphoneIMG);
			cv::imwrite(strframe, *detectImg);
			strframe = "D:\\omniCamara\\19111003\\color_" + to_string(countframe) + ".png";
			cv::imwrite(strframe, *sendphoneIMG);
			countframe++;
			
		}
		//if needn't to show the image on picbox 
		else
		{
			if (this->checkBox1->Checked == false)
			{ //CB_showVision
				delete this->pictureBox1->Image; //PB_visionShow
				this->pictureBox1->Image = nullptr; //PB_visionShow
			}
		}
		QueryPerformanceCounter(&t2); //tick count for the total time cost in show information on form
	}
	catch (System::Exception^ e)
	{
		MessageBox::Show(e->ToString(), "From ImageProcessThread_ProgressChangedEvent!!!", System::Windows::Forms::MessageBoxButtons::OK, System::Windows::Forms::MessageBoxIcon::Error);
	}
}
	private: System::Void BW_imageProcessThread_RunWorkerCompleted(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e) {
		//do things after the imageProcessThread end
		//do nothing...
	}

	public: void oneD_To_twoD(int i)
	{
		address1[0] = -400 + (int)i / 800; //row
		address1[1] = -400 + (int)i % 800; //col
	}
	public: void pixel_to_mm_3D()
	{
		address_temp[0] = address1[0] * zoom;
		address_temp[1] = address1[1] * zoom;
	}
	public: void P3D_to_P2D()
	{
		point_len = sqrt(address_temp[0] * address_temp[0] + address_temp[1] * address_temp[1]);
		if (point_len == 0.0) { cosTheta = 0; sinTheta = 0; }
		else
		{
			cosTheta = (address_temp[0] * 1.0) / point_len;
			sinTheta = (address_temp[1] * 1.0) / point_len;
		}
		cosAlpha = point_len / sqrt((2 * c - (-h)) * (2 * c - (-h)) + address_temp[0] * address_temp[0] + address_temp[1] * address_temp[1]);
		sinAlpha = (2 * c - (-h)) / sqrt((2 * c - (-h)) * (2 * c - (-h)) + address_temp[0] * address_temp[0] + address_temp[1] * address_temp[1]);
		NP_3D[0] = cosAlpha * cosTheta;
		NP_3D[1] = cosAlpha * sinTheta;
		NP_3D[2] = -sinAlpha;
		if (NP_3D[0] == 0 && NP_3D[1] == 0 && NP_3D[2] == 0)
		{
			NP_Distance = 0;
			M_3D[0] = 0;
			M_3D[1] = 0;
			M_3D[2] = 2 * c;
		}
		else
		{
			NP_Distance = sqrt(NP_3D[0] * NP_3D[0] + NP_3D[1] * NP_3D[1] + NP_3D[2] * NP_3D[2]);
			M_3D[0] = ((a * a) / (b * NP_Distance - c * NP_3D[2])) * NP_3D[0];
			M_3D[1] = ((a * a) / (b * NP_Distance - c * NP_3D[2])) * NP_3D[1];
			M_3D[2] = ((a * a) / (b * NP_Distance - c * NP_3D[2])) * NP_3D[2] + 2 * c;
		}
		temp2D[0] = M_3D[0] * f / M_3D[2];
		temp2D[1] = M_3D[1] * f / M_3D[2];
	}
	public: void mm_to_pixel_2D()
	{
		address2[0] = (int)floor(temp2D[0] * 100);
		address2[1] = (int)floor(temp2D[1] * 100);
	}
	public: void makeImg()
	{
		int addr1 = ((address1[0] + 400) * 800 + (address1[1] + 400)) * 3;
		int addr2 = ((address2[0] + 400) * 800 + (address2[1] + 400)) * 3;
		if (address2[0] + 400 >= 0 && address2[1] + 400 >= 0 && address2[0] + 400 < 800 && address2[1] + 400 < 800)
		{
			Omni3DPtr[addr1] = Omni2DPtr[addr2];
			Omni3DPtr[addr1 + 1] = Omni2DPtr[addr2 + 1];
			Omni3DPtr[addr1 + 2] = Omni2DPtr[addr2 + 2];
		}
	}
	
	public: void carLocation() 
	{
		int temp=0, blue, green, red, colorRedVar, colorBlueVar;
		for (int i = 0; i < 800; i++) 
		{
			for (int j = 0; j < 800; j++) 
			{
				blue = Omni3DPtr[temp] * Omni3DPtr[temp];
				green = Omni3DPtr[temp + 1] * Omni3DPtr[temp + 1];
				red= Omni3DPtr[temp + 2] * Omni3DPtr[temp + 2];
				colorRedVar = sqrt(blue + green + 65025-510* Omni3DPtr[temp + 2]+red);
				colorBlueVar = sqrt(65025 - 510 * Omni3DPtr[temp] + green + red);

				if (colorRedVar <200){ redimg->at<uchar>(800-1-i, j) = 128; }
				else if (colorBlueVar < 70) { blueImg->at<uchar>(800 - 1 - i, j) = 64; }
				else{ redimg->at<uchar>(800-1-i, j) = 0; 

				blueImg->at<uchar>(800 - 1 - i, j) = 0;
				detectImg->at<uchar>(i, j) = 0;
				}
				temp += 3;
			}
		}
		cv::erode(*redimg, *redimg, *element);
		cv::erode(*blueImg, *blueImg, *element);
		cv::dilate(*redimg, *redimg, *element);
		cv::dilate(*blueImg, *blueImg, *element);
	}

	public: void carCenter() 
	{
		
		vector<cv::Point> carArea;
		vector<cv::Point> detectArea;
		vector<vector<cv::Point> > contoursRed;
		vector<vector<cv::Point> > contoursBlue;
		findContours(*redimg, contoursRed, RETR_LIST, CHAIN_APPROX_NONE);
		findContours(*blueImg, contoursBlue, RETR_LIST, CHAIN_APPROX_NONE);
		for (size_t i = 0; i < contoursRed.size(); i++)
		{
			double area = contourArea(contoursRed[i]);
			if (area < 1e3 || 1e5 < area) continue;
			int detectsz = static_cast<int>(contoursRed[i].size());
			Mat detectdata_pts = Mat(detectsz, 2, CV_64F);//CV_64F

			for (int j = 0; j < detectdata_pts.rows; j++)
			{
				detectdata_pts.at<double>(j, 0) = contoursRed[i][j].y;
				detectdata_pts.at<double>(j, 1) = contoursRed[i][j].x;
			}
			PCA pca_analysis(detectdata_pts, noArray(), PCA::DATA_AS_ROW); //PCA::DATA_AS_ROW
			drawContours(*detectImg, contoursRed, static_cast<int>(i), Scalar(128), CV_FILLED);
		}
		for (size_t i = 0; i < contoursBlue.size(); i++)
		{
			double area = contourArea(contoursBlue[i]);
			if (area < 1e2 || 1e4 < area) continue;
			int detectsz = static_cast<int>(contoursBlue[i].size());
			Mat detectdata_pts = Mat(detectsz, 2, CV_64F);//CV_64F

			for (int j = 0; j < detectdata_pts.rows; j++)
			{
				detectdata_pts.at<double>(j, 0) = contoursBlue[i][j].y;
				detectdata_pts.at<double>(j, 1) = contoursBlue[i][j].x;
			}
			PCA pca_analysis(detectdata_pts, noArray(), PCA::DATA_AS_ROW); //PCA::DATA_AS_ROW
			drawContours(*detectImg, contoursBlue, static_cast<int>(i), Scalar(64), CV_FILLED);
		}
		carArea.clear();
		for (int i = 0; i < 800; i++) {
			for (int j = 0; j < 800; j++) {
				if (detectImg->at<uchar>(i, j) ==128 ) {
					carArea.push_back(cv::Point(i, j));
				}
				if (detectImg->at<uchar>(i, j) ==64) {
					detectArea.push_back(cv::Point(i, j));
				}
			}
		}
		cv::Point detectcntr;
		if (static_cast<int>(detectArea.size()) > 0) {
			int detectsz = static_cast<int>(detectArea.size());
			Mat detectdata_pts = Mat(detectsz, 2, CV_64F);//CV_64F

			for (int i = 0; i < detectdata_pts.rows; i++)
			{
				detectdata_pts.at<double>(i, 0) = detectArea[i].y;
				detectdata_pts.at<double>(i, 1) = detectArea[i].x;
			}
			int detectmaxComponents;
			PCA pca_analysis(detectdata_pts, noArray(), PCA::DATA_AS_ROW); //PCA::DATA_AS_ROW
			detectcntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
				static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
		}
		if (static_cast<int>(carArea.size()) > 0) {

			int sz = static_cast<int>(carArea.size());
			Mat data_pts = Mat(sz, 2, CV_64F);//CV_64F

			for (int i = 0; i < data_pts.rows; i++)
			{
				//cout << carArea[i] << endl;
				data_pts.at<double>(i, 0) = carArea[i].y;
				data_pts.at<double>(i, 1) = carArea[i].x;
			}


			//data_pts_copy = data_pts;
			int maxComponents;
			//Perform PCA analysis
			//PCA pca_analysis;
			//Mat data = formatImagesForPCA(images);
			//try {
			PCA pca_analysis(data_pts, noArray(), PCA::DATA_AS_ROW); //PCA::DATA_AS_ROW
			//PCA pca_analysis(data_pts, noArray(), PCA::DATA_AS_ROW);
			//cv::PCA pca_analysis(data_pts, cv::Mat(), PCA::DATA_AS_ROW);
		//}
		//catch (int err) {

		//}

		//Store the center of the object
			cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
				static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

			//Store the eigenvalues and eigenvectors
			vector<Point2d> eigen_vecs(2);
			vector<double> eigen_val(2);
			for (int i = 0; i < 2; i++)
			{
				eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
					pca_analysis.eigenvectors.at<double>(i, 1)); ///
				eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
			}


			circle(*detectImg, cntr, 3, Scalar(255, 0, 255), 2);
			circle(*detectImg, detectcntr, 3, Scalar(255, 0, 255), 2);


			double disSize = sqrt(((eigen_vecs[0].x)*(eigen_vecs[0].x) + (eigen_vecs[0].y)*(eigen_vecs[0].y)));
			cv::Point p1 = cntr + cv::Point(static_cast<int>((eigen_vecs[0].x)*-30), static_cast<int>((eigen_vecs[0].y) * -30));
			cv::Point p2 = cv::Point(static_cast<int>((eigen_vecs[0].x) * 30), static_cast<int>((eigen_vecs[0].y) * 30));
			cv::Point p3 = cv::Point(detectcntr.x - cntr.x, detectcntr.y - cntr.y);
			if ((p2.x > 0 && p3.x < 0) || (p2.x < 0 && p3.x > 0)) {
				p2.x *= -1;
			}
			if ((p2.y > 0 && p3.y < 0) || (p2.y < 0 && p3.y > 0)) {
				p2.y *= -1;
			}
			Carsangle = -atan2(p2.y, p2.x) * 180 / 3.14;
			if (Carsangle < 0) {
				Carsangle = 360 + Carsangle;
			}
			//cv::Point p1 = *cntr + cv::Point(detectcntr.x - cntr->x, detectcntr.y - cntr->y);
			realdistance = 75, realangle = 30;
			cv::Point p4 = cv::Point((-400 + cntr.x), -(-400 + cntr.y));
			double p4_dis = sqrt(p4.x*p4.x + p4.y*p4.y)*zoom/10.0;
			carDisSum += p4_dis;
			carDisVarianceSum += (realdistance - p4_dis)*(realdistance - p4_dis);
			if (Carsangle > 300) Carsangle = Carsangle - 360;
			carAngleSum += Carsangle;
			carAngleVarianceSum += (realangle - Carsangle)*(realangle - Carsangle);

			cout <<"LawnMower Distance= " << p4_dis <<"LawnMower Angle "<< Carsangle << endl;
			cout << "Distance Mean= " << carDisSum / framepage << "Distance SD " << sqrt(carDisVarianceSum / framepage) << endl;
			cout << "Angle Mean= " << carAngleSum / framepage << "Angle SD " << sqrt(carAngleVarianceSum / framepage) << endl;
			cout << framepage<< endl;
			framepage++;
			//cout<< p1.x <<" "<< p1.y <<" " << disSize << " "<< eigen_vecs[0].x << " " << eigen_vecs[0].y << endl;
			/*if ((detectcntr.x - cntr.x > 0 && eigen_vecs[0].x < 0) || (detectcntr.x - cntr.x < 0 && eigen_vecs[0].x > 0)) {
				p1.x = -p1.x;
				p1.y = -p1.y;
			}*/
			carG[0] = cntr.x; carG[1] = cntr.y;
			drawAxis(*detectImg, cntr, cntr + p2, Scalar(255), 1);
			//drawAxis(*detectImg, cntr, cntr + p3, Scalar(255), 1);
			//drawAxis(*sendphoneIMG, cntr, p2, Scalar(255, 0, 0), 1);
			//double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
			
		}
		
	}

	public:void carCommand() {
		cli::array<Byte>^ receiveBuffer = gcnew cli::array<Byte>(3);
		if (sockProtocol == ProtocolType::Tcp) {
			clientSocket = serverSocket->Accept();
		}
	}
	private: void drawAxis(Mat& img, cv::Point p, cv::Point q, Scalar colour, const float scale)
	{
		double angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
		double hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
		// Here we lengthen the arrow by a factor of scale
		q.x = (int)(p.x - scale * hypotenuse * cos(angle));
		q.y = (int)(p.y - scale * hypotenuse * sin(angle));
		line(img, p, q, colour, 1, LINE_AA);
		// create the arrow hooks
		p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
		line(img, p, q, colour, 1, LINE_AA);
		p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
		line(img, p, q, colour, 1, LINE_AA);
	}

	public: void correctTheCar() {
		cv::Point vec3 = cv::Point((carG[0] - (nowPosition[0])), (carG[1] -(nowPosition[1])));
		cout << "correctTheCar\n";
		int disP2P = sqrt(vec3.x*vec3.x + vec3.y*vec3.y);
		double angleP2P, angleP2P1;
		int angleTemp;
		System::String^ sff = "";
		int P2PAngleCount;
		//cout << "carG" << cv::Point(carG[0], carG[1])<<" realdis" <<disP2P*zoom/10 << endl;
		circle(*detectImg, cv::Point(nowPosition[0], nowPosition[1]), 5, Scalar(255), 2);
		circle(*detectImg, cv::Point(nextPosition[0], nextPosition[1]), 5, Scalar(255), 2);
		if(nowPoint<2)drawAxis(*detectImg, cv::Point(nowPosition[0], nowPosition[1]), cv::Point(nowPosition[0], nowPosition[1])+vec3, Scalar(255), 1);
		cout << "disP2P= " << disP2P << "\t nowPoint" << nowPoint << endl;
		if ((disP2P) <= 5) nowPoint = 1;
		if (nowPoint == 0 && (disP2P)>5) {
			angleP2P1 = -atan2(vec3.y, vec3.x) * 180 / 3.14;
			angleP2P = angleP2P1;
			if (angleP2P < 0) {
				angleP2P = 360 + angleP2P;
			}
			cout << angleP2P << " " << Carsangle << endl;
			angleP2P = Carsangle- angleP2P;
			if (angleP2P > 180)angleP2P = angleP2P - 360;
			P2PAngleCount = 0;
			if (angleP2P > 5) {
				angleTemp = (int)angleP2P;
				while (angleTemp > 0) {
					angleTemp -= 15;
					P2PAngleCount++;
				}
				sff += "R";
				sff += ((Char)(P2PAngleCount)).ToString();
				//cout << "angleTemp" << angleP2P <<" "<< P2PAngleCount << endl;
				//for (int iii = 0; iii < sff->Length; iii++)
				//	Console::WriteLine(sff[iii]);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
			}
			else if(angleP2P < -5){
				angleTemp = (int)-angleP2P;
				while (angleTemp > 0) {
					angleTemp -= 15;
					P2PAngleCount++;
				}
				sff += "L";
				sff += ((Char)(P2PAngleCount)).ToString();
				//cout << "angleTemp" << angleP2P << " " << P2PAngleCount<< endl;
				//for(int iii=0; iii<sff->Length;iii++)
				//	Console::WriteLine(sff[iii]);
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
			}
			else {
				if (nowPosition[1] > carG[1]) {
					sff += "B";
					sff += ((Char)(int((disP2P*zoom/10)/2))).ToString();
					cout << "dis" << " " << int((disP2P*zoom / 10) / 2) << endl;
					FormatBuffer(sendBuffer, sff);
					cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
					clientSocket->Send(sendBuffer);
					Sleep(1000);
				}
				else {
					sff += "F";
					sff += ((Char)(int((disP2P*zoom / 10) / 2))).ToString();
					cout << "dis" << " " << int((disP2P*zoom / 10) / 2) << endl;
					FormatBuffer(sendBuffer, sff);
					cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
					clientSocket->Send(sendBuffer);
					Sleep(1000);
				}
			}
		}
		if (nowPoint == 1) {
			P2PAngleCount = 0;
			int nowturnAngle =  Carsangle- nowangle;
			cout << "nowturnAngle" << nowturnAngle <<"\t Carsangle = "<< Carsangle << endl;;
			if (nowturnAngle> 5) {
				angleTemp = nowturnAngle;
				while (angleTemp > 0) {
					angleTemp -= 15;
					P2PAngleCount++;
				}
				sff += "R";
				sff += ((Char)(P2PAngleCount)).ToString();
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << P2PAngleCount<<endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
			}
			else if (nowturnAngle < -5) {
				angleTemp = -nowturnAngle;
				while (angleTemp > 0) {
					angleTemp -= 15;
					P2PAngleCount++;
				}
				sff += "L";
				sff += ((Char)(P2PAngleCount)).ToString();
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << P2PAngleCount << endl;
				clientSocket->Send(sendBuffer);
				Sleep(1000);
			}
			else {
				nowPoint = 2;
				cout << Carsangle << endl;
			}
			if (nowPoint == 2) {
				cv::Point p7 = cv::Point((nextPosition[0] - nowPosition[0]), (nextPosition[1] - nowPosition[1]));
				int nextdis = sqrt(p7.x*p7.x + p7.y*p7.y);
				sff += "F";
				sff += ((Char)(int((nextdis*zoom / 10) / 2))).ToString();
				cout << "dis" << " " << int((nextdis*zoom / 10) / 2) << endl;
				FormatBuffer(sendBuffer, sff);
				cout << "sendBuffer:" << sendBuffer[0] << " " << sendBuffer[1] << endl;
				clientSocket->Send(sendBuffer);
				Sleep(2000);
			}
		}
	    
	}

	//------------------
	double step1(int xy, double x1, double x2, double y1, double y2)///求n1、n2的x(u)
	{
		double a = 0, b = 0, c = 0; ///運算用變數
		a = x1 - x2;
		a = a * a;
		b = y1 - y2;
		b = b * b;
		c = a + b;
		c = sqrt(c);
		a = a / c;
		b = b / c;
		if (xy == 1) return a;
		else return b;
	}

	double step2(int xy, double x1, double x2, double y1, double y2) ///求n3的x(u)
	{
		double a = 0, b = 0, c = 0;
		a = (x1 + x2) / 2;
		b = (y1 + y2) / 2;
		c = a * a + b * b;
		c = sqrt(c);
		c = 1 / c;
		a = a * c;
		b = b * c;
		if (xy == 1) return a;
		else return b;
	}

	double step3(int xy, double x, double y) ///計算U1、U2陣列的x
	{
		double a = 0, b = 0, c = 0;
		a = x * x; ///計算||n1||的x
		b = y * y; ///計算||n1||的y
		c = 1 / sqrt(a + b); ///得出||n1||
		a = b = 0; ///準備計算乘上[v1 -u1](垂直),運算用變數歸0
		a = y;
		b = -x;
		a = a * c;
		b = b * c;
		if (xy == 1) return a;
		else return b;
	}

	int FindPoint(int xy, double c_x1, double c_y1, double c_x2, double c_y2, double c_x3, double c_y3)
	{
		double u1; ///計算n1用
		double v1; ///計算n1用
		double u2; ///計算n2用
		double v2; ///計算n2用
		double u3; ///計算n3用
		double v3; ///計算n3用
		double t_d; ///tD
		double t_a; ///tA
		double U1_x; ///U1陣列的x
		double U1_y; ///U1陣列的y
		double U2_x; ///U2陣列的x
		double U2_y; ///U2陣列的y
		double cal_x = 0, cal_y = 0; ///函數傳回數值用
		double x, y; ///ans
	///(1)
		u1 = step1(1, c_x1, c_x2, c_y1, c_y2);
		v1 = step1(2, c_x1, c_x2, c_y1, c_y2);
		u2 = step1(1, c_x2, c_x3, c_y2, c_y3);
		v2 = step1(2, c_x2, c_x3, c_y2, c_y3);
		//    printf("u1=%lf v1=%lf \nu2=%lf v2=%lf\n",u1, v1, u2, v2);
		///(2)
		u3 = step2(1, u1, v1, u2, v2) + c_x1;
		v3 = step2(2, u1, v1, u2, v2) + c_y1;
		//    printf("u3=%lftd v3=%lftd\n", u3, v3);
		///(3)
		U1_x = step3(1, u1, v1);
		U1_y = step3(2, u1, v1);
		//    printf("U1_x=%lf U1_y=%lf\n", U1_x, U1_y);
		///(4)
		U2_x = step3(1, u2, v2);
		U2_y = step3(2, u2, v2);
		///////////////////////////////
		double S = 1, Cx = c_x2, Cy = c_y2, U1x = U1_x, U1y = U1_y, U2x = U2_x, U2y = U2_y, x1 = u1, y1 = v1, x2 = u2, y2 = v2, x3 = u3, y3 = v3;
		double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0, h = 0, det = 0;
		double tDx, tDy, tAx, tAy, ansX, ansY;
		a = 2 * x1 * x2 + 2 * y1 * y2; ///左上
		b = x1 * x3 + y1 * y3 + x2 * x3 + y2 * y3; ///右上
		c = b; ///左下
		d = 2 * x3 * x3 + 2 * y3 * y3; /// 右下
		det = a * d - b * c;
		e = a * x3 + a * y3 + b * -x2 + b * -y2;
		f = a * x3 + a * y3 + b * -x1 + b * -y1;
		g = c * x3 + c * y3 + d * -x2 + d * -y2;
		h = c * x3 + c * y3 + d * -x1 * d * -y1;
		tDx = e * S * U1x + e * Cx + f * S * U2x + f * Cx;
		tDx /= det;
		tDy = e * S * U1y + e * Cy + f * S * U2y + f * Cy;
		tDy /= det;
		tAx = g * S * U1x + g * Cx + h * S * U2x + h * Cx;
		tAy = g * S * U1y + g * Cy + h * S * U2y + h * Cy;
		ansX = tDx * x3 + Cx;
		ansY = tDy * y3 + Cy;
		if (xy == 1) return ansX;
		else return ansY;
	}

	double Distance(double x1, double x2, double y1, double y2)
	{
		double a = 0, b = 0, c = 0; ///運算用變數
		a = x1 - x2;
		a = a * a;
		b = y1 - y2;
		b = b * b;
		c = a + b;
		c = sqrt(c);
		return c;
	}

	//------------------
	private: System::Void checkBox1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		
	}
	private: System::Void openIMG_Click(System::Object^  sender, System::EventArgs^  e) {
		/*
		Mat img = imread("D:\\190709\\300\\30\\fc2_save_2019-07-09-091403-0002.bmp");
		//cvtColor(img, img, CV_BGR2BGRA);
		//cout << " data: "<<img.at<byte>(0,0)+30<< endl;
		//HBITMAP img1= CreateBitmap(img.cols, img.rows, 1, 32, img.data);
		//Bitmap^ bmp = Bitmap::FromHbitmap((IntPtr)img1);
		//pictureBox1->Image = bmp;
		cvtColor(img, img, CV_BGR2GRAY, 1);
		Mat imgGray;
		int step = img.step;
		uchar* oData = (uchar*)img.data;
		//創建一個三通道的iplimage，裡面存放的是原先單通道的灰階圖
		IplImage* nImg = cvCreateImage(cvSize(img.cols, img.rows), IPL_DEPTH_8U, 3);
		int nStep = nImg->widthStep;
		//cout << nStep << endl;
		int nChannels = nImg->nChannels;
		uchar* nData = (uchar *)nImg->imageData;
		for (int j = 0; j < img.cols; j++)
		{
			for (int i = 0; i < img.rows; i++)
			{
				int counter = j * nStep + i * nChannels;
				if(oData[j*step + i]>5)
				{
					nData[counter + 0] = oData[j*step + i];
					nData[counter + 1] = oData[j*step + i];
					nData[counter + 2] = oData[j*step + i];
				}
				else
				{
					nData[counter + 0] = byte(0);
					nData[counter + 1] = byte(0);
					nData[counter + 2] = byte(255);
				}
			}
		}
		//imgGray.create(img.cols, img.rows, CV_8U);
		//imgGray.create(cvSize(img.cols, img.rows), IPL_DEPTH_8U, 3);
		//IplImage* nImg = cvCreateImage(cvSize(img.cols, img.rows), IPL_DEPTH_8U, 3);
		//cvtColor(img, imgGray, CV_BGR2GRAY, 3);
		//Bitmap^ bmpG= gcnew Bitmap(nImg->width, nImg->height, nImg->widthStep, System::Drawing::Imaging::PixelFormat::Format24bppRgb, (System::IntPtr)nImg->imageData);
		System::Drawing::Image^ bmpG = gcnew Bitmap(nImg->width, nImg->height, 2400, System::Drawing::Imaging::PixelFormat::Format24bppRgb, (System::IntPtr)nImg->imageData);
		//Bitmap^ bmpG = gcnew Bitmap(imgGray.cols, imgGray.rows, (int)imgGray.step, System::Drawing::Imaging::PixelFormat::Format24bppRgb, (System::IntPtr)imgGray.data);
		//HBITMAP imgG = CreateBitmap(imgGray.cols, imgGray.rows, 1, 32, imgGray.data);
		//Bitmap^ bmpG = Bitmap::FromHbitmap((IntPtr)imgG);
		pictureBox1->Image = bmpG;
		*/
	}
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
		Mat origin = imread("D:\\190702\\3m\\range50m.jpg");
		Mat img = imread("D:\\190702\\3m\\30\\range50m_aim30m.jpg");
		//Mat img_1= img.clone();
		Mat car= origin.clone();
		cvtColor(origin, origin, CV_BGR2GRAY, 1);
		cvtColor(img, img, CV_BGR2GRAY, 1);
		absdiff(origin, img, car);

		Mat car1 = Mat::zeros(500, 500, CV_8UC3);
		Mat dst = Mat::zeros(500, 500, CV_8UC3);
		resize(car, dst, dst.size());

		Mat element(3, 3, CV_8U, Scalar(1));
		threshold(dst, dst,16,255, THRESH_BINARY);
		erode(dst, dst, element);
		dilate(dst, dst, element);
		

		IplImage* nImg = cvCreateImage(cvSize(img.cols, img.rows), IPL_DEPTH_8U, 3);
		int step = img.step, nStep = nImg->widthStep, nChannels = nImg->nChannels;
		uchar* nData = (uchar *)nImg->imageData, *oData = (uchar*)car.data, *oimg= (uchar*)img.data;
		/*for (int j = 0; j < img.cols; j++)
		{
			for (int i = 0; i < img.rows; i++)
			{
				int counter = j * nStep + i * nChannels;
				nData[counter + 0] = oData[j*step + i];
				nData[counter + 1] = oData[j*step + i];
				nData[counter + 2] = oData[j*step + i];
			}
		}*/
		/*
		Mat_<Vec3b>::iterator itIMG = img.begin<Vec3b>();
		Mat_<Vec3b>::iterator itendIMG = img_1.end<Vec3b>();
		for (; itIMG != itendIMG; itIMG++) {
			(*itIMG)[0] = (byte)0;
			(*itIMG)[1] = (byte)0;
			(*itIMG)[2] = (*itIMG)[2];
		}*/
		//cout << "++";
		//System::Drawing::Image^ bmpG = gcnew Bitmap(nImg->width, nImg->height, nImg->widthStep, System::Drawing::Imaging::PixelFormat::Format24bppRgb, (System::IntPtr)nImg->imageData);
		
		//HBITMAP imgG = CreateBitmap(img.cols, img.rows, 1, 32, img.data);
		//Bitmap^ bmpG = Bitmap::FromHbitmap((IntPtr)imgG);

		//pictureBox1->Image = bmpG;
		//Mat dst = Mat::zeros(400, 400, CV_8UC3);
		//resize(img_1, dst, dst.size());
		//imshow("img", img);
		imshow("img_1", dst);
		//imwrite("C:\\Users\\user\\Desktop\\testG_24_ED.jpg", dst);
		waitKey();

		/*
		Mat_<Vec3b>::iterator it = car.begin<Vec3b>();
		Mat_<Vec3b>::iterator itend = car.end<Vec3b>();
		Mat_<Vec3b>::iterator it1 = origin.begin<Vec3b>();
		Mat_<Vec3b>::iterator it2 = img.begin<Vec3b>();
		
		int step = img.step;
		uchar* oData = (uchar*)img.data;
		IplImage* nImg = cvCreateImage(cvSize(img.cols, img.rows), IPL_DEPTH_8U, 3);
		int nStep = nImg->widthStep;
		int nChannels = nImg->nChannels;
		uchar* nData = (uchar *)nImg->imageData;
		for (; it != itend; it++)
		{
			(*it)[0] = (byte)abs((*it1)[0] - (*it2)[0]);
			(*it)[1] = (byte)abs((*it1)[0] - (*it2)[0]);
			(*it)[2] = (byte)abs((*it1)[0] - (*it2)[0]);
			it1++;
			it2++;
		}
		*/
		//System::Drawing::Image^ bmpG = gcnew Bitmap(car.cols, car.rows, 2400, System::Drawing::Imaging::PixelFormat::Format24bppRgb, (System::IntPtr)car.data);
		//pictureBox1->Image = bmpG;
	}
	private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
		road * rr;
		rr = new road();
		rr->alice();
	}
};
}
