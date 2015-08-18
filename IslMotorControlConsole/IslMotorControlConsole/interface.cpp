#include "interface.h"
#include "packConstructor.cpp"
#include <iostream>
#include <string>
#include <sstream>

class Interface {

private:
	int packetSize;
	static const int dataStart = 9;
	packConstructor* temp;
	char packet2[3];
	CSerial *terminal;
	char *myRecCursor;
	char *packet;


public:
	Interface(){
		Interface(DEFAULT_PACKET_SIZE, NULL);
	}
	Interface(int PacketSize, CSerial *port){
		terminal = port;
		packetSize = PacketSize;
		temp = new packConstructor(packetSize);
		myRecCursor = &(packet[0]);

	}
	int ReceivePacket(char *myRecCursor,int nonEndedPacket) {
	//asagidan gelen paketleri karsilayan fonksiyon.
		int  senderCard ;
		int result;
		unsigned char BUFFER[100];
		//local variables
		int bufferIndex=0, oldbyte1 = 0,oldbyte2 = 0,oldbyte3 = 0;
		DWORD dwBytesRead;
		//read the Buffer into the BUFFER[100]
		do
		{
			//read data from the COM-port
			terminal->Read(BUFFER,sizeof(BUFFER),&dwBytesRead);

		}while (dwBytesRead == sizeof(BUFFER));		//reading of the buffer finished


		if((BUFFER[CARD_ID_0]==0x06)&&(BUFFER[CARD_ID_1]==0x00)&&(BUFFER[CARD_ID_2]==0x00)) //check the sender card  else i yazilacak
			senderCard = 1;
		if((BUFFER[CARD_ID_0]==0x07)&&(BUFFER[CARD_ID_1]==0x00)&&(BUFFER[CARD_ID_2]==0x00))
			senderCard = 2;
		if((BUFFER[CARD_ID_0]==0x08)&&(BUFFER[CARD_ID_1]==0x00)&&(BUFFER[CARD_ID_2]==0x00))
			senderCard = 3;

		//paket tipine bakilarak gelen paketler ilgili fonksiyonlara yonlendiriliyor.
		switch (BUFFER[PACKETTYPE])
		{
			case 0x00:
				result = printMotorCounter(senderCard, BUFFER );
				break;
			case 0x01:
				result = printSpeed(senderCard, BUFFER);
				break;
			case 0x02:
				result = printSpeedPIDParameters(senderCard, BUFFER);
				break;
			case 0x03:
				result = printPositionPIDParameters(senderCard, BUFFER);
				break;
      case 0x17:
        result = printDistances(senderCard, BUFFER);
        break;
      case 0x18:
        result = printLines(senderCard, BUFFER);
        break;
		}	

		return result;
	}

  int printDistances(int Sender, unsigned char* Buffer) {
    //sayac degerinin yazdirilmasinda kullaniliyor.
    int dist1 = 0;
    int dist2 = 0;

    unsigned char a1, a2, a3, a4;
    a1 = Buffer[dataStart + 1];
    a2 = Buffer[dataStart + 2];
    a3 = Buffer[dataStart + 3];
    a4 = Buffer[dataStart + 4];

    dist1 = (int)(((int)(a1 << 8) & 0xFF00) | ((int)(a2)& 0x00FF));
    dist2 = (int)(((int)(a3 << 8) & 0xFF00) | ((int)(a4)& 0x00FF));

    printf("Right: %d and Left: %d mm\n", dist1, dist2);

    return 1;
  }

  int printLines(int Sender, unsigned char* Buffer) {
    //sayac degerinin yazdirilmasinda kullaniliyor.
    int lineR = 0;
    int lineM = 0;
    int lineL = 0;

    char a1, a2, a3;
    a1 = Buffer[dataStart + 1];
    a2 = Buffer[dataStart + 2];
    a3 = Buffer[dataStart + 3];

    lineR = (int)((a1) & 0xFF);
    lineM = (int)((a2) & 0xFF);
    lineL = (int)((a3) & 0xFF);

    printf("Right: %d Middle: %d Left: %d", lineR, lineM, lineL);

    return 1;
  }
	int printMotorCounter(int Sender, unsigned char* Buffer) {
		//sayac degerinin yazdirilmasinda kullaniliyor.
		int motorCounter = 0;
		unsigned char a1, a2, a3, a4;
		a1 = Buffer[dataStart+1];
		a2 = Buffer[dataStart+2];
		a3 = Buffer[dataStart+3];
		a4 = Buffer[dataStart+4];

		
		//ilk gelen byte'ta gelen sayac degerinin hangi motora ait oldugu belirtiliyor.
		if (Buffer[dataStart] == 0x00)
			printf("\nCard #%d(verified)\nNumber of counts of the motor 0 is: ", Sender);
		else if (Buffer[dataStart] == 0x01)
			printf("\nCard #%d(verified)\nNumber of counts of the motor 1 is: ", Sender);


    motorCounter = (int)(((long)(a1 << 24) & 0xFF000000) | ((long)(a2 << 16) & 0x00FF0000) | ((long)(a3 << 8) & 0x0000FF00) | (a4));

		printf("0x%d ", motorCounter);
		printf("counts\n");


		return motorCounter;
	}

	int printSpeed(int Sender, unsigned char* Buffer) {

    __int16 speed;
    char temp;

		temp = Buffer[dataStart];

		printf("\nCard #%d(verified)\nSpeed value of motor %d\n", Sender, temp);

    speed = (((((__int16)Buffer[dataStart + 1]) << 8) & 0xFF00) | ((((__int16)Buffer[dataStart + 2])) & 0x00FF));


		printf("%d", speed);

		return speed;
	}

	int printPositionPIDParameters(int Sender, unsigned char* Buffer) {
		int Kp, Kd, Ki;
		unsigned char motorID = Buffer[dataStart];

		printf("\nCard #%d(verified)\nPosition PID parameters of motor %c are:\n", Sender, motorID); 

		Kp = Buffer[dataStart + 1];
		Ki = Buffer[dataStart + 2];
		Kd = Buffer[dataStart + 3];

		printf(" P : %d\n I : %d\n D : %d\n", Kp, Ki, Kd);

		return 0;
	}
	
	int printFaultpin(int Sender, unsigned char* Buffer) {
		int pin;
		unsigned char motorID = Buffer[dataStart];
		pin = Buffer[dataStart + 1];

		
		printf("\nCard #%d(verified)\nActivated fault pin of motor %c is:\n", Sender, motorID); 



		printf("Pin %d\n",pin);

		return 0;
	}

	int printSpeedPIDParameters(int Sender, unsigned char* Buffer) {
		int Kp, Kd, Ki;
		unsigned char motorID = Buffer[dataStart];

		printf("\nCard #%d(verified)\nSpeed PID parameters of motor %c are:\n", Sender, motorID); 

		Kp = Buffer[dataStart + 1];
		Ki = Buffer[dataStart + 2];
		Kd = Buffer[dataStart + 3];

		printf(" P : %d\n I : %d\n D : %d\n", Kp, Ki, Kd);

		return 0;
	}


	int setMotorCounter(char *ReceiverCard, int MotorID, long counterValue) {
		char* temppack;
		packet = new char[5];   // DATA LENGTH FOR SETMOTORCOUNTER

		for(int i = 0; i<(packetSize-3); i++) 
			packet[i] = 0x00;
	
		packet[0] = (char) MotorID;


		packet[1] = (char) (counterValue >> 24) & 0xFF;
		packet[2] = (char) (counterValue >> 16) & 0xFF;
		packet[3] = (char) (counterValue >> 8) & 0xFF;
		packet[4] = (char) counterValue & 0xFF;


		temppack = temp->makePacket(0x00, packet, 5, 0x10, ReceiverCard);

		terminal->Write(temppack, 17*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);
		
		int result = ReceivePacket(myRecCursor,0);

		return result;
	}

	int getFaultpin(char* ReceiverCard) {
		
		int result;
		char *temppack;
		packet = new char[1];
		packet[0] = (char)0x00;

		temppack = temp->makePacket(0x00, packet, 1, 0x09, ReceiverCard);

		terminal->Write(temppack, 13*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		result = ReceivePacket(myRecCursor, 0);

		return result;		

	}


	int getMotorCounter(char* ReceiverCard, int MotorID) {
		
		int result;
		char *temppack;
		packet = new char[1];

		switch(MotorID) {
			case 0:
				packet[0] = 0x00;
				break;
			case 1:
				packet[0] = 0x01;
				break;
		}

		temppack = temp->makePacket(0x00, packet, 1, 0x00, ReceiverCard);

		terminal->Write(temppack, 13*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		result = ReceivePacket(myRecCursor, 0);

		return result;		
	}


	int setSpeed(char* ReceiverCard, int MotorID, int Speed) {
		char* temppack;
		packet = new char[3];

		switch(MotorID) {
			case 0:
				packet[0] =  (char)0x00;
				break;
			case 1:
				packet[0] =  (char)0x01;
				break;
		}

		if( Speed > SPEED_LIMIT )
			Speed = SPEED_LIMIT;
		else if ( Speed < -SPEED_LIMIT )
			Speed = -SPEED_LIMIT;

		packet[1] = (char)(Speed >> 8);
		packet[2] = (char)(Speed & 0x00FF);

		temppack = temp->makePacket(0x00, packet, 3, 0x11, ReceiverCard);

		terminal->Write(temppack, 15*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		return ReceivePacket(myRecCursor, 0);
	}


	int getSpeed(char* ReceiverCard, int MotorID) {
		
		int result;
		char *temppack;
		packet = new char[1];

		switch(MotorID) {
			case 0:
				packet[0] = 0x00;
				break;
			case 1:
				packet[0] = 0x01;
				break;
		}

		temppack = temp->makePacket(0x00, packet, 1, 0x01, ReceiverCard);

		terminal->Write(temppack, 13*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		result = ReceivePacket(myRecCursor,0);
	
		return result;
	}

	int setSpeedPIDParameters(char *ReceiverCard, int MotorID, int Kp, int Ki, int Kd) {
		
		char* temppack;
		packet = new char[4];

		packet[0] = MotorID;
		packet[1] = Kp;
		packet[2] = Ki;
		packet[3] = Kd;

		temppack = temp->makePacket(0x00, packet, 4, 0x12, ReceiverCard);

		terminal->Write(temppack, 16*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		return ReceivePacket(myRecCursor, 0);

	}


	int getSpeedPIDParameters(char* ReceiverCard, int MotorID) {
		int result;
		char *temppack;
		packet = new char[1];

		switch(MotorID) {
			case 0:
				packet[0] = (char)0x00;
				break;
			case 1:
				packet[0] = (char)0x01;
				break;
		}

		temppack = temp->makePacket(0x00, packet, 1, 0x02, ReceiverCard);

		terminal->Write(temppack, 13*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		result = ReceivePacket(myRecCursor,0);
	
		return result;
	}


	int setPositionPIDParameters(char *ReceiverCard, int MotorID, int Kp, int Ki, int Kd){
		
		char* temppack;
		packet = new char[4];

		packet[0] = MotorID;
		packet[1] = Kp;
		packet[2] = Ki;
		packet[3] = Kd;

		temppack = temp->makePacket(0x00, packet, 4, 0x13, ReceiverCard);

		terminal->Write(temppack, 16*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		return ReceivePacket(myRecCursor, 0);

	}


	int getPositionPIDParameters(char* ReceiverCard, int MotorID) {
		int result;
		char *temppack;
		packet = new char[1];

		switch(MotorID) {
			case 0:
				packet[0] = (char)0x00;
				break;
			case 1:
				packet[0] = (char)0x01;
				break;
		}

		temppack = temp->makePacket(0x00, packet, 1, 0x03, ReceiverCard);

		terminal->Write(temppack, 13*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		result = ReceivePacket(myRecCursor,0);
	
		return result;
	}

	int setMotorCounters(char *ReceiverCard, long counterValue0, long counterValue1) {
		char* temppack;
		packet = new char[8];   // DATA LENGTH FOR SETMOTORCOUNTER

		packet[0] = (char)(counterValue0 >> 24);
    packet[1] = (char)(counterValue0 >> 16);
    packet[2] = (char)(counterValue0 >> 8);
    packet[3] = (char)counterValue0;

    packet[4] = (char)(counterValue1 >> 24);
    packet[5] = (char)(counterValue1 >> 16);
    packet[6] = (char)(counterValue1 >> 8);
    packet[7] = (char)counterValue1 ;


		temppack = temp->makePacket(0x00, packet, 8, 0x15, ReceiverCard);

		terminal->Write(temppack, 20*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);
		
		int result = ReceivePacket(myRecCursor,0);

		return result;
	}

	
	int setSpeeds(char* ReceiverCard, int Speed0, int Speed1) {
		
		char* temppack;
		packet = new char[4];

		if( Speed1 > SPEED_LIMIT )
			Speed1 = SPEED_LIMIT;
		else if ( Speed1 < -SPEED_LIMIT )
			Speed1 = -SPEED_LIMIT;

		if( Speed0 > SPEED_LIMIT )
			Speed0 = SPEED_LIMIT;
		else if ( Speed0 < -SPEED_LIMIT )
			Speed0 = -SPEED_LIMIT;


		packet[0] = (char)(Speed0 >> 8);
		packet[1] = (char)(Speed0);

		packet[2] = (char)(Speed1 >> 8);
		packet[3] = (char)(Speed1);

		temppack = temp->makePacket(0x00, packet, 4, 0x16, ReceiverCard);

		terminal->Write(temppack, 16*sizeof(char));

		Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

		return ReceivePacket(myRecCursor, 0);
	}

  int getDistances() {

    char ReceiverCard[3];
    
    char receiverCard1[3] = { 0x06, 0x00, 0x00 };

    for (int i = 0; i < 3; i++)
      ReceiverCard[i] = receiverCard1[i];
    

    int result;
    char *temppack;
    packet = new char[1];

    temppack = temp->makePacket(0x00, packet, 1, 0x17, ReceiverCard);

    terminal->Write(temppack, 13 * sizeof(char));

    Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

    result = ReceivePacket(myRecCursor, 0);

    return result;
  }

  int getLines() {

    char ReceiverCard[3];

    char receiverCard1[3] = { 0x06, 0x00, 0x00 };

    for (int i = 0; i < 3; i++)
      ReceiverCard[i] = receiverCard1[i];


    int result;
    char *temppack;
    packet = new char[1];

    temppack = temp->makePacket(0x00, packet, 1, 0x18, ReceiverCard);

    terminal->Write(temppack, 13 * sizeof(char));

    Sleep(CONST_ONE_TO_ONE_SLEEP_TIME);

    result = ReceivePacket(myRecCursor, 0);

    return result;
  }
};
