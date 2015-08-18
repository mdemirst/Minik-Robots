
#define DEFAULT_PACKET_SIZE 25

class packConstructor {

private:
	int packetSize;
	static const int dataStart = 9;

public:
	packConstructor(){
		packConstructor(25);
	}
	packConstructor(int PacketSize){
		packetSize = PacketSize;	
	}


	char* makePacket(char Sender, char Data[], int DataLength, char Task, char* ReceiverID) {

		char xor; 
		packetSize = DataLength + 12;
		char *packet = new char[packetSize];

		//paket için yer açýlýyor.
		//packet = new char[packetSize];
		for(int i = 0; i <= packetSize; i++)
			packet[i] = 0x00;		

		//paket olduðu belirtiliyor. deðerler sabit.
		packet[0] =  (char)0xA0;
		packet[1] =  (char)0xA1;
		packet[2] =  (char)0xA2;

		//gönderen. Master:0x00 , Slave:0x04
		packet[3] = Sender;

		//aþaðýda, karþýlanan verinin ilgili fonksiyona iletilmesini saðlayan görev kimliði.
		packet[4] = Task;

		//paket boyutu.
		packet[5] = packetSize;

		//alýcý kimlikleri.  #1:0x06,0x00,0x00 ; #2:0x07,0x00,0x00 ; #3:0x08,0x00,0x00
		for(int i = 6; i <= 8; i++)
			packet[i] = ReceiverID[i - 6];

		//9 ile packetSize-4 arasýndaki bytelar gönderilecek verilere tahsis ediliyor.
		for(int i = dataStart; i < (dataStart + DataLength); i++)
			packet[i] = Data[i - 9];
				
		//veri bitlerinin sona erdiðini belirten sabit.
		packet[dataStart + DataLength] = (char)0xAA;

		//ilk dokuz byte'ýn kontrol amacýyla xor'lanmasý ve 24.byte'a atanmasý. 
		xor = packet[8];

		for(int i = 8; i >= 4; i--)
			xor ^= packet[i - 1];

		packet[dataStart + DataLength + 1] = xor;

		//bütün byte'larýn xor'lanmasý.
		xor = packet[dataStart + DataLength + 1];

		for(int i = packetSize - 2; i >= 4; i--)
			xor ^= packet[i - 1];

		packet[dataStart + DataLength + 2] = xor;

		return packet;

	}


	char* getData(char Packet[]) {

		//paketle gönderilen veri byte'larýný döndürüyor.

		char *temp = new char[packetSize-4-dataStart];
		int i;
		for(i = dataStart; i <= (packetSize-4); i++)
			temp[i-dataStart] = Packet[i];
		return temp;
	}



	bool isValid(char Packet[]) {
		char x1 , x2;

		//parite kontrolü yapýyor.

		//ilk dokuz byte xor'lanýyor, pariteyle karþýlaþtýrýlýyor.
		x1 = Packet[8];

		for(int i = 8; i >= 1; i--)
			x1 ^= Packet[i-1];
		x1 = !(x1 | Packet[packetSize-2]);

		x2 = Packet[packetSize-2];

		//bütün byte'lar xor'lanýyor, pariteyle karþýlaþtýrýlýyor.
		for(int i = packetSize-2; i >= 1; i--)
			x1 ^= Packet[i-1];

		x2 = !(x2 | Packet[packetSize-1]);
		
		//iki kontrolün sonucu da pozitifse "true" döndürülüyor.
		if (!(x1 || x2))
			return 1;
		else 
			return 0;
	}



	char getSender(char Packet[]) {
		return Packet[3];
	}



	char getTask(char Packet[]) {
		return Packet[4];
	}


	char getPacketSize(char Packet[]) {
		return Packet[5];
	}
};