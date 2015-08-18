
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

		//paket i�in yer a��l�yor.
		//packet = new char[packetSize];
		for(int i = 0; i <= packetSize; i++)
			packet[i] = 0x00;		

		//paket oldu�u belirtiliyor. de�erler sabit.
		packet[0] =  (char)0xA0;
		packet[1] =  (char)0xA1;
		packet[2] =  (char)0xA2;

		//g�nderen. Master:0x00 , Slave:0x04
		packet[3] = Sender;

		//a�a��da, kar��lanan verinin ilgili fonksiyona iletilmesini sa�layan g�rev kimli�i.
		packet[4] = Task;

		//paket boyutu.
		packet[5] = packetSize;

		//al�c� kimlikleri.  #1:0x06,0x00,0x00 ; #2:0x07,0x00,0x00 ; #3:0x08,0x00,0x00
		for(int i = 6; i <= 8; i++)
			packet[i] = ReceiverID[i - 6];

		//9 ile packetSize-4 aras�ndaki bytelar g�nderilecek verilere tahsis ediliyor.
		for(int i = dataStart; i < (dataStart + DataLength); i++)
			packet[i] = Data[i - 9];
				
		//veri bitlerinin sona erdi�ini belirten sabit.
		packet[dataStart + DataLength] = (char)0xAA;

		//ilk dokuz byte'�n kontrol amac�yla xor'lanmas� ve 24.byte'a atanmas�. 
		xor = packet[8];

		for(int i = 8; i >= 4; i--)
			xor ^= packet[i - 1];

		packet[dataStart + DataLength + 1] = xor;

		//b�t�n byte'lar�n xor'lanmas�.
		xor = packet[dataStart + DataLength + 1];

		for(int i = packetSize - 2; i >= 4; i--)
			xor ^= packet[i - 1];

		packet[dataStart + DataLength + 2] = xor;

		return packet;

	}


	char* getData(char Packet[]) {

		//paketle g�nderilen veri byte'lar�n� d�nd�r�yor.

		char *temp = new char[packetSize-4-dataStart];
		int i;
		for(i = dataStart; i <= (packetSize-4); i++)
			temp[i-dataStart] = Packet[i];
		return temp;
	}



	bool isValid(char Packet[]) {
		char x1 , x2;

		//parite kontrol� yap�yor.

		//ilk dokuz byte xor'lan�yor, pariteyle kar��la�t�r�l�yor.
		x1 = Packet[8];

		for(int i = 8; i >= 1; i--)
			x1 ^= Packet[i-1];
		x1 = !(x1 | Packet[packetSize-2]);

		x2 = Packet[packetSize-2];

		//b�t�n byte'lar xor'lan�yor, pariteyle kar��la�t�r�l�yor.
		for(int i = packetSize-2; i >= 1; i--)
			x1 ^= Packet[i-1];

		x2 = !(x2 | Packet[packetSize-1]);
		
		//iki kontrol�n sonucu da pozitifse "true" d�nd�r�l�yor.
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