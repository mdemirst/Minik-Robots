////////////////////////////////////////////////////////////////
// Project: IslMotorControlConsole
// Function: An interface for communicating with new developed 
//           motor control card with arduino mega board
// Authors: Mahmut Demir - mahmutdemir@gmail.com
// Previous authors: Semih Cayci
// Edit date: 14.04.2015
//////////////////////////////////////////////////////////////// 

#include "interface.cpp"

//seri iletisim nesnesi
CSerial comm;
//CSerial comm2;  //////////////////////////////////////////////

char receiverCard1[3] = { 0x06, 0x00, 0x00 };
char receiverCard2[3] = { 0x07, 0x00, 0x00 };
char receiverCard3[3] = { 0x08, 0x00, 0x00 };

void printMenu(void) {
  printf("\n\t\t<<Make Your Choice and Press Enter>>\n\n");
  printf("\t1. Set motor counter\n");
  printf("\t2. Set motor counters\n");
  printf("\t3. Get motor counter\n");
  printf("\t4. Set speed\n");
  printf("\t5. Set speeds\n");
  printf("\t6. Get speed\n");
  printf("\t7. Set speed PID coefficients\n");
  printf("\t8. Get speed PID coefficients\n");
  printf("\t9. Set position PID coefficients\n");
  printf("\t10. Get position PID coefficients\n");
  printf("\n\t0 . Exit\n\n\nYour Choice : ");

}


int main()
{

  int choice, data1, data2, data3;
  int MotorID, cardNr;
  long distance, distance0, distance1;
  char recCard[3];


  comm.Open(_T("COM5"), 0, 0, false);
  comm.Setup(CSerial::EBaud19200, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
  Sleep(1000); //Wait for serial port to open

  printf("\t\t\tMerhaba!\n");

  system("cls");

  //arayüz nesnesi
  Interface *terminal = new Interface(25, &comm);

  int SP1, SI1, SD1, SP2, SI2, SD2, PP1, PI1, PD1, PP2, PI2, PD2;///M1 ve M2 icin PID katsayilari (position ve speed)
  SP1 = 10;
  SI1 = 0;
  SD1 = 0;
  SP2 = 10;
  SI2 = 0;
  SD2 = 0;
  PP1 = 120;
  PI1 = 0;
  PD1 = 0;
  PP2 = 200;
  PI2 = 0;
  PD2 = 0;
  Sleep(500);

  /*
  (*terminal).setSpeedPIDParameters(receiverCard1,0,SP1,SI1,SD1);///Yatay motor icin
  Sleep(500);
  (*terminal).setSpeedPIDParameters(receiverCard1,1,SP2,SI2,SD2);///Dikey motor icin
  Sleep(500);
  (*terminal).setPositionPIDParameters(receiverCard1,0,PP1,PI1,PD1);
  Sleep(500);
  (*terminal).setPositionPIDParameters(receiverCard1,1,PP2,PI2,PD2);
  Sleep(500);
  */

  printMenu();

  scanf_s("%d", &choice);

  system("cls");

  while (choice != 0) {
    switch (choice) {
      //setMotorCounter
      case 1:
      {

        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which you want to set the motor counter value :  ");
        scanf_s("%d", &MotorID);

        printf("Enter the desired counter value :  ");
        scanf_s("%ld", &distance);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).setMotorCounter(recCard, MotorID, distance);
        else
          printf("Invalid choice.");

        break;
      }

        //SetMotorCounters
      case 2:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the counter value for Motor 0 :  ");
        scanf_s("%ld", &distance0);
        printf("\nEnter the counter value for Motor 1 :  ");
        scanf_s("%ld", &distance1);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        terminal->setMotorCounters(recCard, distance0, distance1);

        break;
      }

        //getMotorCounter
      case 3:
      {
        printf("Enter the card that you want to use (1-to-3) :");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which you want to get the motor counter value  (0-1) :  ");
        scanf_s("%d", &MotorID);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");


        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).getMotorCounter(recCard, MotorID);
        else
          printf("Invalid choice.");


        break;
      }

        //SetSpeed
      case 4:
      {
        printf("Enter the card that you want to use (1-to-3): ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which you want to set speed values (0-1) :  ");
        scanf_s("%d", &MotorID);

        printf("Set speed value (0,65535) :  ");
        scanf_s("%d", &data1);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).setSpeed(recCard, MotorID, data1);
        else
          printf("Invalid choice.");

        break;
      }

        //SetSpeeds
      case 5:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Set speed value for Motor 0 :  ");
        scanf_s("%d", &data1);

        printf("Set speed value for Motor 1 :  ");
        scanf_s("%d", &data2);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        terminal->setSpeeds(recCard, data1, data2);

        break;

      }

        //getSpeed
      case 6:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which you want to ask the speed values (0-1) :  ");
        scanf_s("%d", &MotorID);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).getSpeed(recCard, MotorID);
        else
          printf("Invalid choice.");

        break;
      }

        //setSpeedPIDParameters
      case 7:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which which you want to set speed PID Coefficients (0-1) :  ");
        scanf_s("%d", &MotorID);

        printf("Enter the (Spd) proportional coefficient for motor %d : ", MotorID);
        scanf_s("%d", &data1);

        printf("Enter the (Spd) differential coefficient for motor %d : ", MotorID);
        scanf_s("%d", &data2);

        printf("Enter the (Spd) integral coefficient for motor %d : ", MotorID);
        scanf_s("%d", &data3);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).setSpeedPIDParameters(recCard, MotorID, data1, data2, data3);


        else
          printf("Invalid choice.");

        break;
      }

        //getSpeedPIDParameters
      case 8:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which which you want to get speed PID Coefficients (0-1) :  ");
        scanf_s("%d", &MotorID);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).getSpeedPIDParameters(recCard, MotorID);
        else
          printf("Invalid choice.");
        break;
      }

        //setPositionPIDParameters
      case 9:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which which you want to set position PID Coefficients (0-1) :  ");
        scanf_s("%d", &MotorID);

        printf("Enter the (Pos.) proportional coefficient for motor %d :", MotorID);
        scanf_s("%d", &data1);

        printf("Enter the (Pos.) differential coefficient for motor %d :", MotorID);
        scanf_s("%d", &data2);

        printf("Enter the (Pos.) integral coefficient for motor %d :", MotorID);
        scanf_s("%d", &data3);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).setPositionPIDParameters(recCard, MotorID, data1, data2, data3);
        else
          printf("Invalid choice.");

        break;
      }

        //getPositionPIDParameters
      case 10:
      {
        printf("Enter the card that you want to use (1-to-3) : ");
        scanf_s("%d", &cardNr);

        printf("Enter the motor, of which which you want to get position PID Coefficients (0-1) :  ");
        scanf_s("%d", &MotorID);

        if (cardNr == 1)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard1[i];
        else if (cardNr == 2)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard2[i];
        else if (cardNr == 3)
          for (int i = 0; i < 3; i++)
            recCard[i] = receiverCard3[i];
        else
          printf("Invalid choice.");

        if ((MotorID == 0) || (MotorID == 1))
          (*terminal).getPositionPIDParameters(recCard, MotorID);
        else
          printf("Invalid choice.");

        break;

      }
    }
    
    printMenu();
    scanf_s("%d", &choice);
    system("cls");
  }

  comm.Close();
  return 0;
}