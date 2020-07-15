/*  Note 1: The correct way would be to use VCC(+5) at 6,5 VDC during programming phase
 *          I'm not doing it here to have the circuit as simple as possible
 *  Note 2: The process to read the electronic signature depends on forcing the A9 to 12,5V, for this 
 *          reason I recommend to use a switch inverter, where the middle pin (C) goes directly to the A9 of EPROM and
 *          the other two pins: (NO)one goes to the Opto controlled by pin 10 of 2560(status read ES), the
 *          other pin(NC), goes to the port responsible for the MSB address - pin(A9)
 *          This allows to easely move bettewen the status to read data or read electronic signature.
 *  Note 3: I use the Mega because this allows to use 2 or 3 ports completly as the Addres BUS,
 *          Offcourse it is possible to use only one port of 8bits complemented withe one or two registers
 *          to keep the address bytes, with the help of 2 or 3 pins to control the registers, this
 *          process allows the use of Arduino Uno instead of Mega.
 *  Note 4: It is possible to extend the address BUS adding more ports.
 *  Note 5: It is possible to work with data length of 16 bits adding more ports to the Data BUS
 *  Note 6: This project is working with the Vpp value of 12,5 but it is possible to use the 21V to Vpp.
 *  Note 7: Used three opto isolators to protect the ports of Mega from any error connection involving Vpp
 *          If you feel secure, you don't need to use the opto isolators.
 *  Note 8: Pay attention to the circuit wich control of the Vpp, use a 2n2222(NPN) and a 2n2907(PNP) this
 *          allows to have control over current and voltage of Vpp.
 *  Note 9: You can use a variable as input from "monitor IDE" with the values of address, exp: start addr: 0000, end addr: 8FFF, 
 *          and with this input you will be able to use a small EPROM or a bigger one.
 *  Note 10: If necessary, you can resize the area of EPROM written, it is possible adjust the block address you 
 *          want to write, this allow to use the EPROM to keep blocks of code and is possible to call a specific block to 
 *          run on your project.        
 */

#include <SPI.h>
#include <SD.h>

//Running on Mega
//Miso 52
//Mosi 51
//SCK 50
//CS 4

// DDRA = DATA  pins 22 - 29                    // This pins represent the data bus of EPROM
// DDRL = ADDRESS LSB pins 49 - 42  (inverted)  // This pins represent the address bus less significative byte of EPROM
// DDRC = ADDRESS MSB pins 37 - 30  (inverted)  // This pins represent the address bus more significative byte of EPROM

//pin 7 - OUTPUT ENABLE // Pin to controle the OE  // Also called as /G, active the data to output port of D0-D7
//pin 8 - CHIP ENABLE   // Pin to controle the CE  // Also called as /E, active the chip selection.
//pin 9 - VPP           // Pin to controle the voltage pulse program(Vpp)

#define OE 7   // The OE sometimes is marked as /G and means Output Enable Pin nº 22 of EPROM 27C256B
#define CE 8   // The CE sometimes is marked as /E and means Chip Enable Pin nº 20 of EPROM 27C256B
#define VPP 9  // Voltage program, to the M27C256B of "ST" is referenced as 12,75 +/- 0.25 Pin nº 1

#define ELECTRONIC_SIGNATURE 10  //This force pin A9 of address BUS, to stay with 12,75V to allow read the electronics signature.
#define VPP_5V 11  // This force 5V at pin of Vpp

const unsigned int timeDelay = 1;

File myFile;
unsigned char portData;
unsigned long register32 = 0b11111111;  //Hex value of 32 bits FF FF FF FF
unsigned long addr = 0; // 32Bits
unsigned char bytePosition[4] = {'0'};
short checkSum;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  pinMode(OE, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(VPP, OUTPUT);
  pinMode(ELECTRONIC_SIGNATURE, OUTPUT);
  pinMode(VPP_5V, OUTPUT);

  digitalWrite(OE, HIGH);
  digitalWrite(CE, HIGH);
  digitalWrite(VPP, LOW);
  digitalWrite(ELECTRONIC_SIGNATURE, LOW);
  digitalWrite(VPP_5V, LOW);

  DDRA = 0xFF;
  PORTA = 0xFF;

  DDRL = 0xFF;
  PORTL = 0xFF;

  DDRC = 0xFF;
  PORTC = 0xFF;

  Serial.println (" MENU:");
  Serial.println (" k: checksum of SOCKET device");
  Serial.println (" d: del file test_wr.txt");
  Serial.println (" p: print file test_wr.txt");
  Serial.println (" s: get electronic signature");
  Serial.println (" f: checksum of file WR");
  Serial.println (" e: checksum of file RD");
  Serial.println (" r: read device to file test_rd.txt");
  Serial.println (" w: write text_wr.txt to device");

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test_wr.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Opened test_wr.txt...");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test_wr.txt");
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
        
            if (c == 'd') // delete file
            {
              Serial.println("Removing test_wr.txt");
              SD.remove("test_wr.txt");
            }
        
                          if (c == 'p'){ // Show print file in the Serial Monitor
                            Serial.println("Reading test_wr.txt");
                      
                            // open the file for reading:
                            myFile = SD.open("test_wr.txt");
                                if (myFile) {
                                  // read from the file until there's nothing else in it:
                                  while (myFile.available()) {
                                    Serial.write(myFile.read());
                                  }
                                  // close the file:
                                  myFile.close();
                                  } else {
                                    // if the file didn't open, print an error:
                                    Serial.println("error opening test.txt");
                                  }
                          }
         
            if (c == 'r'){
              ReadToSD();
            }
            if (c == 'w'){
              WriteToEPROM();
            }
            if(c=='s'){
              GetSignature();
            }
            if(c=='k'){
              GetCheckSum();
            }
            if(c=='f'){
              GetCheckSumFileWR();
            }
             if(c=='e'){
              GetCheckSumFileRD();
            }
  }
}

void GetSignature() {
  DDRA = 0x00; // set PORTA as INPUT
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0

  addr=0;
  SD.remove("signature.txt");
  myFile = SD.open("signature.txt", FILE_WRITE);

  Serial.println("Getting signature");

   digitalWrite(CE, LOW);
    delayMicroseconds(5);
    digitalWrite(OE, LOW);
    delayMicroseconds(5);

  digitalWrite(ELECTRONIC_SIGNATURE, HIGH);  //Apply 12V to addr 9
  digitalWrite(VPP_5V, HIGH);  //Apply 5V to pin of VPP, is a condition to read electronics signature of manufacturer ST

  while (addr < 0x0002){
    bytePosition[1] = (addr & 0x0000ff00UL) >>  8;
    bytePosition[0] = (addr & 0x000000ffUL) >>  0;
    
    PORTL = bytePosition[0];
    PORTC = bytePosition[1];
    delay(4);  //This delay is important to be able of read electronic signature!
    portData = PINA;

    if (!addr) //first reading
      {Serial.print("Manufacturer: "); Serial.println(portData, HEX);}
    else  //second reading
     { Serial.print("Device: "); Serial.println(portData, HEX);}

    myFile.write(portData);
    addr++;

    delay(timeDelay);
  }

  digitalWrite(ELECTRONIC_SIGNATURE, LOW);
  digitalWrite(VPP_5V, LOW);

     digitalWrite(CE, HIGH);
    delayMicroseconds(5);
    digitalWrite(OE, HIGH);

  Serial.println("Done.");

  DDRA = 0xFF;
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0
}

void ReadToSD(){
  DDRA = 0x00; // set PORTA as INPUT
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0

  addr=0;
  SD.remove("test.txt");
  myFile = SD.open("test_rd.txt", FILE_WRITE);

  Serial.println("Starting to read EPROM");

  while (addr < 0x8000){

    bytePosition[1] = (addr & 0x0000ff00UL) >>  8;
    bytePosition[0] = (addr & 0x000000ffUL) >>  0;

    PORTL = bytePosition[0];
    PORTC = bytePosition[1];

    digitalWrite(CE, LOW);
    delayMicroseconds(5);
    digitalWrite(OE, LOW);
    delayMicroseconds(5);

    portData = PINA;

    digitalWrite(CE, HIGH);
    delayMicroseconds(5);
    digitalWrite(OE, HIGH);

    // Serial.print(portData);

    myFile.write(portData);
    addr++;

    delayMicroseconds(5);
  }

  Serial.println("Done.");

  DDRA = 0xFF;
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0
}

void WriteToEPROM(){
  DDRA  = 0xFF;  //set PORTA as OUTPUT
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0

  addr=0;
  myFile = SD.open("test_wr.txt", FILE_READ);

  Serial.println("Starting to write to EPROM");

  digitalWrite(VPP, HIGH);
  digitalWrite(OE, HIGH);  // corresponding to the pin /G wich enable the output pins of data.
  while (addr < 0x8000){
    portData = myFile.read();  // Byte read from file and will be writted to EPROM
    //Serial.print(addr, HEX); Serial.print(" : "), Serial.println(portData, HEX);
    bytePosition[1] = (addr & 0x0000ff00UL) >>  8; // High significative address
    bytePosition[0] = (addr & 0x000000ffUL) >>  0; // Low significative address

    PORTL = bytePosition[0];
    PORTC = bytePosition[1];

    PORTA = portData;
    delayMicroseconds(45);
    digitalWrite(CE, LOW);  // To program only is use the CE, the pin OE stay High!
    delayMicroseconds(100);  // Atmel time is 95 uSec to 105 uSec
    digitalWrite(CE, HIGH);
    delayMicroseconds(50);
    
    //Implement here the verify os data writed, to do it, let CE HIGH and put OE LOW. Compare data
    //Vpp can be keep with same status to read and compare, see waveforms.
    addr++; 
  }

  digitalWrite(VPP, LOW);
  Serial.println("Done.");
}
void GetCheckSumFileRD(){
  unsigned long checkSumResult = 0x0000;
  unsigned char chkSum[4] = {'0'};
  unsigned char dataWord= 0x00;
  addr=0;
  myFile = SD.open("test_rd.txt", FILE_READ);

  Serial.print("Starting to calculate checkSum :"); Serial.println(myFile);

  while (addr < 0x8000){
   dataWord = myFile.read(); 
   checkSumResult += dataWord;
   addr++;
  }
   myFile.close();
   
  chkSum[0] = (checkSumResult & 0x000000ffUL) >>   0;
  chkSum[1] = (checkSumResult & 0x0000ff00UL) >>   8;
  chkSum[3] = (checkSumResult & 0x00ff0000UL) >>  12;
  chkSum[4] = (checkSumResult & 0xff000000UL) >>  16;
  
 Serial.print("CheckSum Ext. of file test_rd.txt: "); Serial.println(checkSumResult, HEX);
 Serial.print("CheckSum of file test_rd.txt: "); Serial.print(chkSum[1], HEX); Serial.println(chkSum[0], HEX); 
}
void GetCheckSumFileWR(){
  unsigned long checkSumResult = 0x0000;
  unsigned char chkSum[4] = {'0'};
  unsigned char dataWord= 0x00;
  addr=0;
  myFile = SD.open("test_wr.txt", FILE_READ);

  Serial.print("Starting calculating checkSum :"); Serial.println(myFile);

  while (addr < 0x8000){
   dataWord = myFile.read(); 
   checkSumResult += dataWord;
   addr++;
  }
   myFile.close();
   
  chkSum[0] = (checkSumResult & 0x000000ffUL) >>   0;
  chkSum[1] = (checkSumResult & 0x0000ff00UL) >>   8;
  chkSum[3] = (checkSumResult & 0x00ff0000UL) >>  12;
  chkSum[4] = (checkSumResult & 0xff000000UL) >>  16;
  
 Serial.print("CheckSum Ext. of file test_wr.txt: "); Serial.println(checkSumResult, HEX);
 Serial.print("CheckSum of file test_wr.txt: "); Serial.print(chkSum[1], HEX); Serial.println(chkSum[0], HEX); 
}

void GetCheckSum(){
  unsigned long checkSumResult = 0x0000;
  unsigned char chkSum[4] = {'0'};
  DDRA = 0x00; // set PORTA as INPUT
  PORTL = 0x00; //address PORT starting at 0
  PORTC = 0x00; //address PORT starting at 0
  addr=0;
  
  Serial.println("Calcul of CheckSum: DEVICE");

  while (addr < 0x8000){
    
    bytePosition[1] = (addr & 0x0000ff00UL) >>  8;
    bytePosition[0] = (addr & 0x000000ffUL) >>  0;

    PORTL = bytePosition[0];
    PORTC = bytePosition[1];
    
    digitalWrite(CE, LOW);
    delayMicroseconds(10);
    digitalWrite(OE, LOW);
    delayMicroseconds(10);

    portData = PINA;

    digitalWrite(CE, HIGH);
    digitalWrite(OE, HIGH);
    checkSumResult += portData;
    addr++;

    delayMicroseconds(5);
  }

  chkSum[0] = (checkSumResult & 0x000000ffUL) >>   0;
  chkSum[1] = (checkSumResult & 0x0000ff00UL) >>   8;
  chkSum[3] = (checkSumResult & 0x00ff0000UL) >>  12;
  chkSum[4] = (checkSumResult & 0xff000000UL) >>  16;
 Serial.print("CheckSum of DEVICE: "); Serial.println(checkSumResult, HEX);
 Serial.print("CheckSum of DEVICE: "); Serial.print(chkSum[1], HEX); Serial.println(chkSum[0], HEX);
}
