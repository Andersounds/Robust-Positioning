#include <pigpio.h>
#include <iostream>
#include <unistd.h> //For sleep
#include <cstring> //for memcpy
#include <bitset> //For cout binary
using namespace std;

void runSlave();
void closeSlave();
int getControlBits(int, bool);

const int slaveAddress = 0x03; // <-- Your address of choice
bsc_xfer_t xfer; // Struct to control data flow

int main(){
    // Chose one of those two lines (comment the other out):
    runSlave();
    //closeSlave();

    return 0;
}

void runSlave() {
cout << "Initializing gpio..."<< endl;
if (gpioInitialise() < 0)
{
   cout << "Could not initialize gpio" << endl;
}
else
{
    cout << "Initialized GPIOs" << endl;
}

    // Close old device (if any)
    xfer.control = getControlBits(slaveAddress, false); // To avoid conflicts when restarting
    bscXfer(&xfer);
    // Set I2C slave Address to 0x0A
    xfer.control = getControlBits(slaveAddress, true);
    int status = bscXfer(&xfer); // Should now be visible in I2C-Scanners


//Test transmit buffer
bool testTX =false;
    uint8_t i = 1;
    while(testTX){
        uint8_t data[i] = {(uint8_t)i,(uint8_t)i+1};
	memcpy(&xfer.txBuf,&data,2);
        //xfer.txBuf = data;
        xfer.txCnt = 2;
        std::cout << "Wrote 2x " <<(int) i << " in tx buffer" << std::endl;
       int status_return = bscXfer(&xfer);
	std::cout << "Status: "<< std::bitset<30>(status_return) << std::endl;
	int secsleep = 2;
        for(int delay = 1;delay<secsleep;delay++){
            std::cout << "Sleeping " <<(int)i << "/"<< secsleep << "seconds." << std::endl;
            usleep(10000);
        }

        i++;
    }
bool testRX =true;
    while(testRX){
    i=1;
    uint8_t data[16]; //Make an array of 16 bytes to be sure that the buffer fits
	//memcpy(&xfer.txBuf,&data,2);
    int status_return = bscXfer(&xfer);//See what happened
    int rxBufSize = xfer.rxCnt;//Read how much there is to be read in rx buff
    if(rxBufSize>0){
        std::cout << "There are " << rxBufSize << " available bytes in rx buffer...";
        memcpy(&data,&xfer.rxBuf,rxBufSize);
        std::cout << "Read: ";
        for(int j=0;j<rxBufSize;j++){
            std::cout << std::bitset<8>(data[j]) << ", ";
        }
        std::cout << std::endl;
    }else{
        std::cout << "No available data in rx buf." << std::endl;
    }
    //int status_return = bscXfer(&xfer);
	//std::cout << "Status: "<< std::bitset<30>(status_return) << std::endl;
	int secsleep = 10;
        for(int delay = 1;delay<secsleep;delay++){
            std::cout << "Sleeping " <<(int)i << "/"<< secsleep << "seconds." << std::endl;
            usleep(1000000);
        }
        i++;
    }


    if (status >= 0)
    {
        cout << "Opened slave\n";
        xfer.rxCnt = 0;
        while(1){
            bscXfer(&xfer);
            if(xfer.rxCnt > 0) {
                cout << "Received " << xfer.rxCnt << " bytes: ";
  //              for(int i = 0; i < xfer.rxCnt; i++){
//		    uint8_t byt = xfer.rxBuf[i];
		    //cout <<"Byte "<< i << ": " <<  byt << endl;
                   // cout <<"Byte "<< i << ": " <<  xfer.rxBuf[i] << endl;
//                }
		cout << "Buffer: " << xfer.rxBuf << endl;//show buffer
		cout << "Buffer size: " << xfer.rxCnt <<endl; //
//		xfer.rxBuf = 0;//Clear buffer
		//for(int i=0;i<xfer.rxCnt;i++){
		  //  xfer.rxBuf[i] = 0;
		//}
		//memset(&xfer.rxBuf[0], 0, sizeof(xfer.rxBuf));//Clear buffer
            }
	    usleep(5000000);
	}
     }else{
         cout << "Failed to open slave!!!\n";
     }
}

void closeSlave() {
    gpioInitialise();
    cout << "Initialized GPIOs\n";

    xfer.control = getControlBits(slaveAddress, false);
    bscXfer(&xfer);
    cout << "Closed slave.\n";

    gpioTerminate();
    cout << "Terminated GPIOs.\n";
}


int getControlBits(int address /* max 127 */, bool open) {
    /*
    Excerpt from http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer regarding the control bits:

    22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    a  a  a  a  a  a  a  -  -  IT HC TF IR RE TE BK EC ES PL PH I2 SP EN

    Bits 0-13 are copied unchanged to the BSC CR register. See pages 163-165 of the Broadcom
    peripherals document for full details.

    aaaaaaa defines the I2C slave address (only relevant in I2C mode)
    IT  invert transmit status flags
    HC  enable host control
    TF  enable test FIFO
    IR  invert receive status flags
    RE  enable receive
    TE  enable transmit
    BK  abort operation and clear FIFOs
    EC  send control register as first I2C byte
    ES  send status register as first I2C byte
    PL  set SPI polarity high
    PH  set SPI phase high
    I2  enable I2C mode
    SP  enable SPI mode
    EN  enable BSC peripheral
    */

    // Flags like this: 0b/*IT:*/0/*HC:*/0/*TF:*/0/*IR:*/0/*RE:*/0/*TE:*/0/*BK:*/0/*EC:*/0/*ES:*/0/*PL:*/0/*PH:*/0/*I2:*/0/*SP:*/0/*EN:*/0;

    int flags;
    if(open)
        flags = /*RE:*/ (1 << 9) | /*TE:*/ (1 << 8) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);
    else // Close/Abort
        flags = /*BK:*/ (1 << 7) | /*I2:*/ (0 << 2) | /*EN:*/ (0 << 0);

    return (address << 16 /*= to the start of significant bits*/) | flags;
}
