#include <pigpio.h>
#include <iostream>
#include <unistd.h> //For sleep

#ifndef I2C_SLAVE_RBP
#define I2C_SLAVE_RBP


/*
http://abyz.me.uk/rpi/pigpio/pdif2.html#bsc_xfer
http://abyz.me.uk/rpi/pigpio/pdif2.html#bsc_i2c
https://robot-electronics.co.uk/i2c-tutorial

There are two functions implemented for slave-i2c for the rpi
    int status = bscXfer(&xfer);            //This one seems to be the core function in c
    int status = bsc_i2c(address, &xfer);   //This one seems to be a wrapper for python?


*/

namespace robustpositioning{

class i2cSlave{
public:
    i2cSlave(int);      //Constructor. Give adress as argument
    ~i2cSlave(void);    //Destructor.
    int RxBufferContentSize(void);
    void clearRxBuffer(void);        //Used when master is repeatedly sending new data. Clear and use next recieved
    int readBuffer(char[]);          //Base method for reading the RX buffer
    int writeBuffer(&);              //Base method for writing a char-array to the TX buffer. Returns number of successfully written bytes


    status ??
private:
    bsc_xfer_t xfer;    //Struct to control data flow
    int address;        //Address of slave device
    int ctrlBitsEnable;
    int ctrlBitsDisable;
};

//Struct to easily convert buffer

//Special class inherited from i2cSlave, with custom endoce/decode functions for robustpositioning-system
class i2cSlave_decode:i2cSlave{
public:
    int readAndDecodeBuffer(float[4]);
    int writeAndEncodeBuffer();     //Special case method for encoding info-x-y-z-yaw - values and writing them to buffer
private:

};
}
#endif
robustpositioning::i2cSlave::i2cSlave(int address_){
    //Init address
    if(address<0 || address>127){std::cout << "i2cSlave:: invalid address: "<< address<<std::endl;return;}
    address=address_;
    std::cout<< "Initializing GPIO...";
    if(gpioInitialise()<0){
        std::cout<<"Failed.\n\ti2cSlave:: Could not initialize gpio"<<std::endl;
        return;
    }else{
        std::cout<<"Done.\n\ti2cSlave:: GPIO initialized"<<std::endl;
    }
    ctrlBitsEnable  = (address<<16)|0x305;//Enable transmit, recieve as i2c, and enable BSC periphial
    ctrlBitsDisable = address<<16; //Just disable
    xfer.control = ctrlBitsDisable;//Only matter that the second argument is false, because...
    bscXfer(&xfer);                       //... we just make sure that it is closed and disabled before tring to activate it
    xfer.control = ctrlBitsEnable
    std::cout << "Initializing i2c slave...";
    int status = bscXfer(&xfer); //Activate again
    if(status<0){
        std::cout<<"Failed.\n\ti2cSlave:: Could not initialize i2c slave. (Error code: " << status << ")"<<std::endl;
    }else{
        std::cout<<"Done.\n\ti2cSlave:: i2c slave initialized with address " << address << "."<<std::endl;
    }
}
robustpositioning::i2cSlave::~i2cSlave(void){
    xfer.control = ctrlBitsDisable;
    bscXfer(&xfer);
    std::cout << "i2cSlave:: Closed i2c node" << std::endl;
    gpioTerminate();
    cout << "Terminated GPIOs."<< std::endl;;
}
int robustpositioning::i2cSlave::RxBufferContentSize(void){
    return xfer.rxCnt;
}


    /*Control bits definition:

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
