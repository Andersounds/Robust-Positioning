#include <pigpio.h>
#include <iostream>
#include <unistd.h> //For sleep

#ifndef I2C_SLAVE_RBP
#define I2C_SLAVE_RBP


/*
http://abyz.me.uk/rpi/pigpio/pdif2.html#bsc_xfer
http://abyz.me.uk/rpi/pigpio/pdif2.html#bsc_i2c



*/

namespace robustpositioning{

class i2cSlave{
public:
    i2cSlave(int);      //Constructor. Give adress as argument
    ~i2cSlave(void);    //Destructor.
    int isRecieved(void);
    readBuffer(void);
    writeBuffer();
    status ??
private:
    bsc_xfer_t xfer;    //Struct to control data flow
    int address;        //Address of slave device
    int getControlBits(int,bool);//Get bsc_xfer_t control bits for enable/disable i2c read +write with address
};

}

#endif

robustpositioning::i2cSlave::i2cSlave(int address_){
    //Init address
    if(address<0 || address>127){std::cout << "i2cSlave:: invalid address: "<< address<<std::endl;return;}
    address=address_;
    std::cout<< "Initializing GPIO...";
    if(gpioInitialise()<0){
        std::cout<<"\ti2cSlave:: Could not initialize gpio\nFailed."<<std::endl;
    }else{
        std::cout<<"\ti2cSlave:: GPIO initialized\nDone."<<std::endl;
    }
    xfer.control = getControlBits(address,false);
    bscXfer(&xfer);//Make sure that it is closed and disabled before tring to activate it
    xfer.control = getControlBits(slaveAddress, true);
    std::cout << "Initializing i2c slave...";
    int status = bscXfer(&xfer); //Activate again
    if(status<0){
        std::cout<<"\ti2cSlave:: Could not initialize i2c slave.\nFailed. (Error code: " << status << ")"<<std::endl;
    }else{
        std::cout<<"\ti2cSlave:: i2c slave initialized with address " << address << ".\nDone."<<std::endl;
    }
}
robustpositioning::i2cSlave::~i2cSlave(void){
    xfer.control = getControlBits(address,false);
    bscXfer(&xfer);
    std::cout << "i2cSlave:: Closed i2c node" << std::endl;
}

int robustpositioning::i2cSlave::getControlBits(int address /* max 127 */, bool open) {
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
