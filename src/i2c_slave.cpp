#include <pigpio.h>
#include <iostream>
#include <vector>
#include <cstring> //for memcpy
#include <unistd.h> //For sleep
#include <bitset> //For cout binary

#ifndef I2C_SLAVE_RBP
#define I2C_SLAVE_RBP


/*
http://abyz.me.uk/rpi/pigpio/pdif2.html#bsc_xfer
https://raspberrypi.stackexchange.com/questions/76109/raspberry-as-an-i2c-slave
https://robot-electronics.co.uk/i2c-tutorial
*/

namespace robustpositioning{

class i2cSlave{
public:
    i2cSlave(int);      //Constructor. Give adress as argument
    ~i2cSlave(void);    //Destructor.
    //Read, write, clear methods
    int writeBuffer(const uint8_t *msg,int size);       //Base method for writing a char-array to the TX buffer. Returns status
    int writeBuffer(const std::vector<uint8_t>& msg);   //Overloaded version that accepts a c vector instead. returns status
    int readBuffer(uint8_t *msg);                       //Base method for reading buffer. Must be sure that msg can hold all values. Returns size
    int readBuffer(std::vector<uint8_t>&);              //Overloaded version that reads into a vector of bytes. Returns size
    void clearRxBuffer(void);                           //Used when master is repeatedly sending new data. Clear and use next recieved. Essentially read buffer and discard data
    //Status readings
    int RxBufferContentSize(void);//This and the ones below have to be called after a read or write operation is attempted
    int TxBufferContentSize(void);//
    int bytesWrittenToTxBuff(void);//
    bool rxBusy(void);//
    bool txBusy(void);//
    int status;//http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer
//private:
    bsc_xfer_t xfer;    //Struct to control data flow
    int address;        //Address of slave device
    uint32_t ctrlBitsEnable;    //Bit sequence to write to BSC peripheral to enable
    uint32_t ctrlBitsDisable;   //Bit sequence to write to BSC peripheral to disable
};

//Special class inherited from i2cSlave, with custom endoce/decode functions for robustpositioning-system
class i2cSlave_decode: public i2cSlave{
public:
    i2cSlave_decode(int address_);
    ~i2cSlave_decode(void);
    void emptyRxBuffer(void);
    int readAndDecodeBuffer(std::vector<float>&);
    int writeAndEncodeBuffer(const std::vector<float>&);     //Special case method for encoding info-x-y-z-yaw - values and writing them to buffer
    int encodeScale;// The scale to use when converting the floats to ints. floor(float*scale)
    int msgSizeRX;    //Number of floats that are in each payload. Excluding the leading 8 bit info byte
    int msgSizeTX;
private:
    std::vector<float> scales;
};
}
#endif

// Methods for base i2c slave class
robustpositioning::i2cSlave::i2cSlave(int address_){
    //Init address
    if(address_<0 || address_>127){std::cout << "i2cSlave:: invalid address: "<< address_<<std::endl;return;}
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
    xfer.control = ctrlBitsEnable;
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
    std::cout << "Terminated GPIOs."<< std::endl;;
}
int robustpositioning::i2cSlave::writeBuffer(const uint8_t *msg,int size){
//    std::memcpy(&xfer.txBuf,&msg,size);      //Copy the specified memory section to txBuf
    std::memcpy(&xfer.txBuf, msg,size); //Did I pass pointer before? wrong?
    xfer.txCnt = size;                  //Specify to low level interface size of memory section
    status = bscXfer(&xfer);            //Hand data over to BSC periphial
    return bytesWrittenToTxBuff();      //Return amount of successfully copied bytes
}
int robustpositioning::i2cSlave::writeBuffer(const std::vector<uint8_t>& msg){
    int size = msg.size();
    return writeBuffer(msg.data(),size);//Give pointer to vector first element address and size of msg
}
int robustpositioning::i2cSlave::readBuffer(uint8_t *msg){
    status = bscXfer(&xfer);//Load data from BSC periphial
    int rxBufSize = xfer.rxCnt;//Read how much there is to be read in rx buff
    std::memcpy(msg,&xfer.rxBuf,rxBufSize);
    return rxBufSize;
}
int robustpositioning::i2cSlave::readBuffer(std::vector<uint8_t>& msg){
    uint8_t msg_c_array[16];//Maximum size of buffer is 16 bytes
    msg.clear();
    int size = readBuffer(msg_c_array);
    for(int i=0;i<size;i++){
        msg.push_back(msg_c_array[i]);
    }
    return size;
}
void robustpositioning::i2cSlave::clearRxBuffer(void){
    uint8_t msg_c_array[16];
    readBuffer(msg_c_array);
}
int robustpositioning::i2cSlave::bytesWrittenToTxBuff(void){
    return (status>>16)&0b11111;
}
int robustpositioning::i2cSlave::RxBufferContentSize(void){
    return xfer.rxCnt;
}
int robustpositioning::i2cSlave::TxBufferContentSize(void){
    return xfer.txCnt;
}

//Constructor If this does not work then rewrite constructor Then also bsc_xfer_t xfer needs tp be made public
robustpositioning::i2cSlave_decode::i2cSlave_decode(int address_):i2cSlave(address_){
    scales = std::vector<float>{1,10,100,1000};//Specify the scaling constants
    encodeScale  = 2;//May give this as argument in overloaded writeAndEncode

}
//Destructor
robustpositioning::i2cSlave_decode::~i2cSlave_decode(void){
    xfer.control = ctrlBitsDisable;
    bscXfer(&xfer);
    std::cout << "i2cSlave:: Closed i2c node" << std::endl;
    gpioTerminate();
    std::cout << "Terminated GPIOs."<< std::endl;;
}
//Additional methods for special case class
int robustpositioning::i2cSlave_decode::readAndDecodeBuffer(std::vector<float>& values){
    uint8_t rxbuffer[16];             //Create array able to hold whole buffer
    int rxSize = readBuffer(rxbuffer);//Read rx buffer
    int infoByte = -1;
    for(int i=0;i<rxSize;i++){        //Find the first info byte
        if(rxbuffer[i]&0x01){
	    infoByte = i;
	    break;//If the first info byte is found then use that
       }
    }
    std::cout << "Info byte index: " << infoByte << std::endl;
    if(infoByte<0){return -1;}//If no infobyte has been found, return
    values.clear();
    int sgnByte   = infoByte+1; //Sign-byte is the one after infobyte always
    int startByte = infoByte+2; //First datafield
    int readableBytes = rxSize-infoByte-2;//Number of databytes
    int floatNmbr = 0;          //Number of the decoded float
    int decodeScale = (int)(rxbuffer[infoByte]>>1)&0b11;
    //std::cout << "Decode scale: " << decodeScale << ": " << scales[decodeScale] << std::endl;
    //std::cout << "Read bytes: " << rxSize << std::endl;
    for(int i=startByte;i<rxSize-1;i+=2){//go through all complete data-pairs. if there is an odd number of data fields then skip the last one
        int value_abs_int = (int)((rxbuffer[i]<<6)|(rxbuffer[i+1]>>1));//build HB and LB to a int
        if((rxbuffer[sgnByte]>>(floatNmbr+1))&0b1){
            value_abs_int*=(-1);//value is negative
        }
        float value = (float)value_abs_int / scales[decodeScale];

        values.push_back(value);
        floatNmbr++;
    }
    return values.size();
}
int robustpositioning::i2cSlave_decode::writeAndEncodeBuffer(const std::vector<float>& values){
    uint8_t txbuffer[16];
    uint8_t info = (encodeScale<<1)|1;//Info byte has LSB=1
    info|= ((uint8_t)values.size())<<3;//encode message length
    uint8_t sign = 0;
    int size = values.size()*2+2;//two bytes for every value, one info byte and one sign byte
    int i = 2;//Start set data at index 2

    for(float value:values){
        uint16_t unsignedScaledValue = (uint16_t) abs( (int)(value*scales[encodeScale]) );//
        std::cout << "unsigned scaled value: " << unsignedScaledValue << std::endl;
        uint8_t HB = (unsignedScaledValue>>6)&0xFE; //use 7 bits and set LSB to 0
        uint8_t LB = (unsignedScaledValue<<1)&0xFE; //Shift one bit and make sure LSB is 0
	std::cout << "HB: " <<(int)HB << ", LB: " << (int)LB << std::endl;
        sign |= ( (value<0)<<(i+1) ); //Mask in the sign bit in the sign byte
        txbuffer[i] = HB;
        txbuffer[i+1] = LB;
        i+=2;
    }
    txbuffer[1] = sign;
    txbuffer[0] = info;

    for(int j=0;j<8;j++){
        std::cout << "BIT " << j << ": " <<std::bitset<8>( txbuffer[j]) << std::endl;
    }


    return writeBuffer(txbuffer,size);
}

/* This method is used to read and discard all contents of the RX buffer in order to recieve new values
 */
void robustpositioning::i2cSlave_decode::emptyRxBuffer(void){
    std::vector<float> discardValues;
    readAndDecodeBuffer(discardValues);
}

/*
//Info byte
07 06 05 04 03 02 01 00
D  D  A  A  A  S  S  ID
//Sign byte
07 06 05 04 03 02 01 00
S7 S6 S5 S4 S3 S2 S1 ID
//Data bytes
07 06 05 04 03 02 01 00
        <data>       ID
//Legend
ID:     bit identifying the info-byte. 1: info byte, 0: data or sign byte
S:      bits encoding the scale of floats. 00: 1, 01:10, 10:100, 11:1000
A:      bits encoding the message length. 1-6 bytes.
D:      bits identifying which message it is (0-3). Can specify before that 0: 3 floats in certain order, 1: 2 ints etc.
SX:     Bit identifying the sign of float X. 1: neg, 0: pos

Each float is encoded as 7 high bits and 7 low bits in that order.


DD message number specification:
bin     dec     description
00       0
                byte 0: info byte
                byte 1: sign byte
                byte 2: lidar dist HB
                byte 3: lidar dist LB
                byte 4: height est HB
                byte 5: height est LB
                byte 6: pitch (rad) HB
                byte 7: pitch (rad) LB
                byte 8: roll (rad) HB
                byte 9: roll (rad) LB
01       1
10       2
11       3
*/
