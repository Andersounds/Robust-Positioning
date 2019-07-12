#include <pigpio.h>
#include <iostream>
#include <unistd.h> //For sleep
#include "../src/i2c_slave.cpp"

// give  a 1 enables. give a 0 closes gpios
// use cin to do clean exit
void closeSlave() {
    gpioInitialise();
    std::cout << "Initialized GPIOs\n";
    bsc_xfer_t xfer; // Struct to control data flow
    xfer.control = 0;
    bscXfer(&xfer);
    std::cout << "Closed slave.\n";
    gpioTerminate();
    std::cout << "Terminated GPIOs." << std::endl;
}



int main(int argc, char** argv){
// Parse arguments
    if(argc!=2){
        std::cout << "Give -one- argument"<< std::endl;
        std::cout << "1: enable gpio, i2c on BSC peripheral and run example" << std::endl;
        std::cout << "0: disable gpio and i2c on BSC peripheral" << std::endl;
        return 0;
    }else{
        if((*argv[1] != '0') || *argv[1]!='1'){
            std::cout << "Valid arguments:" << std::endl;
            std::cout << "1: enable gpio, i2c on BSC peripheral and run example" << std::endl;
            std::cout << "0: disable gpio and i2c on BSC peripheral" << std::endl;
            return 0;
        }
    }
    if(argv[1]==0){
        closeSlave();
        return 0;
    }

    //initialize i2c slave object with the inherited encode/decode class
    robustpositioning::i2cSlave_decode i2cComm(0x03);
    std::vector<float> values;
    while(1){
        if(i2cComm.readAndDecodeBuffer(values)>0){
            std::cout << "Recieved " << i2cComm.RxBufferContentSize() <<"decoded floats" << std::endl;
        }else{std::cout << "No available data in rx buffer" << std::endl;}
        usleep(3000000);
        int writtenBytes = i2cComm.writeAndEncodeBuffer(values);
        if(writtenBytes>0){
            std::cout << "Wrote " << writtenBytes << " bytes to tx buffer" << std::endl;
        }else{ std::cout << "Could not write to tx buffer " << std::endl;}
        usleep(3000000);
    }

}
