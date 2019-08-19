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
// Parse argument
    if(argc!=2){
        std::cout << "Give -one- argument"<< std::endl;
        std::cout << "1: enable gpio, i2c on BSC peripheral and run example" << std::endl;
        std::cout << "0: disable gpio and i2c on BSC peripheral" << std::endl;
        return 0;
    }else{
	std::string arg = argv[1];
	std::cout <<"Given argument: "<< arg << std::endl;
        if((arg != "0") && arg!="1"){
            std::cout << "Valid arguments:" << std::endl;
            std::cout << "1: enable gpio, i2c on BSC peripheral and run example" << std::endl;
            std::cout << "0: disable gpio and i2c on BSC peripheral" << std::endl;
            return 0;
        }
    	if(arg=="0"){
        	closeSlave();
        	return 0;
   	 }
    }
    //initialize i2c slave object with the inherited encode/decode class
    const int slaveAddress = 0x04;
    robustpositioning::i2cSlave_decode i2cComm(slaveAddress);
    std::vector<float> values{0,0,0,0};
    std::vector<float> valuesTX{1.3,2.5,0.8};
    while(1){
        int recievedFloats = i2cComm.readAndDecodeBuffer(values);
        if(recievedFloats>0){
            std::cout << "Recieved " << recievedFloats <<" decoded floats" << std::endl;
            for(float i:values){
                std::cout << i << ", ";
            }
            std::cout << std::endl;
        }else{std::cout << "No available data in rx buffer" << std::endl;}
        /*usleep(3000000);
        int writtenBytes = i2cComm.writeAndEncodeBuffer(valuesTX);
        if(writtenBytes>0){
            std::cout << "Wrote " << writtenBytes << " bytes to tx buffer" << std::endl;
        }else{ std::cout << "Could not write to tx buffer " << std::endl;
                std::cout << "Tx buf content size: " <<i2cComm.TxBufferContentSize() << std::endl;}
        */
        usleep(3000000);
    }

}
