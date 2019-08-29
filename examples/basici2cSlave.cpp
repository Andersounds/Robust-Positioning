#include <pigpio.h>
#include <iostream>
#include <unistd.h> //For sleep
#include "../src/timeStamp.cpp" //For timestamp
#include "../src/i2c_slave.cpp"


int main(int argc, char** argv){
timestamp::timeStamp_ms stamp; //initialize timestamper
float timestamp_data = 0;
// Parse argument
    int lapCounter = 0;
    int lapMax = 0;
    if(argc!=2){
        std::cout << "Give -one- argument"<< std::endl;
        std::cout << "-1: enable gpio, i2c on BSC peripheral and run example indefinitely until ^c" << std::endl;
        std::cout << " 0: disable gpio and i2c on BSC peripheral" << std::endl;
	std::cout << " <non neg number>" << "Run the specified amount of laps and then return. disable BSC with destructor. "<< std::endl;
        return 0;
    }else{
	std::string arg = argv[1];
	std::cout <<"Given argument: "<< arg << std::endl;
	try{
            lapMax = stoi(arg);
        }catch(const std::invalid_argument& ia){
            std::cout << "Gave invalid number of laps to run: " << arg << std::endl;
            return 0;
        }
    }
    //initialize i2c slave object with the inherited encode/decode class
    const int slaveAddress = 0x04;
    robustpositioning::i2cSlave_decode i2cComm(slaveAddress);
    std::vector<float> values{0,0,0,0,0};
    std::vector<float> valuesTX{1.3,2.5,0.8};
    while(lapMax < 0 || lapCounter<lapMax){
	lapCounter++;
	i2cComm.clearRxBuffer();
	stamp.get(timestamp_data); //Get new timestamp
	usleep(25000);//wait approx 25 ms to simulate image collect and log
	int watchdog = 0;
	int recv_amount = i2cComm.readAndDecodeBuffer(values);//-1;
	while(recv_amount<0 && watchdog < 500){
	    stamp.get(timestamp_data);//Get new timestamp
	    usleep(1000);
	    recv_amount = i2cComm.readAndDecodeBuffer(values);
	    watchdog ++;
	}
	if(recv_amount>0){
            //std::cout <<lapCounter <<". WD: "<< watchdog << ". Recieved " << recv_amount <<" decoded floats. roll: "<< values[1]<< ", batt: " << values[4] << std::endl;
        	std::cout << timestamp_data << ", " << values[0] << ", " << values[1] << std::endl;
	}else{std::cout << "No available data in rx buffer" << std::endl;}
        usleep(3000);//Pause a bit more to simulate data logging
        /* int writtenBytes = i2cComm.writeAndEncodeBuffer(valuesTX);
        if(writtenBytes>0){
            std::cout << "Wrote " << writtenBytes << " bytes to tx buffer" << std::endl;
        }else{ std::cout << "Could not write to tx buffer " << std::endl;
                std::cout << "Tx buf content size: " <<i2cComm.TxBufferContentSize() << std::endl;}
        */
    }

}
