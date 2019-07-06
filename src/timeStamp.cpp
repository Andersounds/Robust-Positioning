#ifndef RPOS_TIMESTAMP
#define RPOS_TIMESTAMP
#include <chrono>

//Gör till en klass som på instansiering sätter t_init och också räknar ut när det blir overflow.
//Då behöver man inte utvärdera varje gång huruvida det är första funktionsanropet eller inte
//Man kan dessutom ha flera olika funktioner beroende på om man vill ha double, eller float
// https://en.cppreference.com/w/cpp/chrono/duration

/*
double timeStamp_ms(void){
    static std::chrono::steady_clock::time_point t_init = std::chrono::steady_clock::now();
    static int init = 0;
    if(!init){
        init = 1;
        std::chrono::duration<double,std::milli> init_dur = t_init - t_init;


        float seconds = init_dur.count(); //Count gives in seconds
        std::cout << "T_init is " << seconds << " seconds." << std::endl;
        double maxmilli =  std::chrono::duration<double,std::milli>::max().count();
        double ovrflow_time = maxmilli/1000/3600/24;
        std::cout << "T_max is " << ovrflow_time << " days." << std::endl;
    }




    std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
    std::chrono::duration<double,std::milli> dur = ts - t_init;
    return dur.count();
}

*/


//https://en.cppreference.com/w/cpp/chrono/duration
namespace timestamp{
    class timeStamp_ms{
    public:
        timeStamp_ms(void);
        void get(double&); //native double cast
        void get(float&);//convert to float first
        void timeUntilOverFlow(void);
    private:
        std::chrono::steady_clock::time_point t_init;
    };
}

timestamp::timeStamp_ms::timeStamp_ms(void){
    t_init = std::chrono::steady_clock::now();//Set initial timepoint
    timeUntilOverFlow();                      //Give user info regarding overflow time of timestamp
}
void timestamp::timeStamp_ms::timeUntilOverFlow(void){
    //Get duration <double,std::milli> between init timepoint and clock epoch
    std::cout << "Initializing time stamp object" << std::endl;
    std::cout << "\tClock: std::chrono::steady_clock  " << std::endl;
    std::chrono::duration<double,std::milli> init_dur_ms = t_init.time_since_epoch();
    std::cout << "\tTime since epoch of steady_clock: " << init_dur_ms.count()/1000/3600 << " hours." << std::endl;
    //std::cout << "\tTime since epoch of steady_clock: " << init_dur_ms.count()/1000      << " seconds." << std::endl;
    //std::cout << "\t                                  " << init_dur_ms.count()/1000/3600 << " hours." << std::endl;
    //Get duration <double,std::milli> until overflow
    std::chrono::duration<double,std::milli> dur_until_ovflw_ms = std::chrono::time_point<std::chrono::steady_clock>::max() - t_init;
    //std::cout << "\tTime until overflow:              " << dur_until_ovflw_ms.count()/1000         << " seconds." << std::endl;
    //std::cout << "\t                                  " << dur_until_ovflw_ms.count()/1000/3600    << " hours." << std::endl;
    //std::cout << "\t                                  " << dur_until_ovflw_ms.count()/1000/3600/24 << " days." << std::endl;
    //std::cout << "###################END###################" << std::endl;
    std::cout << "\tTime until overflow:              " << dur_until_ovflw_ms.count()/1000/3600    << " hours." << std::endl;
    std::cout << "Done." << std::endl;
}

void timestamp::timeStamp_ms::get(double& stamp){
    std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
    std::chrono::duration<double,std::milli> duration_ms = ts - t_init;
    stamp = duration_ms.count();
}
void timestamp::timeStamp_ms::get(float& stamp_f){
    double stamp_d;
    get(stamp_d);
    stamp_f = (float) stamp_d;
}


#endif
