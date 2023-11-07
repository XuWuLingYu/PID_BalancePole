#include "simulator.hpp"
using namespace cv;
using namespace std;
double k_P=5,k_I=1,k_D=0.2;
double det=0,inter=0;
double F=0;
long long lastT=0;
double lastError=0;
long long tick=0;
int main(){

    Simulator s(1, 0.2, 0.3 ,100);
    double v=0;
    lastT=getCurrentTimeMillis();
    sleep(0.02);
    while(1){
        tick++;
        long long nowT=getCurrentTimeMillis();
        sleep(0.01);
        double angle=s.step(F);
        double error;
        if(angle>=M_PI)error=2*M_PI-angle;
        if(angle<M_PI)error=0-angle;
        double deltaT=(nowT-lastT)*1.0/1000;
        inter+=error*deltaT;
        if(tick!=1)
            det=(error-lastError)/deltaT;
        else 
            det=0;
        F=-((error)*k_P+inter*k_I+det*k_P);
        lastT=nowT;
        lastError=error;
    }
}