#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP
#include <ctime>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <queue>
#include <stack>
#include <math.h>
#include <sys/time.h>
#include <iostream>
#include <unistd.h>
using namespace std;
long long getCurrentTimeMillis() {
    auto currentTime = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(currentTime).count();
}
class Simulator{
public:
    Simulator(double r,double m,double len,double show_k=50){
        this->r=r;
        length=len;
        this->m=m;
        base_angle=0;
        balance_angle=M_PI;
        initTime=getCurrentTimeMillis();
        previous_t=initTime-1;
        I=1.0/3*r*r*m;
        this->show_k=show_k;
    }
    double step(double F){//v:顺时针（从中心看向右）
        if(F>2)F=2;
        if(F<-2)F=-2;
        long long now_t=getCurrentTimeMillis();
        double delta_t=(now_t-previous_t)*1.0/1000;
        cv::Mat src=cv::Mat(720, 720, CV_8UC3, cv::Scalar(0, 0, 0));
        if(rand()%1000==0){
            double delt=(rand()%1000-500)*1.0/500*0.2;
            balance_angle+=delt;
            balance_w+=0.2*delt;
            show_last_time+=0.4;
            cv::putText(src,"TOUCHED",cv::Point(100,100),1.0,3.0,cv::Scalar(0,0,255));
            cout<<"TOUCHED:"<<delt<<endl;
        }
        if(show_last_time>0){
            show_last_time-=delta_t;
            cv::putText(src,"TOUCHED",cv::Point(100,100),1.0,3.0,cv::Scalar(0,0,255));
        }
        
        double torque=0;
        
        torque+=sin(balance_angle)*m*9.8;
        
        double a=F/m;   //加速度
        v+=a*delta_t;
        x+=v*delta_t;
        v*=(1-(1-decay)*delta_t*abs(v));
        a=(v-previous_v)/delta_t;
        torque-=cos(balance_angle)*m*a;
        double w_a=torque/I;//角加速度
        balance_w+=delta_t*w_a;
        balance_w*=(1-(1-decay)*delta_t);
        balance_angle+=delta_t*balance_w;
        while(x>360.0/show_k)x-=720.0/show_k;
        while(x<-360.0/show_k)x+=720.0/show_k;
        while(balance_angle<0)balance_angle+=2*M_PI;
        while(balance_angle>2*M_PI)balance_angle-=2*M_PI;
        
        cv::line(src,cv::Point(360+x*show_k,360),cv::Point(360+r*200*sin(balance_angle)+x*show_k,360-200*r*cos(balance_angle)),cv::Scalar(255,0,0),5);
        cv::imshow("show",src);
        cv::waitKey(1);   
        //预后操作
        previous_v=v;
        previous_t=now_t;
        return balance_angle;
    }
    double v=0;
private:
    double show_last_time=0;
    double show_k=50;
    
    double x=0;
    double decay=0.99;
    double I;//转动惯量
    struct timeval _time;
    double base_angle;//用于可视化
    long long initTime;
    double balance_angle=2*M_PI;//最上方0 / 2pi，最下方pi,顺时针正
    double balance_w=0;//顺时针正，角速度
    long long previous_t;
    double previous_v=0;
    double m;
    double length;
    double r;
    
};

#endif