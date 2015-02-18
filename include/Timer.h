#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

//https://gist.github.com/gongzhitaao/7062087

#include <chrono>
#include <unordered_map>
#include <string>
#include <sstream>

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    void reset(std::string key) {
        std::get<1>(stopwatch[key]) = 0.d;
        std::get<2>(stopwatch[key]) = 0.d;
        start(key);
    }
    void start(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            reset(key);
        }
        std::get<0>(stopwatch[key]) = clock_::now();
    }
    double pause(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            reset(key);
        }
        else {
            std::get<2>(stopwatch[key]) = std::chrono::duration_cast<second_>
                (clock_::now() - std::get<0>(stopwatch[key])).count();
            std::get<1>(stopwatch[key]) += std::get<2>(stopwatch[key]);
        }
        return getSec(key);
    }
    double getSec(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            return 0.;
        }
        return std::get<1>(stopwatch[key]);
    }
    double getSecLastLap(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            return 0.;
        }
        return std::get<2>(stopwatch[key]);
    }
    double remove(std::string key){
        double tmp = pause(key);
        stopwatch.erase(key);
        return tmp;
    }
    const char* strStopwatch(){
        /*std::stringstream ss;
        for(auto it = stopwatch.begin() ; it != stopwatch.end() ; ++it){
            ss << it->first << " : " << getSec(it->first) << "\n";
        }
        return ss.str().c_str();*/
        char* str = (char*) malloc(500*stopwatch.size());
        str[0] = '\0';
        for(auto it = stopwatch.begin() ; it != stopwatch.end() ; ++it){
            char tmp[500];
            sprintf(tmp, "%s : %lf\n", it->first.c_str(), getSec(it->first));
            strcat(str, tmp);
        }
        return str;
    }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
    std::unordered_map<std::string, std::tuple<std::chrono::time_point<clock_>, double, double > > stopwatch;
};


#endif // TIMER_H_INCLUDED
