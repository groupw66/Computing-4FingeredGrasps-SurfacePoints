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
        stopwatch[key].second = 0.d;
    }void start(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            reset(key);
        }
        stopwatch[key].first = clock_::now();
    }
    double pause(std::string key) {
        if(stopwatch.find(key) == stopwatch.end()){
            reset(key);
        }
        else {
            stopwatch[key].second += std::chrono::duration_cast<second_>
                (clock_::now() - stopwatch[key].first).count();
        }
        return getSec(key);
    }
    double getSec(std::string key) {
        return stopwatch[key].second;
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
    std::unordered_map<std::string, std::pair<std::chrono::time_point<clock_>, double > > stopwatch;
};


#endif // TIMER_H_INCLUDED
