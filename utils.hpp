#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
using namespace Eigen;

#include <random>

//https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}



static int uid()
{
    static int i = 0;
    i++;
    // if (i == 3475)
    //     printf("%d\n", i);
    return i;
}



int randi(int min, int max);
float rand(float min, float max);
int randi2(int min, int max);
float rand2(float min, float max);
float randn(float std);
void setseed(int n);


namespace timer
{
    void tick();
    int ticks();
}


#endif