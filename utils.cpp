
#include "utils.hpp"

constexpr int DEBUG = 0;

static std::default_random_engine  generator;
static std::default_random_engine  generator2;

int randi(int min, int max)
{
    std::uniform_int_distribution<int> dist(min, max);
    int x = dist(generator);
    if (DEBUG) printf("randi %d %d %d\n", min, max, x);
    return x;
}
float rand(float min, float max)
{
    std::uniform_real_distribution<float> dist(min, max);
    float x = dist(generator);
    if (DEBUG) printf("rand  %f %f %f\n", min, max, x);
    return x;
}
int randi2(int min, int max)
{
    std::uniform_int_distribution<int> dist(min, max);
    int x = dist(generator2);
    if (DEBUG) printf("randi %d %d %d\n", min, max, x);
    return x;
}
float rand2(float min, float max)
{
    std::uniform_real_distribution<float> dist(min, max);
    float x = dist(generator2);
    if (DEBUG) printf("rand  %f %f %f\n", min, max, x);
    return x;
}
float randn(float std)
{
    std::normal_distribution<float> dist(0.0, std);
    float x = dist(generator);
    if (DEBUG) printf("randn %f %f\n", std, x);
    return x;
}

void setseed(int n)
{
    //printf("setting seed %d\n", n);
    generator.seed(n);
    generator2.seed(n);
}


namespace timer
{
    static int ticker(int inc)
    {
        static int t = 0;
        t += inc;
        return t;
    }
    void tick()
    {
        ticker(1);
    }
    int ticks()
    {
        return ticker(0);
    }
}