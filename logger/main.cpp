
#include "logger.h"

#include <iostream> 
#include <fstream> 
using namespace std; 

void test() {
    Logger logger(Logger::file_and_terminal, Logger::debug, "Log.log");
    logger.DEBUG("Debug info");
    logger.INFO("This is variable");
    logger.WARNING("This function or variable may be unsafe");
    logger.ERRORS("Memory leak");
}

int main()
{
    test();
    getchar();
    return 0;
}