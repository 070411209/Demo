#include <iostream>
#include<stdio.h>
#include "sophus/se3.hpp"

using namespace std;
 
int main()
{
    char c;
    while ((c=getchar())!='\n')
    {
        if ((c >='a' && c<='z') || c >='A' && c<='Z')
        {
            c = c +4;
            if(c>'Z' && c<='Z'+4 || c>'z')
                c = c-26;
        }
        std::cout << c;
    }
    std::cout << std::endl;
    return 0;
}