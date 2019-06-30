#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include 


using namespace std;

int main(int argc, char **argv)
{
    int ch;
    while (1){
        if (_kbhit()){
            ch = _getch();
            cout << ch;
            if (ch == 27){ break; }
        }
    }
    system("pause");
}