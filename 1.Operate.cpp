#include "1.Operate.h"
#include "math.h"
#include <algorithm>
#include <Wire.h>
#include "function.h"

#define COUNT1 5
#define COUNT2 3
#define COUNT3 2

double idou_1[COUNT1][3] = {
            {0,0,0},
            {45,0,0},
            {45,45,0},
            {0,45,0},
            {0,0,0}
};
double idou_2[COUNT2][3] = {
            { 0,0,0},
            { 60, 60,0},
            {0, 60,PI}
};
double idou_3[COUNT3][3] = {
            { 0,0,0},
            { 30, 0,PI}
};
double damy1[10][3] = {0};


void Auto(int b){

    if(b == 1)Idou(COUNT1,idou_1,damy1);
    if(b == 2)Idou(COUNT1,idou_2,damy1);
    if(b == 3)Idou(COUNT1,idou_3,damy1);

}

    