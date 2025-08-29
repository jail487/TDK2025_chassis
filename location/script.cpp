/*
 * script.cpp
 *
 *  Created on: Jul 5, 2025
 *      Author: 88698
 */
#include "script.h"
#include "chassis_move.h"
#include "chassis.h"
#include "pathsensor.h"
#include "stm32h7xx_hal.h"

#define pi 3.14159

int s = 0;
int coffee_table = 1, cup_color = 0; // coffee_table: 1~4, cup_color: 1~2

bool side = 0; // 0: left, 1: right 

void stage_1() {
   // setPath_finding_line(3, 0);Follow front path till find right and left line
	setPath_finding_line(1, 0);

    while (!ach()) {}
    s++;
}

void set_coffee_table(int _coffee_table) {
    switch (coffee_table) {
        case 0:
            setPath_finding_line(4, 3);// Follow left path till find front and left line
        	//setPath_finding_line(1, 3);// Follow left path till find front and left line
            while (!ach()) {}
            setPath_distance(60, 0);// Follow front path for 60 cm
            while (!ach()) {}
            stop();  
            // 放杯子
            break;

        case 1:
            setPath_finding_line(4, 3);// Follow left path till find front and left line
            //setPath_finding_line(1, 3);// Follow left path till find front and left line
            while (!ach()) {}
            setPath_distance(60, 0);// Follow front path for 60 cm
            while (!ach()) {}
            set_directMove(4, 0, 90); // Rotate left 90 degrees
            while (!ach()) {}
            setPath_distance(60, 0);// Follow front path for 60 cm
            while (!ach()) {}
            stop();   
            // 放杯子
            break;

        case 2:
            setPath_distance(200, 3); // Follow left path for 200 cm
            while (!ach()) {}
            setPath_finding_line(4, 3); // Follow left path till find front and left line
           // setPath_finding_line(1, 3);// Follow left path till find front and left line
            while (!ach()) {}
            setPath_distance(60, 0); // Follow front path for 60 cm
            while (!ach()) {}
            stop();  
            // 放杯子
            break;

        case 3:
            setPath_distance(200, 3); // Follow left path for 200 cm
            while (!ach()) {}
            setPath_finding_line(4, 3); // Follow left path till find front and left line
           // setPath_finding_line(1, 3);// Follow left path till find front and left line
            while (!ach()) {}
            set_directMove(4, 0, 90); // Rotate left 90 degrees
            while (!ach()) {}
            setPath_distance(60, 0); // Follow front path for 60 cm
            while (!ach()) {}
            stop();  
            // 放杯子
            break;
    }
}

void stage_2() {
    setPath_finding_line(4, 2);// Follow right path till find front and left line
//    set_directMove_findLine(4,2);//
   // set_directMove_findLine(1,2);// direct move right till find front and right line
    while (!ach()) {}
    setPath_distance(60, 0);// Follow front path for 60 cm
    while (!ach()) {}
    stop();
    //setPath_finding_line(4, 1);// Follow back path till find front and left line
    set_directMove_findLine(4,1);// direct move back till find front and right line
    set_coffee_table(coffee_table);// Place the coffee cup based on the coffee table number
}


void stage_4() {
    float T = 15.0; // modify T to adjust speed
    float decel_rate;
    if ( side == 0 ) {
        // left
        circular_move(side, 1, T);
        integral_move_w_decel(side, T);
        circular_move(side, 2, T);
        integral_move_w_const_v(side, T);
        circular_move(side, 3, T);
    }
    else {
        // right
        // first a circular
        circular_move(side, 1, T);
        // linear move w decelerate (r 60 to r 50)
        integral_move_w_decel(side, T);    
        circular_move(side, 2, T);
        integral_move_w_const_v(side, T);
        circular_move(side, 3, T);
    }
}



