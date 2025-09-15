/*
 * script.cpp
 *
 *  Created on: Jul 5, 20normalSpeed
 *      Author: 88698
 */
#include "script.h"
#include "location.h"
#include "chassis.h"
#include "pathsensor.h"
#include "stm32h7xx_hal.h"

#define M_PI 3.14159265358979323846

extern int stage;

int s = 0;
int coffeTable = 0, cupColor = 0; // coffeTable: 1~4, cupColor: 1~2

bool side = 0; // 0: left, 1: right 
float normalSpeed = 30.f;
float slowSpeed = 10.f;

void testt(){
	setmoveDistance(0, 200, 0, 10.f,1);
}

void stage_1() {
   // setPath_finding_line(3, 0);Follow front path till find right and left line
	setPath_finding_line(3, 0, normalSpeed);//循跡到過橋遇到第一個左右線
    //stop();

   // while (!ach()) {}
    s++;
}
float xDis_receive = 0, yDis_receive = 0;
void set_coffeTable(int _coffeTable) {
    float deskDistance = 38; // Distance to move for placing the cup

    switch (coffeTable) {//左上桌
        case 1:
            setPath_finding_line(2, 3, normalSpeed);// Follow left path till find front and right line
        	//setPath_finding_line(1, 3);// Follow left path till find front and left line

            setPath_distance(deskDistance, 0, normalSpeed);// move to desk
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 0, normalSpeed,1);// Move to desk
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3);// move back till find left and right line
            setPath_distance(80, 3, normalSpeed);// Follow front path for normalSpeed cm
            setPath_finding_line(6,3, normalSpeed);//follow left path till find front line
            stop();  
            // 放杯子
            break;

        case 2://右上桌
            setmoveDistance(0, 0, 179.7, normalSpeed,0);// move back for normalSpeed cm
            setPath_finding_line(2, 2, normalSpeed);// Follow right path till find front and right line
            setPath_distance(deskDistance, 0, normalSpeed);// move to desk
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 179.7, normalSpeed,1);// Move to desk (保持180度角度)
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3);// move back till find left and right line
            setmoveDistance(-normalSpeed, 0, 90, normalSpeed,0);// move back for normalSpeed cm
            setPath_finding_line(6,3, normalSpeed);//follow left path till find front line
            stop();
          ///  while (!ach()) {}
           // stop();   
            // 放杯子
            break;

        case 3://left down table
            setPath_distance(200, 3, normalSpeed); // Follow left path for 200 cm
            setPath_finding_line(2, 3, normalSpeed); // Follow left path till find front and left line
            setPath_distance(deskDistance, 0, normalSpeed); // move to desk
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 0, normalSpeed,1);// Move to desk
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3); // move back till find left and right line
            //setPath_finding_line(6,3);//follow left path till find front line
            setmoveDistance(-normalSpeed, 0, 0, normalSpeed,0);// move cross the line
            setPath_finding_line(6,3, normalSpeed);//follow left path till find front line
            break;

        case 4://right down table
            setmoveDistance(0, 0, 179.7, normalSpeed,0);// turn for 180 degree
            setPath_finding_line(2, 2, normalSpeed); // Follow right path till find front and right line
            setPath_distance(deskDistance, 0, normalSpeed); // move to desk
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 179.7, normalSpeed,1);// Move to desk (保持180度角度)
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3); // move back till find left and right line
            setmoveDistance(-normalSpeed, 0, 90, normalSpeed,0);// move cross the line
            setPath_finding_line(6,3, normalSpeed);//follow left path till find front line
            stop();  
            // 放杯子
            break;
    }
}
//int cupDistance_x = 20; // Distance to move for placing the cup
void set_coffee_cup() {
    float cupDistance_x = 20; // Distance to move for placing the cup
    float cupDistance_y = 16.5; // Distance to move for placing the cup
    switch (cupColor) {
        case 1: // white cup
            setmoveDistance(-cupDistance_x, 0, 0, slowSpeed,1); // Move forward to place the cup
            setmoveDistance(0, cupDistance_y, 0, slowSpeed,1); // Move upward to place the cup
            break;
        case 2: // black cup
            setmoveDistance(cupDistance_x, 0, 0, slowSpeed,1); // Move forward to place the cup
            setmoveDistance(0, cupDistance_y, 0, slowSpeed,1); // Move upward to place the cup
            break;
    }
}

void stage_2() {
    setPath_finding_line(4, 2, normalSpeed);// Follow right path till find front and left line//
//    set_directMove_findLine(4,2);//
   // set_directMove_findLine(1,2);// direct move right till find front and right line
   // while (!ach()) {}
    //setPath_distance(normalSpeed, 0, 0 ,10.f);// Follow front path for normalSpeed cm//到桌子前
    setPath_distance(33, 0, normalSpeed);
//    while(cupColor == 0){
//
//    }
    waitMissionComplete(21);//wait for receive cup color
    setmoveDistance(0, -5, 0, normalSpeed,1);// Move to desk (保持180度角度)
   // waitMissionComplete(21);//wait for receive desk informatu

    set_coffee_cup();//go tp cup location
    waitMissionComplete(22);//wait for close grip
    set_directMove_findLine(1,5);// 往後移到碰到左線
    set_coffeTable(coffeTable);// Place the coffee cup based on the coffee table number
   // stop();
}



