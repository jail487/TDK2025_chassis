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
int coffeTable = 2, cupColor = 1; // coffeTable: 1~4, cupColor: 1~2

bool side = 0; // 0: left, 1: right 
float normalSpeed = 30.f;
float slowSpeed = 10.f;

float veryfast =70;



void stage_1() {
    //setPath_finding_line(3, 0,veryfast);//Follow front path till find right and left line
	setLinePI(1.5f,0.15,2.f);
    setPath_distance(180,0,veryfast);
    setLinePI(0.6,0.15,0.f);
    setPath_finding_line(3,0,normalSpeed);

    //stop();

   // while (!ach()) {}
    s++;
}
float xDis_receive = 0, yDis_receive = 0;
void set_coffeTable(int coffeeTable) {
    float deskDistance = 28; // Distance to move for placing the cup

    switch (coffeTable) {//左上桌
        case 1:
            setPath_finding_line(2, 3, normalSpeed);// Follow left path till find front and right line
        	//setPath_finding_line(1, 3);// Follow left path till find front and left line

            setPath_distance(deskDistance, 0, slowSpeed);// move to desk
            waitMissionComplete(23);//wait for receiveDistance
            updatePosition(0,0,0.f);
            setmoveDistance(xDis_receive, yDis_receive, 0, slowSpeed,3);// Move to desk
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3, normalSpeed);// move back till find left and right line
            setPath_distance(120, 3, normalSpeed);// Follow front path for normalSpeed cm
            setPath_finding_line(2,3, normalSpeed);//follow left path till find front line
            updatePosition(0,0,0);
            stop();  
            // 放杯子
            break;

        case 2://右上桌
        	 updatePosition(0,0,0);
            setmoveDistance(0, 0, 180.f, normalSpeed,1,-1);// move back for normalSpeed cm
            setPath_distance(150, 2, normalSpeed);// Follow front path for normalSpeed cm
            setPath_finding_line(2, 2, normalSpeed);// Follow right path till find front and right line
            setPath_distance(deskDistance, 0, slowSpeed);// move to desk
            updatePosition(0,0,180.f);
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 180.f, slowSpeed,3);// Move to desk (保持180度角度)
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3, normalSpeed);// move back till find left and right line
            setmoveDistance(0, 0, 0.f, normalSpeed,1,-1);// move back for normalSpeed cm
            setPath_distance(120, 2, normalSpeed);// Follow front path for normalSpeed cm
            setPath_finding_line(2, 2, normalSpeed);//follow left path till find front line
            stop();  
           // while (!ach()) {}
           // stop();
            // 放杯子
            break;

        case 3://left down table
            setPath_distance(200, 3, normalSpeed); // Follow left path for 200 cm
            setPath_finding_line(2, 3, normalSpeed); // Follow left path till find front and left line
            setPath_distance(deskDistance, 0, slowSpeed); // move to desk
            updatePosition(0,0,0.f);
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 0, slowSpeed,3);// Move to desk
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3, normalSpeed); // move back till find left and right line
            setPath_finding_line(2,3, normalSpeed);
            updatePosition(0,0,0);
            stop();
            break;

        case 4://right down table
            setmoveDistance(0, 0, 180, normalSpeed,1,-1);// turn for 180 degree
            setPath_distance(200, 2, normalSpeed);// Follow front path for normalSpeed cm
            setPath_finding_line(2, 2, normalSpeed); // Follow right path till find front and right line
            setPath_distance(deskDistance, 0, slowSpeed); // move to desk
            updatePosition(0,0,180.f);
            waitMissionComplete(23);//wait for receiveDistance
            setmoveDistance(xDis_receive, yDis_receive, 180, slowSpeed,3);// Move to desk (保持180度角度)
            waitMissionComplete(24);//wait for put cup
            set_directMove_findLine(1, 3, normalSpeed); // move back till find left and right line
            setmoveDistance(0, 0, 0.f, normalSpeed,1,-1);// move back for normalSpeed cm
            setPath_finding_line(2,2, normalSpeed);//follow left path till find front line
            stop();  
            // 放杯子
            break;
        default:
        	stop();
        	waitMissionComplete(coffeTable);
        	break;

    }
}
float sslowSpeed = 7;
//int cupDistance_x = 20; // Distance to move for placing the cup
void set_coffee_cup() {
    float cupDistance_x = 16.5; // Distance to move for placing the cup
    float cupDistance_y = 16.5; // Distance to move for placing the cup
    switch (cupColor) {
        case 1: // white cup
            setmoveDistance(-cupDistance_x, 0, 0, sslowSpeed,1); // Move forward to place the cup
            setmoveDistance(0, cupDistance_y, 0, sslowSpeed,1); // Move upward to place the cup
            break;
        case 2: // black cup
            setmoveDistance(cupDistance_x, 0, 0, sslowSpeed,1); // Move forward to place the cup
            setmoveDistance(0, cupDistance_y, 0, sslowSpeed,1); // Move upward to place the cup
            break;
    }
}

int firstDeskDis = 25;
void stage_2() {
    setPath_finding_line(4, 2, normalSpeed);// Follow right path till find front and left line//
//    set_directMove_findLine(4,2);//
   // set_directMove_findLine(1,2);// direct move right till find front and right line
   // while (!ach()) {}
    //setPath_distance(normalSpeed, 0, 0 ,10.f);// Follow front path for normalSpeed cm//到桌子前
    setPath_distance(firstDeskDis, 0, 10);

//    while(cupColor == 0){
//
//    }
    waitMissionComplete(21);//wait for receive cup color
    //setmoveDistance(0, -5, 0, normalSpeed,1);// Move to desk (保持180度角度)
   // waitMissionComplete(21);//wait for receive desk informatu
    updatePosition(0,0,0);
    set_coffee_cup();//go tp cup location
    waitMissionComplete(22);//wait for close grip
    set_directMove_findLine(1,5, normalSpeed);// 往後移到碰到左線

    set_coffeTable(coffeTable);// Place the coffee cup based on the coffee table number
    stop();
}

float basket_x = 68, basket_y = 30,putbasket_x = 80;
void stage_3() {
	updatePosition(0,0,0);
    setmoveDistance(0, 0, 180.f, normalSpeed,1,-1);// rotate counterclockwise 180 degree
    setPath_distance(basket_x , 2, slowSpeed);
    updatePosition(0,0,180);
    setmoveDistance(0, basket_y, 180.f, normalSpeed,1);
    waitMissionComplete(31);//wait for grab basket
    set_directMove_findLine(6,0, normalSpeed);// move forward till find right line
    setPath_finding_line(6,2, normalSpeed);// Follow right path till find front and right line and left line
    setPath_distance(putbasket_x, 2, slowSpeed);// Follow front path for slowSpeed cm
    waitMissionComplete(32);// wait for put basket
    setPath_finding_line(3,0, slowSpeed);//follow front path till find left ,right,speeds

    stop();
}
void testt(){
	updatePosition(0,0,180.f);
	waitMissionComplete(23);//wait for receiveDistance
	setmoveDistance(xDis_receive, yDis_receive, 180, slowSpeed,2);// Move to desk (保持180度角度)
	waitMissionComplete(24);//wait for put cup
	// 測試從 0° 轉到 90°
//	set_coffee_cup();
	//setmoveDistance(0, 0, 180.f, normalSpeed,1,-1);// move back for normalSpeed cm
	//set_coffeTable(2);


	// 順時針應該是：0° → -90° → -180° → -270° (= 90°)，總共轉 -270°
	//setmoveDistance(20, 0, 0, 15, 1, -1);   // 順時針到90°

	// 逆時針應該是：0° → 90°，總共轉 +90°
	//setmoveDistance(0, 0, 180, 15, 1, -1); // 逆時針到90°
}



