
/* mainRob.C
 *
 * Basic Robot Agent
 * Very simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "RobSock.h"

#include "robfunc.h"

#define true 1
#define false 0

void drive(int orientation, double target_x, double target_y, double gps_x, double gps_y);
void align(bool linemem[][7]);

int main(int argc, char *argv[])
{   
    // C1 related
    double err = 0;
    double errors[3];
    bool linemem[3][7];

    // C2 related
    double target[2] = {0, 0};
    double world[21][49]; 
    bool aligned = 1;

    char host[100]="localhost";
    char rob_name[20]="robsample";
    float lPow,rPow;
    int state=STOP, stoppedState=RUN, rob_id = 1;
    char lmap[CELLROWS*2-1][CELLCOLS*2-1]; // in this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to lmap[i*2][j*2].
                                           // to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of lmap[i*2+1][j*2] is space or not

    printf( " Sample Robot\n Copyright (C) 2001-2019 Universidade de Aveiro\n" );

    /* processing arguments */
    while (argc > 2) /* every option has a value, thus argc must be 1, 3, 5, ... */
    {
        if (strcmp(argv[1], "--host") == 0 || strcmp(argv[1], "-h") == 0)
        {
           strncpy(host, argv[2], 99);
           host[99]='\0';
        }
        else if (strcmp(argv[1], "--robname") == 0 || strcmp(argv[1], "-r") == 0)
        {
           strncpy(rob_name, argv[2], 19);
           rob_name[19]='\0';
        }
        else if (strcmp(argv[1], "--pos") == 0 || strcmp(argv[1], "-p") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
               argc=0; /* error message will be printed */
        }
        else if (strcmp(argv[1], "--map") == 0 || strcmp(argv[1], "-m") == 0)
        {
            ReadMap(argv[2],lmap);
            for(int r=CELLROWS*2-2; r>=0; r--) {
               for(int c=0; c<CELLCOLS*2-1; c++) {
                   printf("%c", lmap[r][c]);
               }
               printf("\n");
            }
        }
        else
        {    
            break; /* the while */
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: mainRob [--host hostname] [--robname robotname] [--pos posnumber]\n");

        return 1;
    }

    /* Connect Robot to simulator */
    if(InitRobot(rob_name, rob_id, host)==-1)
    {
       printf( "%s Failed to connect\n", rob_name); 
       exit(1);
    }
    printf( "%s Connected\n", rob_name );
    state=STOP;

    // C2 related
    ReadSensors();
    double offsets[] = {GetX(), GetY()};

    while(1)
    {
        /* Reading next values from Sensors */
        ReadSensors();

        /* show LineSensor values */
        bool line[7];
        GetLineSensor(line);
        for(int i=0;i<N_LINE_ELEMENTS;i++) {
            fprintf(stderr,"%s",line[i]?"1":"0");
        }
        fprintf(stderr,"\n");

        if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
        {
           printf(  "%s Exiting\n", rob_name );
           exit(0);
        }
        if(state==STOP && GetStartButton()) state=stoppedState;  /* Restart     */
        if(state!=STOP && GetStopButton())  {
            stoppedState=state;
            state=STOP; /* Interrupt */
        }
        
        // Line reading history
        int i;
        for(i = 0; i < 2; i++) memcpy(linemem[i], linemem[i+1], 7);
        memcpy(linemem[2], line, 7);

        aligned = (line[2] && line[3] && line[4]);

        // My code:
        double pos_x = GetX() - offsets[0];
        double pos_y = GetY() - offsets[1];
        double compass = GetCompassSensor();

        int orientation = (int) (compass / 45);
        
        // Print values for debugging
        // printf("X: %f\n", pos_x);
        // printf("Y: %f\n", pos_y);
        // printf("Compass: %f\n", compass);

        if (pos_x == target[0] && pos_y == target[1]){
            fprintf(stderr, "Target reached\n");
            if (aligned){
                // turn the next switch into if statements
                if (orientation == 0) target[0] += 2.0;
                else if (orientation == 1) target[0] += 2.0, target[1] -= 2.0;
                else if (orientation == 2) target[1] -= 2.0;
                else if (orientation == 3) target[0] -= 2.0, target[1] -= 2.0;
                else if (orientation == -1) target[0] += 2.0, target[1] += 2.0;
                else if (orientation == -2) target[1] += 2.0;
                else if (orientation == -3) target[0] -= 2.0, target[1] += 2.0;
                else if (orientation == -4 || orientation == 4) target[0] -= 2.0;
            }
        }
        else
        {
            if (aligned)
                drive(orientation, target[0], target[1], pos_x, pos_y);
            else
                align(linemem);
        }
    }

    return 1;
}

// C2 related
void drive(int orientation, double target_x, double target_y, double gps_x, double gps_y)
{
    double speed = 0.0;
    double aux = 0.0;
    double aux2 = 0.0;

    // fprintf(stderr, "\nDrive\n");
    fprintf(stderr, "Target: %f, %f\n", target_x, target_y);
    fprintf(stderr, "GPS: %f, %f\n", gps_x, gps_y);
    // fprintf(stderr, "Orientation: %d\n", orientation);

    if (abs(orientation) == 4 || orientation == 0){   
        aux = (target_x - gps_x);     
        aux *= aux >= 0 ? 1 : -1;
        speed = aux > 0.5 ? 0.15 : 0.02; 
    }
    else if (abs(orientation) == 2){
        aux = (target_y - gps_y);
        aux *= aux > 0 ? 1 : -1;
        speed = aux > 0.5 ? 0.15 : 0.02;
    }
    else{
        aux = (target_x - gps_x);
        aux2 = (target_y - gps_y);     
        speed = (aux*aux)+(aux2*aux2) > 0.5 ? 0.15 : 0.02;
    }

    fprintf(stderr, "Speed: %f\n", speed);
    DriveMotors(speed, speed);
}

void align(bool linemem[][7])
{
    int i, j;
    int left, right = 0;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 2; j++)
        {
            left += linemem[i][j]++;
            right += linemem[i][6 - j]++;
        }
    
    (left > right) ? DriveMotors(-0.05, 0.05) : DriveMotors(0.05, -0.05); 
}