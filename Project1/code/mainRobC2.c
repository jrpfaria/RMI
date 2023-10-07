
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

void driver(double pos_x, double pos_y, double compass);
double error(bool *line);

int main(int argc, char *argv[])
{   
    // C1 related
    double err = 0;
    double errors[3];
    bool linemem[5][7];

    // C2 related
    double crossroads[5][2]; 

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
        
        // My code:
        double pos_x = GetX() - offsets[0];
        double pos_y = GetY() - offsets[1];
        double compass = GetCompassSensor();

        double target_x = 0;
        double target_y = 0;

        // Print values for debugging
        printf("X: %f\n", pos_x);
        printf("Y: %f\n", pos_y);
        printf("Compass: %f\n", compass);

    }

    return 1;
}

// C2 related
void driver(double pos_x, double pos_y, double compass)
{
    // useless for now
    // move robot forward to the next cell   
}


// C1 related
double error(bool *line)
{
    int i, j, k;
    
    // Lookup table for error correction
    double lut_error[12] = {0, 1, 2, 0.05, 4, 0.08, 6, 7, 0.13, 9, 10, 0.08};
    int left = 0;
    int right = 0;

    // Calculate error
    // using the 5 central bits of the line sensor, split into groups of 4
    for (i = 1, j = 5, k = 8; i < 5; i++, j--)
    {
        left += line[i] * k;
        right += line[j] * k;
        k = (k >> 1);
    }

    // Use absolute value of error to index lookup table
    int error = abs(left - right);

    // Return error correction
    double necessaryCorrection = lut_error[error];

    // if left is bigger than right, the robot is too much to the right, so it should turn left
    return (left >= right) ? necessaryCorrection : necessaryCorrection * -1;
}