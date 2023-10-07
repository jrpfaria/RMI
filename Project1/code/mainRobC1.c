
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

int correct(bool line[][7]);
double error(bool *line);

int main(int argc, char *argv[])
{   
    int rotate;
    double err = 0;
    double errors[3];
    bool linemem[5][7];
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
    while(1)
    {
        /* Reading next values from Sensors */
        ReadSensors();

        /* show LineSensor values */
        bool line[7];
        GetLineSensor(line);
        fprintf(stderr,"%u: ", GetTime());
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
        // Count ammount of 1's in line
        int i, j;
        for (i = 0, j = 7; i < 7; i++)
            if (!line[i]) j--;

        // j = amount of 1's detected by the line sensor
        // Discard useless data
        if (j > 1 && j < 6){
            // Update line history (5 readings)
            for(i = 0; i < 4; i++) memcpy(linemem[i], linemem[i+1], 7);
            memcpy(linemem[4], line, 7);
            
            // Calculate error for the current line reading
            err = error(line);
        
            // Update error history (5 readings)
            for(i = 0; i < 2; i++) errors[i] = errors[i+1];
            errors[2] = err;

            // Calculate errors median
            double aux;
            for(i = 0; i < 2; i++)
                if (errors[i] > errors[i+1]){
                    aux = errors[i+1];
                    errors[i+1] = errors[i];
                    errors[i] = aux;
                }
            err = errors[1];
        }

        // Act on error
        // if (j == 0) implementar algo como o rui
        // Perfect scenario
        if (err == 0 && j == 3)
            DriveMotors(0.13, 0.13);
        // Basic error correction
        else DriveMotors(0.02-err,0.02+err);
    }

    return 1;
}

double error(bool *line)
{
    int i, j, k;
    
    // Lookup table for error correction
    double lut_error[12] = {0, 1, 2, 0.08, 4, 0.08, 6, 7, 0.13, 9, 10, 0.08};
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

int correct(bool line[][7])
{  
    int i, j, k;
    int left = 0;
    int right = 0;
    int turn = 0;

    // print line arrays
    fprintf(stderr, "line memory:\n");
    for (i = 0; i < 3; i++){
        for (j = 0; j < 7; j++)
            fprintf(stderr, "%d", line[i][j]);
        fprintf(stderr, "\n");
    }

    // Check for hooks
    for (i = 0; i < 3; i++){
        if (line[i][0]) left++;
        if (line[i][6]) right++;
    }
    turn += (left - right);

    // Failed to pick up hook
    if (turn == 0)
        for (i = 0; i < 3; i++){
            if (line[i][1]) left++;
            if (line[i][5]) right++;
        }
    turn += (left - right);

    fprintf(stderr, "-- %d --\n", turn);

    // No proper memory
    if (turn == 0) return 0;

    // Left / Right turn
    turn = (turn > 0) ? 1 : -1;
    return turn;
}