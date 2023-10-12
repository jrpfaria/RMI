
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

int correct(double *errors, bool lines[][7], int decision);
double error(bool *line);

int main(int argc, char *argv[])
{   
    int decision = 0;
    double err = 0;
    double errors[3];
    double err_mem[3];
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
            
            if (err != 0){
                for(i = 0; i < 2; i++) err_mem[i] = err_mem[i+1];
                err_mem[2] = err;
            }    
        }

        // Act on error
        // if (j == 0) implementar algo como o rui
        // Perfect scenario
        if (err == 0)
            if (j == 2 || j == 3){
                decision = 0;
                DriveMotors(0.15, 0.15);
            }
            else{
                correct(err_mem, linemem, 0); 
            }

        // Basic error correction
        else
            if (j == 1) decision = (decision == 0) ? 0 : correct(err_mem, linemem, decision); 
            else DriveMotors(0.02-err,0.02+err);
    }

    return 1;
}

double error(bool *line)
{
    int i, j, k;
     
    // Lookup table for error correction
    double lut_error[12] = {0, 1, 2, 0.05, 4, 0.01, 6, 7, 0.13, 9, 10, 0.08};
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

int correct(double *errors, bool lines[][7], int decision)
{  
    int i, target = 0;
    int e_left = 0;
    int e_right = 0;

    int l_left = 0;
    int l_right = 0;

    // bool checks_out;

    // Calculate errors median
    for(i = 0; i < 3; i++)
        errors[i] > 0 ? e_left++ : e_right++;
    
    for(i = 0; i < 5; i++){
        if (lines[i][0]) l_left++;
        else if (lines[i][6]) l_right++;
    }

    // checks_out = ((l_left > l_right) && (e_left > e_right)) || ((l_right > l_left) && (e_right > e_left));

    fprintf(stderr, "correcting...\n");
    fprintf(stderr, "e_left: %d, e_right: %d\n", e_left, e_right);
    fprintf(stderr, "l_left: %d, l_right: %d\n", l_left, l_right);

    if (decision == 0)
    // // Act on error
        if (l_left  != l_right){
            target = (l_left > l_right) ? 1 : -1;
            target == 1 ? DriveMotors(-0.08, 0.08) : DriveMotors(0.08, -0.08);
            return target;
        }
        else{
            target = (e_left > e_right) ? 1 : -1;
            target == 1 ? DriveMotors(-0.08, 0.08) : DriveMotors(0.08, -0.08);
            return target;
        }
    else{
        decision == 1 ? DriveMotors(-0.08, 0.08) : DriveMotors(0.08, -0.08);
        return decision;
    }
}

