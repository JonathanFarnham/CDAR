#include "config.h"
#include "motors.h"
#include "grid_control.h"
#include <Arduino.h>

//Grid Params
float grid_len_ft = 0;
float grid_width_ft = 0;
int total_passes = 0;
bool turn_right_first = true;

//Flag for Grid Active
bool isAutoPilotActive = false;

//State Machine Variables
enum State { IDLE, DRIVING_LONG, TURNING_1, DRIVING_SHORT, TURNING_2, FINISHED };
State currentState = IDLE;

int current_pass = 0;
long target_ticks = 0;
bool current_turn_right = true; //tracks next turn direction

//Configuration
void configureGrid(float len, float width, int passes, bool start_right)
{
    grid_len_ft = len;
    grid_width_ft = width;
    total_passes = passes;
    turn_right_first = start_right;
}

void startGridRun()
{
    if (grid_len_ft == 0 || total_passes == 0) return;
    if (isAutoPilotActive) return; //prevent from running if already running

    isAutoPilotActive = true; //lock

    currentState = DRIVING_LONG;
    current_pass = 1;
    current_turn_right = turn_right_first;

    //setup first move (length)
    resetEncoders();
    target_ticks = grid_len_ft * TICKS_PER_FOOT;
    driveStraight(SPEED_GRID_FWD);
    Serial.println("Grid Started");
}

void stopGridRun()
{
    currentState = IDLE;
    stopBot();
    isAutoPilotActive = false; //Unlock
    Serial.println("Grid Stopped/Finished");
}

//Helper for other modules to check status
bool isGridRunning()
{
    return isAutoPilotActive;
}

//main loop logic
void handleGrid()
{
    if (currentState == IDLE || currentState == FINISHED)
    {
        if (currentState == FINISHED)
        {
            stopGridRun(); //Ensure flag gets reset
            currentState = IDLE;
        }
        return;
    }

    long current_dist = getEncoderAvg();

    //check if target distance reached
    if (current_dist >= target_ticks)
    {
        stopBot();
        delay(200);
        resetEncoders();

        //State transition
        switch (currentState)
        {
            case DRIVING_LONG:
                //Just finished long vertical line
                if (current_pass >= total_passes)
                {
                    currentState = FINISHED;
                } else
                {
                    currentState = TURNING_1;
                    target_ticks = TICKS_PER_TURN;
                    if (current_turn_right) turnClockwise(SPEED_GRID_TURN);
                    else turnCounterClockwise(SPEED_GRID_TURN);
                }
                break;
            
            case TURNING_1:
                //Finished First 90 Degree Turn now doing short drive
                currentState = DRIVING_SHORT;
                //calculate spacing (wdith / (passes - 1))
                {
                    float spacing = grid_width_ft / (total_passes - 1);
                    target_ticks = spacing * TICKS_PER_FOOT;
                    driveStraight(SPEED_GRID_FWD);
                }
                break;

            case DRIVING_SHORT:
                //Finished driving short now turn same direction
                currentState = TURNING_2;
                target_ticks = TICKS_PER_TURN;
                if (current_turn_right) turnClockwise(SPEED_GRID_TURN);
                else turnCounterClockwise(SPEED_GRID_TURN);
                break;

            case TURNING_2:
                //Finished Second Turn ready for next length wise run
                currentState = DRIVING_LONG;
                current_pass++;
                //Flip Turn Direction for next loop
                current_turn_right = !current_turn_right;

                target_ticks = grid_len_ft * TICKS_PER_FOOT;
                driveStraight(SPEED_GRID_FWD);
                break;
        }
    }
}