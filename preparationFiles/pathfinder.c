//
//  pathfinder.c
//  
//
//  Created by Jonathan Erskine on 03/04/2018.
//

#include "pathfinder.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int compare(const void *pp0, const void *pp1)
{
    int i0 = **(int **)pp0;
    int i1 = **(int **)pp1;
    if(i0 > i1)return -1;
    if(i0 < i1)return  1;
    return 0;
}

//Movement function
int movement(int nPanels, int nsqPanels, int position, int oldPosition, int north [], int south [], int east [], int west [], int occupied [], int border [], int obstructed [], int corner [], int confidence [] )
/* Function takes current position and outputs new position based on
 logic tree defined within the function.
 
 Inputs (all integer arrays of length 'n' where n is the number of panels on the grid)
 ------
 nPanels: number of panels present in the current square grid, including outside row of border panels
 nsqPanels: square root of number of panels; i.e. the number of panels in one row/column
 position: Panel identifier; Panel which the drone is currently occupying
 oldPosition: Panel identifier; Panel which the drone previously occupied
 north: Panel identifier; Panel which is North of the current panel
 south: Panel identifier; Panel which is South of the current panel
 east: Panel identifier; Panel which is East of the current panel
 west: Panel identifier; Panel which is West of the current panel
 occupied: 1 or 0; Identifies whether or not panel is occupied (True or False)
 border: 1 or 0; Identifies whether or not panel is a border panel (True or False)
 obstructed: 1 or 0; Identifies whether or not panel is obstructed (True or False)
 corner: 1 or 0; Identifies whether or not panel is a corner (inside border) (True or False)
 confidence: Value which is manipulated based on the characteristics of the panel;
 
 Outputs
 -------
 newPosition: Panel identifier; Panel which the drone now occupies
 */
{
    
    //Define variables
    int routeN, routeS, routeE, routeW;
    int i;
    int futurePosition;
    
    //Determine N,S,E,W for each panel
    for (i = 0; i < nPanels ; ++i ){
        
        north[i] = (i + nsqPanels);
        south[i] = (i - nsqPanels);
        east[i] = (i + 1);
        west[i] = (i - 1);
        
    }
    
    // Determine N,S,E,W for current panel
    int mn = north[position];
    int ms = south[position];
    int me = east[position];
    int mw = west[position];
    
    //Determine if N,S,E,W is obstructed/border for current position
    if (mn < 0 || mn > (nPanels -1) || obstructed[mn] == 1 || border[mn] == 1 )
    {
        int routeN = 0;
    }
    else{
        int routeN = 1;
    }
    
    if (ms < 0 || ms > (nPanels -1) || obstructed[ms] == 1 || border[ms] == 1 )
    {
        int routeS = 0;
    }
    else{
        int routeS = 2;
    }
    
    if (me < 0 || me > (nPanels -1) || obstructed[me] == 1 || border[me] == 1 )
    {
        int routeE = 0;
    }
    else{
        int routeE = 3;
    }
    
    if (mw < 0 || mw > (nPanels -1) || obstructed[mw] == 1 || border[mw] == 1 )
    {
        int routeW = 0;
    }
    else{
        int routeW = 4;
    }
    
    //Generate list of available moves - N,S,E,W
    int choiceList[] = {mn, ms, me, mw};
    
    /*Cheack first if a route is obstructed, and if not check if it is a move forward
            - If yes, reduce confidence in that square - Move will be favourable*/
    if (routeN != 0)
    {
        if (mn == (2*nsqPanels) + oldPosition)
        {
            confidence[mn] = confidence[mn] - 4;
        }
    }
    
    if (routeS != 0)
    {
        if (ms == (oldPosition - 2*nsqPanels))
        {
            confidence[ms] = confidence[ms] - 4;
        }
    }
    
    if (routeE != 0)
    {
        if (ms == (oldPosition + 2))
        {
            confidence[me] = confidence[me] - 4;
        }
    }
    
    if (routeW != 0)
    {
        if (mw == (oldPosition - 2))
        {
            confidence[mw] = confidence[mw] - 4;
        }
    }

    // Produce list of confidence for each possible move
    int confList[] = {confidence[mn],confidence[ms],confidence[me],confidence[mw]};

    // Extract largest and smallest values from list of confidence
    int largest = confList[0];
    int smallest = confList[0];
    for (i = 1; i < 4; i++)
    {
        if (largest < confList[i])
        {
            largest = confList[i];
        }
        if (smallest > confList[i])
        {
            smallest = confList[i];
        }
    }

    // If all confidence equal, pick a random direction
    if (smallest == largest) {
        
        srand(time(NULL));
        int randomIndex = rand() % 4;
        int futurePosition = choiceList[randomIndex];
    
    // Else, pick lowest value of confidence from list
    } else
    {
        // Reordering confList from high to low and applying same indexing to choiceList
        int *pConfList[4];
        size_t i, j, k;
        int tConfList, tChoiceList;
        
        /* create array of pointers to confList[] */
        for(i = 0; i < sizeof(confList)/sizeof(confList[0]); i++)
            pConfList[i] = &confList[i];
        
        /* sort array of pointers */
        qsort(pConfList, sizeof(confList)/sizeof(confList[0]), sizeof(pConfList[0]), compare);
        
        /* reorder confList[] and choiceList[] according to the array of pointers */
        for(i = 0; i < sizeof(confList)/sizeof(confList[0]); i++) {
            if(i != pConfList[i]-confList){
                tConfList = confList[i];
                tChoiceList = choiceList[i];
                k = i;
                while(i != (j = pConfList[k]-confList)){
                    confList[k] = confList[j];
                    choiceList[k] = choiceList[j];
                    pConfList[k] = &confList[k];
                    k = j;
                }
                confList[k] = tConfList;
                choiceList[k] = tChoiceList;
                pConfList[k] = &confList[k];
            }
        }
        /* Assign the future position which is the last element in the newly sorted choiceList
                i.e. the direction with the least confidence*/
        futurePosition = choiceList[3];
    }
    return futurePosition;
    }

int main(void)
{
    // Set initial conditions
    int nPanels = 81; // Number of panels on grid
    int nsqPanels = sqrt(nPanels); // Square root of number of panels;
    //printf("%d\n",nsqPanels);
    int startPosition = nsqPanels + 1; // Initial position of drone
    int position = startPosition;
    int i;
    
    // Arrays which store information for each panel
    int north [nPanels];
    int south [nPanels];
    int east [nPanels];
    int west [nPanels];
    int occupied [nPanels];
    int border [nPanels];
    int obstructed [nPanels];
    int corner [nPanels];
    int confidence [nPanels];
    
    // Add borders
    for (i = 0; i < nsqPanels ; ++i ){
        
        border[i] = 1;
        border[(i*nsqPanels)] = 1;
        border[((i*nsqPanels)-1)] = 1;
    }
    for (i = (nPanels-nsqPanels); i < nPanels ; ++i ){
        
        border[i] = 1;
    }
    
    //Add corners
    corner[nsqPanels+1] = 1;
    corner[2*nsqPanels-2] = 1;
    corner[nPanels - nsqPanels - 2] = 1;
    corner[nPanels - 2*nsqPanels+1] = 1;
    
    for ( i = 0; i < nPanels ; ++i ) {
        
        //Add movement functions for each panel
        north[i] = (i + nsqPanels);
        south[i] = (i - nsqPanels);
        east[i] = (i + 1);
        west[i] = (i - 1);
        
        //Set obstructions
        obstructed[i] = 0;
        
        //Set base level of confidence
        confidence[i] = 5;
    }
    
    // For testing purposes, generates 10 example iterations
    int iteration = 0;
    while (iteration < 10) {
        for ( i = 0; i < nPanels ; ++i ) {
            if (obstructed[i] == 1) {
                confidence[i] = 50;
                obstructed[i] = 0;
            }
            if (border[i] == 1) {
                confidence[i] = 50;
            }
            if (corner[i] == 1) {
                confidence[i] = 10;
            }
        }
        
        int oldPosition = position;
        confidence[oldPosition] = confidence[oldPosition] + 10;
        int newPosition = movement(nPanels, nsqPanels, position,oldPosition, north, south, east, west, occupied, border, obstructed, corner, confidence);
        
        for ( i = 0; i < nPanels ; ++i )
        {
            occupied[i] = 0;
            confidence[i] = confidence[i] - 1;
            
            if ((confidence[i] < 5))
            {
                confidence[i] = 5;
            }
        }
        
        position = newPosition;
        occupied[position] = 1;
        printf("%d\n",iteration);
        printf("%d\n",position);
        iteration ++;
    }
    return 0;
    
    
}
