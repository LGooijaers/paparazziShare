#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 22 10:43:21 2018

@author: Jonny
"""
import numpy as np
from matplotlib import pyplot
import random

# Define grid sizing and arena geometry
n_panels = 81
nsq_panels = int(np.sqrt(n_panels))

x_start, x_end = 0.0, 10.0  # in metres
y_start, y_end = 0.0, 10.0  # in metres

# Generate discretized grid spacing
x = np.linspace(x_start, x_end, nsq_panels+1)
y = np.linspace(y_start, y_end, nsq_panels+1)
X, Y = np.meshgrid(x, y)

# Create an object class called a panel which stores all relevant information
class class_panel:
    """
    Stores panel information.
        * Panel x and y beginning and end coordinates
        * Length
        * Mid-point
        * Neighbouring panel identifiers and associated direction (N,S,E,W)
        * Panel characteristics, primarily panel confidence based on;
            - Is it occupied by the drone?
            - Is it a border panel?
            - Is is obstructed?
            - Is it a corner?
    """
    def __init__(self, x_start, y_start, x_end, y_end):
        """
        Initialisation.
        --------------
        Inputs
        ------
        x_start:    x-coordinate at the start of the panel.(float)
        y_start:    y-coordinate at the start of the panel.(float)
        x_end:      x-coordinate at the end of the panel.(float)
        y_end:      y-coordinate at the end of the panel.(float)
        """
        self.x_start, self.y_start = x_start, y_start  # panel starting-point
        self.x_end, self.y_end = x_end, y_end  # panel ending-point
        self.length = x_end-x_start  # length
        self.x_mid = (x_start+x_end)/2  # 'x' mid-point
        self.y_mid = (y_start+y_end)/2  # 'y' mid-point
        # Corner Points
        self.lbx = x_start
        self.lby = y_start
        self.rbx = x_end
        self.rby = y_start
        self.ltx = x_start
        self.lty = y_end
        self.rtx = x_end
        self.rty = y_end
        self.N, self.E, self.S, self.W = 0, 0, 0, 0  # Initialising headings
        #  Other Parameters
        self.confidence = 5.0  
        self.occupied = False
        self.border = False
        self.obstructed = False
        self.corner = False


def func_panels(x, y, N):
    """
    Generates an array of 'n' number of panels using the discretized geometry.

    Inputs
    ------
    x: x-coordinate of the panel. (Array of floats)
    y: y-coordinate of the panel. (Array of floats)
    N: Number of panels. (Integer)

    Outputs
    -------
    panels:List of corresponding panels (Array of 'panels')
    """

    panels = np.empty(N, dtype=object)  # Empty array called panels

    # Creates the numbering scheme for all panels on the grid
    # Starts from bottom row, left to right then repeats for each row

    k = 0

    for i in range(n_panels):

        j = i

        if i < nsq_panels:

            panels[i] = class_panel(x[j], y[k], x[j+1], y[k+1])

        else:

            j = j - (int(i/nsq_panels) * nsq_panels)
            k = int((i)/nsq_panels)
            panels[i] = class_panel(x[j], y[k], x[j+1], y[k+1])

        # Neighboring panels stored in variable which denotes direction
        panels[i].N = int(i + nsq_panels)
        panels[i].S = int(i - nsq_panels)
        panels[i].E = int(i + 1)
        panels[i].W = int(i - 1)

    return panels


def movement(position, position_memory, i):
    """
    Take current location and reset "occupied" marker to new location

    Inputs
    ------
    position: location of drone on grid. (Integer)
    position memory: list
    i: current iteration

    Outputs
    -------
    position: Overwrites current location
    """

    #  Find old position
    old_position = int(position_memory[i-1])

    #  Set N,S,E,W panels for current position
    mn = panels[position].N
    ms = panels[position].S
    me = panels[position].E
    mw = panels[position].W

    #  Checking if N,S,E,W are blocked
    if (mn < 0 or mn > (n_panels - 1) or
            panels[mn].obstructed is True or
            panels[mn].border is True):

        routeN = 5

    else:

        routeN = 0

    if (ms < 0 or ms > (n_panels - 1) or
            panels[ms].obstructed is True or
            panels[ms].border is True):

        routeS = 5

    else:

        routeS = 1

    if (me < 0 or me > (n_panels - 1) or
            panels[me].obstructed is True or
            panels[me].border is True):

        routeE = 5

    else:

        routeE = 2

    if (mw < 0 or mw > (n_panels - 1) or
            panels[mw].obstructed is True or
            panels[mw].border is True):

        routeW = 5

    else:

        routeW = 3

    #  List of all unblocked routes
    route_list = [routeN, routeS, routeE, routeW]
    possible_routes = [i for i, e in enumerate(route_list) if e != 5]
    direction_list = [mn, ms, me, mw]
    choice_list = []
    choice_list = [direction_list[i] for i in possible_routes]
    print(choice_list)
    if routeN != 5:

        if mn == (2*nsq_panels) + old_position:

            panels[mn].confidence = panels[mn].confidence - 4

    if routeS != 5:

        if ms == old_position - (2*nsq_panels):

            panels[ms].confidence = panels[ms].confidence - 4

    if routeE != 5:

        if me == old_position + 2:

            panels[me].confidence = panels[me].confidence - 4

    if routeW != 5:

        if mw == old_position - 2:

            panels[mw].confidence = panels[mw].confidence - 4

    # Set current position to undesirable level of confidence
    panels[position].confidence = panels[position].confidence + 10

    # Create list of panel confidence corresponding to each possible heading
    conf_list = []

    for i in range(n_panels):

        conf_list = np.append(conf_list, panels[i].confidence)

    conf_list = [conf_list[i] for i in choice_list]

    # Take random direction if minimum confidence exists in two or more panels
    rand_direction = random.choice(choice_list)

    # If there is discrepancy between panels, choose lowest confidence panel
    if max(conf_list) != min(conf_list):

        choice_array = np.array(choice_list)
        conf_array = np.array(conf_list)
        sorted_conf = conf_array.argsort()
        path_of_least_resistance = choice_array[sorted_conf]

        # Selects the lowest (first) value in this list
        future_position = path_of_least_resistance[0]

    # Or else pick a random direction
    else:

        rand_direction = random.choice(choice_list)
        future_position = rand_direction

    position = future_position

    return position


#  create panels
panels = func_panels(x, y, n_panels)

#  initial conditions
start_position = int(nsq_panels+1)
position = int(start_position)
position_memory = []
panels[position].occupied = True

#  add borders
for i in range(nsq_panels):

    panels[i].border = True
    panels[int(i*nsq_panels)].border = True
    panels[int((i*nsq_panels)-1)].border = True

for i in range(n_panels-nsq_panels, n_panels):

        panels[i].border = True

# Add Corners
panels[nsq_panels + 1].corner = True
panels[2*nsq_panels - 2].corner = True
panels[n_panels - nsq_panels - 2].corner = True
panels[n_panels - 2 * nsq_panels + 1].corner = True

#  Add randomly generated obstructions
number_of_obstructions = 7
obstructions = []
int_obstructions = []
rand_numb = []
o = 0
while o < number_of_obstructions:
    obstructions = (np.append(obstructions,
                              int(n_panels - random.choice(range(n_panels)))))
    o += 1

obstructions = list(map(int, obstructions))

for i in obstructions:
    panels[i].obstructed = True

#  Begin Flight
iteration = 0
total_conf = []

# For visulisation purposes, simulate a run of 40 grid displacements
while iteration < 40:

    print(iteration)

    # Plot Arena and Position
    pyplot.show()
    pyplot.figure(figsize=(7, 7))
    pyplot.xlim(x_start, x_end)
    pyplot.ylim(y_start, y_end)
    pyplot.xlabel('x', fontsize=20)
    pyplot.ylabel('y', fontsize=20)
    pyplot.plot(X[0], Y[1], color='g', linestyle='-', linewidth=3)
    pyplot.plot(X[0], Y[nsq_panels-1], color='g', linestyle='-', linewidth=3)
    pyplot.plot(X[:, 1], Y[:, 0], color='g', linestyle='-', linewidth=3)
    pyplot.plot(X[:, nsq_panels-1], Y[:, 0], color='g',
                linestyle='-', linewidth=3)
    for panel in panels:
        pyplot.plot(panel.lbx, panel.lby,
                    linestyle='-', linewidth=1, marker='+',
                    markersize=10, color='k')
        pyplot.plot(panel.rbx, panel.rby,
                    linestyle='-', linewidth=1, marker='+',
                    markersize=10, color='k')
        pyplot.plot(panel.ltx, panel.lty,
                    linestyle='-', linewidth=1, marker='+',
                    markersize=10, color='k')
        pyplot.plot(panel.rtx, panel.rty,
                    linestyle='-', linewidth=1, marker='+',
                    markersize=10, color='k')

    # Plot occupied panel (green)
    for i in range(n_panels):
        if panels[i].occupied is True:
            pyplot.plot(panels[i].x_mid, panels[i].y_mid,
                        linestyle='-', linewidth=1, marker='o',
                        markersize=10, color='g')

        # Assign panel obstruction & border confidence
        if panels[i].obstructed is True:
            panels[i].confidence = 40
            panels[i].obstructed = False  # Enables memory loss of obstacles

        if panels[i].border is True:
            panels[i].confidence = 50
            pyplot.plot(panels[i].x_mid, panels[i].y_mid,
                        linestyle='-', linewidth=1, marker='s',
                        markersize=50, color='b')

        if panels[i].border is False:
            markersize_n = panels[i].confidence
            if markersize_n < 20:
                pyplot.plot(panels[i].x_mid, panels[i].y_mid,
                            linestyle='-', linewidth=1, marker='o',
                            markersize=markersize_n-5, color='r')
            else:
                pyplot.plot(panels[i].x_mid, panels[i].y_mid,
                            linestyle='-', linewidth=1, marker='s',
                            markersize=markersize_n-5, color='r')

        if panels[i].corner is True:
            panels[i].confidence = 10

            pyplot.plot(panels[i].x_mid, panels[i].y_mid,
                        linestyle='-', linewidth=1, marker='d',
                        markersize=10, color='orange')

    # Move Position Counter
    old_position = position
    position_memory = np.append(position_memory, old_position)
    new_position = movement(position, position_memory, iteration)

    # Sum confidence of proposed route
    total_conf = np.append(total_conf, panels[new_position].confidence)
    position = int(new_position)

    # reset panel occupation and reduce confidence of all squares by 1
    for i in range(n_panels):
        panels[i].occupied = False
        panels[i].confidence = panels[i].confidence - 1

        if panels[i].confidence < 5.0:

            panels[i].confidence = 5.0

    panels[position].occupied = True

    pyplot.show()

    iteration += 1

print(total_conf)
