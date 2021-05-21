# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 14:32:47 2021

@author: tanch

    if(check == 1):
                    if (potentialValue > seuil - 20):
                x = robot.x
                y = robot.y - 10
                v = [x,y]
                WPlist.append(v)
                WPManager = rob.WPManager(WPlist, epsilonWP)
                WPManager.xr= WPManager.WPList[index][0]
                WPManager.yr= WPManager.WPList[index][1]
            else:
                del WPlist[-1]
                a = list_add[n][0]
                b = list_add[n][1]
                x = robot.x + a
                y = robot.y + b
                v=[x,y]
                WPlist.append(v)
                WPManager = rob.WPManager(WPlist, epsilonWP)
                WPManager.xr= WPManager.WPList[index][0]
                WPManager.yr= WPManager.WPList[index][1]
                if (n == 8):
                    n=0
                else:
                    n = n +1
                    
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential
# A travailler 

#Cherche la valeur max dans les chemins parcourus
def max_mat(L_po):
    maxi = 0
    for i in L_po:
        for k in i:
            if(k > maxi):
                maxi = k
    return int(maxi)

#Cherche le rang ou se trouve le max
def max_list(L):
    maxi = 0
    k=0
    for i in range (len(L)):
        if(L[i] > maxi):
            maxi = L[i]
            k = i
    return k

#Compte pour chaque chemin (une ligne droite), le nombre de max
def list_max_mat(L_po):
    maxi = max_mat(L_po) - 2
    index_diag = []
    index_nb_diag = []
    for i in L_po:
        nb_max = 0
        n=0
        list_n = []
        for k in i:
            if(k > maxi ):
                nb_max = nb_max + 1
                list_n.append(n)
            n=n+1
        index_nb_diag.append(list_n)
        index_diag.append(nb_max)
    return index_diag , index_nb_diag

#Permet de defnir le point d arret
def point_stop(L_po , P_po):
    L , N= list_max_mat(L_po)
    id_max = max_list(L) #dans le liste des nombres de max, on cherche le chemin comprtant le plus de max
    v = P_po[id_max]
    a = len(N[id_max])
    a = int(a/2)
    a = int(N[id_max][a])
    v_n = v[a]
    return v_n

# robot
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=3, random=False)


# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2 #0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
#WPlist = [ [x0,y0] ]
#WPlist = [ [-10.0 , -20.0] , [-20.0 , -10.0] , [-20.0 , 0.0] , [0 , -20.0] , [10.0 , -20.0] , [-20.0 , 10] , [-20.0 , 20.0] ,
#  [-10.0 , 20.0] , [20.0 , -10.0] , [20.0 , 0.0] , [0.0 , 20.0] , [10.0 , 20.0] , [20.0 , 10.0] , [20.0 , 20.0]]

WPlist = [ [-20.0, -20.0],[-20.0 , 20.0] , [20.0 , 20.0] , [20.0 , -20.0], [-10.0 , -20.0] , [-10.0 , 10] ,
 [10.0 , 10.0] , [10.0 , -10.0] , [0.0 , -10.0] , [0.0 , 0.0] , [0.0 , 0.0]  ]

#threshold for change to next WP
epsilonWP = 0.2
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)


# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 75.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True

Listpotential = []
potentialdiag = []
Listpoints = []
points_parcourus = []
index=0
check = 0
n_List = len(WPlist)
x_stop=0
y_stop=0
seuil = 0
PotPLUS = 0

# loop on simulation time
for t in simu.t: 
    if (np.sqrt((robot.y-WPManager.yr)**2 + (robot.x-WPManager.xr)**2) < epsilonWP): # robot est dans zone en cercle
        if (index < n_List-2 ):
            Listpotential.append(potentialdiag)
            Listpoints.append(points_parcourus)
            potentialdiag = [] # chaque droite on a une liste de potentiel
            points_parcourus = []
            WPManager.xr= WPManager.WPList[index][0]
            WPManager.yr= WPManager.WPList[index][1]
            index = index + 1
        
        elif ( index < n_List-1 ):
            del WPlist[-1]
            v= point_stop(Listpotential ,Listpoints)
            WPlist.append(v)
            seuil = max_mat(Listpotential)
            print(seuil)
            WPManager = rob.WPManager(WPlist, epsilonWP)
            WPManager.xr= WPManager.WPList[index][0]
            WPManager.yr= WPManager.WPList[index][1]
            index = index + 1
            check = 1
            x_stop = v[0]
            y_stop = v[1]

            
            
        elif ( index == n_List - 1 ):

            if(check ==1):
                WPlist.append([x_stop , y_stop - 10 ])
                WPManager.xr= WPManager.WPList[index][0]
                WPManager.yr= WPManager.WPList[index][1]
                index = index + 1
                PotPLUS = 1
        

            

    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        potentialValue = pot.value([robot.x, robot.y])
        potentialdiag.append(potentialValue)
        #print(potentialValue)
        points_parcourus.append([robot.x, robot.y])
        
        # velocity control input
        Vr = 0.0
        k1= 1
        if( PotPLUS == 1):
            if(potentialValue < seuil - 5):
                Vr = 0
                check = 2
                print('a')
            
            else:
                Vr = k1*np.sqrt((WPManager.xr-robot.x)**2 + (WPManager.yr-robot.y)**2 )
                print('a')
        else:
            Vr = k1*np.sqrt((WPManager.xr-robot.x)**2 + (WPManager.yr-robot.y)**2 )
        
        
        # reference orientation
        thetar = theta0
        thetar = np.arctan2(WPManager.yr-robot.y,WPManager.xr-robot.x)
        
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)        
        
        
        
    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input        
        omegar = 0.0
        k2=10
        omegar= k2*(thetar-robot.theta)
    
    
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    
    
# end of loop on simulation time




# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************
# fig = plt.figure()
# ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
# ax.grid()
# ax.set_xlabel('x (m)')
# ax.set_ylabel('y (m)')

# robotBody, = ax.plot([], [], 'o-', lw=2)
# robotDirection, = ax.plot([], [], '-', lw=1, color='k')
# wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
# time_template = 'time = %.1fs'
# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
# potential_template = 'potential = %.1f'
# potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
# WPArea, = ax.plot([], [], ':', lw=1, color='b')

# thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
# xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
# yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

# def initAnimation():
#     robotDirection.set_data([], [])
#     robotBody.set_data([], [])
#     wayPoint.set_data([], [])
#     WPArea.set_data([], [])
#     robotBody.set_color('r')
#     robotBody.set_markersize(20)    
#     time_text.set_text('')
#     potential_text.set_text('')
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
    
# def animate(i):  
#     robotBody.set_data(simu.x[i], simu.y[i])          
#     wayPoint.set_data(simu.xr[i], simu.yr[i])
#     WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
#     thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
#     thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
#     robotDirection.set_data(thisx, thisy)
#     time_text.set_text(time_template%(i*simu.dt))
#     potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#     interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)