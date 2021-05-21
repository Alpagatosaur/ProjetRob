# -*- coding: utf-8 -*-
"""
Created on Fri Apr  9 21:42:31 2021

@author: tanch

"""

"""
                            Bibliotheques
"""
import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

#determine le point ou se trouve le max potential
def vecteur_stop(List_pot , List_pts):
    
    #definition du plus grand potential lu
    max_pot = 0
    for i in range (len(List_pot)):
        for j in range(len(List_pot[i])):
            if (List_pot[i][j] > max_pot):
                
                max_pot = List_pot[i][j]

    #Cb pour chaque chemin il y a t il de valeur proche du max
    Cpt_val_max = []
    List_vec_max = []
    for i in range (len(List_pot)):
        L_val = []
        val=0
        for j in range(len(List_pot[i])):
            if (List_pot[i][j] > max_pot - 10 ):
                val = val + 1
                
            else:
                L_val.append(val)
                if(val>0):
                    Cpt_val_max.append(val)
                    u = int(val/2)
                    List_vec_max.append(List_pts[i][j-u]) #debut ou on a lu le max
                val = 0
        
    #Quel chemin contient le plus de valeur proche du max
    val = 0
    vecteur = []
    for n in range (len(Cpt_val_max)):
        if(Cpt_val_max[n] > val):
           val = Cpt_val_max[n]
           vecteur=List_vec_max[n]
    return vecteur , max_pot

#check quel sommet sommes nous ?

def check_carre (carre, i):
    for k in carre:
        if (k == i):
            return True
    return False


# robot
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=3, random=True)


# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
WPlist = [ [-20.0, -20.0],[-20.0 , 20.0] , [20.0 , 20.0] , [20.0 , -20.0], [-10.0 , -20.0] , [-10.0 , 10] ,
 [10.0 , 10.0] , [10.0 , -10.0] , [0.0 , -10.0] , [0.0 , 0.0] , [0.0 , 0.0] ]
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

#Variables

n_List = len(WPlist) #taille de WPlist

Listpotential = []
potentialdiag = []
Listpoints = []
pointsdiag = []
index=0
x_stop=0
y_stop=0
ListCercle=[]
PotPLUS = 0
STOP = 0
Part = 0
carre = 1
historique_carre = []
cpt_carre = 1
potentialValue=0
"""
                                Simulation
"""
for t in simu.t: 
    
    if (np.sqrt((robot.y-WPManager.yr)**2 + (robot.x-WPManager.xr)**2) < epsilonWP): # robot est dans zone en cercle
        
    # Robot parcourt les pts definis
        if (index < n_List-2):
            Listpotential.append(potentialdiag)
            Listpoints.append(pointsdiag)
            potentialdiag = [] # chaque droite on a une liste de potentiel
            pointsdiag = []
            WPManager.xr= WPManager.WPList[index][0]
            WPManager.yr= WPManager.WPList[index][1]
            index = index + 1
        
        #Fin mapping de la carte, on cherche le moment ou le potential lu etait le max
        elif ( index < n_List-1):
            
            del WPlist[-1]
            v , seuil= vecteur_stop(Listpotential ,Listpoints)
            WPlist.append(v)
            print(seuil)
            WPManager = rob.WPManager(WPlist, epsilonWP)
            WPManager.xr= WPManager.WPList[index][0]
            WPManager.yr= WPManager.WPList[index][1]
            index = index + 1
        
        #On demande au robot de descendre jusqu au seuil du potential max
        elif ( index == n_List-1):
            Part = 1
            WPlist.append([robot.x +20, robot.y - 20 ])
            WPManager = rob.WPManager(WPlist, epsilonWP)
            WPManager.xr= WPManager.WPList[index][0]
            WPManager.yr= WPManager.WPList[index][1]
            index = index + 1


            
        elif( index > n_List-1 & int(Vr) == 0):
            
            
            if(carre == 1):
                if (check_carre ( historique_carre , 1) == False):
                    historique_carre.append(carre)
                    ListCercle.append([robot.x, robot.y])
                    WPlist.append([robot.x - 20 , robot.y])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue

                
                
            elif(carre == 2):
                if (check_carre ( historique_carre , 2) == False):
                    historique_carre.append(carre)
                    ListCercle.append([robot.x, robot.y])
                    WPlist.append([robot.x  , robot.y + 20])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue

                
            elif(carre == 3):
                if (check_carre ( historique_carre , 3) == False):
                    historique_carre.append(carre)
                    ListCercle.append([robot.x, robot.y])
                    WPlist.append([robot.x +20 , robot.y])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue

                
            elif(carre == 4):
                if (check_carre ( historique_carre , 4) == False):
                    historique_carre.append(carre)
                    ListCercle.append([robot.x, robot.y])
                    WPlist.append([robot.x  , robot.y - 20])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue
                    
                    
            elif(carre == 5):
                if (check_carre ( historique_carre , 5) == False):
                    historique_carre.append(carre)
                    x_stop , y_stop = ListCercle[2]
                    WPlist.append([x_stop , y_stop])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue
                
            elif(carre == 6):
                if (check_carre ( historique_carre , 6) == False):
                    historique_carre.append(carre)
                    x_stop , y_stop = ListCercle[3]
                    WPlist.append([x_stop , y_stop])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue
                    
            elif(carre == 7):
                if (check_carre ( historique_carre , 7) == False):
                    historique_carre.append(carre)
                    x_stop , y_stop = ListCercle[1]
                    WPlist.append([x_stop , y_stop])
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue
                    
                    
            elif(carre == 8):
                if (check_carre ( historique_carre , 8) == False):
                    historique_carre.append(carre)
                    x1 , y1 = ListCercle[1]
                    x2 , y2 = ListCercle[3]
                    
                    x0 = (x1 + x2)/2
                    y0 = (y1 + y2)/2
                    WPlist.append([x0 , y0])
                    
                    WPManager = rob.WPManager(WPlist, epsilonWP)
                    WPManager.xr= WPManager.WPList[index][0]
                    WPManager.yr= WPManager.WPList[index][1]
                    index = index + 1
                    carre = 0
                    Part = 1
                    seuil = potentialValue

        

    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        potentialValue = pot.value([robot.x, robot.y])
        potentialdiag.append(potentialValue)
        #print(potentialValue)
        pointsdiag.append([robot.x, robot.y])
        
        # velocity control input
        k1= 1
        if (Part == 0):
            Vr = k1*np.sqrt((WPManager.xr-robot.x)**2 + (WPManager.yr-robot.y)**2 )
            
        elif(Part == 1):
            
            if(potentialValue < seuil - 4):
                Vr = 0.0
                epsilonWP = 30 
                Part = 2
            else:
                Vr = k1*np.sqrt((WPManager.xr-robot.x)**2 + (WPManager.yr-robot.y)**2 )
        
        elif(Part == 2):
            if(potentialValue > seuil - 4):
                Vr = 0.0
                epsilonWP = 30 
                Part = 3
            else:
                Vr = k1*np.sqrt((WPManager.xr-robot.x)**2 + (WPManager.yr-robot.y)**2 )
                
                
        elif(Part == 3):
            
            if(potentialValue < seuil - 4):
                Vr = 0.0
                epsilonWP = 30 
                cpt_carre = cpt_carre+1
                carre = cpt_carre
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

#simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



#simu.plotPotential3D(5)


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




