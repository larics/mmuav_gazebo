#!/usr/bin/env python
import math, copy
#import mathutils
import numpy as np
import time
    
def spin_transform(theta,d,alpha,a):
    #This function can be used accorss systems
    #Calculate transfer matrix w.r.t. DH table row |theta | d | alpha | a |
    T = np.matrix(((math.cos(theta), -math.cos(alpha)*math.sin(theta),math.sin(alpha)*math.sin(theta), a*math.cos(theta)),(math.sin(theta), math.cos(alpha)*math.cos(theta), -math.sin(alpha)*math.cos(theta), a*math.sin(theta)),(0, math.sin(alpha), math.cos(alpha), d),(0, 0,0,1)))
    return T

def dk_calculate(q,DH):
    #This function can be used accorss systems
    # we are ignoring potential translation joints
    #DH is a standard Denavit Hartenberg table
    (m, n) = DH.shape
    T = np.eye(4)
    T_array = []
    for i in range(m):
        if DH[i,4]>0:
            T = T*spin_transform(DH[i,0]+q[i],DH[i,1],DH[i,2],DH[i,3])
        else:
            T = T*spin_transform(DH[i,0],DH[i,1]+q[i],DH[i,2],DH[i,3])
        T_array.append(T)
    return T_array


def ik_2R_MMUAV(Goal,L1,L2):
    #This function can be used accorss systems
    #The function calculates the inverse kinematic solution of the 2DOF scara manipulator
    #DH is a standard Denavit Hartenberg table
    q=np.zeros((3,2))
    for i in range(2):
        n=i
        if i:
            q[1][n]= np.arccos((Goal[0]**2+Goal[1]**2-L1**2-L2**2)/(2*L1*L2+1e-16))#q2 solution
        else:
            q[1][n]= -np.arccos((Goal[0]**2+Goal[1]**2-L1**2-L2**2)/(2*L1*L2+1e-16))#q2 solution
        p1 = L1+L2*np.cos(q[1][n])
        p2 = L2*np.sin(q[1][n])
        q[0][n]=np.arctan2(p1*Goal[1]-p2*Goal[0],p1*Goal[0]+p2*Goal[1])#  (np.pi/2-q[2][n] #q1 solution
        q[2][n]=-(q[0][n]+q[1][n])
    return q

def ik_2R_MMUAV_closest(q_start,Goal,DH,L1,L2):
    #This function can be used accorss systems
    #The function returns the closes solution of IK problem w.r.t. q_start
    #DH is a standard Denavit Hartenberg table
    min_dist = np.infty
    min_i = 0
    q=ik_2R_MMUAV(Goal,L1,L2)
    for i in range(2):
        if np.linalg.norm(np.transpose(np.matrix(Goal))-dk_calculate(q[:,i],DH)[1][0:3,3]) < 1e-2:
            qdistance = q_start - np.transpose(q[:,min_i])
            qdistance = ( qdistance + np.pi) % (2.0 * np.pi ) - np.pi
            # Wrap to PI for qdistance
            if np.linalg.norm(qdistance) < min_dist:
                min_dist = np.linalg.norm(qdistance)
                min_i = i          
    return q[:,min_i]

def ik_both_arms(q01_list, q02_list, goal, L1, L2, L3):
    # First define DH table      
    # | theta | d | alpha | a | R/T
    DH = np.matrix('0.0 0.0 0.0 1.0 1.0;0.0 0.0 0.0 1.0 1.0')
    DH[0,3] = L1
    DH[1,3] = L2
    L3 = 0.08

    q01 = [q01_list[0], q01_list[1], q01_list[2]]
    q02 = [q02_list[0], q02_list[1], q02_list[2]]
    Q1 = copy.deepcopy(q01)
    Q2 = copy.deepcopy(q02)
    #We want to use the null space of the arms to find the closes solution of inverse kinematics
    DQ_min = np.infty
    #Passive joint at the endeffector can rotate +/-30 degrees 
    N=90
    PHI=100.0
    # -64, -34
    for phi in range(-65,-33,3):
        phi=(phi*1)*np.pi/180
        #Find the goal position for the arms
        #We are searching for a circle around the goal, where the arm can reach, keeping in mind the angle N
        #goal = [0,0]
        goal1 = ((goal[0]+L1+L2-L3*np.cos(phi)),-(goal[1]+L3*np.sin(phi)),0)
        q01=(q01_list[0], q01_list[1], 0.0)#Arm A
        #t0 = time.time()
        q1=ik_2R_MMUAV_closest(q01,goal1,DH,L1,L2)
        #t1 = time.time()
        q02=(q02_list[0], q02_list[1], 0.0)#Arm B
        goal2 = (-goal[0]+L1+L2-L3*np.cos(phi),(goal[1]-L3*np.sin(phi)),0)
        #t2 = time.time()
        q2=ik_2R_MMUAV_closest(q02,goal2,DH,L1,L2)
        #t3 = time.time()
        #print "Left: ", t1-t0, "Right: ", t3-t2
        DQ=(q1[0]-q01[0])**2+(q1[1]-q01[1])**2+(q1[2]-q01[2])**2+(q2[0]-q02[0])**2+(q2[1]-q02[1])**2+(q2[2]-q02[2])**2
        #print(DQ)
        #Find minimal distance in Q space
        if (DQ<DQ_min):
            DQ_min=DQ
            #Q1=[0,0,0]
            #Q2=[0,0,0]
            for i in range(2):
                Q1[i] = q1[i]
                Q2[i] = q2[i]
            #Q1=list(q1)
            #Q2=list(q2)
            PHI=phi
            Q1[2] = -PHI
            Q2[2] = -PHI

    #print "PHI: ", PHI*180/math.pi

    # Put phi in q1 and q2
    #print Q1    
    #print PHI*180.0/np.pi
    return [Q1, Q2]



"""
########################################### MAIN FUNCTION #####################################
#Link sizes
L1 = 0.095
L2 = 0.0527
L3 = 0.05 # Passive joint length
# First define DH table      
# | theta | d | alpha | a | R/T
DH = np.matrix('0.0 0.0 0.0 1.0 1.0;0.0 0.0 0.0 1.0 1.0')
DH[0,3] = L1
DH[1,3] = L2
#We want to use the null space of the arms to find the closes solution of inverse kinematics
DQ_min = np.infty
#Passive joint at the endeffector can rotate +/-30 degrees 
N=30
for phi in range(-N,N):
    phi=phi*np.pi/180
    #Find the goal position for the arms
    #We are searching for a circle around the goal, where the arm can reach, keeping in mind the angle N
    goal = [0,0]
    goal1 = ((goal[0]+L1+L2-L3*np.cos(phi)),-(goal[1]+L3*np.sin(phi)),0)
    q01=(0.74,-1.6,0.0)#Arm A
    q1=ik_2R_MMUAV_closest(q01,goal1,DH,L1,L2)
    q02=(0.74,-1.6,0.0)#Arm B
    goal2 = (-goal[0]+L1+L2-L3*np.cos(phi),(goal[1]-L3*np.sin(phi)),0)
    q2=ik_2R_MMUAV_closest(q02,goal2,DH,L1,L2)
    DQ=(q1[0]-q01[0])**2+(q1[1]-q01[1])**2+(q1[2]-q01[2])**2+(q2[0]-q02[0])**2+(q2[1]-q02[1])**2+(q2[2]-q02[2])**2
    print(DQ)
    #Find minimal distance in Q space
    if (DQ<DQ_min):
        DQ_min=DQ
        Q1=q1
        Q2=q2
        PHI= phi
#Initial value of joints
#q01=(-np.pi/2,0.0,PHI)
#q02=(0,0.0,PHI)

print Q1
print Q2
print phi
"""