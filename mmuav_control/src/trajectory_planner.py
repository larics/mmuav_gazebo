#!/usr/bin/env python
import rospy
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rospkg import RosPack
import copy

class trajectory_planner():

    def __init__(self,Points, polyorder):

        self.Points=Points
        self.polyorder=polyorder
        self.segments=Points.size-1
        Aeq=self.gen_Aeq(self.gen_T(Points))
        deq=self.gen_deq(Points)
        Q=self.gen_Q(self.gen_T(Points))
        dQ=self.gen_dQ(self.gen_T(Points))
        print Aeq.shape, deq.shape


    def gen_Aeq(self,T,derivation_order=4):      #calculating matrix of equality constraints

        # initializing basic polynome of a given order

        polynom=np.poly1d(np.ones(self.polyorder+1))

        #calculating coeffitients of polynom derivatives

        poly_derivations=np.array(polynom.coef)

        for n in range(1, derivation_order+1):
            polynom=np.polyder(polynom)
            poly_derivations=np.vstack((poly_derivations,np.append(polynom.coef,np.zeros(n))))

        #skeleton of matrix for first segment
        if self.polyorder >=9:
            A1=np.zeros((derivation_order+1,self.polyorder+1))
            for i in range(derivation_order+1):
                A1[i,self.polyorder-i]=poly_derivations[i,self.polyorder-i]
        else:
            A1=np.zeros((3,self.polyorder+1))
            for i in range(3):
                A1[i,self.polyorder-i]=poly_derivations[i,self.polyorder-i]

        #skeleton of matrix for other segments
        Ai=np.zeros((derivation_order+1,self.polyorder+1))
        for i in range(derivation_order):
            Ai[i,self.polyorder-i-1]=-poly_derivations[i+1,self.polyorder-1-i]
        Ai[derivation_order,self.polyorder]=poly_derivations[0,self.polyorder]

        #constructing complete Aeq

        deltaAiRow = derivation_order + 2 #difference between rows of adjacent Ai, A(i+1) matrices

        #expected size of Aeq

        if self.polyorder >= 9:
            sizeAeq = derivation_order + deltaAiRow * (self.segments)
        else:
            sizeAeq = 6 + deltaAiRow * (self.segments-1)
            deltaAiRow=4

        for i in range(self.segments):
            Aj=np.array([])
            if i==0:
                Aj=np.copy(A1)
            else:
                Aj=np.vstack((np.zeros((deltaAiRow,self.polyorder+1)),Ai))
                deltaAiRow=deltaAiRow+derivation_order+2 #current rows + position + position + derivatives

            Ak=np.array([])
            for j in range(derivation_order+1):
                temporaryA=np.copy(poly_derivations[j,:])
                for k in range(self.polyorder-j+1):
                    temporaryA[self.polyorder-k-j]=temporaryA[self.polyorder-k-j]*T[i]**(k)
                if j==0:
                    Ak=np.copy(temporaryA)
                elif (i<self.segments-1 or self.polyorder>=9):
                    Ak=np.vstack((Ak,temporaryA))
                elif j<3:
                    Ak=np.vstack((Ak,temporaryA))

            Aj=np.vstack((Aj,Ak))
            (a,b)=Aj.shape
            Aj=np.vstack((Aj,np.zeros((sizeAeq-a,self.polyorder+1)))) #fill rest of the columns with zeros

            if i==0:
                Aeq=np.copy(Aj)
            else:
                Aeq=np.hstack((Aeq,Aj))
        return Aeq

    def gen_deq(self,Points,derivation_order=4):

        d1=np.array([])
        for i in range(self.segments):
            zeros=np.zeros(derivation_order)
            if self.polyorder >=9:
                if i==0:
                    dx=np.transpose(np.matrix(np.hstack(([Points[i].x], zeros,[Points[i+1].x], zeros))))
                    dy=np.transpose(np.matrix(np.hstack(([Points[i].y], zeros,[Points[i+1].y], zeros))))
                    dz=np.transpose(np.matrix(np.hstack(([Points[i].z], zeros,[Points[i+1].z], zeros))))

                else:
                    dx=np.vstack((dx,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))
                    dy=np.vstack((dy,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))
                    dz=np.vstack((dz,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))
            else:
                if i==0:
                    dx=np.transpose(np.matrix(np.hstack(([Points[i].x, 0, 0,Points[i+1].x], zeros))))
                    dy=np.transpose(np.matrix(np.hstack(([Points[i].y, 0, 0,Points[i+1].y], zeros))))
                    dz=np.transpose(np.matrix(np.hstack(([Points[i].z, 0, 0,Points[i+1].z], zeros))))
                elif i==self.segments-1:
                    dx=np.vstack((dx,np.transpose(np.matrix([Points[i].x, Points[i+1].x, 0, 0]))))
                    dy=np.vstack((dy,np.transpose(np.matrix([Points[i].y, Points[i+1].y, 0, 0]))))
                    dz=np.vstack((dz,np.transpose(np.matrix([Points[i].z, Points[i+1].z, 0, 0]))))
                else:
                    dx=np.vstack((dx,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))
                    dy=np.vstack((dy,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))
                    dz=np.vstack((dz,np.transpose(np.matrix(np.hstack(([Points[i].x],[Points[i+1].x], zeros))))))

        deq=np.vstack((dx,dy,dz))
        return deq

    def gen_Q(self,T,derivation_order=4): #cost function

        # initializing basic polynom of a given order

        polynom=np.poly1d(np.ones(self.polyorder+1))

        for n in range(1, derivation_order+1):
            polynom=np.polyder(polynom)


        polynom=np.matrix(polynom.coef)
        polynom=np.hstack((polynom,[np.zeros(derivation_order)]))
        #constructing matrix Qi(T)


        deltaQiRow = self.polyorder + 1     #alignment of adjacent Qi, Qi+1 matrices

        #expected size of Q
        sizeQ = deltaQiRow * self.segments

        for i in range(self.segments):


            #T matrix to be used for Hadamard product

            Qi=np.matmul(np.transpose(polynom),polynom)
            polynomT=np.hstack(([np.ones(self.polyorder+1-derivation_order)],[np.zeros(derivation_order)]))
            for j in range(self.polyorder+1-derivation_order):
                polynomT[0,j]=polynomT[0,j]*T[i]**(self.polyorder-j-derivation_order)

            matrixT=np.matmul(np.transpose(polynomT),polynomT)
            Qi=np.multiply(matrixT,Qi)
            if i==0:
                Qi=np.vstack((Qi,np.zeros((sizeQ-deltaQiRow,self.polyorder+1))))
                Q=np.copy(Qi)
            else:
                Qi=np.vstack((np.zeros((deltaQiRow,self.polyorder+1)),Qi))
                deltaQiRow=deltaQiRow+self.polyorder+1
                Qi=np.vstack((Qi,np.zeros((sizeQ-deltaQiRow,self.polyorder+1))))
                Q=np.hstack((Q,Qi))
        return Q

    def gen_dQ(self,T,derivation_order=4):

        # initializing basic polynom of a given order

        polynom=np.poly1d(np.ones(self.polyorder+1))

        for n in range(1, derivation_order+2):
            polynom=np.polyder(polynom)

        polynom=np.matrix(polynom.coef)
        polynom=np.hstack((polynom,[np.zeros(derivation_order+1)]))

        #constructing matrix dQi(T)

        deltadQiRow = self.polyorder + 1     #alignment of adjacent dQi, dQi+1 matrices

        #expected size of dQ
        sizedQ = deltadQiRow * self.segments

        for i in range(self.segments):
            dQi=np.matmul(np.transpose(polynom),polynom)
            #T matrix to be used for Hadamard product
            polynomT=np.hstack(([np.ones(self.polyorder-derivation_order)],[np.zeros(derivation_order+1)]))

            for j in range(self.polyorder-derivation_order):
                polynomT[0,j]=polynomT[0,j]*T[i]**(self.polyorder-j-derivation_order-1)


            matrixT=np.matmul(np.transpose(polynomT),polynomT)
            dQi=np.multiply(matrixT,dQi)
            if i==0:
                dQi=np.vstack((dQi,np.zeros((sizedQ-deltadQiRow,self.polyorder+1))))
                dQ=np.copy(dQi)
            else:
                dQi=np.vstack((np.zeros((deltadQiRow,self.polyorder+1)),dQi))
                deltadQiRow=deltadQiRow+self.polyorder+1
                dQi=np.vstack((dQi,np.zeros((sizedQ-deltadQiRow,self.polyorder+1))))
                dQ=np.hstack((dQ,dQi))
        return dQ

    def gen_T(self,Points):     #approximate segment times based on distance
        T=np.zeros(self.segments)
        for i in range(self.segments):
            tx=float(abs(Points[i].x-Points[i+1].x))
            ty=float(abs(Points[i].y-Points[i+1].y))
            tz=float(abs(Points[i].z-Points[i+1].z))
            T[i]=max(tx,ty,tz)
        return T

if __name__=='__main__':
    Points=np.array([Vector3(1,0,0)])
    Points=np.append(Points,Vector3(2,.5,0))
    Points=np.append(Points,Vector3(3,.5,0))
    Points=np.append(Points,Vector3(4,0,0))
    Points=np.append(Points,Vector3(5,0,0))
    trajectory=trajectory_planner(Points,9)
