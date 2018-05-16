#!/usr/bin/env python
import rospy
import numpy as np
import tf
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
from rospkg import RosPack
import copy

class TrajectoryPlanner():

    def __init__(self, polyorder):

        self.keyframes=[];
        self.segments=0
        rospy.Subscriber(   'multi_dof_trajectory', MultiDOFJointTrajectory,\
                            self.keyframes_callback,queue_size=1)
        self.polyorder=polyorder



    def generate_Aeq(self,T,derivation_order=4):     # equality constraints matrix

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

    def generate_deq(self,keyframes,derivation_order=4,derivation_order_yaw=2):

        for i in range(self.segments):
            zeros=np.zeros(derivation_order)
            zeros_yaw=np.zeros(derivation_order_yaw)
            if self.polyorder >=9:
                if i==0:
                    dx=np.transpose(np.matrix(np.hstack(([keyframes[0,i]], zeros,[keyframes[0,i+1]], zeros))))
                    dy=np.transpose(np.matrix(np.hstack(([keyframes[1,i]], zeros,[keyframes[1,i+1]], zeros))))
                    dz=np.transpose(np.matrix(np.hstack(([keyframes[2,i]], zeros,[keyframes[2,i+1]], zeros))))
                    dyaw=np.transpose(np.matrix(np.hstack(([keyframes[3,i]], zeros_yaw,[keyframes[3,i+1]],zeros_yaw))))

                else:
                    dx=np.vstack((dx,np.transpose(np.matrix(np.hstack(([keyframes[0,i]],[keyframes[0,i+1]], zeros))))))
                    dy=np.vstack((dy,np.transpose(np.matrix(np.hstack(([keyframes[1,i]],[keyframes[1,i+1]], zeros))))))
                    dz=np.vstack((dz,np.transpose(np.matrix(np.hstack(([keyframes[2,i]],[keyframes[2,i+1]], zeros))))))
                    dyaw=np.vstack((dyaw,np.transpose(np.matrix(np.hstack(([keyframes[3,i]], [keyframes[3,i+1]],zeros_yaw))))))

            else:
                if i==0:
                    dx=np.transpose(np.matrix(np.hstack(([keyframes[0,i], 0, 0,keyframes[0,i+1]], zeros))))
                    dy=np.transpose(np.matrix(np.hstack(([keyframes[1,i], 0, 0,keyframes[1,i+1]], zeros))))
                    dz=np.transpose(np.matrix(np.hstack(([keyframes[2,i], 0, 0,keyframes[2,i+1]], zeros))))
                    dyaw=np.transpose(np.matrix(np.hstack(([keyframes[3,i]], zeros_yaw,[keyframes[3,i+1]],zeros_yaw))))
                elif i==self.segments-1:
                    dx=np.vstack((dx,np.transpose(np.matrix(np.hstack(([keyframes[0,i]],[keyframes[0,i+1]], 0, 0))))))
                    dy=np.vstack((dy,np.transpose(np.matrix(np.hstack(([keyframes[1,i]],[keyframes[1,i+1]], 0, 0))))))
                    dz=np.vstack((dz,np.transpose(np.matrix(np.hstack(([keyframes[2,i]],[keyframes[2,i+1]], 0, 0))))))
                    dyaw=np.vstack((dyaw,np.transpose(np.matrix(np.hstack(([keyframes[3,i]], [keyframes[3,i+1]],zeros_yaw))))))
                else:
                    dx=np.vstack((dx,np.transpose(np.matrix(np.hstack(([keyframes[0,i]],[keyframes[0,i+1]], zeros))))))
                    dy=np.vstack((dy,np.transpose(np.matrix(np.hstack(([keyframes[1,i]],[keyframes[1,i+1]], zeros))))))
                    dz=np.vstack((dz,np.transpose(np.matrix(np.hstack(([keyframes[2,i]],[keyframes[2,i+1]], zeros))))))
                    dyaw=np.vstack((dyaw,np.transpose(np.matrix(np.hstack(([keyframes[3,i]], [keyframes[3,i+1]],zeros_yaw))))))

        deq=np.vstack((dx,dy,dz,dyaw))
        return deq #equality constraints

    def generate_C(self,derivation_order=4,derivation_order_yaw=2):    #permutation matrix for equality constraints

        #constructing C matrix
        for i in range(self.segments):


            if self.polyorder >=9:
                if i==0:
                    deltaCiRow = derivation_order + 2 #difference between rows of adjacent Ci, C(i+1) matrices
                    sizeC = derivation_order + deltaCiRow * (self.segments) #expected size of C matrix
                    dC=np.eye(deltaCiRow)
                    zeros=np.zeros((sizeC-deltaCiRow,deltaCiRow))
                    C=np.vstack((dC,zeros))
                    deltaCiRow=deltaCiRow+derivation_order

                elif i==self.segments-1:

                    zeros=np.zeros((deltaCiRow,derivation_order+2))
                    deltaCiRow=deltaCiRow+derivation_order+2
                    dC=np.eye(derivation_order+2)
                    dC=np.vstack((zeros,dC,np.zeros((sizeC-deltaCiRow,derivation_order+2))))
                    C=np.hstack((C,dC))

                else:

                    zeros=np.zeros((deltaCiRow,2))
                    dC=np.eye(2)
                    dC=np.vstack((zeros,dC,np.zeros((sizeC-deltaCiRow-2,2))))
                    deltaCiRow=deltaCiRow+derivation_order+2
                    C=np.hstack((C,dC))
            else:

                if i==0:
                    sizeC = 6 + (derivation_order + 2) * (self.segments-1)
                    deltaCiRow=4
                    dC=np.eye(deltaCiRow)
                    zeros=np.zeros((sizeC-deltaCiRow,deltaCiRow))
                    C=np.vstack((dC,zeros))
                    deltaCiRow=deltaCiRow+derivation_order
                    C=np.vstack((dC,zeros))

                elif i==self.segments-1:
                    zeros=np.zeros((deltaCiRow,4))
                    dC=np.eye(4)
                    deltaCiRow=deltaCiRow+4
                    dC=np.vstack((zeros,dC,np.zeros((sizeC-deltaCiRow,4))))
                    C=np.hstack((C,dC))
                else:
                    zeros=np.zeros((deltaCiRow,2))
                    dC=np.eye(2)
                    dC=np.vstack((zeros,dC,np.zeros((sizeC-deltaCiRow-2,2))))
                    deltaCiRow=deltaCiRow+derivation_order+2
                    C=np.hstack((C,dC))

        (rows,columns)=C.shape
        C=np.hstack((C,np.zeros((rows,sizeC-columns))))
        return C

    def generate_Q(self,T,derivation_order=4): #cost matrix

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

            Qi=np.matmul(np.transpose(polynom),polynom)
            for j in range(self.polyorder - derivation_order+1):
                for k in range(self.polyorder - derivation_order+1):
                    exponent=2*(self.polyorder - derivation_order)
                    Qi[j,k]=Qi[j,k]*T[i]**(exponent-j-k+1)/(exponent-j-k+1)
            if i==0:
                Qi=np.vstack((Qi,np.zeros((sizeQ-deltaQiRow,self.polyorder+1))))
                Q=np.copy(Qi)
            else:
                Qi=np.vstack((np.zeros((deltaQiRow,self.polyorder+1)),Qi))
                deltaQiRow=deltaQiRow+self.polyorder+1
                Qi=np.vstack((Qi,np.zeros((sizeQ-deltaQiRow,self.polyorder+1))))
                Q=np.hstack((Q,Qi))
        return Q

    def generate_dQ(self,T,derivation_order=4): #cost matrix derivative

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
        return dQ   #cost function derivative

    def generate_T(self,keyframes):     #approximate segment times based on distance
        T=np.zeros(self.segments)
        for i in range(self.segments):
            tx=float(abs(keyframes[0,i]-keyframes[0,i+1]))
            ty=float(abs(keyframes[1,i]-keyframes[1,i+1]))
            tz=float(abs(keyframes[2,i]-keyframes[2,i+1]))
            tyaw=float(abs(keyframes[3,i]-keyframes[3,i+1]))
            T[i]=max(tx,ty,tz,tyaw)
        return T

    def keyframes_callback(self,data):
        self.segments=len(data.points)-1

        for i in range(self.segments+1):

            dkeyframe=np.zeros((4,1))

            dkeyframe[0]=data.points[i].transforms[0].translation.x
            dkeyframe[1]=data.points[i].transforms[0].translation.y
            dkeyframe[2]=data.points[i].transforms[0].translation.z

            quaternion=(data.points[i].transforms[0].rotation.x, \
                        data.points[i].transforms[0].rotation.y, \
                        data.points[i].transforms[0].rotation.z, \
                        data.points[i].transforms[0].rotation.w  )
            euler=tf.transformations.euler_from_quaternion(quaternion)
            dkeyframe[3]=euler[2] #yaw
            if i==0:
                keyframes=np.copy(dkeyframe)
            else:
                keyframes=np.hstack((keyframes,dkeyframe))


        self.keyframes=np.copy(keyframes)

        print "Keyframes received, segments: ", self.segments

    def run(self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('trajectory_planner')
    trajectory=TrajectoryPlanner(7).run()
