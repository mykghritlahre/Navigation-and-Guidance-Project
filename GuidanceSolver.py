from enum import Enum

import os
import numpy as np
import matplotlib.pyplot as plt

def ToRad(deg):
    return deg * np.pi / 180

Dist    = 20            # km
Vm      = 1             # km/s
Vt      = (2/5) * Vm    # km/s
Theta   = ToRad(30)     # rad
AlphaT  = ToRad(120)    # rad
Delta   = ToRad(30)     # rad

DeltaTime = 0.1         # s
TimeSpan = np.arange(0, 300.1, DeltaTime)
NumSteps = len(TimeSpan)

class GuidanceType(Enum):
    Pure_Pursuit = 1
    Deviated_Pursuit = 2
    True_Proportional_Navigation = 3

    def fetchName(self):
        return self.name.replace('_', ' ')
    
    def fetchInitials(self):
        return ''.join([word[0] for word in self.name.split('_')])

class Solver:
    
    def __init__(self):
        self.MissileStart = np.array( [ 0.0           , 0.0           ] )
        self.TargetStart  = np.array( [ np.cos(Theta) , np.sin(Theta) ] ) * Dist
        
        self.GuidanceFunc = {
            GuidanceType.Pure_Pursuit: self.PP,
            GuidanceType.Deviated_Pursuit: self.DP,
            GuidanceType.True_Proportional_Navigation: self.TPN
        }

    def reset(self, type):
        self.R          = np.zeros(NumSteps)
        self.LosAngle   = np.zeros(NumSteps)
        self.Guidance   = np.zeros(NumSteps)

        self.R[0] = Dist
        self.LosAngle[0] = Theta

        self.MissilePos = np.array( [ self.MissileStart[0] , self.MissileStart[1] ] )
        self.TargetPos  = np.array( [ self.TargetStart [0] , self.TargetStart [1] ] )

        if type == GuidanceType.True_Proportional_Navigation:
            self.AlphaM = np.zeros(NumSteps)
            self.AlphaM[0] = Theta

            self.VM = np.zeros(NumSteps)
            self.VM[0] = Vm

            Vr, Vth = self.CalculateVelocity(Vm, self.AlphaM[0], Vt, AlphaT, Theta)

            N = 3
            self.c = -N * Vr

    @staticmethod
    def CalculateVelocity(Vm, Am, Vt, At, LosAngle):
        Vr  = Vt * np.cos(At - LosAngle) - Vm * np.cos(Am - LosAngle)
        Vth = Vt * np.sin(At - LosAngle) - Vm * np.sin(Am - LosAngle)
        return Vr, Vth

    def PP(self, Iter):
        LosAngle = self.LosAngle[Iter]
        RadialDistance = self.R[Iter]

        Vr, Vth = self.CalculateVelocity(Vm, LosAngle, Vt, AlphaT, LosAngle)

        self.R[Iter + 1] = RadialDistance + Vr * DeltaTime
        self.LosAngle[Iter + 1] = LosAngle + (Vth / RadialDistance) * DeltaTime

        self.MissilePos[0] += Vm * np.cos(LosAngle) * DeltaTime
        self.MissilePos[1] += Vm * np.sin(LosAngle) * DeltaTime

        self.TargetPos[0] += Vt * np.cos(AlphaT) * DeltaTime
        self.TargetPos[1] += Vt * np.sin(AlphaT) * DeltaTime

        self.Guidance[Iter + 1] = (Vm * Vth) / RadialDistance

        return self.MissilePos, self.TargetPos

    def DP(self, Iter):
        LosAngle = self.LosAngle[Iter]
        RadialDistance = self.R[Iter]

        Vr, Vth = self.CalculateVelocity(Vm, LosAngle + Delta, Vt, AlphaT, LosAngle)

        self.R[Iter + 1] = RadialDistance + Vr * DeltaTime
        self.LosAngle[Iter + 1] = LosAngle + (Vth / RadialDistance) * DeltaTime

        self.MissilePos[0] += Vm * np.cos(LosAngle + Delta) * DeltaTime
        self.MissilePos[1] += Vm * np.sin(LosAngle + Delta) * DeltaTime

        self.TargetPos[0] += Vt * np.cos(AlphaT) * DeltaTime
        self.TargetPos[1] += Vt * np.sin(AlphaT) * DeltaTime

        self.Guidance[Iter + 1] = (Vm * Vth) / RadialDistance

        return self.MissilePos, self.TargetPos
    
    def TPN(self, Iter):
        LosAngle = self.LosAngle[Iter]
        RadialDistance = self.R[Iter]
        CurrentVm = self.VM[Iter]
        CurrentAm = self.AlphaM[Iter]

        Vr, Vth = self.CalculateVelocity(CurrentVm, CurrentAm, Vt, AlphaT, LosAngle)

        self.Guidance[Iter] = (self.c * Vth) / RadialDistance
        self.VM[Iter + 1] = CurrentAm + CurrentAm * np.sin(CurrentAm - LosAngle) * DeltaTime
        self.AlphaM[Iter + 1] = CurrentAm + (self.Guidance[Iter] / CurrentVm) * np.cos(CurrentAm - LosAngle) * DeltaTime

        self.R[Iter + 1] = RadialDistance + Vr * DeltaTime
        self.LosAngle[Iter + 1] = LosAngle + (Vth / RadialDistance) * DeltaTime

        self.MissilePos[0] += Vm * np.cos(CurrentAm) * DeltaTime
        self.MissilePos[1] += Vm * np.sin(CurrentAm) * DeltaTime

        self.TargetPos[0] += Vt * np.cos(AlphaT) * DeltaTime
        self.TargetPos[1] += Vt * np.sin(AlphaT) * DeltaTime

        return self.MissilePos, self.TargetPos

    def CalculateTrajectory(self, type):
        
        self.reset(type)

        Mx = [ self.MissilePos[0] ]; My = [ self.MissilePos[1] ]
        Tx = [ self.TargetPos [0] ]; Ty = [ self.TargetPos [1] ]

        func = self.GuidanceFunc[type]

        for Iter in range(NumSteps - 1):
            
            mp, tp = func(Iter)

            Mx.append(mp[0]); My.append(mp[1])
            Tx.append(tp[0]); Ty.append(tp[1])

            distance = np.linalg.norm(mp - tp)
            
            if(self.R[Iter + 1] < 0 or distance < 0.001):
                self.R = self.R[:Iter + 1]
                self.LosAngle = self.LosAngle[:Iter + 1]
                self.Guidance = self.Guidance[:Iter + 1]
                break
        
        return np.array(Mx), np.array(My), np.array(Tx), np.array(Ty)

    def TpnDebugPlot(self):
        self.reset(GuidanceType.True_Proportional_Navigation)

        Mx = [ self.MissilePos[0] ]; My = [ self.MissilePos[1] ]
        Tx = [ self.TargetPos [0] ]; Ty = [ self.TargetPos [1] ]

        plt.figure(); plt.ion()
        plt.axis('equal')
        plt.xlim([-10, 30]); plt.ylim([-10, 30]); plt.grid(True)

        missile_plot, = plt.plot(Mx, My, 'bo', markersize=4, label='Missile')
        target_plot, = plt.plot(Tx, Ty, 'go', markersize=4, label='Target')

        plt.legend()
        plt.show()

        for Iter in range(NumSteps - 1):

            mp, tp = self.TPN(Iter)
            
            distance = np.linalg.norm(mp - tp)
            
            if(self.R[Iter + 1] < 0 or distance < 0.001):
                self.R = self.R[:Iter + 1]
                self.LosAngle = self.LosAngle[:Iter + 1]
                self.Guidance = self.Guidance[:Iter + 1]
                break
            
            Mx.append(mp[0]); My.append(mp[1])
            Tx.append(tp[0]); Ty.append(tp[1])

            missile_plot.set_data(Mx, My)
            target_plot.set_data(Tx, Ty)

            plt.pause(0.01)

        plt.ioff()


    def Plot(self, GuidanceType, outputFolder = "", showPlot = True):
        Mx, My, Tx, Ty = self.CalculateTrajectory(GuidanceType)

        typeName = GuidanceType.fetchName()
        typeInitials = GuidanceType.fetchInitials()

        enableSave = outputFolder != ""

        plt.figure(typeName + ': Trajectory') 

        plt.scatter(self.MissileStart[0], self.MissileStart[1], c='black', label='Missile Start')
        plt.scatter(self.TargetStart[0], self.TargetStart[1], c='orange', label='Target Start')
        plt.plot(Mx, My, 'black', markersize=4, label='Missile')
        plt.plot(Tx, Ty, 'orange', markersize=4, label='Target')
        plt.axis('equal')
        plt.grid(True)
        plt.xlabel(r'X Position $(km)$')
        plt.ylabel(r'Y Position $(km)$')
        plt.title("Trajectory")
        plt.legend()

        if enableSave:
            plt.savefig(outputFolder + typeInitials + "_Trajectory")

        plt.figure(typeName + ': Time Variance')
        plt.subplot(2, 1, 1)
        plt.plot(TimeSpan[:len(self.Guidance)], self.Guidance, 'orange', label='Guidance Command')
        plt.grid(True)
        plt.xlabel(r'Time $(s)$')
        plt.ylabel(r'Guidance Cmd $(km/s^2)$')
        plt.title("Guidance Cmd vs Time")
        
        plt.subplot(2, 1, 2)
        plt.plot(TimeSpan[:len(self.R)], self.R, 'black', label='Range')
        plt.grid(True)
        plt.xlabel(r'Time $(s)$')
        plt.ylabel(r'Range $(km)$')
        plt.title("Range vs Time")

        plt.tight_layout()

        if enableSave:
            plt.savefig(outputFolder + typeInitials + "_TimeVariance")
        
        if showPlot:
            plt.show()  
        
SAVE_IMAGE = True
SHOW_IMAGE = not SAVE_IMAGE

OUTPUT = "./plots/" if SAVE_IMAGE else "" 

if SAVE_IMAGE and not os.path.exists(OUTPUT):
    os.makedirs(OUTPUT)

solver = Solver()
solver.Plot(GuidanceType.Pure_Pursuit, OUTPUT, SHOW_IMAGE)
solver.Plot(GuidanceType.Deviated_Pursuit, OUTPUT, SHOW_IMAGE)
solver.Plot(GuidanceType.True_Proportional_Navigation, OUTPUT, SHOW_IMAGE)