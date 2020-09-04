import numpy as np
import scipy.linalg
class PathPlanner:
    def __init__(self):
        self.S = [[0,0,-1,0,0,0], [0,0,0,0,-1,0], [0,0,0,-1,0,0]]   # basic joint configurations as Screws
        self.th = [0.0,0.0,0.0]       # theta, x, y input to the actuation system
        self.w = np.pi/1800
        self.v = 0.005
        self.x = np.array([0.0, 0.0, 0.0])    # theta, x, y of the robot
        
   def skewSym(self, v):
        return np.array([[0, -v[2], v[1]], [v[2], 0 , -v[0]],[-v[1], v[0], 0]])
    
    def screwToBraket(self, screw):
        w = screw[:3]
        v = np.array(screw[3:])[np.newaxis]
        w_hat = self.skewSym(w)
        v_vec = v.T
        S_braket = np.concatenate((w_hat, v_vec), axis=1)
        final_row = np.zeros((1,4))
        S_braket = np.concatenate((S_braket, final_row), axis = 0)        
        return S_braket

    def fwdKinematics(self):
        SB1 = self.screwToBraket(self.S[0])
        SB2 = self.screwToBraket(self.S[1])
        SB3 = self.screwToBraket(self.S[2])
        
        T1 = scipy.linalg.expm(SB1*self.th[0])
        T2 = scipy.linalg.expm(SB2*self.th[1])
        T3 = scipy.linalg.expm(SB3*self.th[2])
        T = np.matmul(np.matmul(T1, T2), T3)
        return T
    
    def spatialJacobian(self):
        V0 = np.array(self.S[0])[np.newaxis].T        
        SB0 = self.screwToBraket(self.S[0])
        E0 = scipy.linalg.expm(SB0*self.th[0])

        V1 = np.matmul(self.adjointMap(E0), self.S[1])[np.newaxis].T
        
        SB1 = self.screwToBraket(self.S[1])
        E1 = scipy.linalg.expm(SB1*self.th[1])
        
        V2 = np.matmul(self.adjointMap(np.matmul(E0, E1)), self.S[2])[np.newaxis].T
        
        return np.concatenate((V0, V1, V2), axis = 1)[2:5, :]
        
        
    def adjointMap(self, T):
        R = T[:3, :3]
        p = T[:3, 3]
        p_braket = self.skewSym(p)
        adjtop = np.concatenate((R, np.zeros((3,3))), axis =1)
        adjbottom = np.concatenate((np.matmul(p_braket, R), R), axis = 1)
        adj = np.concatenate((adjtop, adjbottom), axis = 0)
        return adj
    
    def followPoint(self, x_end):
        diff = np.array(x_end) - self.x
        dxy = diff[1:]
        dtheta = diff[0]
        th_log = []
        while max(abs(dxy))>0.1 or abs(dtheta)>np.pi/90:
            T = self.fwdKinematics()
            T_inv = np.linalg.inv(T)
            self.x[0] = -self.th[0]                        
            self.x[1:3]=T[:2, 3]
                        
            diff = np.array(x_end) - self.x
            dxy = diff[1:]
            dtheta = diff[0]
            
            
            th_log.append(self.th)
            position_norm = np.linalg.norm(dxy)
            
            if position_norm > 0.01:
                xy_direction = dxy / position_norm
            else :
                    xy_direction = np.array([0.0,0.0])
            
            if abs(dtheta) > np.pi/90:
                th_direction = dtheta/abs(dtheta)
            else:
                th_direction = 0
            
            v_xy = xy_direction * self.v
            w = th_direction * self.w
            V = np.append(w, v_xy)
            #V = np.concatenate((w,v_xy))[np.newaxis].T
            
            dq = np.matmul(np.linalg.inv(self.spatialJacobian()), V)
            
            self.th += dq
            
            print(dq)
            print(self.x)
            
        return th_log
    
            
            
        
           
            
            
    
        

a = PathPlanner()

th_log  = a.followPoint([np.pi,1,0])