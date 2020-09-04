import numpy as np
import scipy.linalg
class PathPlanner:
    def __init__(self, x_target):
        self.S = [[0,0,-1,0,0,0], [0,0,0,0,-1,0], [0,0,0,-1,0,0]]   # basic joint configurations as Screws seen by gelatin
        self.q = [0.0,0.0,0.0]       # actuation system joint space: theta, x, y
        self.x = np.array([0.0, 0.0, 0.0])    # Manipulator space: theta, x, y of the robot seen by gelatin
        self.w = np.pi/180*30       # angular speed         
        self.v = 0.005              # translation speed
        self.dt = 0.001 
        self.x_target = x_target    # goal position
                
    def skewSym(self, v):
        
       return np.array([[0, -v[2], v[1]], [v[2], 0 , -v[0]],[-v[1], v[0], 0]])
    
    def screwToLA(self, screw):     # screw to Lie Algebra se(3)
        w = screw[:3]
        v = np.array(screw[3:])[np.newaxis]
        w_hat = self.skewSym(w)
        v_vec = v.T
        S_braket = np.concatenate((w_hat, v_vec), axis=1)
        final_row = np.zeros((1,4))
        S_braket = np.concatenate((S_braket, final_row), axis = 0)        
        return S_braket

    def fwdKinematics(self):
        SB1 = self.screwToLA(self.S[0])
        SB2 = self.screwToLA(self.S[1])
        SB3 = self.screwToLA(self.S[2])
        
        T1 = scipy.linalg.expm(SB1*self.q[0])
        T2 = scipy.linalg.expm(SB2*self.q[1])
        T3 = scipy.linalg.expm(SB3*self.q[2])
        T = np.matmul(np.matmul(T1, T2), T3)
        self.x = np.array([-self.q[0], T[0,3], T[1,3]])     # manipulator state update
        return T
    
    def spatialJacobian(self):      
        V0 = np.array(self.S[0])[np.newaxis].T               
        SB0 = self.screwToLA(self.S[0])
        E0 = scipy.linalg.expm(SB0*self.q[0])

        V1 = np.matmul(self.adjointMap(E0), self.S[1])[np.newaxis].T
        
        SB1 = self.screwToLA(self.S[1])
        E1 = scipy.linalg.expm(SB1*self.q[1])
        
        V2 = np.matmul(self.adjointMap(np.matmul(E0, E1)), self.S[2])[np.newaxis].T
        
        return np.concatenate((V0, V1, V2), axis = 1)[2:5, :]       # extract the sub matrix of interest
        
        
    def adjointMap(self, T):        # Adjoint map expressed in 6x6 matrix
        R = T[:3, :3]
        p = T[:3, 3]
        p_braket = self.skewSym(p)
        adjtop = np.concatenate((R, np.zeros((3,3))), axis =1)
        adjbottom = np.concatenate((np.matmul(p_braket, R), R), axis = 1)
        adj = np.concatenate((adjtop, adjbottom), axis = 0)
        return adj
    
    def checkArrivalAndDirection(self):
        diff = self.x_target - self.x        # this includes angular distance
        
        dxy = diff[1:]
        dtheta = diff[0]
        
        position_error = np.linalg.norm(dxy)
        
        if position_error > 0.0001:
            self.xy_direction = dxy / position_error
        else:
            self.xy_direction = np.array([0.0,0.0])
        
        if abs(dtheta) > np.pi/180/10:
            self.th_direction = dtheta/abs(dtheta)
        else:
            self.th_direction = 0
            
        return position_error < 0.001 and abs(dtheta) < np.pi/90
        
        
    
    def followPoint(self):
        while not self.checkArrivalAndDirection():
            self.fwdKinematics()                                    
            v_xy = self.xy_direction * self.v * self.dt
            w = self.th_direction * self.w * self.dt 
            V = np.append(w, v_xy)
                        
            dq = np.matmul(np.linalg.inv(self.spatialJacobian()), V)
            
            self.q += dq    # joint space update
            
            print('joint angles:', self.q, '   position:', self.x)
        return 1
    
a = PathPlanner([np.pi,0.02,0.01])
th_log  = a.followPoint()