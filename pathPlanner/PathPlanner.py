#!/usr/bin/env python3
import numpy as np
import scipy.linalg

class PathPlanner:
    def __init__(self, x_target):
        self.S = [[0,0,-1,0,0,0], [0,0,0,0,-1,0], [0,0,0,-1,0,0]]   # basic joint configurations as Screws seen by gelatin (order of theta, y, x)
        self.S_table = [[0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,1,0,0,0]]   # basic joint configurations as Screws seen by gelatin (order of x, y, theta)
        
       
        self.q = np.array([0.0,0.0,0.0])       # actuation system joint space: theta, y, x 
        self.x = np.array([0.0, 0.0, 0.0])    # Manipulator space: theta, x, y of the robot seen by gelatin

        self.w = np.pi/180.*10.*10      # angular speed         
        self.v = 0.02              # translation speed
        self.dt = 0.001         

        self.x_target = x_target    # goal position
        self.q_log = []
        self.converted_joint_angles = []
                
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
        dtheta = np.sin(diff[0])
        
        position_error = np.linalg.norm(dxy)
        
        if position_error > 0.0001:
            self.xy_direction = dxy / position_error
        else:
            self.xy_direction = np.array([0.0,0.0])
        
        if abs(dtheta) > np.sin(np.pi/180./10.):
            v_target = [np.cos(self.x_target[0]), np.sin(self.x_target[0])]
            v = [np.cos(self.x[0]), np.sin(self.x[0])]
            
            self.th_direction = np.cross(v, v_target)
        else:
            self.th_direction = 0
            
        return position_error < 0.0005 and abs(dtheta) < np.pi/90
        
        
    
    def followPoint(self, publish = True):
        
        while not self.checkArrivalAndDirection():
            self.fwdKinematics()                                    
            v_xy = self.xy_direction * self.v * self.dt
            w = self.th_direction * self.w * self.dt 
            V = np.append(w, v_xy)
                        
            dq = np.matmul(np.linalg.inv(self.spatialJacobian()), V)
            
            self.q += dq    # joint space update
            
            # note that the published message has q0, q2, q1 : theta, x, y
            converted_joint_angles = [self.q[0], self.w, self.q[2]*1000., self.v*1000., self.q[1]*1000., self.v*1000.]
            self.q_log.append(self.q.tolist())
            self.converted_joint_angles.append(converted_joint_angles)
            
        return 1
    
    
        
    def fwdKinematics_from_table(self, q):
        SB1 = self.screwToLA(self.S_table[0])
        SB2 = self.screwToLA(self.S_table[1])
        SB3 = self.screwToLA(self.S_table[2])
        
        T1 = scipy.linalg.expm(SB1*q[2])
        T2 = scipy.linalg.expm(SB2*q[1])
        T3 = scipy.linalg.expm(SB3*q[0])
        T = np.matmul(np.matmul(T1, T2), T3)
        T = np.linalg.inv(T)        
        x = np.array([q[0], T[0,3], T[1,3]])     # manipulator state update
        return x
        
    
    def plot_trajectory_from_base(self):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(16,8))
        points = []
        q_log = np.array(self.q_log)
        for q in q_log:
            points.append(self.fwdKinematics_from_table(q))
        
        points = np.array(points)            
        plt.subplot(241)
        plt.plot(points[:,1], points[:,2])        
        plt.title('xy plot')
        plt.subplot(242)
        plt.plot(points[:,0])        
        plt.title('w')
        plt.subplot(243)
        plt.plot(points[:,1])        
        plt.title('x')
        plt.subplot(244)
        plt.plot(points[:,2])        
        plt.title('y')
        
        plt.subplot(245)
        plt.plot(q_log[:, 0])
        plt.title('theta stage input')
        
        plt.subplot(246)
        plt.plot(q_log[:, 1])
        plt.title('X stage input')
        
        plt.subplot(247)
        plt.plot(q_log[:, 2])
        plt.title('Y stage input')

        plt.show()
        
   
# test here with a simple square path
# four points to follow

t = np.linspace(0, 2*np.pi, 20)
radius = 0.005
x = np.cos(t)*radius
y = np.sin(t)*radius
th = t + np.pi/2

# paths = np.concatenate((th[:, None], x[:, None], y[:, None]), axis = 1)

paths = [[5*np.pi/4., 0.005, 0.005], 
        [5*np.pi/4., 0., 0.], 
        [0., 0., 0.], 
        [0., 0., 0.01], 
        [-np.pi/2, 0., 0.01],
        [-np.pi/2, 0.01, 0.01],
        [-np.pi, 0.01, 0.01],
        [-np.pi, 0.01, 0.0],
        [-np.pi*3/2, 0.01, 0.0],
        [-np.pi*3/2, 0.0, 0.0],
        ]

paths = np.array(paths)

paths_offset = np.ones_like(np.array(paths)[:,1])*0.005

paths[:,1] -= paths_offset
paths[:,2] -= paths_offset

paths = paths.tolist()


publish = False

    
import time
a = PathPlanner([0,0,0])
for count, path in enumerate(paths):
    a.x_target = path 
    a.followPoint(publish)
    print('point {0} reached'.format(count))
    #time.sleep(1)

if publish is False:    
    a.plot_trajectory_from_base()
        


if publish is True:
    import rospy
    from std_msgs.msg import Float64MultiArray, String
    rospy.init_node('master_node', anonymous=True)
    global pub
    pub = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)
    
    for i, converted_joint_angles in enumerate(a.converted_joint_angles):

        pos_vel = Float64MultiArray()
        pos_vel.data = converted_joint_angles
        pub.publish(pos_vel)
        time.sleep(a.dt)


    

# import IPython
# IPython.embed()


# print('joint angles: %.2f %.2f %.2f %.2f %.2f %.2f '%(converted_joint_angles[0], converted_joint_angles[1], 
                                                      # converted_joint_angles[2], converted_joint_angles[3], converted_joint_angles[4], converted_joint_angles[5]
                                                      # ), '   position:', self.x)            

                
