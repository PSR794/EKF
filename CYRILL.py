import pickle
import numpy as np
import matplotlib.pyplot as plt

with (open("data.pickle", "rb")) as f:
    data=pickle.load(f)
    
#getting values

#initial orientation
x_init = data['x_init']
y_init=data['y_init']
th_init=data['th_init']

#controls
vel=data['v']
vel_var=data['v_var']
w=data['om']
w_var=data['om_var']


t=data['t']

#observation
l=data['l']
phi=data['b']
phi_var=data['b_var']
r=data['r']
r_var=data['r_var']

#distance between COG of robot and sensor
d=data['d']

#mean and covariances matrices
pos_mean=np.zeros([501,3],dtype=float)
pos_mean[0]=np.array([x_init, y_init, th_init],dtype=float)

pos_sigma=np.zeros([501,3,3],dtype=float)
pos_sigma[0]=np.diag([0,0,0])

control_sigma=np.diag([vel_var,w_var])
z_sigma=np.diag([r_var,phi_var])


#angle wrap
def wraptopi(z):
    if z > np.pi:
        z = z - (np.floor(z / (2 * np.pi)) + 1) * 2 * np.pi
    elif z < -np.pi:
        z = z + (np.floor(z / (-2 * np.pi)) + 1) * 2 * np.pi
    return z

#prediction steps
sigma1=pos_sigma[0]
x=np.zeros([3])
x=pos_mean[0]
x1=x.reshape([3,1])

for i in range(1,501):
    dt=t[i]-t[i-1]
    v=x1[2][0]
    
    #for controls
    
    control=np.zeros([3,1],dtype=float)
    control[0][0]=( np.sin(v+(w[i-1]*dt)) - np.sin(v) )*(vel[i-1]/w[i-1])
    control[1][0]=( np.cos(v) - np.cos(v+(w[i-1]*dt)) )*(vel[i-1]/w[i-1])
    control[2][0]=w[i-1]*dt

    #prediction mean
    x1=x1+control #prediction first step (gives us ut')
 
    x1[2][0]=wraptopi(x1[2][0])
    
    #jacobian 1 + I(wrt x,y,theta)
    G=np.zeros([3,3],dtype=float)
    G[1][1]=G[0][0]=G[2][2]=1
    G[0][2]=( np.cos(v+(w[i-1]*dt)) - np.cos(v) )*(vel[i-1]/w[i-1])
    G[1][2]=( np.sin(v+(w[i-1]*dt)) - np.sin(v) )*(vel[i-1]/w[i-1])

    #jacobian 2(wrt v,w)
    L=np.zeros([3,2],dtype=float)
    L[0][0]=(np.sin(v+(w[i-1]*dt)) - np.sin(v))/w[i-1]
    L[1][0]=(np.cos(v) - np.cos(v+(w[i-1]*dt)))/w[i-1]
    L[2][0]=0 
    L[0][1]=( ( w[i-1]*dt*np.cos(v+(w[i-1]*dt)) ) - np.sin(v+(w[i-1]*dt)) + np.sin(v) )*((vel[i-1])/(w[i-1]**2))
    L[1][1]=( np.cos(v+(w[i-1]*dt)) + (w[i-1]*dt*np.sin(v+(w[i-1]*dt))) - np.cos(v) )*((vel[i-1])/(w[i-1]**2))
    L[2][1]=dt
    #prediction covariance
    sigma1=G@(sigma1@(G.T)) + L@(control_sigma@(L.T))

    #correction
    for j in range(len(r[i])):
        
        theta=x1[2][0]
        #landmark and robot positions
        xk=x1[0][0]
        yk=x1[1][0]
        xj=l[j][0]
        yj=l[j][1]

        #location of landmark wrt robot
        dx= xj-xk-(d[0]*np.cos(theta))
        dy= yj-yk-(d[0]*np.sin(theta))
        q=np.sqrt((dx**2) + (dy**2))
        angle=np.arctan2(dy,dx) - theta
        
#        print('sd',angle)

        #(jacobian of h) H, 3 variables and 2 functions (wrt x,y,theta)
        H=np.zeros([2,3],dtype=float)
#        print(dx,q)
        #jacobian of h
        H[0][0]=(-dx)/q
        H[0][1]=(-dy)/q
        H[0][2]=((dy*np.cos(theta)*d[0])-(dx*np.sin(theta)*d[0]))/q
        H[1][0]=(-dy)/(q**2)
        H[1][1]=(dx)/(q**2)
        H[1][2]=(-1)-(d[0]*((dx*np.cos(theta))+(dy*np.sin(theta))))/(q**2)

        #kalman gain
        K= sigma1@((H.T)@(np.linalg.inv( (H@(sigma1@(H.T))) + np.identity(2)@z_sigma@(np.identity(2).T) )))
        #print(K.shape)
        #observation
        zt=np.array([ [r[i,j]],[wraptopi(phi[i,j])]],dtype=float)#measured
        h=np.array([ [q],[wraptopi(angle)] ],dtype=float)#calculated

        #final mean
        x1 = x1 + K@(zt-h)
        if i==1 and j==1:
            print(x1)       

        #final covariance
        sigma1=(np.identity(3)-(K@H))@(sigma1)
        x1[2][0]=wraptopi(x1[2][0])

    pos_mean[i,:]=x1.T
    pos_sigma[i,:,:]=sigma1


e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(pos_mean[:, 0], pos_mean[:, 1])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Estimated trajectory')
plt.show()

e_fig1 = plt.figure()
ax1 = e_fig1.add_subplot(111)
ax1.plot(t[:], pos_mean[:, 2])
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('theta [rad]')
ax1.set_title('Estimated trajectory')
plt.show()
