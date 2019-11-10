import matplotlib.pyplot as plt
import numpy as np

from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from numpy import linalg

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")
        
        
#FUNCTION DEFINITION
def J(A, B, u):
    return (1/2)* np.inner(np.matmul(A,u),u) - np.inner(B, u)
    
def gradJ(A, B, u):
    return np.matmul(A,u) - B

def rho(rk, A):
    return np.inner(rk,rk) / np.inner(np.matmul(A,rk),rk)
    
def buildMNL(a, b, dx, dt, u):
    n = u.shape[0]
    delta = b * dt / (dx**2)
    M = np.zeros([n, n])
    
    diag = 1 + 2*delta
    upper = -delta
    lower = -delta
    
    for i in range(n):
        M[i,i] = diag
        if(i + 1 < n):
            M[i,i+1] = upper
        if(i - 1 >= 0):
            M[i,i-1] = lower
    M = M + np.diag(u**3)*dt
    return M    
    
def buildM(a, b, dx, dt, u):
    n = u.shape[0]
    gamma = a * dt / (2 * dx)
    delta = b * dt / (dx**2)
    M = np.zeros([n, n])
    
    diag = 1 + 2*delta
    upper = gamma - delta
    lower = -gamma - delta
    
    for i in range(n):
        M[i,i] = diag
        if(i + 1 < n):
            M[i,i+1] = upper
        if(i - 1 >= 0):
            M[i,i-1] = lower
    return M 
    
def gradientOptimal(A, B, u0, th):
    u = np.copy(u0)  
    rk0 = gradJ(A, B, u)
    rk0norm = linalg.norm(rk0)
    rknorm = rk0norm
    while(rknorm/rk0norm > th):
        rk = gradJ(A, B, u)
        rknorm = linalg.norm(rk)
        rhok = rho(rk, A)
        u = u - rhok * rk
    return u, rknorm/rk0norm

def implicitIterative(a, b, dx, dt, T, u0, buildF):
    u = np.copy(u0)
    n = int(T/dt)
    uhist = [u]
    dhist = []
    for n in range(n):
        
        M = buildF(a, b, dx, dt, u)
        MT = np.transpose(M)
        A = np.matmul(MT, M)
        B = np.matmul(MT, u)
        u, d = gradientOptimal(A, B, u, 1.0e-5)
        dhist.append(d)
        uhist.append(u)
    print("Avarage relative precision:",np.mean(dhist))
    return uhist

a = 1
b = 1    
X = 10
dx = 0.1
dt = 0.0001
T = 1
x0 = np.arange(-X,X+dx,dx)
u0 = (1/np.sqrt(2*np.pi)) * np.exp((-1/2)* x0**2)
tic()
uhist = implicitIterative(a, b, dx, dt, T, u0, buildM)
toc()
for i in range(int(T/dt)):
    if(i % 200 == 0):
        plt.plot(x0, uhist[i],color = [i*dt/T,0,0])
    
plt.gca().set_xlabel(r'$x$')
plt.gca().set_ylabel(r'$u$')
plt.title(r'Representation de la solution implicite')  
plt.savefig('III23.eps', format='eps')
plt.show()

tic()
uhist = implicitIterative(a, b, dx, dt, T, u0, buildMNL)
toc()
for i in range(int(T/dt)):
    if(i % 200 == 0):
        plt.plot(x0, uhist[i],color = [i*dt/T,0,0])
    
plt.gca().set_xlabel(r'$x$')
plt.gca().set_ylabel(r'$u$')
plt.title(r'Représentation de la solution implicite')  
plt.savefig('IV12.eps', format='eps')
plt.show()