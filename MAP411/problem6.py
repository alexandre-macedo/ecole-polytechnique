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
    
def G(b, dx, u):
    n = u.shape[0]
    g = np.zeros([n])
    for i in range(n):
        g[i] = u[i]**4 + 2*b*u[i]/ (dx**2) -1 
        if(i-1 >= 0):
            g[i] -= b*u[i-1]/ (dx**2)
        if(i+1 < n):
            g[i] -= b*u[i+1]/ (dx**2)
    return g
    
def buildDG(b, dx, u0):
    u = np.copy(u0)
    n = u.shape[0]
    theta = b / (dx**2)
    M = np.zeros([n, n])
    
    diag = 2*theta
    upper = -theta
    lower = -theta
    
    for i in range(n):
        M[i,i] = diag
        if(i + 1 < n):
            M[i,i+1] = upper
        if(i - 1 >= 0):
            M[i,i-1] = lower
            
    M = M + np.diag(4*u**3)
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
        print(rknorm/rk0norm)
    return u

def newtonMethod(b, dx, u0):
    u = np.copy(u0)
    uhist = [u]
    #dhist = []
    n = 20
    for i in range(n):
        DG = buildDG(b, dx, u)
        B = G(b, dx, u)
        u = u - np.matmul(linalg.inv(DG),B)
        uhist.append(u)
    return uhist
    
b = 1    
X = 10
dx = 0.01
x0 = np.arange(-X,X+dx,dx)
u0 = 1-(x0**2)/100
    

tic()
uhist = newtonMethod(b, dx, u0)
toc()
n = len(uhist)
for i in range(n):
        plt.plot(x0, uhist[i],color = [i/n,0,0])
    
plt.gca().set_xlabel(r'$x$')
plt.gca().set_ylabel(r'$u$')
plt.title(r'Representation de la solution du problème stationnaire')  
plt.savefig('IV22.eps', format='eps')
plt.show()