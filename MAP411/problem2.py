import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from numpy import linalg


#FUNCTION DEFINITION
def J(x, y, l):
    return (x**2+l*y**2)/2 - (x+y)
def Jv(u, l):
    return J(u[0],u[1],l)
    
def gradJ(x, y, l):
    return np.array([x-1,l*y-1]);
def gradJv(u,l):
    return gradJ(u[0],u[1],l)
def gradJvNorm(u,l):
    return linalg.norm(gradJv(u, l))
    
def rho(x,y,l):
    if(x == 1 and y == 1/l):
        return 2/(1+l)
    return ((x-1)**2 + (l*y-1)**2)/((x-1)**2 + l*(l*y-1)**2)
def rhov(u,l):
    return rho(u[0],u[1], l)
    

#(I.1.1) PLOT CONTOUR LINES
def plotContour(lamb, colorMap):
    #PLOT VARIABLES
    N = 100
    x = np.linspace(0, 2, N)
    y = np.linspace(0, 2, N)
    L = 30
    X, Y = np.meshgrid(x, y)
    
    #CALCULATE THE FUNCTION IN THE MESHGRID
    z = J(X, Y, lamb)
    plt.contour(X, Y, z, L, colors='k')
    plt.contourf(X, Y, z, L , cmap = colorMap)
    plt.axis([0,2,0,2])
    plt.colorbar()
    
def linePlot(U, colorMap, width):
    z = np.linspace(0, 1, len(U))
    plt.gca().add_collection( LineCollection(zip(U[:-1], U[1:]), array=z, cmap = colorMap, linewidths= width))
    UM = np.matrix(U)
    plt.plot(UM[:,0],UM[:,1],'ow')
    
#(I.1.2) METHODE DE GRADIENT
def gradient(u, l, th):
    r_0 = gradJvNorm(u, l);
    if(r_0 == 0):
        print("Le minimum est déjà atteint")
        return
    iterations = 0
    uhist = [u]
    rhist = [1]
    while(gradJvNorm(u, l) > th * r_0 and iterations < 3000):
        u = u - rhov(u, l) * gradJv(u, lamb)   
        uhist.append(u)
        rhist.append(gradJvNorm(u, l)/r_0)
        iterations += 1
    if(gradJvNorm(u, l) > th * r_0):
        print("La méthode ne converge pas aprés 5000 pas")
    else:
        print("u = ", u)
        print("|r_k|/|r_0| =", gradJvNorm(u, l)/r_0)
        print(iterations, " iterations sont necessaires pour attendre J(u) <", th)
        
    return uhist,rhist

#COLOR MAP DEFINITION
cmap1 = LinearSegmentedColormap.from_list("my_colormap", ((0.25,0.25,1), (1, 0.25,0.25)), gamma=0.3)
cmap2 = LinearSegmentedColormap.from_list("my_colormap", ( (0, 0,0), (1,1,1)), gamma=0.2)

lamb = 3

uhist,rhist = gradient(np.array([0,1]), lamb, 1.0e-2)
#PLOT GRADIENT METHOD PATH FOR p = 0.002
plt.figure(figsize = plt.figaspect(0.5))
plotContour(lamb, cmap1)
linePlot(uhist, cmap2, 2)
plt.title(r'Le parcours pour le pas optimal à partir de $(0,1)$')  
plt.savefig('I123a.eps', format='eps')
plt.show()


#PLOT SEMILOG GRAPH FOR THE RATIO ||rk||/||r0||
plt.semilogy(rhist,'-o')
plt.title(r'$||r_k||/||r_0||$ en fonction de $k$')  
plt.gca().grid(True)
plt.gca().set_xlabel(r'$k$')
plt.gca().set_ylabel(r'$||r_k||/||r_0||$')
plt.savefig('I123b.eps', format='eps')
plt.show()