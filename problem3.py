import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.collections import LineCollection 

#FUNCTION DEFINITION
def J(u1, u2):
    return (u1-1)**2 + 100*(u1**2-u2)**2
    
def Jv(u):
    return J(u[0],u[1])

def gradJ(u1, u2):
    return np.array((2 *(-1 + u1 + 200 *u1* (u1**2 - u2)), -200 *(u1**2 - u2)))

def gradJv(u):
    return gradJ(u[0],u[1])

def plotContour(colorMap):
    N = 50
    x = np.linspace(-1.5, 1.5, N)
    y = np.linspace(-0.5, 1.5, N)
    L = 30
    X, Y = np.meshgrid(x, y)
    Z = J(X,Y)
    plt.contour(X, Y, Z, L, colors = 'k')
    plt.contourf(X, Y, Z, L,  cmap = colorMap)
    plt.axis([-1.5, 1.5, -0.5, 1.5])
    plt.colorbar()

    
def linePlot(U, colorMap, width):
    plt.plot(U[0][0], U[0][1],'ko')
    plt.plot(U[-1][0], U[-1][1],'wo')
    z = np.linspace(0, 1, len(U))
    plt.gca().add_collection( LineCollection(zip(U[:-1], U[1:]), array=z, cmap = colorMap, linewidths= width))

# METHODE DE GRADIENT
def gradient(u, th):
    print('Methode de gradient pour u_0 =',u,' th=',th);
    iterations = 0
    uhist = [u]
    while(Jv(u) > th and iterations < 3000 and not np.isinf(u[0])  and not np.isinf(u[1]) ):
        rho = 5
        count = 0
        while(Jv(u - rho * gradJv(u)) >= Jv(u) and count < 20):
            rho = rho/2
            count += 1
        u = u - rho * gradJv(u)  
        uhist.append(u)
        iterations += 1
    
    if(np.isinf(u[0])  or np.isinf(u[1])):
        #if(Jv(u) > th):
        print("La méthode ne converge pas aprés 5000 pas")
    else:
        print("u = ", u)
        print(iterations, " iterations sont necessaires pour attendre J(u) <", th)
    return uhist
    
#COLOR MAP DEFINITION
cmap1 = LinearSegmentedColormap.from_list("my_colormap", ((0.25,0.25,1), (1, 0.25,0.25)), gamma=0.3)
cmap2 = LinearSegmentedColormap.from_list("my_colormap", ( (0, 0,0), (1,1,1)), gamma=0.2)

#CALCULATE GRADIENT METHOD PATH
uhist = gradient(np.array((0,0)), 1.0e-3)

#PLOT GRADIENT METHOD PATH
plt.figure(figsize = plt.figaspect(0.5))
plotContour(cmap1)
linePlot(uhist, cmap2, 2)
plt.title(r"Le procédé de dichotomie pour l'équation Rosenbrock") 
plt.show()