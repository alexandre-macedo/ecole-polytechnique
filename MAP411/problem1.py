import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.collections import LineCollection 
from matplotlib.colors import LinearSegmentedColormap

#FUNCTION DEFINITION
def J(u1, u2):
    return (u1-1)**2 + 100*(u1**2-u2)**2
    
def Jv(u):
    return J(u[0],u[1])

def gradJ(u1, u2):
    return np.array((2 *(-1 + u1 + 200 *u1* (u1**2 - u2)), -200 *(u1**2 - u2)))

def gradJv(u):
    return gradJ(u[0],u[1])
    
#PLOT CONTOUR LINES
def plotContour(colorMap, ystart):
    N = 50
    x = np.linspace(-1.5, 1.5, N)
    y = np.linspace(ystart, 1.5, N)
    L = 30
    X, Y = np.meshgrid(x, y)
    Z = J(X,Y)
    plt.contour(X, Y, Z, L, colors = 'k')
    plt.contourf(X, Y, Z, L,  cmap = colorMap)
    plt.axis([-1.5, 1.5, ystart, 1.5])
    plt.colorbar()

def plotSurface(colorMap):
    N = 150
    x = np.linspace(-1.5, 1.5, N)
    y = np.linspace(0.5, 1.5, N)
    X, Y = np.meshgrid(x, y)
    Z = J(X,Y)        
    fig = plt.figure(figsize=plt.figaspect(0.35))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X, Y, Z,rstride=1,cstride=1,cmap=colorMap,linewidth=0,antialiased=False)
    ax.set_zlim3d( 0 ,  np.max(Z) )
    fig.colorbar(surf,shrink=0.5,aspect=10)
    plt.title('Graphique de la fonction de Rosenbrock')    
    			

def linePlot(U, colorMap, width):
    plt.plot(U[0][0], U[0][1],'ko')
    plt.plot(U[-1][0], U[-1][1],'wo')
    z = np.linspace(0, 1, len(U))
    plt.gca().add_collection( LineCollection(zip(U[:-1], U[1:]), array=z, cmap = colorMap, linewidths= width))

#METHODE DE GRADIENT
def gradient(u, rho, th):
    print('Methode de gradient pour u_0 =',u,' rho =', rho,' th =',th);
    iterations = 0
    uhist = [u]
    while(Jv(u) > th and iterations < 3000 and not np.isinf(u[0])  and not np.isinf(u[1]) ):
        u = u - rho * gradJv(u)   
        uhist.append(u)
        #plt.plot([un[0],u[0]],[un[1],u[1]],color = [inter,inter,inter])
        iterations += 1
    if(np.isinf(u[0])  or np.isinf(u[1])):
        #if(Jv(u) > th):
        print("La méthode ne converge pas aprés 5000 pas")
    else:
        print("u = ", u)
        print(iterations, " iterations sont necessaires pour attendre J(u) <", th)
    print('\n')
    return uhist

#COLOR MAP DEFINITION
cmap1 = LinearSegmentedColormap.from_list("my_colormap", ((0.25,0.25,1), (1, 0.25,0.25)), gamma=0.3)
cmap2 = LinearSegmentedColormap.from_list("my_colormap", ( (0, 0,0), (1,1,1)), gamma=0.2)

#CALCULATE GRADIENT METHOD PATH
uhist002 = gradient(np.array((1,0)), 0.002, 1.0e-3) 
uhist045 = gradient(np.array((1,0)), 0.0045, 1.0e-3)
uhist01 = gradient(np.array((1,0)), 0.01, 1.0e-3)
   

#PLOT CONTOUR LINES
plt.figure(figsize = plt.figaspect(0.4))
plotContour(cmap1, 0.5)
plt.title('Lignes de niveau de $J(u_1,u_2)$')   
plt.savefig('I11.eps', format='eps')
plt.show()


#PLOT GRADIENT METHOD PATH FOR p = 0.002
plt.figure(figsize = plt.figaspect(0.5))
plotContour(cmap1,-0.5)
linePlot(uhist002, cmap2, 2)
plt.title(r'Le parcours pour $\rho = 0.002$') 
plt.savefig('I12002.eps', format='eps') 
plt.show()

#PLOT GRADIENT METHOD PATH FOR p = 0.0045 and p 0.01
plt.figure(figsize = plt.figaspect(0.3))
plt.subplot(1, 2, 1)
plt.title(r'Le parcours pour $\rho = 0.0045$')  
plotContour(cmap1,-0.5)
linePlot(uhist045, cmap2, 2)
plt.subplot(1, 2, 2)
plt.title(r'Le parcours pour $\rho = 0.01$')  
plotContour(cmap1,-0.5)
linePlot(uhist01, cmap2, 2)
plt.savefig('I12d.eps', format='eps')
plt.show()

#PLOT 3D SURFACE
plotSurface(cm.coolwarm)
plt.savefig('I13d.png', format='png', dpi=500)
plt.show()