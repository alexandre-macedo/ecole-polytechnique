import matplotlib.pyplot as plt
import numpy as np
from numpy import matrix
from numpy import linalg


def f(x, y, B):
    K = int(B.shape[0]/3)
    a = B[0:K]
    o = B[K:2*K]
    x0 = B[2*K:3*K]
    N = x.shape[0]
    out = y.copy()
    for i in range(N):
        for j in range(K):
            out[i] -= a[j]*np.exp((-1/2) * ((x[i]-x0[j])/o[j])**2)
    return out
    
def buildJ(x, y, B):
    K = int(B.shape[0]/3)
    a = B[0:K]
    o = B[K:2*K]
    x0 = B[2*K:3*K]
    N = x.shape[0]
    J = np.zeros([N,K*3])
    for i in range(N):
        for j in range(K):
            expP = np.exp((-1/2) * ((x[i]-x0[j])/o[j])**2)
            J[i][j] = -expP
            J[i][K + j] = -expP * a[j] * (((x[i]-x0[j])**2) / (o[j]**3))
            J[i][2*K + j] = -expP * a[j] * ((x[i]-x0[j]) / (o[j]**2))
    return J

def GaussNewton(steps, X, Y, B):
    print('Running Gauss-Newton method for B =',B)
    J = linalg.norm(f(X,Y,B))**2
    Jo = J
    histB = [B]
    for i in range(steps):
        Jac = buildJ(X, Y, B)
        JacT = matrix.transpose(Jac)    
        B = B - np.matmul(np.linalg.inv(np.matmul(JacT, Jac)), np.matmul(JacT, f(X, Y, B)))
        J = linalg.norm(f(X,Y,B))**2
        histB.append(B)
        
    histB = np.matrix(histB)
    if(J > Jo):
        print("Diverged!")
    else:
        print("Converged to", B)
    return histB

def plotPointCloud(histB):
    labels = [r'\alpha', r'\sigma','x^0']
    K = histB.shape[1]
    steps = histB.shape[0]
    for j in range(K):
        plt.plot(range(steps),histB[:,j],'-o',label = ('$'+labels[int(3*j/K)]+'_{0}$').format(1 + j%int(K/3)))
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2,  ncol=1)
    plt.gca().grid(True)
    plt.title(r'Nuage de points pour la méthode de Gauss-Newton')  
    
X = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
Y = np.array([0.127, 0.2, 0.3, 0.25, 0.32, 0.5, 0.7, 0.9])

histB7 = GaussNewton(10, X, Y, np.array([1.0, 1.0, 1.0, 1.0, 3.0, 7.0])) 
histB6 = GaussNewton(11, X, Y, np.array([1.0, 1.0, 1.0, 1.0, 3.0, 6.0]))


#PLOT POINT CLOUD OF GAUSS NEWTON FOR X^0_2 = 7
fig = plt.figure(figsize = plt.figaspect(0.5))
fig.add_axes([0.1, 0.1, 0.7, 0.75])
plotPointCloud(histB7)
plt.savefig('I22.eps', format='eps')
plt.show()

#PLOT POINT CLOUD OF GAUSS NEWTON FOR X^0_2 = 6
fig = plt.figure(figsize = plt.figaspect(0.5))
fig.add_axes([0.1, 0.1, 0.7, 0.75])
plotPointCloud(histB6)
plt.show()