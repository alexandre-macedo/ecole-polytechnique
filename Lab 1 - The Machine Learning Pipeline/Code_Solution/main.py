from numpy import *
set_printoptions(precision=3)
from matplotlib.pyplot import *
from scipy.linalg import inv

# Load the data

data = loadtxt('data/data_train.csv', delimiter=',')

# Prepare the data

X = data[:,0:-1]
y = data[:,-1]
y[y >= 8] = 0       # <-- Task 7 data cleaning

# Inspect the data

figure()
hist(X[:,1], 10)
#savefig("../Description/fig/hist1.pdf")

figure()
hist(X[:,2], 10)
#savefig("../Description/fig/hist2.pdf")

figure()
plot(X[:,1],X[:,2], 'o')
xlabel('x2')
ylabel('x3')
#savefig("../Description/fig/data.pdf")

figure()
plot(X[:,0],y, 'o')
xlabel('x1')
ylabel('y')
#savefig("../Description/fig/data_y.pdf")
show()

# Standardization

m = mean(X,axis=0)
s = std(X,axis=0)
X = (X - m) / s

# Feature creation

from tools import poly_exp

# Polynomial degree
degree = 3

Z = poly_exp(X,degree)

Z = column_stack([ones(len(Z)), Z])

# Building a model

w = dot(inv(dot(Z.T,Z)),dot(Z.T,y))
print("w = ", w)

from tools import MSE

yp = dot(Z,w)

# Evaluation 

data = loadtxt('data/data_test.csv', delimiter=',')

# Preparation and preprocessing 

X_test = data[:,0:-1]
X_test = (X_test - m) / s
Z_test = poly_exp(X_test,degree)
Z_test = column_stack([ones(len(Z_test)), Z_test])
y_test = data[:,-1]

# Prediction

y_pred = dot(Z_test,w)

# Score

print("MSE on test data  ", MSE(y_test,y_pred))
print("MSE baseline      ", MSE(y_test,ones(len(y_test))*mean(y)))
