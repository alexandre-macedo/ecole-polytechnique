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

# ...
# (see main.py)
# ...

# Standardization

m = mean(X,axis=0)
s = std(X,axis=0)
X = (X - m) / s

# Feature creation

from tools import poly_exp

degrees = [0,1,2,3,4,5,6,7]
MSE_train = zeros(len(degrees))
MSE_test = zeros(len(degrees))

for degree in degrees:

    Z = poly_exp(X,degree)

    Z = column_stack([ones(len(Z)), Z])

    # Building a model

    w = dot(inv(dot(Z.T,Z)),dot(Z.T,y))
    print(w)

    from tools import MSE

    yp = dot(Z,w)
    MSE_train[degree] = MSE(y,yp)
    print("MSE on train data ", MSE(y,yp))

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

    MSE_test[degree] = MSE(y_test,y_pred)
    print("MSE on test data  ", MSE(y_test,y_pred))

    print("MSE baseline      ", MSE(y_test,ones(len(y_test))*mean(y)))

figure()
plot(degrees[1:],MSE_train[1:], 'r-', label="Training data")
plot(degrees[1:],MSE_test[1:], 'b-', label="Test data")
xlabel('degree of polynomial')
ylabel('MSE')
legend()
#savefig("../Description/fig/overfitting.pdf")
show()
