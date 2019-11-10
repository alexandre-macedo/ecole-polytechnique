from numpy import *
import matplotlib.pyplot as plt
from loadMnist import loadMnist
from kNN import kNN    

# Load training and test data
trainingImages, trainingLabels = loadMnist('training')
testImages, testLabels = loadMnist('testing')

# Keep a subset of the training and test data
trainingImages = trainingImages[:2000,:]
trainingLabels = trainingLabels[:2000]

testImages = testImages[:50,:]
testLabels = testLabels[:50]

# Show the first ten digits
fig = plt.figure('First 10 Digits') 
for i in range(10):
    a = fig.add_subplot(2,5,i+1) 
    plt.imshow(testImages[i,:].reshape(28,28), cmap=plt.cm.gray)
    plt.axis('off')

plt.show()

# Run kNN algorithm
k = 5
predictedDigits = zeros(testImages.shape[0])

for i in range(testImages.shape[0]):
    print("Current Test Instance: %d" % (i+1))
    predictedDigits[i] = kNN(k, trainingImages, trainingLabels, testImages[i,:])
    
# Calculate accuracy
correct = 0

for i in range(testImages.shape[0]):
    if predictedDigits[i] == testLabels[i]:
        correct += 1
        
accuracy = correct/float(testImages.shape[0])
print("\nAccuracy: %4.3f" % accuracy)
