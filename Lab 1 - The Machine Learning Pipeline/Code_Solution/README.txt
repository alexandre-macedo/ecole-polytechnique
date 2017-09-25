# Notes:

## Task 7

The results found up until Task 6 are given in the Lab Description. Results after removing the outlier (Task 7) are:

w =  [ 1.838  0.069  0.669  0.185 -0.63   0.28  -0.043]
MSE on test data   0.509881463006
MSE baseline       1.72845474467

Replacing the outlier with 0 can be achieved by either using the line commented at the beginning of the file main.py, 
#y[y >= 8] = 0       # <-- Task 7 data cleaning
or by manual replacement in the data file (the outlier is at line 47).

Regarding the second part of Task 7: it is easy to achieve even better results by, e.g., changing from degree '2' to '3' in the call to poly_exp: 

MSE on test data   0.444276226552
MSE baseline       1.72845474467

## Task 8

Since Task 8 requires refactorization of some of the code, a possible solution is provided separately: in main_bonus.py
