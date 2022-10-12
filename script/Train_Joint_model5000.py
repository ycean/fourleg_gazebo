#!/usr/bin/env python

# import the necessary packages
import sys
import csv
import pandas as pd
import numpy as np
from numpy import mean
from numpy import std
from numpy import asarray
import tensorflow as tf
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense
from sklearn.model_selection import RepeatedKFold
import matplotlib.pyplot as plt


file_path = "/home/ycean/Documents/backup/raw_data_0623/overall_label.csv"
df = pd.read_csv(file_path, names=['L1J1', 'L1J2', 'L1J3', 'L2J1', 'L2J2', 'L2J3', 'L3J1', 'L3J2', 'L3J3', 'L4J1', 'L4J2', 'L4J3'])
len(df)

##extract the column data out from the table
a1 = df['L1J1']
a2 = df['L1J2']
a3 = df['L1J3']
a4 = df['L2J1']
a5 = df['L2J2']
a6 = df['L2J3']
a7 = df['L3J1']
a8 = df['L3J2']
a9 = df['L3J3']
a10 = df['L4J1']
a11 = df['L4J2']
a12 = df['L4J3']

##convert the type of the extracted column data into numpy array
a1 = a1.to_numpy()
a2 = a2.to_numpy()
a3 = a3.to_numpy()
a4 = a4.to_numpy()
a5 = a5.to_numpy()
a6 = a6.to_numpy()
a7 = a7.to_numpy()
a8 = a8.to_numpy()
a9 = a9.to_numpy()
a10 = a10.to_numpy()
a11 = a11.to_numpy()
a12 = a12.to_numpy()

##rearrange the dataset vector's dimension
L1_j1 = a1.reshape(int(len(df)/240), 240)
L1_j2 = a2.reshape(int(len(df)/240), 240)
L1_j3 = a3.reshape(int(len(df)/240), 240)
L2_j1 = a4.reshape(int(len(df)/240), 240)
L2_j2 = a5.reshape(int(len(df)/240), 240)
L2_j3 = a6.reshape(int(len(df)/240), 240)
L3_j1 = a7.reshape(int(len(df)/240), 240)
L3_j2 = a8.reshape(int(len(df)/240), 240)
L3_j3 = a9.reshape(int(len(df)/240), 240)
L4_j1 = a10.reshape(int(len(df)/240), 240)
L4_j2 = a11.reshape(int(len(df)/240), 240)
L4_j3 = a12.reshape(int(len(df)/240), 240)


##extract the dataset of the input data
input_path = "/home/ycean/Documents/backup/raw_data_0623/re_com_overall.csv"
input_df = pd.read_csv(input_path, names=['S_x', 'S_y', 'G_x', 'G_y'])
len(input_df)

##convert the type of the dataset into numpy array
input_df = input_df.to_numpy()

##stack the input data along side the output joint dataset
Leg1_j1 = np.hstack((input_df,L1_j1))
Leg1_j2 = np.hstack((input_df,L1_j2))
Leg1_j3 = np.hstack((input_df,L1_j3))
Leg2_j1 = np.hstack((input_df,L2_j1))
Leg2_j2 = np.hstack((input_df,L2_j2))
Leg2_j3 = np.hstack((input_df,L2_j3))
Leg3_j1 = np.hstack((input_df,L3_j1))
Leg3_j2 = np.hstack((input_df,L3_j2))
Leg3_j3 = np.hstack((input_df,L3_j3))
Leg4_j1 = np.hstack((input_df,L4_j1))
Leg4_j2 = np.hstack((input_df,L4_j2))
Leg4_j3 = np.hstack((input_df,L4_j3))

##split the dataset for training set and testing set for each direction (front,left,right ,back)
##for leg 1 joint 1
L1_J1_ftrain = Leg1_j1[0:561]
L1_J1_ltrain = Leg1_j1[781:1294]
L1_J1_rtrain = Leg1_j1[1420:1745]
L1_J1_btrain = Leg1_j1[1825:2372]
L1_J1_train = np.vstack((L1_J1_ftrain,L1_J1_ltrain,L1_J1_rtrain,L1_J1_btrain))

L1_J1_ftest = Leg1_j1[561:781]
L1_J1_ltest = Leg1_j1[1294:1420]
L1_J1_rtest = Leg1_j1[1745:1825]
L1_J1_btest = Leg1_j1[2372:]
L1_J1_test = np.vstack((L1_J1_ftest,L1_J1_ltest,L1_J1_rtest,L1_J1_btest))

##for leg 1 joint 2
L1_J2_ftrain = Leg1_j2[0:561]
L1_J2_ltrain = Leg1_j2[781:1294]
L1_J2_rtrain = Leg1_j2[1420:1745]
L1_J2_btrain = Leg1_j2[1825:2372]
L1_J2_train = np.vstack((L1_J2_ftrain,L1_J2_ltrain,L1_J2_rtrain,L1_J2_btrain))

L1_J2_ftest = Leg1_j2[561:781]
L1_J2_ltest = Leg1_j2[1294:1420]
L1_J2_rtest = Leg1_j2[1745:1825]
L1_J2_btest = Leg1_j2[2372:]
L1_J2_test = np.vstack((L1_J2_ftest,L1_J2_ltest,L1_J2_rtest,L1_J2_btest))

##for  leg 1 joint 3
L1_J3_ftrain = Leg1_j3[0:561]
L1_J3_ltrain = Leg1_j3[781:1294]
L1_J3_rtrain = Leg1_j3[1420:1745]
L1_J3_btrain = Leg1_j3[1825:2372]
L1_J3_train = np.vstack((L1_J3_ftrain,L1_J3_ltrain,L1_J3_rtrain,L1_J3_btrain))

L1_J3_ftest = Leg1_j3[561:781]
L1_J3_ltest = Leg1_j3[1294:1420]
L1_J3_rtest = Leg1_j3[1745:1825]
L1_J3_btest = Leg1_j3[2372:]
L1_J3_test = np.vstack((L1_J3_ftest,L1_J3_ltest,L1_J3_rtest,L1_J3_btest))

##for leg 2 joint 1
L2_J1_ftrain = Leg2_j1[0:561]
L2_J1_ltrain = Leg2_j1[781:1294]
L2_J1_rtrain = Leg2_j1[1420:1745]
L2_J1_btrain = Leg2_j1[1825:2372]
L2_J1_train = np.vstack((L2_J1_ftrain,L2_J1_ltrain,L2_J1_rtrain,L2_J1_btrain))

L2_J1_ftest = Leg2_j1[561:781]
L2_J1_ltest = Leg2_j1[1294:1420]
L2_J1_rtest = Leg2_j1[1745:1825]
L2_J1_btest = Leg2_j1[2372:]
L2_J1_test = np.vstack((L2_J1_ftest,L2_J1_ltest,L2_J1_rtest,L2_J1_btest))

##for leg 2 joint 2
L2_J2_ftrain = Leg2_j2[0:561]
L2_J2_ltrain = Leg2_j2[781:1294]
L2_J2_rtrain = Leg2_j2[1420:1745]
L2_J2_btrain = Leg2_j2[1825:2372]
L2_J2_train = np.vstack((L2_J2_ftrain,L2_J2_ltrain,L2_J2_rtrain,L2_J2_btrain))

L2_J2_ftest = Leg2_j2[561:781]
L2_J2_ltest = Leg2_j2[1294:1420]
L2_J2_rtest = Leg2_j2[1745:1825]
L2_J2_btest = Leg2_j2[2372:]
L2_J2_test = np.vstack((L2_J2_ftest,L2_J2_ltest,L2_J2_rtest,L2_J2_btest))

##for leg 2 joint 3
L2_J3_ftrain = Leg2_j3[0:561]
L2_J3_ltrain = Leg2_j3[781:1294]
L2_J3_rtrain = Leg2_j3[1420:1745]
L2_J3_btrain = Leg2_j3[1825:2372]
L2_J3_train = np.vstack((L2_J3_ftrain,L2_J3_ltrain,L2_J3_rtrain,L2_J3_btrain))

L2_J3_ftest = Leg2_j3[561:781]
L2_J3_ltest = Leg2_j3[1294:1420]
L2_J3_rtest = Leg2_j3[1745:1825]
L2_J3_btest = Leg2_j3[2372:]
L2_J3_test = np.vstack((L2_J3_ftest,L2_J3_ltest,L2_J3_rtest,L2_J3_btest))

##for leg 3 joint 1
L3_J1_ftrain = Leg3_j1[0:561]
L3_J1_ltrain = Leg3_j1[781:1294]
L3_J1_rtrain = Leg3_j1[1420:1745]
L3_J1_btrain = Leg3_j1[1825:2372]
L3_J1_train = np.vstack((L3_J1_ftrain,L3_J1_ltrain,L3_J1_rtrain,L3_J1_btrain))

L3_J1_ftest = Leg3_j1[561:781]
L3_J1_ltest = Leg3_j1[1294:1420]
L3_J1_rtest = Leg3_j1[1745:1825]
L3_J1_btest = Leg3_j1[2372:]
L3_J1_test = np.vstack((L3_J1_ftest,L3_J1_ltest,L3_J1_rtest,L3_J1_btest))

##for leg 3 joint 2
L3_J2_ftrain = Leg3_j2[0:561]
L3_J2_ltrain = Leg3_j2[781:1294]
L3_J2_rtrain = Leg3_j2[1420:1745]
L3_J2_btrain = Leg3_j2[1825:2372]
L3_J2_train = np.vstack((L3_J2_ftrain,L3_J2_ltrain,L3_J2_rtrain,L3_J2_btrain))

L3_J2_ftest = Leg3_j2[561:781]
L3_J2_ltest = Leg3_j2[1294:1420]
L3_J2_rtest = Leg3_j2[1745:1825]
L3_J2_btest = Leg3_j2[2372:]
L3_J2_test = np.vstack((L3_J2_ftest,L3_J2_ltest,L3_J2_rtest,L3_J2_btest))

##for leg 3 joint 3
L3_J3_ftrain = Leg3_j3[0:561]
L3_J3_ltrain = Leg3_j3[781:1294]
L3_J3_rtrain = Leg3_j3[1420:1745]
L3_J3_btrain = Leg3_j3[1825:2372]
L3_J3_train = np.vstack((L3_J3_ftrain,L3_J3_ltrain,L3_J3_rtrain,L3_J3_btrain))

L3_J3_ftest = Leg3_j3[561:781]
L3_J3_ltest = Leg3_j3[1294:1420]
L3_J3_rtest = Leg3_j3[1745:1825]
L3_J3_btest = Leg3_j3[2372:]
L3_J3_test = np.vstack((L3_J3_ftest,L3_J3_ltest,L3_J3_rtest,L3_J3_btest))

##for leg 4 joint 1
L4_J1_ftrain = Leg4_j1[0:561]
L4_J1_ltrain = Leg4_j1[781:1294]
L4_J1_rtrain = Leg4_j1[1420:1745]
L4_J1_btrain = Leg4_j1[1825:2372]
L4_J1_train = np.vstack((L4_J1_ftrain,L4_J1_ltrain,L4_J1_rtrain,L4_J1_btrain))

L4_J1_ftest = Leg4_j1[561:781]
L4_J1_ltest = Leg4_j1[1294:1420]
L4_J1_rtest = Leg4_j1[1745:1825]
L4_J1_btest = Leg4_j1[2372:]
L4_J1_test = np.vstack((L4_J1_ftest,L4_J1_ltest,L4_J1_rtest,L4_J1_btest))

##for leg 4 joint 2
L4_J2_ftrain = Leg4_j2[0:561]
L4_J2_ltrain = Leg4_j2[781:1294]
L4_J2_rtrain = Leg4_j2[1420:1745]
L4_J2_btrain = Leg4_j2[1825:2372]
L4_J2_train = np.vstack((L4_J2_ftrain,L4_J2_ltrain,L4_J2_rtrain,L4_J2_btrain))

L4_J2_ftest = Leg4_j2[561:781]
L4_J2_ltest = Leg4_j2[1294:1420]
L4_J2_rtest = Leg4_j2[1745:1825]
L4_J2_btest = Leg4_j2[2372:]
L4_J2_test = np.vstack((L4_J2_ftest,L4_J2_ltest,L4_J2_rtest,L4_J2_btest))

##for leg 4 joint 3
L4_J3_ftrain = Leg4_j3[0:561]
L4_J3_ltrain = Leg4_j3[781:1294]
L4_J3_rtrain = Leg4_j3[1420:1745]
L4_J3_btrain = Leg4_j3[1825:2372]
L4_J3_train = np.vstack((L4_J3_ftrain,L4_J3_ltrain,L4_J3_rtrain,L4_J3_btrain))

L4_J3_ftest = Leg4_j3[561:781]
L4_J3_ltest = Leg4_j3[1294:1420]
L4_J3_rtest = Leg4_j3[1745:1825]
L4_J3_btest = Leg4_j3[2372:]
L4_J3_test = np.vstack((L4_J3_ftest,L4_J3_ltest,L4_J3_rtest,L4_J3_btest))

###!!!SINCE THE DATASET WILL BE TRAINED IN THE KERAS MODEL, THE SPLITTING ALGORITHM WILL BE INCLUDED,#
##SO THE FOLLOWING ACTON IS TO STACK BACK THE TRAIN SET AND TEST SET DATA TOGETHER AND SET X AS INPUT DATASET#
# AND Y AS THE OUTPUT DATASET ##
data_set_L1J1 = np.vstack((L1_J1_train,L1_J1_test))
X_L1J1 = data_set_L1J1[:,0:4]
data_set_L1J2 = np.vstack((L1_J2_train,L1_J2_test))
X_L1J2 = data_set_L1J2[:,0:4]
data_set_L1J3 = np.vstack((L1_J3_train,L1_J3_test))
X_L1J3 = data_set_L1J3[:,0:4]
data_set_L2J1 = np.vstack((L2_J1_train,L2_J1_test))
X_L2J1 = data_set_L2J1[:,0:4]
data_set_L2J2 = np.vstack((L2_J2_train,L2_J2_test))
X_L2J2 = data_set_L2J2[:,0:4]
data_set_L2J3 = np.vstack((L2_J3_train,L2_J3_test))
X_L2J3 = data_set_L2J3[:,0:4]
data_set_L3J1 = np.vstack((L3_J1_train,L3_J1_test))
X_L3J1 = data_set_L3J1[:,0:4]
data_set_L3J2 = np.vstack((L3_J2_train,L3_J2_test))
X_L3J2 = data_set_L3J2[:,0:4]
data_set_L3J3 = np.vstack((L3_J3_train,L3_J3_test))
X_L3J3 = data_set_L3J3[:,0:4]
data_set_L4J1 = np.vstack((L4_J1_train,L4_J1_test))
X_L4J1 = data_set_L4J1[:,0:4]
data_set_L4J2 = np.vstack((L4_J2_train,L4_J2_test))
X_L4J2 = data_set_L4J2[:,0:4]
data_set_L4J3 = np.vstack((L4_J3_train,L4_J3_test))
X_L4J3 = data_set_L4J3[:,0:4]

y_L1J1 = data_set_L1J1[:,4:]
y_L1J2 = data_set_L1J2[:,4:]
y_L1J3 = data_set_L1J3[:,4:]
y_L2J1 = data_set_L2J1[:,4:]
y_L2J2 = data_set_L2J2[:,4:]
y_L2J3 = data_set_L2J3[:,4:]
y_L3J1 = data_set_L3J1[:,4:]
y_L3J2 = data_set_L3J2[:,4:]
y_L3J3 = data_set_L3J3[:,4:]
y_L4J1 = data_set_L4J1[:,4:]
y_L4J2 = data_set_L4J2[:,4:]
y_L4J3 = data_set_L4J3[:,4:]


###NOW CREATE THE TRAINING MODEL ALGORITHM##
def get_model(n_inputs,n_outputs):
    model = Sequential()
    model.add(Dense(4,input_dim =n_inputs,kernel_initializer = 'he_uniform',activation='relu'))
    model.add(Dense(16,activation='relu'))
    model.add(Dense(48,activation='relu'))
    model.add(Dense(n_outputs))
    model.compile(loss='mse',optimizer='adam')
    return model

####NOW CREATE THE EVALUATING  MODEL ALGORITHM
def evaluate_model(X,y):
    results = list()
    n_inputs, n_outputs = X.shape[1], y.shape[1]
    cv = RepeatedKFold(n_splits=4, n_repeats=240, random_state=1)
    for train_ix, test_ix in cv.split(X):
        X_train, X_test = X_L1J1[train_ix], X_L1J1[test_ix]
        y_train, y_test = y_L1J1[train_ix], y_L1J1[test_ix]
        model = get_model(n_inputs, n_outputs)
        model.fit(X_train, y_train, verbose=0, epochs=5000)
        mse = model.evaluate(X_test, y_test, verbose=0)
        print('>%.3f' % mse)
        results.append(mse)
    return results


print("For re_5000_h16_h48:")

## WITH THE PREPARED DATASET FOR EACH JOINT, APPLY THE TRAINING MODEL ALGORITHM, START THE DATA TRAINING##
#FOR LEG 1 JOINT 1
#print("model training...")

#X, y = X_L1J1, y_L1J1
#n_input, n_output = X.shape[1], y.shape[1]
#model1 = get_model(n_input, n_output)
#model1.fit(X, y, verbose=0, epochs=5000)

#print("model 1 done...")

#FOR LEG 1 JOINT 2
#X, y = X_L1J2, y_L1J2
#n_input, n_output = X.shape[1], y.shape[1]
#model2 = get_model(n_input, n_output)
#model2.fit(X, y, verbose=0, epochs=5000)

#print("model 2 done...")

#FOR LEG 1 JOINT 3
#X, y = X_L1J3, y_L1J3
#n_input, n_output = X.shape[1], y.shape[1]
#model3 = get_model(n_input, n_output)
#model3.fit(X, y, verbose=0, epochs=5000)

#print("model 3 done...")

#FOR LEG 2 JOINT 1
#X, y = X_L2J1, y_L2J1
#n_input, n_output = X.shape[1], y.shape[1]
#model4 = get_model(n_input, n_output)
#model4.fit(X, y, verbose=0, epochs=5000)

#print("model 4 done...")

#FOR LEG 2 JOINT 2
#X, y = X_L2J2, y_L2J2
#n_input, n_output = X.shape[1], y.shape[1]
#model5 = get_model(n_input, n_output)
#model5.fit(X, y, verbose=0, epochs=5000)

#print("model 5 done...")

#FOR LEG 2 JOINT 3
#X, y = X_L2J3, y_L2J3
#n_input, n_output = X.shape[1], y.shape[1]
#model6 = get_model(n_input, n_output)
#model6.fit(X, y, verbose=0, epochs=5000)

#print("model 6 done...")

#FOR LEG 3 JOINT 1
#X, y = X_L3J1, y_L3J1
#n_input, n_output = X.shape[1], y.shape[1]
#model7 = get_model(n_input, n_output)
#model7.fit(X, y, verbose=0, epochs=5000)

#print("model 7 done...")

#FOR LEG 3 JOINT 2
#X, y = X_L3J2, y_L3J2
#n_input, n_output = X.shape[1], y.shape[1]
#model8 = get_model(n_input, n_output)
#model8.fit(X, y, verbose=0, epochs=5000)

#print("model 8 done...")

#FOR LEG 3 JOINT 3
#X, y = X_L3J3, y_L3J3
#n_input, n_output = X.shape[1], y.shape[1]
#model9 = get_model(n_input, n_output)
#model9.fit(X, y, verbose=0, epochs=5000)

#print("model 9 done...")

#FOR LEG 4 JOINT 1
#X, y = X_L4J1, y_L4J1
#n_input, n_output = X.shape[1], y.shape[1]
#model10 = get_model(n_input, n_output)
#model10.fit(X, y, verbose=0, epochs=5000)

#print("model 10 done...")

#FOR LEG 4 JOINT 2
#X, y = X_L4J2, y_L4J2
#n_input, n_output = X.shape[1], y.shape[1]
#model11 = get_model(n_input, n_output)
#model11.fit(X, y, verbose=0, epochs=5000)

#print("model 11 done...")

#FOR LEG 4 JOINT 3
#X, y = X_L4J3, y_L4J3
#n_input, n_output = X.shape[1], y.shape[1]
#model12 = get_model(n_input, n_output)
#model12.fit(X, y, verbose=0, epochs=5000)

#print("model 12 done...")

#print("model saving...")

##save model
#model1.save("re_5000_h16_h48_model1.h5")
#model2.save("re_5000_h16_h48_model2.h5")
#model3.save("re_5000_h16_h48_model3.h5")
#model4.save("re_5000_h16_h48_model4.h5")
#model5.save("re_5000_h16_h48_model5.h5")
#model6.save("re_5000_h16_h48_model6.h5")
#model7.save("re_5000_h16_h48_model7.h5")
#model8.save("re_5000_h16_h48_model8.h5")
#model9.save("re_5000_h16_h48_model9.h5")
#model10.save("re_5000_h16_h48_model10.h5")
#model11.save("re_5000_h16_h48_model11.h5")
#model12.save("re_5000_h16_h48_model12.h5")

#print("model save done!")

### WITH THE PREPARED DATASET FOR EACH JOINT, APPLY THE TRAINING MODEL ALGORITHM, EVALUATE THE TRAINING SET##
#FOR LEG 1 JOINT 1
print("model 1 evaluation result:")
X, y = X_L1J1, y_L1J1
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 1 JOINT 2
print("model 2 evaluation result:")
X, y = X_L1J2, y_L1J2
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 1 JOINT 3
print("model 3 evaluation result:")
X, y = X_L1J3, y_L1J3
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 2 JOINT 1
print("model 4 evaluation result:")
X, y = X_L2J1, y_L2J1
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 2 JOINT 2
print("model 5 evaluation result:")
X, y = X_L2J2, y_L2J2
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 2 JOINT 3
print("model 6 evaluation result:")
X, y = X_L2J3, y_L2J3
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 3 JOINT 1
print("model 7 evaluation result:")
X, y = X_L3J1, y_L3J1
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 3 JOINT 2
print("model 8 evaluation result:")
X, y = X_L3J2, y_L3J2
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 3 JOINT 3
print("model 9 evaluation result:")
X, y = X_L3J3, y_L3J3
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 4 JOINT 1
print("model 10 evaluation result:")
X, y = X_L4J1, y_L4J1
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 4 JOINT 2
print("model 11 evaluation result:")
X, y = X_L4J2, y_L4J2
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))

#FOR LEG 4 JOINT 3
print("model 12 evaluation result:")
X, y = X_L4J3, y_L4J3
results1 = evaluate_model(X, y)
print('MSE: %.3f (%.3f)' % (mean(results1), std(results1)))




#Load model
#model1 = keras.models.load_model("re_model1.h5")
#model2 = keras.models.load_model("re_model2.h5")
#model3 = keras.models.load_model("re_model3.h5")
#model4 = keras.models.load_model("re_model4.h5")
#model5 = keras.models.load_model("re_model5.h5")
#model6 = keras.models.load_model("re_model6.h5")
#model7 = keras.models.load_model("re_model7.h5")
#model8 = keras.models.load_model("re_model8.h5")
#model9 = keras.models.load_model("re_model9.h5")
#model10 = keras.models.load_model("re_model10.h5")
#model11 = keras.models.load_model("re_model11.h5")
#model12 = keras.models.load_model("re_model12.h5")



## AFTER DONE THE TRAINING , PREDICT FOR NEW OUTPUT##
#FOR LEG 1
#def predict_L1J(S_x,S_y,G_x,G_y):
#    test_in = [S_x,S_y,G_x,G_y]
#    newX = asarray([test_in])
#    L1J1_sln = model1.predict(newX)
#    L1J2_sln = model2.predict(newX)
#    L1J3_sln = model3.predict(newX)
#    LEG1_Joint = np.column_stack((L1J1_sln[0].reshape(240,1),L1J2_sln[0].reshape(240,1),L1J3_sln[0].reshape(240,1)))
#    return LEG1_Joint

#FOR LEG 2
#def predict_L2J(S_x,S_y,G_x,G_y):
#    test_in = [S_x,S_y,G_x,G_y]
#    newX = asarray([test_in])
#    L2J1_sln = model4.predict(newX)
#    L2J2_sln = model5.predict(newX)
#    L2J3_sln = model6.predict(newX)
#    LEG2_Joint = np.column_stack((L2J1_sln[0].reshape(240,1),L2J2_sln[0].reshape(240,1),L2J3_sln[0].reshape(240,1)))
#    return LEG2_Joint

#FOR LEG 3
#def predict_L3J(S_x,S_y,G_x,G_y):
#    test_in = [S_x,S_y,G_x,G_y]
#    newX = asarray([test_in])
#    L3J1_sln = model7.predict(newX)
#    L3J2_sln = model8.predict(newX)
#    L3J3_sln = model9.predict(newX)
#    LEG3_Joint = np.column_stack((L3J1_sln[0].reshape(240,1),L3J2_sln[0].reshape(240,1),L3J3_sln[0].reshape(240,1)))
#    return LEG3_Joint

#FOR LEG 4
#def predict_L4J(S_x,S_y,G_x,G_y):
#    test_in = [S_x,S_y,G_x,G_y]
#    newX = asarray([test_in])
#    L4J1_sln = model10.predict(newX)
#    L4J2_sln = model11.predict(newX)
#    L4J3_sln = model12.predict(newX)
#    LEG4_Joint = np.column_stack((L4J1_sln[0].reshape(240,1),L4J2_sln[0].reshape(240,1),L4J3_sln[0].reshape(240,1)))
#    return LEG4_Joint

##PLOT THE PREPARE DATASET AND THE PREDICT DATASET THEN COMPARE THE RESULT FOR BOTH
#FOR LEG 1 JOINT 1
#
#t_time = np.linspace(0, 240, 240)
#line1, = plt.plot(t_time, y_L1J1[700,:], label='experiment_raw_data')
#line2, = plt.plot(t_time, L1J1_sln[0], label='learning_solution')
#plt.legend(handles=[line1, line2], loc='lower right')
##major grid lines
#plt.grid(b=True, which='major', color='gray', alpha=0.6, linestyle='dashdot', lw=1.5)
##minor grid lines
#plt.minorticks_on()
#plt.grid(b=True, which='minor', color='beige', alpha=0.8, ls='-', lw=1)
#plt.show()
