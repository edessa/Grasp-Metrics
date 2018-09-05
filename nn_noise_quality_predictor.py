import tensorflow as tf
import numpy as np
import os
import math
import glob
import random
from ast import literal_eval
import tflearn
from tflearn.layers.core import input_data, dropout, fully_connected
from tflearn.layers.conv import conv_2d, max_pool_2d
from tflearn.layers.estimator import regression
from tflearn.metrics import Accuracy
from tflearn.data_preprocessing import ImagePreprocessing
from tflearn.data_augmentation import ImageAugmentation
tf.reset_default_graph()


def getFilenames(base_directory):
    matrixFname = []
    labelFname = []
    for filename in os.listdir(base_directory):
        if '_labels' in str(filename):
            split = filename.split('_labels')
            matrixFname.append(base_directory + split[0] + split[1])
            labelFname.append(base_directory + filename)
    c = zip(matrixFname, labelFname)
    random.shuffle(c)
    matrixFname, labelFname = zip(*c)
    print len(labelFname)
    return matrixFname, labelFname

def applyThreshold(label, threshold):
    if math.fabs(label[1] - label[0]) < threshold:
        return 1
    return 0

def stripLine(fname):
    lines = open(fname, 'r').readlines()
    lines[0] = ""
    file = open(fname, 'w')
    for line in lines:
        file.write(line)
    file.close()

def getLabel(labelFilename):
    eps_labels = np.genfromtxt(labelFilename, delimiter=",")
    for i in range(0, len(eps_labels)):
        if eps_labels[i] == -1:
            eps_labels[i] = 0
    average = np.average(eps_labels)
    return [average]

def parseData(base_directory, n_pads):
    matrices = []
    labels = []
    numAligned = 0.0
    matricesFname, labelsFname = getFilenames(base_directory)
    maxVal = -1.0
    for i in range(0, len(labelsFname)):
        try:
            matrixToAppend = []
            matrix = np.genfromtxt(matricesFname[i], delimiter='\n', dtype='unicode') #Filter normals
        #    matrix = np.array(list(matrix)[0:54] + list(matrix)[54:63]) backward selection stuff
            #matrix = np.array(text.split()).astype(np.float) #Filter normals
            for j in range(1, len(matrix)): #skip first empty [] in matrix
                np_mat = np.array(literal_eval(matrix[j])).flatten()[0::2]
                np_mat = np.reshape(np_mat, (-1, 21, 21))
                matrixToAppend.append(np_mat)
            matrices.append(matrixToAppend)
            label = getLabel(labelsFname[i])
            labels.append(label)
        except IOError:
            continue
    return matrices, labels

def CNN(trainInputs, trainLabels, valInputs, valLabels, epochs, n_grid):
    acc = Accuracy()
    cnn = input_data(shape=[None, 21, 21, n_grid])
    cnn = conv_2d(cnn, 128, 5, strides=1, activation='relu', name='conv1')
    cnn = max_pool_2d(cnn, 2, strides=2)
    cnn = conv_2d(cnn, 64, 3, strides=1, activation='relu', name='conv2')
    cnn = conv_2d(cnn, 64, 3, strides=1, activation='relu', name='conv3')
    cnn = max_pool_2d(cnn, 2, strides=2)
    cnn = fully_connected(cnn, 1)
    cnn = regression(cnn, optimizer='adam', loss='mean_square', learning_rate=0.001, metric='R2')
    model = tflearn.DNN(cnn)
    #model.fit(trainInputs, trainLabels, n_epoch=epochs, shuffle=True, validation_set=(valInputs, valLabels), show_metric=True, run_id='cnn')
    model.fit(trainInputs, trainLabels, n_epoch=epochs, show_metric=True, run_id='cnn')

    return model

def compareOutputs(y_est_np, labels):
    return np.mean(np.abs((labels - y_est_np)))

#Barrett: 7 padds

num_training = 5800
num_testing = 300
n_pads = 7 #7 for barrett, 
matrices, labels = parseData('/home/eadom/Grasp-Metrics/Barrett_Data/', n_pads)
print len(matrices)
print len(labels)
matrices = np.reshape(np.array(matrices)[:,:,0], (-1, 21, 21, n_pads))
labels = np.array(labels)

model = CNN(matrices[0:num_training], labels[0:num_training], matrices[num_training:num_testing],  labels[num_training:num_testing], 100, n_pads)
score = model.evaluate(matrices[num_training:num_training+num_testing], labels[num_training:num_training+num_testing])
