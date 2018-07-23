import tensorflow as tf
import numpy as np
import os
import math
import glob
import random


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

def parseData(base_directory):
    matrices = []
    labels = []
    numAligned = 0.0
    matricesFname, labelsFname = getFilenames(base_directory)
    maxVal = -1.0
    for i in range(0, len(labelsFname)):
        with open(matricesFname[i], 'r') as my_file:
            text = my_file.read()
            text = text.replace("[", " ")
            text = text.replace("]", " ")
            text = text.replace(",", " ")
            text = text.replace("\n", " ")
            matrix = np.array(text.split()).astype(np.float)[0::2] #Filter normals
            #matrix = np.array(text.split()).astype(np.float) #Filter normals

        matrices.append(matrix)
        fingerIters = np.loadtxt(str(labelsFname[i]))[2:4]
        label = math.fabs(fingerIters[1] - fingerIters[0])
        labels.append(label)

    return matrices, labels

def oneHotLabels(labels):
    hotLabels = []
    for label in labels:
        if label == 0:
            hotLabels.append([0])
        else:
            hotLabels.append([1])
    return hotLabels

def NN(matrices, labels, hidden_nodes, num_iters):
    lRate = 0.01
    loss_plot = []
    matrices = np.array(matrices)

    tf.reset_default_graph()
    X = tf.placeholder(shape=(None, len(matrices[0])), dtype=tf.float64, name='X')
    y = tf.placeholder(shape=(None, 1), dtype=tf.float64, name='y')

    W1 = tf.Variable(np.random.rand(len(matrices[0]), hidden_nodes), dtype=tf.float64)
    W2 = tf.Variable(np.random.rand(hidden_nodes, 1), dtype=tf.float64)

    # Create the neural net graph
    #A1 = tf.sigmoid(tf.matmul(X, W1))
    #y_est = tf.sigmoid(tf.matmul(A1, W2))
    A1 = tf.nn.sigmoid(tf.matmul(X, W1))
    y_est = tf.nn.sigmoid(tf.matmul(A1, W2))
#    y_est = tf.matmul(A1, W2)


    # Define a loss function
    deltas = tf.square(y_est - y)
    loss = tf.reduce_sum(deltas)

    # Define a train operation to minimize the loss
#    optimizer = tf.train.GradientDescentOptimizer(lRate)
    optimizer = tf.train.AdamOptimizer(learning_rate=lRate)
#    optimizer = tf.train.MomentumOptimizer(lRate,momentum=0.005)
    train = optimizer.minimize(loss)

    # Initialize variables and run session
    init = tf.global_variables_initializer()
    sess = tf.Session()
    sess.run(init)

    # Go through num_iters iterations
    for i in range(num_iters):
        sess.run(train, feed_dict={X: matrices, y: labels})
        loss_plot.append(sess.run(loss, feed_dict={X: matrices, y: labels}))
        weights1 = sess.run(W1)
        weights2 = sess.run(W2)

    print("loss (hidden nodes: %d, iterations: %d learning rate: %.2f training size: %d): %.2f" % (hidden_nodes, num_iters, lRate, len(matrices), loss_plot[-1]))
    sess.close()
    return weights1, weights2, loss_plot

def removeNaN(y_est_np, labels):
    for i in range(0, len(y_est_np)):
        if i >= len(labels):
            break
        if y_est_np[i][0] == 0 or labels[i][0] == 0:
            y_est_np = np.delete(y_est_np, i, 0)
            labels = np.delete(labels, i, 0)

    return y_est_np, labels

def compareOutputs(y_est_np, labels):
    return np.mean(np.abs((labels - y_est_np)))

def testing(matrices, labels):
    X = tf.placeholder(shape=(None, len(matrices[0])), dtype=tf.float64, name='X')
    y = tf.placeholder(shape=(None, 1), dtype=tf.float64, name='y')
    W1 = tf.Variable(weights1)
    W2 = tf.Variable(weights2)
#    A1 = tf.sigmoid(tf.matmul(X, W1))
    A1 = tf.nn.sigmoid(tf.matmul(X, W1))
    y_est = tf.nn.sigmoid(tf.matmul(A1, W2))
#    y_est = tf.matmul(A1, W2)

    init = tf.global_variables_initializer()
    with tf.Session() as sess:
        sess.run(init)
        y_est_np = sess.run(y_est, feed_dict={X: matrices, y: labels})

    y_est_np, labels = removeNaN(y_est_np, labels)
    accuracy = compareOutputs(y_est_np, labels)
    return accuracy, y_est_np

num_training = 10000
num_testing = 5000
matrices, labels = parseData('/home/eadom/Grasp-Metrics/6x6Res/')

matrices = np.array(matrices)
labels = np.array(labels)/max(labels)
labels = labels.reshape(-1,1)



weights1, weights2, loss_plot = NN(matrices[0:num_training], labels[0:num_training], 30, 5000)

test_deviation, test_pred = testing(matrices[num_training:num_training+num_testing], labels[num_training:num_training+num_testing])
train_deviation, train_pred = testing(matrices[0:num_training], labels[0:num_training])

print "test deviation: " + str(test_deviation)
print "train deviation: " + str(train_deviation)
