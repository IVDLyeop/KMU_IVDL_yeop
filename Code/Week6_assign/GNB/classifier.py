import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").

    #define variables and train the given data set
    def train(self, X, Y):  
        self.obslen = len(X[0])  
        self.label = ['left', 'keep', 'right']  
        self.statistics = {label: {} for label in self.label}  
        map_XY = {label: [] for label in self.label}  
        for x, y in zip(X, Y):  
            map_XY[y].append(x)  

        for label, data in map_XY.items():  
            data = np.array(data)  
            self.statistics[label].update({  
                'mu': data.mean(axis=0),  
                'sigma': data.std(axis=0),  
                'shape': data.shape[0],  
            })
        

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):  
        prob = {label: [] for label in self.label}  
        for label in self.label:  
            for i in range(self.obslen):  
                g_prob = float(  
                    gaussian_prob(observation[i], self.statistics[label]['mu'][i], self.statistics[label]['sigma'][i]))  # calculate probability
                prob[label].append(g_prob)  
            shape = self.statistics[label]['shape']  
            prob[label].append(float(shape))  

        cond_prob = {label: 1.0 for label in self.label}  
        for i in range(self.obslen + 1):  
            norm = sum(prob[label][i] for label in self.label)  # calculate normalization factor
            for label in self.label:  
                p = prob[label][i] / norm  
                cond_prob[label] *= p  

        max_label = self.label[0]  
        max_cond_prob = 0.0  
        for label in self.label:   # return the higest probability
            if max_cond_prob < cond_prob[label]:  
                max_label = label  
            max_cond_prob = cond_prob[label]  
        return max_label 

