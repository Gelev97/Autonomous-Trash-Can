from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

predictions = np.load("predictions.npy", allow_pickle=True)
for each in predictions:
	print(each)