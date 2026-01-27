import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

data = pd.read_csv('A.csv')
data_2 = pd.read_csv('B.csv')


plt.plot(data)
plt.plot(data_2)
plt.show()