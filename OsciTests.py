import pigpio as pg
import numpy as np


pi = pg.pi()
A = []
B = []
pi.set_mode(16,pg.INPUT)
pi.set_mode(12,pg.INPUT)

try:
    while True:
        a = pi.read(16)
        A.append(a)
        print('16:',a)
        b = pi.read(12)
        print(b)
        B.append(b)

except KeyboardInterrupt:
    np.array(A).tofile("A.csv",sep='\n')
    np.array(B).tofile("B.csv",sep='\n')

    pi.stop()
