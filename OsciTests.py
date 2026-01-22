import pigpio as pg
import numpy as np


pi = pg.pi()
A = []
B = []
pi.set_mode(21,pg.INPUT)
pi.set_mode(20,pg.INPUT)
pi.set_pull_up_down(20,pg.PUD_UP)
pi.set_pull_up_down(21,pg.PUD_UP)

try:
    while True:
        a = pi.read(21)
        A.append(a)
        print('16:',a)
        b = pi.read(20)
        print(b)
        B.append(b)

except KeyboardInterrupt:
    np.array(A).tofile("A.csv",sep='\n')
    np.array(B).tofile("B.csv",sep='\n')

    pi.stop()
