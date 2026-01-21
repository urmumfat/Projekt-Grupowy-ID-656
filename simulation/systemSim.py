import numpy as np
import matplotlib.pyplot as plt

#parameters of engine
To = 0.1 #seconds
Ko = 0.1 #m/sV

#zadajnik
Vmax = 1#m/s
Kz = 255

#regulator
Vss = 12#V
Vdrop = 2#V
Kp = (Vss - Vdrop)/255 

#czujnik
Kt = 0.255#s/m

#symulation
timeSpace = np.linspace(0,0.99,100)
dt = 0.01

R = 0.5#m/s
Y = [0]
#zakladamy ze U(s) jest skokiem jednostkowym

for t in timeSpace:

    yPrev = Y[-1]

    r = Kz * R

    error = r - Kt*yPrev

    u = error*Kp

    dy = (Ko*u - yPrev)*dt/To

    Y.append(yPrev+dy)

Y = Y[-len(timeSpace):]

plt.figure(figsize=(12,8))
plt.plot(timeSpace,Y)

plt.grid()
plt.show()
