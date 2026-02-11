import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

"""
Obliczenia zmiany położenie robota: odometria

Mamy do dyspozycji 4 czujniki pomiarowe:
-dwa enkodery dla każdego z kół, pomira obrotów na sekunde
-akcelerometr do pomiaru przyspieszenie robota
-żyroskop do pomiaru prędkości kątowej robota

Zaczniemy od przetworzenia danych z enkoderów:
(wyprowadzenie wzorów i uzupełnienie znadują się na stronie 76)

Mając pomiar prędkości obrotowej z kół w_i możemy policzyć przemieszczenie koła ze wzoru:

    d_i = 2*pi*r*w_i(t)*dt ; i:l,r

    r - promień koła[cm],
    w_i(t) - pedkosc oborotwa koła [obr/s],
    dt - próbkowkowanie czasu, co ile wierzymy prędkość [s].

Robot posiada trzy stopnie swobody polozenie wzdłóż osi x, y oraz kąt oborotu wokół własnej osi
Inczej możemy nazwać to jego posturą zapisując: (x[cm],y[cm],phi[stopnie])

Żeby policzyć zmianę postury musimy przyjąć następujace założenie:

    -przemieszczenie jest dość małe żeby móc przybliżyć wartość kąta wartościami długości
    -wynika z powyzszego ze musimy mieć częsty pomiar prędkości obrotowej

Wzory: 

    1) theta = (d_r - d_l)/b 

     theat - kąt o jaki zmienia swoją posture robot,
     b - odległość miedzy kołami

    2) d_c = (d_l +d_r)/2

     d_c - wartość przemieszczenie środka robota

    3) d_x = d_c*cos(phi+theta)

     d_x - zmiana polozenie wzloz osi x

    4) d_y = d_c*sin(phi+theta)

"""

class odometryAlgorithm:
    def __init__ (self,
                  dataFromAccelerator: np.array,
                  dataFromGyroscope: np.array,
                  dataFromEncoders: np.array,
                  startPosture,
                  timeVec):
        
        self.posX = np.array([startPosture[0]]) #cm
        self.posY = np.array([startPosture[1]])
        self.phi = np.array([startPosture[2]]) #stopnie

        self.dataFromAccelerator = dataFromAccelerator
        self.dataFromGyroscope   = dataFromGyroscope
        self.dataFromEncoders    = {
                        'rightWheel' : dataFromEncoders[0],
                         'leftWheel' : dataFromEncoders[1]
                        }

        self.timeStep = timeVec[1]-timeVec[0]
        self.distanceLeftWheel = 0
        self.distanceRightWheel = 0
        self.vecTemporaryDistanceLW = np.array([])
        self.vecTemporaryDistanceRW = np.array([])
        self.radiusOfWheel = 5
        self.distanceBetweenwheels = 20

    def calculatePath(self):
        for i in range(min(len(self.dataFromEncoders['rightWheel']),len(self.dataFromEncoders['leftWheel']))):
            dl = np.pi * self.radiusOfWheel * 2 * self.dataFromEncoders['leftWheel'][i] * self.timeStep
            dr = np.pi * self.radiusOfWheel * 2 * self.dataFromEncoders['rightWheel'][i] * self.timeStep
            self.vecTemporaryDistanceLW = np.append(self.vecTemporaryDistanceLW,dl)
            self.vecTemporaryDistanceRW = np.append(self.vecTemporaryDistanceRW,dr)
        
        for i in range(len(dataFromLeftWheel)):
            #theta = (d_r - d_l)/b
            theta = (self.vecTemporaryDistanceRW[i]-self.vecTemporaryDistanceLW[i])/self.distanceBetweenwheels
            self.phi = np.append(self.phi,self.phi[i]+theta)

            #d_c = (d_l +d_r)/2
            d_c = (self.vecTemporaryDistanceRW[i]+self.vecTemporaryDistanceLW[i])/2

            d_x = d_c * np.cos((self.phi[i]+theta)*(np.pi/180))
            d_y = d_c * np.sin((self.phi[i]+theta)*(np.pi/180))
            self.posX = np.append(self.posX,self.posX[i]+d_x)
            self.posY = np.append(self.posY,self.posY[i]+d_y)

            #print(f"postura: (x:{self.posX[i]}, y:{posY[i]}, phi:{phi[i]})",end='\n')

    def visPath(self):
        fig,ax = plt.subplots(figsize=(8,8))

        margin = 20
        ax.set_xlim(min(self.posX)-margin, max(self.posX)+margin)
        ax.set_ylim(min(self.posY)-margin, max(self.posY)+margin)

        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title("Wizulaziacja ruchu robota")
        ax.set_xlabel("X [cm]")
        ax.set_ylabel("Y [cm]")

        robotPath, = ax.plot([],[],'b-',linewidth = 1, label='Ścierzka')
        pathPoints, = ax.plot([],[],'ro',markersize = 8, label='Ścierzka')
        robotNose, = ax.plot([],[],'r-',linewidth = 2)

        plt.legend()

        noseLenght = 5 #cm

        def update(frame):

            robotPath.set_data(self.posX[:frame], self.posY[:frame])
            pathPoints.set_data([self.posX[frame]],[self.posY[frame]])

            noseCorner = np.radians(self.phi[frame])

            noseX = self.posX[frame] + noseLenght * np.cos(noseCorner)
            noseY = self.posY[frame] + noseLenght * np.sin(noseCorner)

            robotNose.set_data([self.posX[frame],noseX],[self.posY[frame],noseY])

            return robotPath, pathPoints, robotNose

        animation = FuncAnimation(fig,update,frames = len(self.posX),interval = 20, blit=True)

        plt.show()

    

# Konfiguracja czasu (tak jak miałeś ostatnio)
timeStep = 0.01
left_1 = np.ones(100) * 1.0
right_1 = np.ones(100) * 1.0
left_2 = np.ones(200) * -15
right_2 = np.ones(200) * 15
left_3 = np.ones(200) * 1.5  # Szybko
right_3 = np.ones(200) * 0.8 # Wolniej
dataFromLeftWheel = np.concatenate([left_1, left_2, left_3])
dataFromRightWheel = np.concatenate([right_1, right_2, right_3])

dataEncoders = np.array([dataFromRightWheel,dataFromLeftWheel])

simulation = odometryAlgorithm(
    dataFromAccelerator= None,
    dataFromGyroscope= None,
    dataFromEncoders= dataEncoders,
    startPosture= [100.0,0.0,90.0],
    timeVec= [0.0,0.01]
)

simulation.calculatePath()
simulation.visPath()