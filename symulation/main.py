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

# Konfiguracja czasu (tak jak miałeś ostatnio)
timeStep = 0.01

# --- Etap 1: Jazda prosto (1 sekunda / 100 próbek) ---
# Obie strony jadą z prędkością 1.0 obr/s
left_1 = np.ones(100) * 1.0
right_1 = np.ones(100) * 1.0

# --- Etap 2: Obrót w miejscu w lewo (1.5 sekundy / 150 próbek) ---
# Żeby obrócić się w miejscu (nie zmieniając pozycji x,y):
# Lewe koło musi cofać (-0.5), Prawe jechać do przodu (0.5)
left_2 = np.ones(200) * -15
right_2 = np.ones(200) * 15

# --- Etap 3: Łuk w prawo (2 sekundy / 200 próbek) ---
# Żeby skręcać w prawo jadąc do przodu:
# Lewe koło musi pokonywać większy dystans (zewnętrzne) niż prawe (wewnętrzne)
left_3 = np.ones(200) * 1.5  # Szybko
right_3 = np.ones(200) * 0.8 # Wolniej

# --- Łączenie danych ---
dataFromLeftWheel = np.concatenate([left_1, left_2, left_3])
dataFromRightWheel = np.concatenate([right_1, right_2, right_3])
dataFromAccelerator = np.array([])
dataFromGyroscope = np.array([])
timeStep = 0.01 #czas co jaki pobierana jest wartosc z czujników
radiusOfWheel = 5 #cm
distanceBetweenwheels = 20 #cm

#wyznaczenie chwilowych zmian przemieszczenia
distanceLeftWheel = 0
distanceRightWheel = 0
vecTemporaryDistanceLW = np.array([])
vecTemporaryDistanceRW = np.array([])

for i in range(min(len(dataFromLeftWheel),len(dataFromRightWheel))):
    dl = np.pi * radiusOfWheel * 2 * dataFromLeftWheel[i] * timeStep
    dr = np.pi * radiusOfWheel * 2 * dataFromRightWheel[i] * timeStep
    vecTemporaryDistanceLW = np.append(vecTemporaryDistanceLW,dl)
    vecTemporaryDistanceRW = np.append(vecTemporaryDistanceRW,dr)

#obliczanie zmiany pozycji
posX = np.array([100]) #cm
posY = np.array([0])
phi = np.array([90]) #stopnie

for i in range(len(dataFromLeftWheel)):

    #theta = (d_r - d_l)/b
    theta = (vecTemporaryDistanceRW[i]-vecTemporaryDistanceLW[i])/distanceBetweenwheels
    phi = np.append(phi,phi[i]+theta)

    #d_c = (d_l +d_r)/2
    d_c = (vecTemporaryDistanceRW[i]+vecTemporaryDistanceLW[i])/2

    d_x = d_c * np.cos((phi[i]+theta)*(np.pi/180))
    d_y = d_c * np.sin((phi[i]+theta)*(np.pi/180))
    posX = np.append(posX,posX[i]+d_x)
    posY = np.append(posY,posY[i]+d_y)

    print(f"postura: (x:{posX[i]}, y:{posY[i]}, phi:{phi[i]})",end='\n')


#wizulaizacjas
fig,ax = plt.subplots(figsize=(8,8))

margin = 20
ax.set_xlim(min(posX)-margin, max(posX)+margin)
ax.set_ylim(min(posY)-margin, max(posY)+margin)

ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Wizulaziacja ruchu robota")
ax.set_xlabel("X [cm]")
ax.set_ylabel("Y [cm]")

robotPath, = ax.plot([],[],'b-',linewidth = 1, label='Ścierzka')
pathPoints, = ax.plot([],[],'ro',markersize = 8, label='Ścierzka')
robotNose, = ax.plot([],[],'r-',linewidth = 2)

plt.legend()

def update(frame):

    robotPath.set_data(posX[:frame], posY[:frame])
    pathPoints.set_data([posX[frame]],[posY[frame]])

    noseLenght = 5 #cm
    noseCorner = np.radians(phi[frame])

    noseX = posX[frame] + noseLenght * np.cos(noseCorner)
    noseY = posY[frame] + noseLenght * np.sin(noseCorner)

    robotNose.set_data([posX[frame],noseX],[posY[frame],noseY])

    return robotPath, pathPoints, robotNose

animation = FuncAnimation(fig,update,frames = len(posX),interval = 20, blit=True)

plt.show()