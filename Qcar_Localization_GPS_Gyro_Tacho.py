import numpy as np
from threading import Thread

import time
import signal


from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup(
        initialPosition = [0,0,0],
        initialOrientation = [0,0,0]
    )

tf = 10
controllerUpdateRate = 100

global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

class QcarEKF:

    def __init__(self,x0,P0,Q,R):
        self.L = 0.257

        self.I = np.eye(3)
        self.xHat = x0
        self.P = P0
        self.Q = Q
        self.R = R

        self.C = np.eye(3)
# motion model
    def f(self,X,u,dt):
        x,y,theta = X
        v = u[0]
        delta = u[1]
        x += dt*v*np.cos(theta)
        y += dt*v*np.sin(theta)
        theta += dt*v*np.tan(delta)/self.L
    
        return [x,y,theta]
    

    def JF(self,X,u,dt):
        x,y,theta = X
        v = u[0]
        delta = u[1]

        J = np.array([[1, 0, -dt*v*np.sin(theta)],
                      [0, 1, dt*v*np.cos(theta)],
                      [0, 0, 1]])
        return J
    
    def prediction(self, dt,u):

        F = self.JF(self.xHat,dt,u)
        self.P = F@self.P@F.T + self.Q
        self.xHat = self.f(self.xHat,u,dt)
        self.xHat[2] = wrap_to_pi(self.xHat[2])

        return [self.xHat,self.P]
    
    def correction(self,y):
        H_k = self.C
        K_k = self.P@H_k.T@ np.linalg.inv(H_k @ self.P @ H_k.T + self.R)
        z = (y - H_k@self.xHat)
        if len(y) > 1:
            z[2] = wrap_to_pi(z[2])
        else:
            z = wrap_to_pi(z)

        self.xHat = self.xHat + K_k@z

        self.xHat[2]= wrap_to_pi(self.xHat[2])
        self.P = self.P - K_k@H_k@self.P_k

        return 
    

class GyroKF:

    def __init__(self, x0, P0, Q, R):

        self.I = np.eye(2)
        self.xHat = x0
        self.P = P0
        self.Q = Q
        self.R = R

        # State Space Representation Matrices
        self.A = np.array([
            [0, -1],
            [0, 0]
        ])
        self.B = np.array([
            [1],
            [0]
        ])
        self.C = np.array([
            [1, 0]
        ])

        def prediction (self, dt,u):
            Ad = self.I + self.A*dt
            self.xHat = Ad @ self.xHat + dt*self.B* u
            self.P = Ad@self.P@Ad.T + self.Q
            return

        def correction(self, y):
            H_k = self.C
            K_k = self.P@H_k.T@ np.linalg.inv(H_k @ self.P @ H_k.T + self.R)
            z = (y - H_k@self.xHat)
            z = wrap_to_pi(z)

            self.xHat = self.xHat + K_k@z

            self.xHat[0]= wrap_to_pi(self.XHat[0])
            self.P = self.P - K_k@H_k@self.P_k
            return

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    #endregion  
    # 
    x0 = np.zeros((3,1))
    P0 = np.eye(3)

    ekf_dr = QcarEKF(
        x0=x0,
        P0=P0,
        Q=np.diagflat([0.01, 0.01, 0.01]),
        R=None
    )

    ekf_gps = QcarEKF(
        x0=x0,
        P0=P0,
        Q=np.diagflat([0.01, 0.01, 0.01]),
        R=np.diagflat([0.2, 0.2, 0.1])
    )

    kf = GyroKF(
        x0=x0,
        P0=P0,
        Q=np.diagflat([0.00001, 0.00001]),
        R= np.diagflat([0.1])
    )

    C_combined = np.eye(3)
    C_headingOnly = np.array([[0, 0, 1]])
    R_combined = np.diagflat([0.8, 0.8, 0.01])
    R_headingOnly = np.diagflat([0.01])


    ekf_sf = QcarEKF(
        x0=x0,
        P0=P0,
        Q=np.diagflat([0.0001, 0.0001, 0.0001]),
        R=R_combined
    ) 
    ### main

    qcar = QCar(readMode = 1, frequency = controllerUpdateRate)
    gps = QCarGPS(initialPose= x0)
    with qcar, gps:
        t0 = time.time()
        t = 0

        while(t<tf) and (not KILL_THREAD):
            tp = t
            t = time.time() - t0 
            dt = t-tp

            qcar.read()
            speed_tach  = qcar.motorTach()
            th_gyro = qcar.gyroscope[2]

            u = 0.1
            delta = np.pi/12
            qcar.write(u,delta)

            # ======= Section For enabling Kalman Filter Estimators =======
            ekf_dr.prediction(dt, [speed_tach, delta])
            ekf_gps.prediction(dt, [speed_tach, delta])
            # ekf_sf.prediction(dt, [speed_tach, delta])
            # kf.prediction(dt, th_gyro)

            if gps.readGPS():
                x_gps = gps.position[0]
                y_gps = gps.position[1]
                th_gps = gps.orientation[2]


                y = np.array([[x_gps],[y_gps],[th_gps]])
                ekf_gps.correction(y)

                ekf_sf.C = C_combined
                ekf_sf.R = R_combined
                kf.Correction(th_gps)


                y = np.array([[x_gps],[y_gps],[kf.xHat[0,0]]])


                ekf_sf.correction(y)

            else:
                y = np.array([[kf.xHat[0,0]]]) 
                ekf_sf.C = C_headingOnly
                ekf_sf.R = R_headingOnly
                ekf_sf.correction(y)


            count += 1
            if count >= countMax:
                # x position estimates
                scope.axes[0].sample(t, [
                    ekf_dr.xHat[0,0],
                    ekf_gps.xHat[0,0],
                    ekf_sf.xHat[0,0]
                ])
                # y position estimates
                scope.axes[1].sample(t, [
                    ekf_dr.xHat[1,0],
                    ekf_gps.xHat[1,0],
                    ekf_sf.xHat[1,0]
                ])
                # Heading angle estimate
                scope.axes[2].sample(t, [
                    ekf_dr.xHat[2,0],
                    ekf_gps.xHat[2,0],
                    ekf_sf.xHat[2,0]
                ])
                # x-y view of estimated trajectory
                scope.axes[3].sample(t, [
                    [ekf_dr.xHat[0,0], ekf_dr.xHat[1,0]],
                    [ekf_gps.xHat[0,0], ekf_gps.xHat[1,0]],
                    [ekf_sf.xHat[0,0], ekf_sf.xHat[1,0]],
                ])

                biasScope.axes[0].sample(t, [kf.xHat[0]])
                biasScope.axes[1].sample(t, [kf.xHat[1]])

                count = 0
            #endregion
            continue

    #endregion
    return



#region : Setup and run experiment
if __name__ == '__main__':

    #region : Setup Scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
    # Scope for displaying estimated gyroscope bias
    biasScope = MultiScope(
        rows=2,
        cols=1,
        title='Heading Kalman Filter',
        fps=fps
    )
    biasScope.addAxis(
        row=0,
        col=0,
        xLabel='Time [s]',
        yLabel='Heading Angle [rad]',
        timeWindow=tf
    )
    biasScope.axes[0].attachSignal()
    biasScope.addAxis(
        row=1,
        col=0,
        xLabel='Time [s]',
        yLabel='Gyroscope Bias [rad/s]',
        timeWindow=tf
    )
    biasScope.axes[1].attachSignal()

    # Scope for comparing performance of various estimator types
    scope = MultiScope(rows=3, cols=2, title='QCar State Estimation', fps=fps)

    scope.addAxis(row=0, col=0, timeWindow=tf, yLabel='x Position [m]')
    scope.axes[0].attachSignal(name='x_dr')
    scope.axes[0].attachSignal(name='x_ekf_gps')
    scope.axes[0].attachSignal(name='x_ekf_sf')

    scope.addAxis(row=1, col=0, timeWindow=tf, yLabel='y Position [m]')
    scope.axes[1].attachSignal(name='y_dr')
    scope.axes[1].attachSignal(name='y_ekf_gps')
    scope.axes[1].attachSignal(name='y_ekf_sf')

    scope.addAxis(row=2, col=0, timeWindow=tf, yLabel='Heading Angle [rad]')
    scope.axes[2].xLabel = 'Time [s]'
    scope.axes[2].attachSignal(name='th_dr')
    scope.axes[2].attachSignal(name='th_ekf_gps')
    scope.axes[2].attachSignal(name='th_ekf_sf')

    scope.addXYAxis(
        row=0,
        col=1,
        rowSpan=3,
        xLabel='x Position [m]',
        yLabel='y Position [m]',
        xLim=(-1.5, 1.5),
        yLim=(-0.5, 2.5)
    )
    scope.axes[3].attachSignal(name='ekf_dr')
    scope.axes[3].attachSignal(name='ekf_gps')
    scope.axes[3].attachSignal(name='ekf_sf')
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True
    #endregion
    if not IS_PHYSICAL_QCAR:
            qlabs_setup.terminate()

    input('Experiment complete. Press any key to exit...')
#endregion

    
    












