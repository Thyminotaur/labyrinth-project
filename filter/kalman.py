import numpy as np

IDX_PX = 0
IDX_PY = 1
IDX_THETA = 2
IDX_V = 3
IDX_W = 4

IDX_CAM = range(0,3)
IDX_SPEED = range(3,5)

IDX_PX_MEASURE = 0
IDX_PY_MEASURE = 1
IDX_THETA_MEASURE = 2

IDX_V_MEASURE = 0
IDX_THETA_MEASURE = 1

IDX_SPEED_R = 0
IDX_SPEED_L = 1

NB_STATES = 5
NB_CAM_STATES = 3
NB_SPEED_STATES = 2

PXL_TO_MM = 1.268346
MM_TO_PXL = 1/PXL_TO_MM
SPEED_FACTOR_TO_THYMIO = 3.75 #2.775 # [pxl/s] to thymio's unit
SPEED_FACTOR_TO_WORLD = 0.28411 # thymio's unit to [mm/s]

THYMIO_DIA = 95. # [mm]
#THYMIO_DIA = 120

ALMOST_ZERO = 1e-6

class kalmanEKF():
    def __init__(self):
        # initialization matrix dimensions
        self.X = np.zeros((NB_STATES,1))
        self.F = np.zeros((NB_STATES,NB_STATES))
        self.Q = np.zeros((NB_STATES,NB_STATES))
        self.P = np.zeros((NB_STATES,NB_STATES))

        self.Zs = np.zeros(NB_SPEED_STATES,1)
        self.Hs = np.zeros((NB_SPEED_STATES,NB_STATES))
        self.Rs = np.zeros((NB_SPEED_STATES,NB_SPEED_STATES))

        self.Zc = np.zeros((NB_CAM_STATES,1))
        self.Hc = np.zeros((NB_CAM_STATES, NB_STATES))
        self.Rc = np.zeros((NB_CAM_STATES, NB_CAM_STATES))
        self.camAvailable = False


        # default values
        self.predict_X() # zeros for all except w which is almost zeros
        self.set_Q([9., 9., 0.0076, 4., 0.0685]) # ecart-type eq: [3mm, 3mm, 5deg=0.0873rad, 2mm/s, 15deg/s=0.2618rad/s] #original: [1., 1., 1., 5., 5.] (px^2,px^2,rad^2,px^2/s^2, rad^2/s^2)
        #self.set_Q([5., 5., 0.087, 6.15, 0.1]) # some values to test
        self.set_P(1000)

        self.Hs[:,IDX_SPEED] = np.eye(NB_SPEED_STATES) #[speed_state_to_vr_measure, speed_state_to_vl_measure]
        self.set_Rs([1., 0.0076]) # ecart-type eq: [1 mm/s, 5deg/s=0.0873rad/s] #original: [9.32, 3.89] (px^2/s^2) (update good [6.32, 3.89])
        #self.set_Rs([6.15, 0.01]) # some values to test
        # speed_state_to_vr_measure = np.array([1, (THYMIO_DIA/2)*MM_TO_PXL]) * SPEED_FACTOR
        # speed_state_to_vl_measure = np.array([1, -(THYMIO_DIA/2)*MM_TO_PXL]) * SPEED_FACTOR

        self.Hc[:, IDX_CAM] = np.eye(NB_CAM_STATES)
        self.set_Rc([1., 0.25, 0.031]) #ecart-type eq: [1mm, 0.5mm, 1deg=0.175rad] original: [0.406, 0.030, 0.117] (px^2,px^2,deg^2)
        #self.set_Rc([4., 4., 0.05]) # some values to test
        self.dt = 1/10

    def filter(self, dt, speed_measure, camera_measure = None):

        self.Zs = np.asarray(speed_measure).reshape((NB_SPEED_STATES,1))
        if camera_measure is not None:
            self.camAvailable = True
            self.Zc = np.asarray(camera_measure).reshape((NB_CAM_STATES,1))
            # if Zc[IDX_THETA] < 0: Zc[IDX_THETA] += 2*np.pi
        else:
            self.camAvailable = False
        self.dt = dt

        self.predict()
        self.update(Zs, Zc)
        
        # if Zc is not None:
        #     if Zc[IDX_THETA] > np.pi: Zc[IDX_THETA] -= 2*np.pi
        return self.X, self.P

    def predict(self):
        self.evaluate_F()
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.predict_X()

    def update(self, Zs, Zc):
        # update with speed measurement
        Y = Zs - self.Hs @ self.X
        S = self.Hs @ self.P @ self.Hs.T + self.Rs
        K = self.P @ self.Hs.T @ np.linalg.inv(S)
        self.X = self.X + K @ Y
        self.P = (np.eye(NB_STATES)- K @ self.Hs) @ self.P

        # update with camera measurement
        if self.camAvailable:
            Y = Zc - self.Hc @ self.X
            S = self.Hc @ self.P @ self.Hc.T + self.Rc
            K = self.P @ self.Hc.T @ np.linalg.inv(S)
            self.P = (np.eye(NB_STATES) - K @ self.Hc) @ self.P
            self.X = self.X + K @ Y


    def predict_X(self):
        # Prediction of the states, model approximation: v and w are constant
        x = self.X[IDX_PX]
        y = self.X[IDX_PY]
        theta = self.X[IDX_THETA]
        v = self.X[IDX_V]
        w = self.X[IDX_W]
        dt = self.dt

        if np.abs(w)<=ALMOST_ZERO: # Driving straight, theta is constant
            self.X[IDX_PX] = x + v * np.cos(theta) * dt
            self.X[IDX_PY] = y + v * np.sin(theta) * dt
            self.X[IDX_W] = ALMOST_ZERO #
        else: # otherwise
            self.X[IDX_PX] = x + (v/w) * (np.sin(theta + w*dt) - np.sin(theta))
            self.X[IDX_PY] = y - (v/w) * (np.cos(theta + w*dt) - np.cos(theta))
            self.X[IDX_THETA] = (theta + w*dt) % (2.0*np.pi)

    def evaluate_F(self):
        # Calculation of the Jacobian of our non linear state prediction system f
        x = self.X[IDX_PX]
        y = self.X[IDX_PY]
        theta = self.X[IDX_THETA]
        v = self.X[IDX_V]
        w = self.X[IDX_W]
        dt = self.dt

        f13 = (v/w) * (np.cos(theta + w*dt) - np.cos(theta))
        f14 = (1.0/w) * (np.sin(theta + w*dt) - np.sin(theta))
        f15 = (v*dt/w) * np.cos(theta + w*dt) - (v/w**2) * (np.sin(theta + w*dt) - np.sin(theta))
        f23 = (v/w) * (np.sin(theta + w*dt) - np.sin(theta))
        f24 = (-1.0/w) * (np.cos(theta + w*dt) - np.cos(theta))
        f25 = (v*dt/w) * np.sin(theta + w*dt) + (v/w**2) * (np.cos(theta + w*dt) - np.cos(theta))
        self.F = np.array([[1.0, 0.0, f13, f14, f15],
                           [0.0, 1.0, f23, f24, f25],
                           [0.0, 0.0, 1.0, 0.0, dt],
                           [0.0, 0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32)


    def set_Q(self, values):
        if np.isscalar(values):
            self.Q = np.eye(NB_STATES)*values
        elif len(np.asarray(values).shape) == 1 and len(values) == NB_STATES:
            np.fill_diagonal(self.Q, values)
        elif np.asarray(values).shape == (NB_STATES, NB_STATES):
            self.Q = values.copy()
        else:
            print("Dimension of Q is not correct")

    def set_P(self, values):
        if np.isscalar(values):
            self.P = np.eye(NB_STATES)*values
        elif len(np.asarray(values).shape) == 1 and len(values) == NB_STATES:
            np.fill_diagonal(self.P, values)
        elif np.asarray(values).shape == (NB_STATES, NB_STATES):
            self.P = values.copy()
        else:
            print("Dimension of P is not correct")

    def set_Rc(self, values):
        if np.isscalar(values):
            self.Rc = np.eye(NB_CAM_STATES)*values
        elif len(np.asarray(values).shape) == 1 and len(values) == NB_CAM_STATES:
            np.fill_diagonal(self.Rc, values)
        elif np.asarray(values).shape == (NB_CAM_STATES, NB_CAM_STATES):
            self.Rc = values.copy()
        else:
            print("Dimension of Rc is not correct")

    def set_Rs(self, values):
        if np.isscalar(values):
            self.Rs = np.eye(NB_SPEED_STATES)*values
        elif len(np.asarray(values).shape) == 1 and len(values) == NB_SPEED_STATES:
            np.fill_diagonal(self.Rs, values)
        elif np.asarray(values).shape == (NB_SPEED_STATES, NB_SPEED_STATES):
            self.Rs = values.copy()
        else:
            print("Dimension of Rs is not correct")
