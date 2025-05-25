import numpy as np 

# Importación de modelos de movimiento y observación personalizados
from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

low_noise_std_KF1 = [0.03, 0.03, 0.02]
low_noise_std_KF2 = [0.03, 0.03, 0.02, 0.03, 0.03, 0.02]
high_noise_std_KF1 = [0.06, 0.06, 0.04]
high_noise_std_KF2 = [0.06, 0.06, 0.04, 0.06, 0.06, 0.04]

class KalmanFilter:
    """
    Implementación de un filtro de Kalman para estimar el estado de un sistema con 3 variables de estado: [x, y, theta].
    """

    def __init__(self, initial_state, initial_covariance, proc_noise_std=low_noise_std_KF1, obs_noise_std=low_noise_std_KF1):
        # Inicialización del estado y la covarianza inicial
        self.mu = np.array(initial_state)  # Estimación inicial del estado [x, y, theta]
        self.Sigma = np.array(initial_covariance)  # Incertidumbre inicial

        # Modelos de movimiento
        self.state_transition_matrix_A, self.control_input_matrix_B = velocity_motion_model()

        # Inicialización de las matrices de transición de estado (A) y de entrada de control (B)
        self.A = self.state_transition_matrix_A()
        self.B = self.control_input_matrix_B(self.mu, 1.0)

        # Ruido del modelo de proceso
        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Covarianza del ruido del proceso

        # Modelo de observación
        self.C = odometry_observation_model()  # Matriz de observación

        # Ruido del modelo de observación
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Covarianza del ruido de observación
            
    def predict(self, u, dt):
        """
        Paso de predicción del filtro de Kalman.
        - u: Vector de control [vx, omega].
        - dt: Intervalo de tiempo.
        """
        # Actualiza las matrices A y B según el modelo de movimiento
        self.A = self.state_transition_matrix_A()
        self.B = self.control_input_matrix_B(self.mu, dt)

        # Asegura que las dimensiones de los vectores sean correctas
        self.mu = np.array(self.mu).reshape(-1, 1)  # (3,1)
        u = np.array(u).reshape(-1, 1)              # (2,1)

        # Predicción del estado
        self.mu = self.A @ self.mu + self.B @ u
        self.Sigma = self.A @ self.Sigma @ self.A.T + self.R

        # Devuelve mu como vector plano para facilitar su uso fuera
        self.mu = self.mu.flatten()
        return self.mu, self.Sigma

    def update(self, z):
        """
        Paso de actualización del filtro de Kalman.
        - z: Vector de observación.
        """
        z = np.array(z).reshape(-1, 1)  # Asegura que z sea un vector columna
        mu = self.mu.reshape(-1, 1)  # Convierte mu a vector columna

        # Cálculo de la ganancia de Kalman
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)

        # Actualización del estado y la covarianza
        mu = mu + K @ (z - self.C @ mu)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma

        # Devuelve mu como vector plano
        self.mu = mu.flatten()
        return self.mu, self.Sigma

class KalmanFilter_2:
    """
    Implementación de un filtro de Kalman para estimar el estado de un sistema con 6 variables de estado: [x, y, theta, vx, vy, omega].
    """

    def __init__(self, initial_state, initial_covariance, proc_noise_std=low_noise_std_KF2, obs_noise_std=low_noise_std_KF2):
        # Inicialización del estado y la covarianza inicial
        self.mu = initial_state  # Estimación inicial del estado [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Incertidumbre inicial

        # Modelos de movimiento
        self.A, self.B = velocity_motion_model_2()  # Matrices del modelo de movimiento

        # Ruido del modelo de proceso
        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Covarianza del ruido del proceso

        # Modelo de observación
        self.C = odometry_observation_model_2()  # Matriz de observación

        # Ruido del modelo de observación
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Covarianza del ruido de observación

    def predict(self, u=None, dt=1.0):
        """
        Paso de predicción del filtro de Kalman.
        - u: Vector de control (opcional).
        - dt: Intervalo de tiempo.
        """
        A = self.A(dt)  # Actualiza la matriz A según el intervalo de tiempo
        self.mu = np.array(self.mu).reshape(-1, 1)  # Asegura que mu sea un vector columna (6,1)

        # Predicción del estado
        self.mu = A @ self.mu
        self.Sigma = A @ self.Sigma @ A.T + self.R

        # Devuelve mu como vector plano
        self.mu = self.mu.flatten()
        return self.mu, self.Sigma

    def update(self, z):
        """
        Paso de actualización del filtro de Kalman.
        - z: Vector de observación.
        """
        z = np.array(z).reshape(-1, 1)  # Asegura que z sea un vector columna
        mu = self.mu.reshape(-1, 1)  # Convierte mu a vector columna

        # Cálculo de la ganancia de Kalman
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)

        # Actualización del estado y la covarianza
        mu = mu + K @ (z - self.C @ mu)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma

        # Devuelve mu como vector plano
        self.mu = mu.flatten()
        return self.mu, self.Sigma