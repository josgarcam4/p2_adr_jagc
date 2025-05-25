import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # Matriz de transición de estado (3x3 identidad)
        return np.eye(3)

    def control_input_matrix_B(mu, delta_t):
        # Matriz de entrada de control (3x2)
        # Aplica velocidades lineales y angulares al estado
        theta = mu[2]  # Ángulo actual (theta)
        return np.array([
            [np.cos(theta) * delta_t, 0],  # Cambio en x
            [np.sin(theta) * delta_t, 0],  # Cambio en y
            [0, delta_t]                  # Cambio en theta
        ])

    return state_transition_matrix_A, control_input_matrix_B

def velocity_motion_model_2():
    def A(dt):
        # Matriz de transición de estado (6x6)
        # Relaciona posiciones con velocidades y velocidades con aceleraciones
        return np.array([
            [1, 0, 0, dt,  0,  0],  # x depende de vx
            [0, 1, 0,  0, dt,  0],  # y depende de vy
            [0, 0, 1,  0,  0, dt],  # theta depende de omega
            [0, 0, 0,  1,  0,  0],  # vx no depende de nada más
            [0, 0, 0,  0,  1,  0],  # vy no depende de nada más
            [0, 0, 0,  0,  0,  1]   # omega no depende de nada más
        ])

    def B(mu, dt):
        # Matriz de entrada de control (6x2)
        # Relaciona aceleraciones con velocidades
        return np.zeros((6,2))

    return A, B
