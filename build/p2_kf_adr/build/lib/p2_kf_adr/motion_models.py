import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # TODO: Define and return the 3x3 identity matrix A
        pass

    def control_input_matrix_B(mu, delta_t):
        # TODO: Define B using current theta and timestep delta_t
        # B should apply linear and angular velocity to position and heading
        pass

    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A():
        # TODO: Define and return the 6x6 constant velocity model transition matrix
        pass

    def B(mu, dt):
        # TODO: Return 6x2 zero matrix (no control input used in pure KF)
        pass

    return A, B
