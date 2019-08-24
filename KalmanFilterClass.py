import numpy as np
import matplotlib.pyplot as plt
%matplotlib inline

class KalmanFilter_1D:
    
    def __init__(self, initial_state_vector, initial_state_covariance, noise_covariance_matrix, control_variables, measurement_covariance_matrix,dt):
        self.initial_state_vector = initial_state_vector
        self.initial_state_covariance = initial_statae_covariance
        self.noise_covariance_matrix = noise_covariance_matrix
        self.control_variable_vector = control_variables
        self.measurement_error = measurement_covariance_matrix
        self.dt = dt
        
    # calculating the predicted state
    # nothing to input
    def calculate_predicted_state(state_vector, control_variable_vector, dt, noise_matrix): 
        # defining the A matrix
        A = np.matrix([[1, dt], [0, 1]])
        #defining the B matrix
        B = np.matrix([[0.5*(dt**2)],[1]])
        #calculating the predicted state
        predicted_x = A * state_vector + B * control_variable_vector[0] + noise_matrix
        return predicted_x
    
    #nothing to input
    def calculate_new_covariance_matrix(previous_covariance_matrix, dt ,noise_covariance_matrix=np.zeros((2,1))): 
        # defining the A matrix
        A = np.matrix([[1, dt], [0, 1]])
        #defining the Q matrix - for now we will set this to 0
        Q = np.zeros((2,2))
        #calculating the new P matrix
        p_new = A * previous_covariance_matrix * A.transpose() + Q
        #for now, we will just get the diagonal terms from the matrix
        p_new_diag = np.diag(np.diag(p_new))
        return p_new_diag
    
    # nothing to input
    def calculate_kalman_gain(state_covariance_matrix, measurement_noise_covariance_matrix): 
        #defining the H matrix
        H = np.identity(2)
        # calculating the kalman gain
        kalman_gain_numerator = state_covariance_matrix * H.transpose()
        kalman_gain_denominator = H * state_covariance_matrix * H.transpose() + measurement_noise_covariance_matrix
        # based on the assumption that the numerator and denominators are diagonal matrices, we can extract
        # the diagonal elements into a list and divide them and then put back into a matrix
        kalman_numerator_list = np.diag(kalman_gain_numerator)
        kalman_denominator_list = np.diag(kalman_gain_denominator)
        #getting the kalman gain matrix
        kalman_gain_matrix = np.matrix(np.diag(np.divide(kalman_numerator_list,kalman_denominator_list)))
        return kalman_gain_matrix
    
    # nothing to input
    def process_next_observation(observation_vector,adjustment_noise= np.zeros((2,1))):
        #defining the C matrix
        C = np.identity(2)

        # processing the observation
        y_new = C * observation_vector + adjustment_noise

        return y_new
    
    # nothing to input
    def calculate_updated_state(kalman_gain_matrix,predicted_state_vector,observation_vector): 
        # defining the H matrix
        H = np.identity(2)

        #calculating the new x vector
        x_updated_vector = predicted_state_vector + kalman_gain_matrix * (observation_vector - predicted_state_vector)

        return x_updated_vector
    
    # nothing to input
    def calculate_updated_process_covariance_matrix(kalman_gain_matrix,process_covariance_matrix): 
        #defining the I and H matrices
        I = np.identity(2)
        H = np.identity(2)

        # calculating the updated covariance matrix
        p_updated = (I - kalman_gain_matrix * H) * process_covariance_matrix

        return p_updated
    
    # need to input the list of observations
    def main_track(observation_matrix,dt=self.dt,inital_state_vector=self.initial_state_vector, initial_state_covariance=self.initial_state_covariance,noise_matrix=self.covariance_matrix,control_variables=self.control_variable_vector, measurement_error = self.measurement_error):
        x_initial = initial_state_vector
        p_initial = initial_state_covariance

        # set up the current variables
        x_current = x_initial
        p_current = p_initial

        #set up the empty list
        x_kalman = [x_initial[0]]
        p_kalman1 = [p_initial.item(0,0)]
        p_kalman2 = [p_initial.item(1,1)]

        n = len(observation_matrix)

        for k in range(1,n):
            #calculate the predicted state
            x_predicted = calculate_predicted_state(x_current,control_variables,dt,noise_matrix)
            # calculate the predicted covariance matrix
            p_predicted = calculate_new_covariance_matrix(p_current, dt ,noise_matrix)
            # calculate the Kalman Gain matrix
            kalman_gain_matrix = calculate_kalman_gain(p_predicted, measurement_error) 
            # process the newest observation
            observation = np.matrix([[x_obs[k]],[v_obs[k]]])
            processed_observation = process_next_observation(observation,noise_matrix)
            # calculate the updated state
            x_updated = calculate_updated_state(kalman_gain_matrix,x_predicted,processed_observation)
            # calculating the updated process covariance matrix
            p_updated = calculate_updated_process_covariance_matrix(kalman_gain_matrix,p_predicted)
            x_kalman.append(x_updated[0])
            p_kalman1.append(p_updated.item(0,0))
            p_kalman2.append(p_updated.item(1,1))
        return x_kalman
    
    # need to put in a list of observations, times, and x_kalman
    def track_plot(observation_times,position_observations,x_kalman):
        plt.plot(t_obs,x_obs,label='Observed Data')
        plt.plot(t_obs,np.reshape(x_kalman,(n,)),label='Kalman Filter')
        plt.grid(True)
        plt.title('Kalman Filter Tracking Falling Ball')
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')
        plt.legend()
        plt.show()