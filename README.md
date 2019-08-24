# Tracking_Algorithms
My exploration of simple tracking algorithms

I have written a simple 2D Kalman Filter tracking algorithm

# Assumptions of Kalman Filter
1. Assumes that the state estimation is a linear function of the previous state and the control variables
2. Assumes that the error is Gaussian

# Falling Ball Kalman Filter Demonstration
Notes:
1. Filter is fairly decent, even when initialised far-off from the track
2. You can change the errors to improve the tracker

# How to use 2D Kalman Filter ipynb
1. Input data into observation_matrix in the penultimate cell at the bottom in the form:
observation_matrix = np.matrix([[x0, y0, x_dot_0, y_dot_0],
                                [x1, y1, x_dot_1, y_dot_1],
                                [x2, y2, x_dot_2, y_dot_2],.....]), where the 0th row is the initial/starting point
2. Initialise the diagonal entries in the covariance matrix in the final cell 
3. Change 'n' in final line to the number of rows that you have in your observation_matrix
4. Run

# Future updates
- I am considering adding an extra feature to plot the 3D track of the object
- I may also look into future models which are able to incorporate non-uniform acceleration as well (e.g. particle filter)
