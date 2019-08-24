# Tracking_Algorithms
My exploration of simple tracking algorithms

I have written a simple 2D Kalman Filter tracking algorithm

# How to use 2D Kalman Filter
1. Input data into observation_matrix in the penultimate cell at the bottom in the form:
observation_matrix = np.matrix([[x0, y0, x_dot_0, y_dot_0],
                                [x1, y1, x_dot_1, y_dot_1],
                                [x2, y2, x_dot_2, y_dot_2],.....]), where the 0th row is the initial/starting points
2. Initialise the diagonal entries in the covariance matrix in the final cell 
3. Change 'n' in final line to the number of rows that you have in your observation_matrix
4. Run

