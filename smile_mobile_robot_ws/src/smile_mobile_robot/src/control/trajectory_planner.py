#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 06/30/2020
Description: Given some reference points (x, y) of the center line to follow,
             generate a trajectory that best follows the ference line.
'''
import numpy as np
from scipy.interpolate import splrep, splev, splder, CubicSpline
from scipy import optimize
import math

class Trajectory:
    '''
    Plan the trajectory given the reference line.
    '''
    def __init__(self, x_ref, y_ref, x_0, y_0, theta_0, v_0, v_1, a_0, a_1, v_limit=1e3, max_accel_lat=1e3, ds=0.01):
        '''
        Initialize the trajectory

        Parameters:

        Returns:

        '''
        #Generate the path to follow
        self.x_path, self.y_path, self.theta, self.s_path, self.center_line_ref, s_start = self._generate_path(x_ref, y_ref, x_0, y_0, theta_0, ds)


        #Parameterize the new path with arc_length in order to get the curvature
        #Get the new path (x_path(s), y_path(s)), this lets us calculate curvature
        path = Arc_Spline_2D(self.x_path, self.y_path, s_start=s_start)

        #Calculate the curvature of the path
        curvatures = np.abs(path.get_curvature(self.s_path))
        max_curvature = np.amax(curvatures)

        Q_init = (self.s_path[0], v_0, a_0)
        Q_goal = (self.s_path[-1], v_1, a_1)

        #Generate the speed profile
        v_s = self._speed_generation(Q_init, Q_goal, v_limit, max_curvature, max_accel_lat)
        self.velocity = v_s(self.s_path)

        #Generate the time profile
        self.time = self.velocity / ds
        self.time = np.cumsum(1.0 / self.time)

    def _generate_path(self, x_ref, y_ref, x_0, y_0, theta_0, ds):
        '''
        Generate the path for the vehicle to follow that is closest to the reference line.

        Parameters:

        Returns:

        '''
        #Generate a cubic spline of the center line reference for the vehicle
        #to follow.
        center_line_ref = Arc_Spline_2D(x_ref, y_ref)

        #arc length points spanning the line of center_line_reference
        s_ref = np.arange(0, center_line_ref.s[-1], ds)

        #Opimization to find closest point on x(s), y(s) to (x_0, y_0)
        s_start, dist_start = get_closest_point(x_0, y_0, center_line_ref)

        #Gradient unit vector of the start point on the reference line T(t) = [dx, dy] / ||[dx, dy]||
        gradient_ref = np.array([center_line_ref.dx_s(s_start), center_line_ref.dy_s(s_start)])
        gradient_ref = gradient_ref / np.linalg.norm(gradient_ref)

        #Get the unit vector of the vehicles direction
        vehicle_dir = np.array([np.cos(theta_0), np.sin(theta_0)])
        vehicle_dir = vehicle_dir / np.linalg.norm(vehicle_dir)

        #Get the unit vector pointing from the reference line to the vehicle
        vehicle_ref = np.array([x_0 - center_line_ref.x_s(s_start), y_0 - center_line_ref.y_s(s_start)])
        vehicle_ref = vehicle_ref / np.linalg.norm(vehicle_ref)

        #Determine the start angle of the vehicle with reference to the start line.
        dtheta_start = np.arccos(np.dot(gradient_ref, vehicle_dir))
        #Use cross product to help determine the sign of the angle, since dot product always gives positive.
        cross = np.cross(gradient_ref, vehicle_dir)
        if(cross < 0):
            dtheta_start *= -1

        #Use another cross product to determine line is the the left of the vehicle or right of the vehicle
        cross_2 = np.cross(gradient_ref, vehicle_ref)
        if(cross_2 < 0):
            dist_start *= -1

        #Determine the boundry condition slope for the start of the spline d(s) spline.
        deriv_d_s_start = np.tan(dtheta_start)

        #Path is in frenet coordinates which corresponds well for lane following
        #TODO: Figure out the end boundry condition. ALSO figure out end offset
        d_s = CubicSpline([s_start, center_line_ref.s[-1]], [dist_start, 0], bc_type=((1, deriv_d_s_start), (1, 0)))

        s_path = np.arange(s_start, center_line_ref.s[-1], ds)

        #Get the x and y coordinates of the trajectory
        x_path, y_path = self.frenet_to_cartesian(s_path, d_s, center_line_ref)

        #Get the new path (x_path(s), y_path(s)), this lets us calculate curvature
        #path = Arc_Spline_2D(x_path, y_path, s_start=s_start)

        #Get the angles along the trajectory.
        dy = np.diff(y_path)
        dy = np.append([dy[0]], dy)
        dx = np.diff(x_path)
        dx = np.append([dx[0]], dx)
        theta = np.arctan2(dy, dx)


        return x_path, y_path, theta, s_path, center_line_ref, s_start
    def _speed_generation(self, Q_init, Q_goal, max_vel, max_curvature, max_accel_lat):
        '''
        Generate the speed (velocity) profile of the vehicle along the given path.
        Limit the speed and accleration based on the limits of the vehicle.
        Parmeters:
            Q_init: A tuple of the initial speed state (s_0, v_0, a_0)
            Q_goal: A tuple of the goal speed state (s_1, v_1, a_1)
            max_vel: The maximum velocity set by of the vehicle
            max_curvature: The maximum curvature (kappa) on the current path.
        Returns:
            v(s): The function handle for the cubic polynomial of the velocity as a function of arclength
        '''
        s_0, v_0, a_0 = Q_init
        s_1, v_1, a_1 = Q_goal

        #Determine the speed limit based on the maximum lateral acceleration and the maximum curvature of the the path.
        v_limit = np.sqrt(max_accel_lat / max_curvature)

        #Bound the limit of velocity to v_limit.
        if(abs(v_0) > v_limit):
            print("[WARNING]: Limiting speed because desired start speed was too high")
            if(v_0 < 0):
                v_0 = -1 * v_limit
            else:
                v_0 = v_limit
        if(abs(v_1) > v_limit):
            print("[WARNING]: Limiting speed because desired end speed was too high")
            if(v_1 < 0):
                v_1 = -1 * v_limit
            else:
                v_1 = v_limit

        #Check what the theoretical maximum accleration is
        #TODO: USE THIS LATER
        a_max = abs((v_limit**2 - v_0**2)/ (2*(s_1 - s_0)))

        if (abs(a_0) > a_max):
            print("[ERROR]: Starting Acceleration a_0=%0.2f is greater than maximum calculated acceleration a_max=%0.2f" % (a_0, a_max))
            return None

        if(abs(a_1) > a_max):
            print("[ERROR]: Goal Acceleration a_1=%0.2f is greater than maximum calculated acceleration a_max=%0.2f" % (a_1, a_max))
            return None

        #Calculate the coefficients of the cubic polynomial
        p_0 = v_0
        p_1 = a_0

        s_1_squared = s_1**2
        s_1_cubed = s_1_squared * s_1
        p_2 = (-2*a_0 / s_1) - (a_1 / s_1) - (3*v_0 / s_1_squared) + (3*v_1 / s_1_squared)
        p_3 = (a_0 / s_1_squared) + (a_1 / s_1_squared) + (2*v_0 / s_1_cubed) - (2*v_1 / s_1_cubed)

        v_s = lambda s : (p_3 * s**3) + (p_2 * s**2) + (p_1 * s) + p_0

        return v_s

    def frenet_to_cartesian(self, s, d_s, arc_spline):
        '''
        Convert the frenet function with the given reference line to cartestian coordinates.
        Parameters:
            s: A scalar or array like object. This is the arclength position.
            d_s: The function handle for the offset distance cubic spline (frenet function)
            arc_spline. The Arc_Spline_2D object of the reference line.
        Returns:
            x: The x coordinate outputs.
            y: The y coordinate outputs.
        '''
        normal_reference = np.hstack((-1*arc_spline.dy_s(s).reshape(s.size, 1), arc_spline.dx_s(s).reshape(s.size, 1)))
        norm = np.linalg.norm(normal_reference, axis=1)
        normal_reference[:, 0] = normal_reference[:, 0] / norm
        normal_reference[:, 1] = normal_reference[:, 1] / norm

        x = arc_spline.x_s(s) + d_s(s) * normal_reference[:, 0]
        y = arc_spline.y_s(s) + d_s(s) * normal_reference[:, 1]

        return x, y

class Arc_Spline_2D:
    '''
    Given the cartesian path points. Generate a cubic spline as a function of the arclength
    '''
    def __init__(self, x, y, s_start=0.0):
        '''
        Parameters:
            x: A 1D numpy array containing the cartestian x-coordiantes of the path
            y: cartestian y-coordinates of the path
            s_start: Starting offset for arc_length (shifts horizontally)
        Returns:
            N/A
        '''
        #Get the arclength from the first point to the reset of the points
        self.s, self.dl = self._get_arclength(x, y)
        self.s = self.s + s_start #Account for horizontal offset

        #Calculate the spline path for x and y
        #x(s)

        #CubicSpline([s_k, arc_spline.s[-1]], [dist, 0], bc_type=((1, deriv_d_s_start), (1, 0)))
        self.x_s = CubicSpline(self.s, x)
        self.y_s = CubicSpline(self.s, y)

        self.dx_s = self.x_s.derivative()
        self.ddx_s = self.dx_s.derivative()
        self.dy_s = self.y_s.derivative()
        self.ddy_s = self.dy_s.derivative()

    def _get_arclength(self, x, y):
        '''
        Calculate the arclength from the first point to the consecutive points. The x and y distances are expected to be
        relatively close.
        Parameteres:
            x: x-coordinates
            y: y-coordinates
        Returns:
            s: A 1-D array of length len(x) containing arclengths.
            dl: The differential length ifference between each point.
        '''
        #The arclength formula is given by

        dx = np.diff(x)
        dy = np.diff(y)
        dl = np.sqrt(dx**2 + dy**2)

        s = np.zeros(1)
        s = np.append(s, np.cumsum(dl))
        return s, dl

    def get_position(self, s):
        '''
        At a given arclength point, calculate the (x, y) position
        Parameters:
            s: The arclength point
        Returns:
            (x, y): The cartesian (x, y) point
        '''

        x = self.x_s(s)
        y = self.y_s(s)

        return x, y

    def get_heading_angle(self, s):
        '''
        At a given arclength point, calculate the heading angle theta.
        Parameters:
            s: The arclength point
        Returns:
            theta: The angle (between -pi and pi).
        '''
        dy = self.dy_s(s)
        dx = self.dx_s(s)

        theta = np.arctan2(dy, dx)
        return(theta)

    def get_curvature(self, s):
        '''
        At a given arclength point, calculate the curvature k.
        Parameters:
            s: The arclength point
        Returns:
            kappa: The curvature of the line
        '''
        dy = self.dy_s(s)
        ddy = self.ddy_s(s)
        dx = self.dx_s(s)
        ddx = self.ddx_s(s)

        kappa = (ddy*dx - dy*ddx) / (dx**2 + dy**2)
        return(kappa)

def dist_poly_interp(s, s_1, s_2, s_3, D):
    '''
    The polynomial interoplation of the distance formula to be used for quadratic minimization.
    Parameters:
        s: The point to estimate along the the polynomial
        s_1: First initial guess
        s_2: Second initial guess
        s_3: Third initial guess
        D: The square distance function handle D(s)
    '''
    Q_1 = ((s - s_2)*(s - s_3)) / ((s_1 - s_2)*(s_1 - s_3))
    Q_2 = ((s - s_1)*(s - s_3)) / ((s_2 - s_1)*(s_2 - s_3))
    Q_3 = ((s - s_1)*(s - s_2)) / ((s_3 - s_1)*(s_3 - s_2))

    P = (Q_1*D(s_1)) + (Q_2*D(s_2)) + (Q_3*D(s_3))

    return(P)

def quadratic_minimization(s_1, s_2, s_3, D, iter_completed=0, max_iter=100, tol=0.001):
    '''
    '''
    y_23 = s_2**2 - s_3**2
    y_31 = s_3**2 - s_1**2
    y_12 = s_1**2 - s_2**2

    s_23 = s_2 - s_3
    s_31 = s_3 - s_1
    s_12 = s_1 - s_2

    #Approximate minimum of P(s) ==> approximate minimum of D(s)
    s_k = (1.0/2.0) * (y_23*D(s_1) + y_31*D(s_2) + y_12*D(s_3)) / (s_23*D(s_1) + s_31*D(s_2) + s_12*D(s_3))

    #Compute the P(s_1), P(s_2), P(s_3), P(s_k), and eliminate the one that gives the highest value.
    P_s_1 = dist_poly_interp(s_1, s_1, s_2, s_3, D)
    P_s_2 = dist_poly_interp(s_2, s_1, s_2, s_3, D)
    P_s_3 = dist_poly_interp(s_3, s_1, s_2, s_3, D)
    P_s_k = dist_poly_interp(s_k, s_1, s_2, s_3, D)

    #Determine which one gives the biggest value, eliminate that s value
    P_vals = np.array([P_s_1, P_s_2, P_s_3, P_s_k])
    max_index = np.argmax(P_vals)

    if(iter_completed >= max_iter):
        print("Maximum Iteration Occured!")
        return(s_k, iter_completed, False)

    iter_completed += 1

    if(abs(np.ptp(P_vals)) < tol):
        return(s_k, iter_completed, True)

    elif(max_index == 0):
        s_1 = s_2
        s_2 = s_3
        s_3 = s_k

    elif(max_index == 1):
        s_1 = s_1
        s_2 = s_3
        s_3 = s_k

    elif(max_index == 2):
        s_1 = s_1
        s_2 = s_2
        s_3 = s_k

    elif(max_index == 3):
        s_1 = s_1
        s_2 = s_2
        s_3 = s_3


    return(quadratic_minimization(s_1, s_2, s_3, D, iter_completed=iter_completed, max_iter=max_iter, tol=tol))



def get_dist_func(x_0, y_0, x_s_func, y_s_func):
    '''
    Return the function handle D(s) for the distance function from the cubic spline defined as (x(s), y(s)).
    Parameters:
        x_0: x-coordinate to compute distance from
        y_0: y-coordinate to compute distance from
        x_s_func: The function handle for the x(s) cubic spline
        y_s_func: The function handle for the y(s) cubic spline
    Returns:
        D(s): The function handle for the square distance equation.
    '''
    D = lambda s : (x_s_func(s) - x_0)**2 + (y_s_func(s) - y_0)**2
    return D

def get_first_deriv_dist_func(x_0, y_0, x_s_func, y_s_func, dx_s_func, dy_s_func):
    '''
    Get the first derivative of the square distance function. Returns a function handle.
     Parameters:
        x_0: x-coordinate to compute distance from
        y_0: y-coordinate to compute distance from
        x_s_func: The function handle for the x(s) cubic spline
        y_s_func: The function handle for the y(s) cubic spline
        dx_s_func: The function handle for the dx(s) quadratic spline
        dy_s_func: The function handle for the dy(s) quadratic spline
    Returns:
       dD(s): The function handle for the square distance equation.
    '''

    dD = lambda s : (2 * (x_s_func(s) - x_0) * dx_s_func(s)) + (2 * (y_s_func(s) - y_0) * dy_s_func(s))
    return dD

def get_second_deriv_dist_func(x_0, y_0, x_s_func, y_s_func, dx_s_func, dy_s_func, ddx_s_func, ddy_s_func):
    '''
    Get the second derivative of the square distance function. Returns a function handle.
     Parameters:
        x_0: x-coordinate to compute distance from
        y_0: y-coordinate to compute distance from
        x_s_func: The function handle for the x(s) cubic spline
        y_s_func: The function handle for the y(s) cubic spline
        dx_s_func: The function handle for the dx(s) quadratic spline
        dy_s_func: The function handle for the dy(s) quadratic spline
    Returns:
       dD(s): The function handle for the square distance equation.
    '''

    ddD = lambda s : (2 * (dx_s_func(s))**2) + (2 * (dy_s_func(s))**2) + \
                     (2 * (x_s_func(s) - x_0) * ddx_s_func(s)) + (2 * (y_s_func(s) - y_0) * ddy_s_func(s))
    return ddD



def newtons_method(dD, ddD, s_guess, tol=0.001, max_iter=100):
    '''
    Perform newtons method for optimization.
    Parameters:
        dD: The function handle for the first derivative of the square distance equation
        ddD: The function handle for the second derivative of the square distance equation
        s_guess: The initial guess of the zero of dD.
    Returns
        s_star: The approximate zero found.
        iteration: The number of iterations it took
        success: True if convergence was reached.
    '''

    for iteration in range(max_iter):
        dD_calc = dD(s_guess)
        if(abs(dD_calc) < tol):
            return(s_guess, iteration+1, True)

        ddD_calc = ddD(s_guess)
        s_guess = s_guess - dD_calc / ddD_calc

    print('EXCEEDED MAXIMUM ITERATIONS!')
    return s_guess, iteration+1, False

def get_closest_point(x_0, y_0, arc_spline, tol=0.001):
    '''
    Calculate the closest point from (x_0, y_0) to the reference spline (x(s), y(s)).
    This algorithm used to find the closest point is a two step optimization procedure
    of quadratic optimization and newtons method.
    Parameters:
        x_0: The absolute x position
        y_0: The absolute y position.
        arc_spline: A Cubic spline defined by the class Arc_Spline_2D
    Returns:
        s: The arclength position of where the closest point is
        dist: The distance between the point (x_0, y_0) and the closest point on the cubic spline
    '''
    #The square distance formulas D(s), 1st deriv of D(s), & 2nd deriv of D(s). Each of these return a function handle
    D = get_dist_func(x_0, y_0, arc_spline.x_s, arc_spline.y_s)

    #Make initial guesses of the best segment of which s_k is on.
    num_s_points = len(arc_spline.s)
    s_1 = arc_spline.s[0]; s_2 = arc_spline.s[num_s_points//2]; s_3 = arc_spline.s[-1]

    #Perfrom quadratic minimization (maximum of 4 iterations).
    #The result is considered the initial guess for newtons method. However, it is possible that
    #this minimization is sufficient.
    s_k, iter_completed_1, converged = quadratic_minimization(s_1, s_2, s_3, D, max_iter=4, tol=tol)

    #Quadratic minimization was good enough!
    if(converged):
        return s_k, np.sqrt(D(s_k))

    #Since quadratic minimization was NOT good enough, perform netwons method.
    #Get the first and second derivative of the square distance equation.
    dD = get_first_deriv_dist_func(x_0, y_0, arc_spline.x_s, arc_spline.y_s, arc_spline.dx_s, arc_spline.dy_s)
    ddD = get_second_deriv_dist_func(x_0, y_0, arc_spline.x_s, arc_spline.y_s, arc_spline.dx_s, arc_spline.dy_s,
                                arc_spline.ddx_s, arc_spline.ddy_s)

    s_k, iter_completed_2, success = newtons_method(dD, ddD, s_k, max_iter=5, tol=tol)
    print("Quadratic Min. Iterations:", iter_completed_1, "Newtons Method Iterations:", iter_completed_2)

    if not success:
        print("OPTIMIZATION TO FIND CLOSEST POINT FAILED!")


    return s_k, np.sqrt(D(s_k))
