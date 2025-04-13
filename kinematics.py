
# assumes coordinates align, will need a new function later for 

# we should use an average of these to get a better path
def trajectory_prediction(x1, y1, time1, x2, y2, time2):
    """

    @param center1: (x, y) coordinates at time1
    @param time1: time stamp of first set of coordinates

    @param center2: (x,y) coordinates at time2
    @param time2: time stamp of second set of coordinates

    return m, b: parameters of the projected line 
    """

    # Placeholder, will likely need to change based on format of time1 and time2
    delta_t = time2 - time1

    # Calculate time derivatives of x and y, assuming frictionless surface
    v_x = (x2 - x1) / delta_t
    v_y = (y2 - y1) / delta_t

    # Solve for slope and y-intercept
    m = v_y / v_x
    b = m * -1 * x2 + y2

    return m, b

