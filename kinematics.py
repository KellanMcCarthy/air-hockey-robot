
# assumes coordinates align, will need a new function later for 

# we should use an average of these to get a better path
def trajectory_prediction(center1, time1, center2, time2):
    """

    @param center1: (x, y) coordinates at time1
    @param time1: time stamp of first set of coordinates

    @param center2: (x,y) coordinates at time2
    @param time2: time stamp of second set of coordinates

    return m, b: parameters of the projected line 
    """

    # these are placeholders and will be changed once we have a way to 
    # relate camera frame and real life
    x_table_edge = 0
    y_table_edge_max = 100
    y_table_edge_min = 0

    # Placeholder, will likely need to change based on format of time1 and time2
    delta_t = time2 - time1

    # Calculate time derivatives of x and y, assuming frictionless surface
    v_x = (center2[0] - center1[0]) / 2
    v_y = (center2[1] - center1[1]) / 2

    # Solve for slope and y-intercept
    m = v_y / v_x
    b = m * -1 * center2[0] + center2[1]

    return m, b