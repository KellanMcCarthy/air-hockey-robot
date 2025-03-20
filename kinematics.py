
# assumes coordinates align, will need a new function later for 

# we should use an average of these to get a better path
def trajectory_prediction(center1, time1, center2, time2, x_end, y_end, goal_start=20, goal_end=80):
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

    x1,y1 = center1
    x2, y2 = center2


    # Placeholder, will likely need to change based on format of time1 and time2
    delta_t = time2 - time1

    # Calculate time derivatives of x and y, assuming frictionless surface
    v_x = (x2 - x1) / delta_t
    v_y = (y2 - y1) / delta_t

    # Solve for slope and y-intercept
    m = v_y / v_x
    b = m * -1 * x2 + y2

    y_predict = m*x_end +b



    while v_x > 0:
        if goal_start <= y_predict <= goal_end:
            #move robot to y_intercept
            True  
        else:
            x_collision = (y_end-b)/m
            y_new = -m * ( x_end+ (b-y_end))+ (x_collision +b)


        

    return m, b

