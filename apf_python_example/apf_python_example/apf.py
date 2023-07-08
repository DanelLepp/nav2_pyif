#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import math

goal_pose = PoseStamped()
position_all = []

def mult(x, y):
    return np.array([
        x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3],
        x[0]*y[1] + x[1]*y[0] + x[2]*y[3] - x[3]*y[2],
        x[0]*y[2] - x[1]*y[3] + x[2]*y[0] + x[3]*y[1],
        x[0]*y[3] + x[1]*y[2] - x[2]*y[1] + x[3]*y[0]
    ])

def cross(x, y):
    return np.array( (x[1]*y[2] - x[2]*y[1], x[2]*y[0] - x[0]*y[2], x[0]*y[1] - x[1]*y[0]) )

def slow_rot(q, v):
    # print("slow_rot")
    qbar = -q
    qbar[0] = q[0]
    # print("slow_rot return")
    return np.array(mult(mult(q, v), qbar)[1:3])

def getRanges(costmap, pose, q_star):
    # global ranges

    # Convert pose to grid coordinates
    range_origin = [ 
        int((pose.pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution),
        int((pose.pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution)]


    vector1 = np.array([1, 0])

    min_x = int(max(0, range_origin[0] - q_star))
    min_y = int(max(0, range_origin[1] - q_star))
    max_x = int(min(costmap.info.width, range_origin[0] + q_star))
    max_y = int(min(costmap.info.height, range_origin[1] + q_star))
    # vector1 = np.array([1, 0])
    ranges = np.full((360), np.inf)

    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            vector2 = [x-range_origin[0], y-range_origin[1]]
            distance = np.linalg.norm(vector2)
            if(distance<q_star and distance != 0):
                cost = costmap.data[y * costmap.info.width + x]
                if (cost >= 100):
                    angle = int(np.rad2deg(np.arccos(np.dot(vector1, vector2)/distance)))

                    if (angle >= 360):
                        angle -= 360
                    
                    if (angle < 0):
                        angle += 360
                    
                    if (distance < ranges[angle]):
                        ranges[angle] = distance * costmap.info.resolution

    # print(ranges)
    # logging.debug(ranges)
    return ranges

def computeRepulsiveForce(occupancy_grid, pose):
    # global ranges
    eta = 4 # scaling factor for repulsive force
    q_star = 1.0 # threshold distance for obstacles

    vector_sum = np.array([0.0,0.0])
    vector_num = 0

    ranges = getRanges(occupancy_grid, pose, q_star)
    angle = 0
    resolution = 1
    
    # The lidar outputs 360 degree data and therefore we get data of 360 points within the range of the sensor.
    # Each obstacle detected by the sensor will occupy some points out of these 360 points. We treat each single point as 
    # an individual obstacle.
    for distance in ranges:
        if(distance<q_star and distance != 0):
            mag = np.absolute(eta*(1/q_star - 1/distance)*(1/distance**2)) 

            # This is the negative gradient direction
            vector_sum = vector_sum + (-1*np.array([np.cos(angle),np.sin(angle)]) * mag)
            vector_num = vector_num + 1

        angle = angle + resolution
            
    # Normalization of the repulsive forces
    # logging.debug([vector_sum[0],vector_sum[1]])*(1/len(ranges))
    # return np.array([vector_sum[0],vector_sum[1]])/vector_num
    if (vector_num > 0):
        # print(vector_sum)
        return vector_sum/vector_num
    else:
        # print(vector_sum)
        return vector_sum
#------------------------------------------

def computeAttractiveForce(pose, goal_pose, position_all):
    # print(position_all)
    pose = pose
    pos_x = pose.pose.position.x
    pos_y = pose.pose.position.y
    pos = []
    pos.append(pos_x)
    pos.append(pos_y)
    zeta = 5 # scaling factor for attractive force
    d_star = 1.5 # threshoild distance for goal
    closest_waypoint = []
    closest_waypoint = getClosestWaypoint(pos, position_all)

    dist_to_goal = np.sqrt((pos_x - goal_pose.pose.position.x)**2 + (pos_y - goal_pose.pose.position.y)**2)

    index_waypoint = closest_waypoint[0]

    # If the closest waypoint is the last point on the global path then direct the robot towards the goal position
    if(index_waypoint == len(position_all)-1):
        angle = np.arctan2(goal_pose.pose.position.y - pos_y, goal_pose.pose.position.x - pos_x)
        if (dist_to_goal<=d_star):
            mag = np.absolute(zeta*dist_to_goal)
        else :
            mag = d_star*zeta
        vector = 1*np.array([np.cos(angle), np.sin(angle)])
        vector = vector * mag
        return np.array([vector[0],vector[1]])

    # We use a lookahead distance concept in order to drive the robot. Feel free to customize this block of code.
    # The robot follows the waypoint which is 10 points ahead of the closest waypoint. If the robot is within 10 points
    # from the goal ,use the goal position instead to drive the robot.
    if(index_waypoint + 10 <len(position_all)-2):
        pt_next = position_all[index_waypoint + 10]
        angle = np.arctan2(pt_next[1]-pos_y,pt_next[0]-pos_x)
    else:
        angle = np.arctan2(goal_pose.pose.position.y - pos_y, goal_pose.pose.position.x - pos_x)

    if (dist_to_goal<=d_star):
        mag = np.absolute(zeta*dist_to_goal)
    else :
        mag = d_star*zeta

    vector = 1*np.array([np.cos(angle), np.sin(angle)])
    vector = vector * mag
    return np.array([vector[0],vector[1]])
#------------------------------------------

def getClosestWaypoint(point, points):
    i=0
    pt=[]
    dist = math.inf
    for p in points:
        if(math.dist(p,point)<dist):
            dist = math.dist(p,point)
            pt = p
            i = points.index(pt)
    return [i,pt]

#------------------------------------------

def handleGlobalPlan(global_path):
    position_x = []
    position_y = []
    i=0
    while(i <= len(global_path.poses)-1):
        position_x.append(global_path.poses[i].pose.position.x)
        position_y.append(global_path.poses[i].pose.position.y)
        i=i+1
    position_all = [list(double) for double in zip(position_x,position_y)]
    
    return position_all


def computeVelocity(obstacle_vector, pose, twist):
    cmd = TwistStamped()
    cmd.header = pose.header
    min_vel_x = -0.2
    max_vec_x = 0.2
    min_vel_y = -0.2
    max_vec_y = 0.2
    min_vel_z = -1.0
    max_vec_z = 1.0
    drive_scale = 0.03 # scaling factor to scale the net force

    if (obstacle_vector is None):
        return cmd
        
    else:
        # We use velocity based potential field,that is, the gradient/force is directly commanded as velocities
        # instead of force or acceleration. 

        vel_x = obstacle_vector[0] * drive_scale
        vel_y = obstacle_vector[1] * drive_scale

        q = np.array([pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z])
        v = np.array([0, vel_x , -vel_y, 0])
        f_v = slow_rot(q, v)

        mag = np.sqrt(f_v[0]**2 + f_v[1]**2)
        cross = np.cross(np.array([1.0 , 0.0, 0.0]), np.array([f_v[0], f_v[1], 0.0])) / mag

        mag_twist = np.sqrt(twist.linear.x**2) / max_vec_x

        cmd.twist.linear.x = np.clip(f_v[0] * np.abs((1.0 - cross[2])), min_vel_x, max_vec_x)
        cmd.twist.linear.y = np.clip(f_v[1] * np.abs((1.0 - cross[2])), min_vel_y, max_vec_y)
        cmd.twist.angular.z = np.clip(-cross[2] * np.abs(1.0 - mag_twist), min_vel_z, max_vec_z)

    return cmd

def computeVelocityCommands(occupancy_grid, pose, twist):
    cmd_vel = TwistStamped()
    net_force = computeRepulsiveForce(occupancy_grid, pose) + computeAttractiveForce(pose, goal_pose, position_all)
    obstacle_vector = np.array([net_force[0], net_force[1]])
    cmd_vel  = computeVelocity(obstacle_vector, pose, twist)

    return cmd_vel

def setPath(global_plan):
    global goal_pose 
    goal_pose = global_plan.poses[-1]
    global position_all
    position_all = handleGlobalPlan(global_plan)
    return

def setSpeedLimit(speed_limit, is_percentage):
    return
