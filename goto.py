import math
import sys
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

motionTopic = '/turtle1/cmd_vel'
poseTopic = '/turtle1/pose'
gLoc = Pose()

def poseCallback(data):
    global gLoc
    gLoc = data

def dist(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    delx = x2 - x1
    dely = y2 - y1
    return math.sqrt(delx * delx + dely * dely)

def goto_node(goalx, goaly, threshold):
    """Move the turtle to a specified (goalx, goaly) position."""
    global gLoc
    rospy.init_node('Goto', anonymous=True)
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    rospy.Subscriber(poseTopic, Pose, poseCallback)
    rospy.sleep(1)

    rate = rospy.Rate(10)  # Control loop rate
    msg = Twist()
    
    rotGain = 10  # Angular gain
    transGain = 0.5  # Translational gain
    gVelList = []  # List to store linear velocities

    while not rospy.is_shutdown() and dist(goalx, goaly, gLoc.x, gLoc.y) > threshold:
        current_distance = dist(goalx, goaly, gLoc.x, gLoc.y)
        
        # Calculate desired angle and wrap it
        delTheta = math.atan2(goaly - gLoc.y, goalx - gLoc.x) - gLoc.theta
        delTheta = (delTheta + math.pi) % (2 * math.pi) - math.pi  # Wrap to [-pi, pi]

        # Set velocities
        msg.angular.z = rotGain * delTheta
        msg.linear.x = transGain * current_distance
        pub.publish(msg)
        
        # Store the linear velocity for plotting
        gVelList.append(msg.linear.x)
        rate.sleep()

    # Stop the robot after reaching the goal
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

    return gVelList

if __name__ == '__main__':
    try:
        waypoints = [(5, 5)]  # Single waypoint
        thresholds = [1.0, 0.5, 0.25, 0.1, 0.05, 0.01]
        
        plt.figure(figsize=(10, 6))
        
        for threshold in thresholds:
            velocities = goto_node(waypoints[0][0], waypoints[0][1], threshold)
            plt.plot(velocities, label=f'Threshold: {threshold}')
        
        plt.xlabel('Iterations')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity vs. Iterations for Different Thresholds')
        plt.legend()
        plt.grid()
        plt.show()

    except rospy.ROSInterruptException:
        pass
