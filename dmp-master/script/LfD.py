#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

def lineToData(line):
 
    return [float(line[0]),float(line[1]),float(line[2])]

def readData(fileName):
    data = []
    with open(fileName) as f:
        for line in f.readlines():
            data.append(lineToData(line.split()))
    return data

def writeData(fileName, plan, dims):
	with open(fileName, 'w') as f:
		i = 0
		while i < len(plan.plan.points):
				j = 0
				while j < 3:
					f.write(str(plan.plan.points[i].positions[j]))
					f.write(" ")
					j+= 1
				f.write("\n")
				i +=1

def writeData2(fileName, plan, dims): # real robot translations
	with open(fileName, 'w') as f:
		i = 0
		while i < len(plan.plan.points):
				j = 0
				while j < 3:

					if j == 0: #x
						f.write(str(plan.plan.points[i].positions[j] - 0.0159))

					

					elif j == 2 : #z
						f.write(str(plan.plan.points[i].positions[j] - 0.1898))
					else:	#y
						f.write(str(plan.plan.points[i].positions[j]))
					f.write(" ")
					j+= 1
				f.write("\n")
				i +=1
			


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 3-D trajectory
    dims = 3                
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4 
    traj = []
    traj = readData("/home/hwadong/catkin_ws/src/lwr_gbz/data/data_mes.txt")
    #print data
         
    #traj = [[1.0,1.0, 1.0],[2.0,2.0, 2.0],[3.0,4.0, 5.0],[6.0,8.0, 10.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [-0.419659, -0.0468611, 0.150443]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0]   
    t_0 = 0                
    goal = [-0.419659, -0.5468611, 0.150443]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)
    
    writeData("/home/hwadong/catkin_ws/src/lwr_gbz/data/plan.txt", plan, dims)


    writeData2("/home/hwadong/catkin_ws/src/lwr_youssef/data/DemonstratedTrajectory_sim_to_real.txt", plan, dims) #for real robot package


    print plan
    #print len(traj)




