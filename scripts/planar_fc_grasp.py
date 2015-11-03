#!/usr/bin/env python

import os
import sys
import numpy as np

current_path = os.getcwd()

try:
    from planar_fc_grasp import planar_fc_grasp
except:
    sys.path.insert(0, current_path + '/../')
    from planar_fc_grasp import planar_fc_grasp

origin = np.array([[0.0,0.0]])
major_axis_length = 4.0
minor_axis_length = 1.0
number_of_ellipse_points = 360
contact_locations = [45,90,225,325]
force_at_contact_locations = np.array([[10],[10],[10],[10]])
friction_coefficient = 0.5

pg = planar_fc_grasp.PlanarFCGrasp()

# Ellipse, with major axis length = 4, minor axis = 1, centered at (0,0)
pts = pg.create_ellipse(a=major_axis_length, b=minor_axis_length, x=origin[0][0], y=origin[0][1], k=number_of_ellipse_points)

contact_pts = pg.get_contact_points(pts=pts, locations=contact_locations)

normals_at_contacts=pg.get_normals_at_contacts(contact_pts, major_axis_length, minor_axis_length)

rotation_at_contacts = np.zeros((len(normals_at_contacts),1))
rotation_at_contacts[:,0]=pg.get_vector_angles(np.array([0,1]),np.array([normals_at_contacts[:,0],normals_at_contacts[:,1]]))

wrench_at_contacts = np.zeros((len(normals_at_contacts)*2,3))
contact_wise_wrenches = []
for i in range(len(contact_locations)):
    wrench_at_contacts[i*2,:] = pg.get_wrench_at_contacts(rotation_at_contacts[i,0],contact_pts[i],force_at_contact_locations[i,0],friction_coefficient)
    wrench_at_contacts[i*2+1,:] = pg.get_wrench_at_contacts(rotation_at_contacts[i,0],contact_pts[i],force_at_contact_locations[i,0],-friction_coefficient)
    contact_wrench = []
    contact_wrench.append(wrench_at_contacts[i*2,:])
    contact_wrench.append(wrench_at_contacts[i*2+1,:])
    contact_wise_wrenches.append(contact_wrench)

hull = pg.get_convex_hull(wrench_at_contacts)

minkowski_sum_of_wrenches = np.asarray(pg.get_minkowski_sum_of_wrenches(contact_wise_wrenches))
minkowski_hull = pg.get_convex_hull(minkowski_sum_of_wrenches)
        
'''
hull_with_origin = pg.get_convex_hull(np.vstack((wrench_at_contacts,np.array([[0,0,0]]))))
is_force_closure = pg.is_hulls_same(hull,hull_with_origin)

if is_force_closure:
    print 'GRASP IS FORCE CLOSURE'
else:
    print 'GRASP IS NOT FORCE CLOSURE'
''' 

print 'Object Wrench Space'
if pg.is_point_inside_hull(hull,np.array([0,0,0])):
    print 'GRASP IS FORCE CLOSURE'
    print 'Force closure grasp metric: ', pg.get_distance_to_nearest_hull_simplex(hull,np.array([0,0,0]))
else:
    print 'GRASP IS NOT FORCE CLOSURE'
    
print 'Minkowski Sum of Object Wrenches'
if pg.is_point_inside_hull(minkowski_hull,np.array([0,0,0])):
    print 'GRASP IS FORCE CLOSURE'
    print 'Force closure grasp metric: ', pg.get_distance_to_nearest_hull_simplex(minkowski_hull,np.array([0,0,0]))
else:
    print 'GRASP IS NOT FORCE CLOSURE'

pg.plot_2d_surface(pts)
pg.plot_2d_points(np.vstack((contact_pts,origin)))
pg.plot_normals_at_points(normals_at_contacts,contact_pts)
pg.plot_hull_with_points(hull,wrench_at_contacts)
pg.plot_hull_with_points(minkowski_hull,minkowski_sum_of_wrenches)
