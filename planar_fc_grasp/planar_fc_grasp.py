#!/usr/bin/env python

__author__ = 'ashok'

import numpy as np
from pylab import plot, quiver
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

class PlanarFCGrasp(object):
    
    def __init__(self):
        pass   
        
    def create_ellipse(self, a=0.0, b=0.0, x=0.0, y=0.0, angle=0.0, k=360):
        """ 
        Draws an ellipse using (360*k + 1) discrete points; based on pseudo code
        given at http://en.wikipedia.org/wiki/Ellipse
        k = 1 means 361 points (degree by degree)
        a = major axis distance,
        b = minor axis distance,
        x = offset along the x-axis
        y = offset along the y-axis
        angle = clockwise rotation [in degrees] of the ellipse;
            * angle=0  : the ellipse is aligned with the positive x-axis
            * angle=30 : rotated 30 degrees clockwise from positive x-axis
        """
        pts = np.zeros((k, 2))
        beta = -angle * np.pi/180.0
        sin_beta = np.sin(beta)
        cos_beta = np.cos(beta)
        alpha = np.radians(np.r_[0.:360.:1j*k])
     
        sin_alpha = np.sin(alpha)
        cos_alpha = np.cos(alpha)
        
        pts[:, 0] = x + (a * cos_alpha * cos_beta - b * sin_alpha * sin_beta)
        pts[:, 1] = y + (a * cos_alpha * sin_beta + b * sin_alpha * cos_beta)
    
        return pts
        
    def get_contact_points(self, pts=np.zeros((1, 2)), locations=[]):
        contact_pts = np.zeros((len(locations),2))
        if len(locations)>0:
            i = 0
            for location in locations:
                contact_pts[i,:] = pts[location-1,:]
                i = i+1
        return contact_pts                        

    def get_normals_at_contacts(self, pts, a, b):
        normals = np.zeros((len(pts), 2))
        normals[:,0] = -pts[:,0]/(pow(a,2)*(np.sqrt((np.power(pts[:,0],2)/pow(a,4))+(np.power(pts[:,1],2)/pow(b,4)))))
        normals[:,1] = -pts[:,1]/(pow(b,2)*(np.sqrt((np.power(pts[:,0],2)/pow(a,4))+(np.power(pts[:,1],2)/pow(b,4)))))
        return normals    
    
    def get_vector_angles(self, v1, v2):
        
        def dotproduct(v1, v2):
          return sum((a*b) for a, b in zip(v1, v2))

        def length(v):
          return np.sqrt(dotproduct(v, v))   
        
        return np.arccos(dotproduct(v1, v2) / (length(v1) * length(v2)))
        
    def get_wrench_at_contacts(self,theta, point, force, friction_coeff):
        theta = theta + np.arctan(-friction_coeff)
        rot = np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        position = np.matrix([[-point[1],point[0]]])
        transformation = np.matrix([ [rot[0,0],rot[0,1],0], [rot[1,0],rot[1,1],0], [np.dot(position,rot)[0,0],np.dot(position,rot)[0,1],1] ])
        wrench_basis = np.matrix([[1,0], [0,1], [0,0]])
        force_at_contact = np.matrix([[force*friction_coeff], [force]])
        total_wrench = np.dot(np.dot(transformation, wrench_basis),force_at_contact)        
        return total_wrench.transpose(1,0)
        
    def get_convex_hull(self, points):
        hull = ConvexHull(points)
        return hull
    
    def get_centroid_of_hull(self, hull):
        cx = np.mean(hull.points[hull.vertices,0])
        cy = np.mean(hull.points[hull.vertices,1])
        cz = np.mean(hull.points[hull.vertices,2])
        return np.array([cx,cy,cz])
        
    def get_point_to_plane_distance(self,plane,point):
        return (point[0]*plane[0] + point[1]*plane[1] + point[2]*plane[2] + plane[3])/np.sqrt(np.power(plane[0],2)+np.power(plane[1],2)+np.power(plane[2],2))
    
    def is_point_inside_hull(self, hull, point):
        centroid = self.get_centroid_of_hull(hull)
        for plane in hull.equations:
            distance_to_point = self.get_point_to_plane_distance(plane,point)
            distance_to_centroid = self.get_point_to_plane_distance(plane,centroid)
            if((distance_to_point>0.0 and distance_to_centroid<0.0) or (distance_to_point<0.0 and distance_to_centroid>0.0)):
                return False
        return True
        
    def get_distance_to_nearest_hull_simplex(self,hull,point):
        distance = []
        for plane in hull.equations:
            distance.append(np.fabs(self.get_point_to_plane_distance(plane,point)))
        distance = map(float, distance)
        return min(distance)
        
    def is_hulls_same(self, hull1, hull2):
        return np.array_equal(hull1.vertices,hull2.vertices)

    def plot_2d_surface(self, pts=np.zeros((1, 2))):
        """
        Plots the given 2D array of points 
        """
        plot(pts[:,0], pts[:,1])

    def plot_2d_points(self, pts=np.zeros((1, 2))):
        """
        Plots the given 2D array of points 
        """
        plot(pts[:,0], pts[:,1],'ro')

    def plot_normals_at_points(self,normals,pts):
        quiver(pts[:,0], pts[:,1], normals[:,0], normals[:,1], width=0.005)
        
    def plot_hull_with_points(self, hull, points):
        fig = plt.figure()
        cx = fig.add_subplot(111, projection='3d')
        cx.scatter(points[:,0], points[:,1], points[:,2],color='black',marker='*',s=100)
        for simplex in hull.simplices:
            xs, ys, zs = points[simplex].T
            xs = np.r_[xs, xs[0]] # close polygons
            ys = np.r_[ys, ys[0]]
            zs = np.r_[zs, zs[0]]
            cx.plot(xs, ys, zs)
        cx.set_xlabel('Fx')
        cx.set_ylabel('Fy')
        cx.set_zlabel('Tau')

            
def main():
    planar_fc_grasp = PlanarFCGrasp()
    planar_fc_grasp.create_ellipse()
    