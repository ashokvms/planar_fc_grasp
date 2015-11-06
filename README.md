# Force Closure Grasp Analysis
## for Planar Objects SE(2)

* Functionalities
  * Grasp Analysis
    * Force Closure Verfication - based on contact wrenches in object space
    * Force closure verification - based on Minkowski sum of contact wrenches in object space
    * Force closure verification - based on combinatorial Minkowski sum of contact wrenches in object space
    * Force closure Grasp Metrics - largest minimum resisted wrench
  * Planar Surfaces
    * Ellipse

* TODO
  * Grasp Analysis
    * Force Closure Grasp Metrics - volume of the convex hull
    * Position of the object with largest minimum resisted wrench 
  * Planar Surfaces
    * Trianlgle, Rectangle, Circle
  * Code Restructring
    * Create separate module for surfaces: Different classes for each surface type with functions for surface creation, contact point selection and contact normals
    * Create separate module for grasp analysis: Different classes for grasp anaysis and helper tools
    * Make the current script as just generic example to show usage of functions
    * Make new script to take different command line arguments (surface type, friction coeefficient, number of points, contact locations, contact forces) and perform grasp analysis
