# ClosestPointLinearConstraints
Finds the closest point to a defined P point that fulfills
 - linear constraints : inequalities, like 2x - 3y &lt; 2.3
 - and circle constraints : (x - u)^2 + (y - v)^2 < R^2 
if there is such a point. The constraints, and the P point are all in 2D.
If there are only linear constraints, this is a special case of the convex optimization problem called quadratic programming.

In case the 2d problem is not feasible, linear constraints are relaxed (circle constraints stay the same):
Ax + By < C  --> normalize --> Ax + By - d < C
This way we get a 3d problem, which should always be feasible if the circle constraints overlap each other.
The resulting point this way is a point which minimizes the maximum distance (d) from this point to the linear constraints,
this is called the safest point (P point might not influence the result when 2d lin. constraints are relaxed).
