import math
import numpy as np
import matplotlib.pyplot as plt

'''
Checking if the line intersects closed figure
1. Circle
2. Ellipse
3. Polygon
'''

# Circle: 
# @Params: Segment pt1, pt2,radius,center
# Return type: bool
def is_intersecting_with_circle(p1,p2,r,center):
    m,y_int,x_int = lineModelGenerator(p1,p2)
    q = center[1]
    p = center[0]
    
    if m == np.inf:
        A = 1
        B = -2*q
        C = q**2 + x_int**2 + p**2 - r**2 - 2*x_int*p
        flag  = 1
    else:
        A = 1+m**2
        B = 2*(m*y_int - m*q - p)
        C = q**2 - r**2 + p**2 -2*y_int*q + y_int**2
        flag = 0
        
    disc = B**2 - 4*A*C
   
    if disc < 0:
        return False
    else:
        if flag == 0:
            root1 = (-B+math.sqrt(disc))/(2*A)
            root2 = (-B-math.sqrt(disc))/(2*A)
            rooty1 = m*root1 + y_int
            rooty2 = m*root2+y_int
        elif flag == 1:
            root1 = x_int
            root2 = root1
            rooty1 = (-B+math.sqrt(disc))/(2*A)
            rooty2 = (-B-math.sqrt(disc))/(2*A)

    if (min(p1[0],p2[0])<=root1<=max(p1[0],p2[0]) and min(p1[1],p2[1])<=rooty1<=max(p1[1],p2[1])) or (min(p1[0],p2[0])<=root2<=max(p1[0],p2[0]) and min(p1[1],p2[1])<=rooty2<=max(p1[1],p2[1])):
        return True
    else:
        return False



# Ellipse: 
# @Params: Segment pt1, pt2, ellipse params,center
# Return type: bool
def is_intersecting_with_ellipse(p1,p2,ell_param,center):
    m,y_int,x_int = lineModelGenerator(p1,p2)
    
    h = center[0]
    k = center[1]
    a = ell_param[0]/2
    b = ell_param[1]/2
    if m == np.inf:
        A = 1
        B = -2*k
        C = k**2 - b**2 + ((x_int-h)**2)*((b**2)/(a**2))
        flag = 1
    else:    
        A = (a**2)*(m**2) + b**2
        B = 2*(-((h)*(b**2))+(a**2)*m*(y_int-k))
        C = (h**2)*(b**2) + (a**2)*((y_int-k)**2) - (a**2)*(b**2)
        flag = 0
    
    #Checking if the quadratic equation has unique real roots
    disc = B**2 - 4*A*C
    if disc < 0:
        return False
    else:
        if flag == 0:
            root1 = (-B+math.sqrt(disc))/(2*A)
            root2 = (-B-math.sqrt(disc))/(2*A)
            rooty1 = m*root1 + y_int
            rooty2 = m*root2 + y_int
        elif flag == 1:
            root1 = x_int
            root2 = root1
            rooty1 = (-B+math.sqrt(disc))/(2*A)
            rooty2 = (-B-math.sqrt(disc))/(2*A)

    if (min(p1[0],p2[0])<=root1<=max(p1[0],p2[0]) and min(p1[1],p2[1])<=rooty1<=max(p1[1],p2[1]))or (min(p1[0],p2[0])<=root2<=max(p1[0],p2[0]) and min(p1[1],p2[1])<=rooty2<=max(p1[1],p2[1])):
        return True
    else:
        return False



# Polygon: 
# @Params: Segment pt1, pt2, coordinates of polygon in order
# Return type: bool
def is_intersecting_with_polygon(p1,p2,coord):
    x_poly_coord = coord[:,0]
    y_poly_coord = coord[:,1]
    polyLines_model = [] 
    M,Y_int,X_int = lineModelGenerator(p1,p2)
    intercept_mat = []

    if M == np.inf:
        eqn_line_seg = [1, 0]
        intercept_mat.append(X_int)
    else:
        eqn_line_seg = [-M, 1]
        intercept_mat.append(Y_int)
    
    for i in range(len(coord)):
        if i+1 >= len(coord):
            polyLines_model.append(lineModelGenerator(coord[i],coord[0]))
        else:    
            polyLines_model.append(lineModelGenerator(coord[i],coord[i+1]))
    

    is_intersecting = False
    for ix, model in enumerate(polyLines_model):
        m_poly, y_int_poly, x_int_poly = model

        if m_poly == M:
            continue

        intercept_mat_copy = intercept_mat.copy()
        if m_poly == np.inf:
            eqn_line_poly = [1, 0]
            intercept_mat_copy.append(x_int_poly)
        else:
            eqn_line_poly = [-m_poly, 1]
            intercept_mat_copy.append(y_int_poly)
        

        eqn_mat = [eqn_line_seg, eqn_line_poly]
        x, y = np.linalg.solve(eqn_mat, intercept_mat_copy)

        if min(p1[0],p2[0]) <= x <= max(p1[0],p2[0]) and min(p1[1],p2[1]) <= y <= max(p1[1],p2[1]):
            if ix+1 >= len(coord):
                if min(x_poly_coord[ix],x_poly_coord[0])<=x<=max(x_poly_coord[ix],x_poly_coord[0]) and min(y_poly_coord[ix],y_poly_coord[0])<=y<=max(y_poly_coord[ix],y_poly_coord[0]):
                    is_intersecting = True
                    break
                
            else:
                if min(x_poly_coord[ix],x_poly_coord[ix+1])<=x<=max(x_poly_coord[ix],x_poly_coord[ix+1]) and min(y_poly_coord[ix],y_poly_coord[ix+1])<=y<=max(y_poly_coord[ix],y_poly_coord[ix+1]):
                    is_intersecting = True
                    break
                
    return is_intersecting        

def is_intersecting_with_boundary(x, y, width, height, padding):
    return (x < padding) or (y < padding) or ((width-1 - x) < padding) or ((height-1 - y) < padding)

def lineModelGenerator(p1,p2):
    if p1[0] == p2[0]:
        m = np.inf
        y_intercept = np.inf
        x_intercept = p1[0]
    else:
        m = (p2[1]-p1[1])/(p2[0]-p1[0])
        y_intercept = p2[1]-m*p2[0]
        if m == 0:
            x_intercept = np.inf
        else:
            x_intercept = -y_intercept/m
    return m, y_intercept, x_intercept
