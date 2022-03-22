import numpy as np
import cv2


'''
 This supplies the predicates (functions that return boolean value) based on whether a specific point satisfy some equations or not
 Here, the equations are of the obstacles that are there in the configuration space provided 

 The equation of lines were formulated on the basis of considering two points at a time from the list of coordinates:
    >> Point form of the line equation is being used here, i.e. : (y-y1) - m(x-x1) = 0, 
        where m = (y1-y2)/(x1-x2), and (x1,y1) and (x2,y2) are set of points of the polygon/rectangle/rhombus taken at a time

    >> Equation used for circle: (x-x1)^2 + (y-y1)^2 - r^2 = 0, 
        where (x1,y1) are the co-ordinates of the centre and r = radius of the circle

    >> Equation used for ellipse: ((x-x1)^2)/a^2 + ((y-y1)^2)/b^2 - 1 = 0,
        where (x1,y1) are the co-ordinates of the centre and a,b are the lengths of the axes of the ellipse
 
 The obstacle space is padded for rigid robot. The padding is equal to the sum of radius of the robot and the clearance required by the robot. In case of a point robot, the padding is zero
'''

class CSpacePredicateSupplier:

    '''
    This polygon's predicate comprises of equations of 6 different lines
    '''
    def get_polygon_predicate(self, height, padding):
        X, Y = 0, 1
        coord_poly = np.array([(25-padding, (185+padding)),
                                (75+padding, (185+padding)),
                                (100+padding, (150+padding)),
                                (75+padding, (120-padding)),
                                (50+padding, (150-padding)),
                                (20-padding, (120-padding))], dtype=np.int32)

        pred_poly_line_1 = lambda x, y: (y - coord_poly[0][Y] - (coord_poly[0][Y]-coord_poly[1][Y])/(coord_poly[0][X]-coord_poly[1][X])*(x - coord_poly[0][X])) >= 0
        pred_poly_line_2 = lambda x, y: (y - coord_poly[1][Y] - (coord_poly[1][Y]-coord_poly[2][Y])/(coord_poly[1][X]-coord_poly[2][X])*(x - coord_poly[1][X])) >= 0
        pred_poly_line_3 = lambda x, y: (y - coord_poly[2][Y] - (coord_poly[2][Y]-coord_poly[3][Y])/(coord_poly[2][X]-coord_poly[3][X])*(x - coord_poly[2][X])) <= 0
        pred_poly_line_4 = lambda x, y: (y - coord_poly[3][Y] - (coord_poly[3][Y]-coord_poly[4][Y])/(coord_poly[3][X]-coord_poly[4][X])*(x - coord_poly[3][X])) <= 0
        pred_poly_line_5 = lambda x, y: (y - coord_poly[4][Y] - (coord_poly[4][Y]-coord_poly[5][Y])/(coord_poly[4][X]-coord_poly[5][X])*(x - coord_poly[4][X])) <= 0
        pred_poly_line_6 = lambda x, y: (y - coord_poly[5][Y] - (coord_poly[5][Y]-coord_poly[0][Y])/(coord_poly[5][X]-coord_poly[0][X])*(x - coord_poly[5][X])) >= 0

        predicate_poly_lines = lambda x,y: pred_poly_line_1(x,y) and pred_poly_line_2(x,y) and pred_poly_line_3(x,y) and (pred_poly_line_4(x,y) or pred_poly_line_5(x,y)) and pred_poly_line_6(x,y)
        return predicate_poly_lines

    #The rectangle predicate is found by calculating the coordinates of the rectangle from the dimensions and slope of the figure given in configuration space specifications (check here -> /media/cspace_spec.PNG)

    def get_rect_predicate(self, height, padding):
        X, Y = 0, 1
        coord_rect = np.array([(30-padding, (67.5+padding)),
                                (35+padding, (76+padding)),
                                (100+padding, (38.6-padding)),
                                (95-padding, (30-padding))], dtype = np.int32)

        pred_rect_line_1 = lambda x, y: (y - coord_rect[0][Y] - (coord_rect[0][Y]-coord_rect[1][Y])/(coord_rect[0][X]-coord_rect[1][X])*(x - coord_rect[0][X])) >= 0
        pred_rect_line_2 = lambda x, y: (y - coord_rect[1][Y] - (coord_rect[1][Y]-coord_rect[2][Y])/(coord_rect[1][X]-coord_rect[2][X])*(x - coord_rect[1][X])) >= 0
        pred_rect_line_3 = lambda x, y: (y - coord_rect[2][Y] - (coord_rect[2][Y]-coord_rect[3][Y])/(coord_rect[2][X]-coord_rect[3][X])*(x - coord_rect[2][X])) <= 0
        pred_rect_line_4 = lambda x, y: (y - coord_rect[3][Y] - (coord_rect[3][Y]-coord_rect[0][Y])/(coord_rect[3][X]-coord_rect[0][X])*(x - coord_rect[3][X])) <= 0

        predicate_rect_lines = lambda x,y: pred_rect_line_1(x,y) and pred_rect_line_2(x,y) and pred_rect_line_3(x,y) and pred_rect_line_4(x,y)
        return predicate_rect_lines

    
    def get_rhombus_predicate(self, height, padding):
        X, Y = 0, 1
        coord_rhom = np.array([(225, (40+padding)),
                                (250+padding, 25),
                                (225, (10-padding)),
                                (200-padding, 25)], dtype=np.int32)

        pred_rhom_line_1 = lambda x, y: (y - coord_rhom[0][Y] - (coord_rhom[0][Y]-coord_rhom[1][Y])/(coord_rhom[0][X]-coord_rhom[1][X])*(x - coord_rhom[0][X])) >= 0
        pred_rhom_line_2 = lambda x, y: (y - coord_rhom[1][Y] - (coord_rhom[1][Y]-coord_rhom[2][Y])/(coord_rhom[1][X]-coord_rhom[2][X])*(x - coord_rhom[1][X])) <= 0
        pred_rhom_line_3 = lambda x, y: (y - coord_rhom[2][Y] - (coord_rhom[2][Y]-coord_rhom[3][Y])/(coord_rhom[2][X]-coord_rhom[3][X])*(x - coord_rhom[2][X])) <= 0
        pred_rhom_line_4 = lambda x, y: (y - coord_rhom[3][Y] - (coord_rhom[3][Y]-coord_rhom[0][Y])/(coord_rhom[3][X]-coord_rhom[0][X])*(x - coord_rhom[3][X])) >= 0

        predicate_rhom_lines = lambda x,y: pred_rhom_line_1(x,y) and pred_rhom_line_2(x,y) and pred_rhom_line_3(x,y) and pred_rhom_line_4(x,y)
        return predicate_rhom_lines


    def get_circle_predicate(self, height, padding):
        predicate_circle = lambda x,y: ((x-225)**2 + (y - (150))**2 - (25+padding)**2) <=0
        return predicate_circle

    def get_ellipse_predicate(self, height, padding):
        predicate_ellipse = lambda x,y: (((x - 150)**2)/((40+padding)**2) + ((y - (100))**2)/((20+padding)**2) - 1) <= 0
        return predicate_ellipse

    
    def get_boundary_padding_predicate(self, height, width, padding):
        predicate_boundary_padding = lambda x,y: (x < padding) or (y < padding) or ((width-1 - x) < padding) or ((height-1 - y) < padding)
        return predicate_boundary_padding


    def get_cspace_predicates(self, height, width, padding):
        c_space_predicates = np.array([
            self.get_polygon_predicate(height, padding),
            self.get_rect_predicate(height, padding),
            self.get_rhombus_predicate(height, padding),
            self.get_circle_predicate(height, padding),
            self.get_ellipse_predicate(height, padding),
            self.get_boundary_padding_predicate(height, width, padding)
        ])
        return c_space_predicates



