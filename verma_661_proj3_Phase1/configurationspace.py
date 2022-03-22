import numpy as np

from cspacepredicatesupplier import CSpacePredicateSupplier
from obstacle_check import *
from functools import reduce
from Obstacle import *

x_limit = 400
y_limit = 250


#----------------------------------------------------------------------------------------------------------------
# circle
circle_diameter = 80 
circle_offset_x = 300
circle_offset_y = 185
circle_radius = int(circle_diameter/2)

# concave quadrilateral
offset = 5
con_quad_p1_x = 36
con_quad_p1_y = 185

con_quad_p2_x = 115
con_quad_p2_y = 210

con_quad_p3_x = 80
con_quad_p3_y = 180

con_quad_p4_x = 105
con_quad_p4_y = 100

con_quad = np.array([[con_quad_p1_x, con_quad_p1_y], [con_quad_p2_x, con_quad_p2_y], [con_quad_p3_x, con_quad_p3_y], [con_quad_p4_x, con_quad_p4_y]], np.int32)

m_l1 = (con_quad_p2_y - con_quad_p1_y) / (con_quad_p2_x - con_quad_p1_x)
c_l1 = - m_l1 * con_quad_p1_x + con_quad_p1_y 
theta_l1 = math.atan(m_l1)
c_l1_dash = c_l1 + offset / math.cos(theta_l1)

m_l2 = (con_quad_p3_y - con_quad_p2_y) / (con_quad_p3_x - con_quad_p2_x)
c_l2 = - m_l2 * con_quad_p2_x + con_quad_p2_y 
theta_l2 = math.atan(m_l2)
c_l2_dash = c_l2 - offset / math.cos(theta_l2)

m_l3 = (con_quad_p3_y - con_quad_p4_y) / (con_quad_p3_x - con_quad_p4_x)
c_l3 = - m_l3 * con_quad_p3_x + con_quad_p3_y 
theta_l3 = math.atan(m_l3)
c_l3_dash = c_l3 + offset / math.cos(theta_l3)

m_l4 = (con_quad_p4_y - con_quad_p1_y) / (con_quad_p4_x - con_quad_p1_x)
c_l4 = - m_l4 * con_quad_p4_x + con_quad_p4_y 
theta_l4 = math.atan(m_l4)
c_l4_dash = c_l4 - offset / math.cos(theta_l4)

# Expanded Area

new_l1_l2_x =  (c_l1_dash - c_l2_dash)/(m_l2 - m_l1)
new_l1_l2_y =  (m_l1 * new_l1_l2_x) + c_l1_dash

new_l2_l3_x =  (c_l2_dash - c_l3_dash)/(m_l3 - m_l2)
new_l2_l3_y =  (m_l2 * new_l2_l3_x) + c_l2_dash

new_l3_l4_x =  (c_l3_dash - c_l4_dash)/(m_l4 - m_l3)
new_l3_l4_y =  (m_l3 * new_l3_l4_x) + c_l3_dash

new_l1_l4_x =  (c_l1_dash - c_l4_dash)/(m_l4 - m_l1)
new_l1_l4_y =  (m_l1 * new_l1_l4_x) + c_l1_dash

m_divide = (new_l2_l3_y - new_l1_l4_y)/ (new_l2_l3_x - new_l1_l4_y)
c_divide = - m_divide * new_l2_l3_x + new_l2_l3_y 
y_divide = m_divide * new_l2_l3_x + c_divide


new_con_quad = np.array([[new_l1_l2_x, new_l1_l2_y], [new_l2_l3_x, new_l2_l3_y], [new_l3_l4_x, new_l3_l4_y], [new_l1_l4_x, new_l1_l4_y]], dtype='int')


# Hexagon

offset = 5 
hex_p1_x = 200
hex_p1_y = 140

hex_p2_x = 235
hex_p2_y = 120

hex_p3_x = 235
hex_p3_y = 80

hex_p4_x = 200
hex_p4_y = 60

hex_p5_x = 165
hex_p5_y = 80

hex_p6_x = 165
hex_p6_y = 120

arr_hex = np.array([[hex_p1_x, hex_p1_y], [hex_p2_x, hex_p2_y], [hex_p3_x, hex_p3_y], [hex_p4_x, hex_p4_y],[hex_p5_x, hex_p5_y], [hex_p6_x, hex_p6_y]], np.int32)

m_hl1 = (hex_p2_y - hex_p1_y) / (hex_p2_x - hex_p1_x)
c_hl1 = - m_hl1 * hex_p1_x + hex_p1_y 
theta_hl1 = math.atan(m_hl1)
c_hl1_dash = c_hl1 + offset / math.cos(theta_hl1)


m_hl3 = (hex_p4_y - hex_p3_y) / (hex_p4_x - hex_p3_x)
c_hl3 = - m_hl3 * hex_p3_x + hex_p3_y 
theta_hl3 = math.atan(m_hl3)
c_hl3_dash = c_hl3 - offset / math.cos(theta_hl3)


m_hl4 = (hex_p5_y - hex_p4_y) / (hex_p5_x - hex_p4_x)
c_hl4 = - m_hl4 * hex_p4_x + hex_p4_y 
theta_hl4 = math.atan(m_hl4)
c_hl4_dash = c_hl4 - offset / math.cos(theta_hl4)


m_hl6 = (hex_p6_y - hex_p1_y) / (hex_p6_x - hex_p1_x)
c_hl6 = - m_hl6 * hex_p6_x + hex_p6_y 
theta_hl6 = math.atan(m_hl6)
c_hl6_dash = c_hl6 + offset / math.cos(theta_hl6)


hex_l1_l2_x =  240
hex_l1_l2_y =  (m_hl1 * hex_l1_l2_x) + c_hl1_dash

hex_l2_l3_x =  240
hex_l2_l3_y =  (m_hl3 * hex_l2_l3_x) + c_hl3_dash

hex_l3_l4_x =  (c_hl3_dash - c_hl4_dash)/(m_hl4 - m_hl3)
hex_l3_l4_y =  (m_hl3 * hex_l3_l4_x) + c_hl3_dash

hex_l4_l5_x =  160
hex_l4_l5_y =  (m_hl4 * hex_l4_l5_x) + c_hl4_dash

hex_l5_l6_x =  160
hex_l5_l6_y =  (m_hl6 * hex_l5_l6_x) + c_hl6_dash

hex_l1_l6_x =  (c_hl6_dash - c_hl1_dash)/(m_hl1 - m_hl6)
hex_l1_l6_y =  (m_hl6 * hex_l1_l6_x) + c_hl6_dash


new_hex = np.array([[hex_l1_l2_x, hex_l1_l2_y], [hex_l2_l3_x, hex_l2_l3_y], [hex_l3_l4_x, hex_l3_l4_y], [hex_l4_l5_x, hex_l4_l5_y],[hex_l5_l6_x, hex_l5_l6_y], [hex_l1_l6_x, hex_l1_l6_y]], dtype='int')

#----------------------------------------------------------------------------------------------------------------

class ConfigurationSpace:
    # def __init__(self, height, width, radius_of_bot=0, clearance=0):
    def __init__(self, height, width, radius_of_bot, clearance):

        self.height = height
        self.width = width
        self.padding = radius_of_bot + clearance
        padding = self.padding
        
        self.hex = new_hex
        self.con_quad = new_con_quad
        self.circle = [(circle_radius+clearance),(circle_offset_x,circle_offset_y)]

        self.qc1 = [5,(con_quad_p1_x,con_quad_p1_y)]
        self.qc2 = [5,(con_quad_p2_x,con_quad_p2_y)]
        self.qc3 = [5,(con_quad_p3_x,con_quad_p3_y)]
        self.qc4 = [5,(con_quad_p4_x,con_quad_p4_y)]

        self._hex = arr_hex
        self._con_quad = con_quad
        self._circle = [circle_radius,(circle_offset_x,circle_offset_y)]


    def is_obstacle_in_path(self, p1, p2):
        predicate_or_op = lambda predicate1, predicate2: predicate1 or predicate2
        is_action_invalid = reduce(predicate_or_op, [
            is_intersecting_with_boundary(p2[1], p2[0], self.width, self.height, self.padding),
            is_intersecting_with_circle(p1, p2, self.circle[0], self.circle[1]),
            is_intersecting_with_polygon(p1, p2, self.con_quad),
            is_intersecting_with_polygon(p1, p2, self.hex)
            ])
        return is_action_invalid

    def is_point_in_obstacle(self, x, y):
        
        if (x > 394 or x < 6 or y < 6 or y > 244):
            return True
        
        IsInCircle = ( (x - circle_offset_x)**2 + (y - circle_offset_y)**2 <= (circle_radius) ** 2)

        IsInHex = (y >= m_hl3*x + c_hl3_dash and y >= m_hl4*x + c_hl4_dash and x >= 160 and x <= 240 and y <= m_hl1*x + c_hl1_dash and y <= m_hl6*x + c_hl6_dash) 

        Polly = (y <= m_l1*x + c_l1_dash and y >= m_l2*x + c_l2_dash and y >= m_divide*x + c_divide) or (y <= m_l3*x + c_l3_dash and y >= m_l4*x + c_l4_dash and y <= m_divide*x + c_divide)

        IsInPolly = Polly

        return (IsInCircle or IsInHex or IsInPolly)


    
