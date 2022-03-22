from turtle import color
import matplotlib.pyplot as plt

class CSpacePlotter:
    def __init__(self, c_space):
        self.c_space = c_space

    def plotMap(self, fig, ax):
        height = self.c_space.height
        width = self.c_space.width
        padding =  self.c_space.padding

        poly_pts = self.c_space.hex
        rect_pts = self.c_space.con_quad
        circle = self.c_space.circle

        _poly_pts = self.c_space._hex
        _rect_pts = self.c_space._con_quad
        _circle = self.c_space._circle
        
        # cq1 = self.c_space.qc1
        # cq2 = self.c_space.qc2
        # cq3 = self.c_space.qc3
        # cq4 = self.c_space.qc4
        
        #fig = plt.figure()
        fig.set_dpi(100)
        fig.set_size_inches(8.5,6)
        #ax = plt.axes(xlim=(0,width),ylim=(0,height))
        borders = plt.Rectangle((0,0),width,height,alpha=1,fill=None,ec='b',linewidth=padding)
        
        cir = plt.Circle((circle[1]),circle[0], color="blue")
        rect = plt.Polygon(rect_pts, color="blue")
        poly = plt.Polygon(poly_pts, color="blue")

        _cir = plt.Circle((_circle[1]),_circle[0], color="red")
        _rect = plt.Polygon(_rect_pts, color="red")
        _poly = plt.Polygon(_poly_pts, color="red")

        # cir1 = plt.Circle((cq1[1]),cq1[0], color="blue")
        # cir2 = plt.Circle((cq2[1]),cq2[0], color="blue")
        # cir3 = plt.Circle((cq3[1]),cq3[0], color="blue")
        # cir4 = plt.Circle((cq4[1]),cq4[0], color="blue")
        


        shapes = [cir,rect,poly,_cir,_rect,_poly,borders]
        for shape in shapes:
            #plt.gca().add_patch(shape)
            ax.add_patch(shape)
        return fig