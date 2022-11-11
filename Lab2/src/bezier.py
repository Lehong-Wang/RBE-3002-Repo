
import math

import matplotlib.pyplot as plt



class BezierCurve():
    def __init__(self, p0x= 0, p0y= 0, p1x= 0, p1y= 0, p2x= 0, p2y= 0, p3x= 0, p3y= 0, total_time=10):
        self.p0x = p0x
        self.p0y = p0y
        self.p1x = p1x
        self.p1y = p1y
        self.p2x = p2x
        self.p2y = p2y
        self.p3x = p3x
        self.p3y = p3y
        self.total_time = total_time
        self.gradient = 20






    def calc_curve(self, time):
        """
        Given time less than self.total_time
        return the (x,y) at that time
        """
        if time > self.total_time:
            # raise ValueError(f"Given time {time} greater than total time {self.total_time}")
            time = self.total_time
        t = time / self.total_time
        x = ((1 - t) ** 3) * self.p0x + 3 * ((1 - t) ** 2) * t * self.p1x + 3 * (1 - t) * (t ** 2) * self.p2x + (t ** 3) * self.p3x
        y = ((1 - t) ** 3) * self.p0y + 3 * ((1 - t) ** 2) * t * self.p1y + 3 * (1 - t) * (t ** 2) * self.p2y + (t ** 3) * self.p3y
        
        return (x,y)


    def plot(self):
        plot_x = []
        plot_y = []
        for i in range(self.gradient+1):
            t = i / self.gradient * self.total_time
            ret = self.calc_curve(t)
            plot_x.append(ret[0])
            plot_y.append(ret[1])
        plt.plot(plot_x, plot_y)
        plt.scatter([self.p0x,self.p1x,self.p2x,self.p3x], [self.p0y,self.p1y,self.p2y,self.p3y])

        plt.axis('equal')
        plt.show()



if __name__ == '__main__':
    b = BezierCurve(0,0,1,0,4,3,6,2,10)
    b.plot()








