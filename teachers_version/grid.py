import numpy as np 
import matplotlib.pyplot as plt
import matplotlib as mpl

from matplotlib.font_manager import FontProperties

class Grid:
    def __init__(self, size, goal):
        self.obstacles = []
        self.sizeX = size[0]
        self.sizeY = size[1]
        self.goalX = goal[0]
        self.goalY = goal[1]

        self.costmap = np.ones((self.sizeY, self.sizeX))
        self.costmap[self.goalY][self.goalX] = 0.0
        
        self.directions = ["u", "d", "l", "r", "s"] # Up, Down, Left, Right, Stay

        # Solver parameters

        self.maxiter = 100
        self.tolerance = 1e-3

    def addObstacle(self, X, Y, cost):
        # Sets the cost of all quadrants X[0] < x < X[1] and Y[0] < y < Y[1]

        for j in range(Y[0], Y[1]+1):
            for i in range(X[0], X[1]+1):
                self.costmap[j][i] = cost

    def setGoal(self, X, Y):
        # Set new goal, restore old goal's value to 1.0 in the map

        self.costmap[self.goalY][self.goalX] = 1.0

        self.goalX = X
        self.goalY = Y

        self.costmap[self.goalY][self.goalX] = 0.0

    def getPath(self,x0,y0):
        # Returns path from (s0, y0) starting point to goal

        actions_star = []
        J_star = []
        cell = np.array([x0, y0])
        cells = cell

        while not (cell == np.array([self.goalX, self.goalY])).all():
            actions_star.append(self.policy[cell[1]][cell[0]])
            J_star.append(self.cost_to_gomap[cell[1]][cell[0]])
            cell = self.update_position(cell[0], cell[1], self.policy[cell[1]][cell[0]])
            cells = np.vstack((cells, cell))
        print(cell)
        print(self.goalX, self.goalY)
        print ((cell != np.array([self.goalX, self.goalY])).all())
        return actions_star, J_star, cells

    def printMap(self):
        print(self.costmap)

    def drawMap(self):
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(self.costmap, cmap = "hot_r", interpolation = "none")

        fig.colorbar(im, ax=ax)

        fig.tight_layout()
        plt.show()

    def drawCTGMap(self):
        prop = FontProperties()
        prop.set_file('STIXGeneral.ttf')
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(self.cost_to_gomap, cmap = "hot_r", interpolation = "none")

        fig.colorbar(im, ax=ax)

        for j in range(0, self.sizeY):
            for i in range(0, self.sizeX):
                
                if self.policy[j][i] == "u":
                    arrow = u"\u2191"
                elif self.policy[j][i] == "d":
                    arrow = u"\u2193"
                elif self.policy[j][i] == "l":
                    arrow = u"\u2190"
                elif self.policy[j][i] == "r":
                    arrow = u"\u2192"
                else:
                    arrow = ""
                text = ax.text(i, j, arrow, ha="center", va="center", color="b")

        fig.tight_layout()
        plt.show()

    def drawCTGMap_path(self, x0, y0):

        actions_star, J_star, cells = self.getPath(x0, y0)
        print(actions_star)
        print(J_star)
        print(cells)

        prop = FontProperties()
        prop.set_file('STIXGeneral.ttf')
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(self.cost_to_gomap, cmap = "hot_r", interpolation = "none")

        fig.colorbar(im, ax=ax)

        for j in range(0, self.sizeY):
            for i in range(0, self.sizeX):
                
                if self.policy[j][i] == "u":
                    arrow = u"\u2191"
                elif self.policy[j][i] == "d":
                    arrow = u"\u2193"
                elif self.policy[j][i] == "l":
                    arrow = u"\u2190"
                elif self.policy[j][i] == "r":
                    arrow = u"\u2192"
                else:
                    arrow = ""

                if [i,j] in cells.tolist():
                    text = ax.text(i, j, arrow, ha="center", va="center", color="g")
                else:
                    text = ax.text(i, j, arrow, ha="center", va="center", color="b")

        fig.tight_layout()
        plt.show()