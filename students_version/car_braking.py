import numpy as np
import copy
from set_theory_utils import precursorSet, findFeasibleSet
import matplotlib.pyplot as plt
import matplotlib as mpl

class CarBrakingDiscrete:
    def __init__(self):
        self.incr = 1

        self.maxiter = 100
        self.tolerance = 1e-3

    def setULimit(self, umin, umax):
        self.umin = umin
        self.umax = umax

    def setGoal(self, goal):
        # Set state goal
        self.goal = goal

    def setPosLimit(self, qmin, qmax):
        self.qmin = qmin
        self.qmax = qmax

    def setVelLimit(self, qdmin, qdmax):
        self.qdmin = qdmin
        self.qdmax = qdmax

    def setCostFunction(self, Q, R):

        self.Q = Q
        self.R = R
        
    
    def cost_function_action(self, u):
        cost = self.cost_function(np.zeros((2,)), np.array([u]))
        return cost


    def generateMeshes(self):
        self.qmin_discrete = int(self.qmin / self.incr)
        self.qmax_discrete = int(self.qmax / self.incr)
        self.qdmin_discrete = int(self.qdmin / self.incr)
        self.qdmax_discrete = int(self.qdmax / self.incr)

    def setAllowedActions(self, allowed_actions):
        self.allowed_actions = allowed_actions
    
    def evaluatePolicy(self):
        # Calculate cost associated to each node following the current policy

        ctg_map = np.zeros((self.sizeX, self.sizeY))

        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                if len(self.allowed_actions[j][i]) != 0:

                    xnext = self.update_position(np.array([i+self.qmin, j+self.qdmin]), np.array([self.policy[i][j]]))
    
                    ctg_map[i,j] = self.cost_function(np.array([i+self.qmin, j+self.qdmin]), np.array([self.policy[i][j]])) + self.cost_to_gomap[xnext[0]-self.qmin, xnext[1]-self.qdmin]
    
        err = np.max(np.abs(ctg_map - self.cost_to_gomap))
        self.cost_to_gomap = ctg_map

        return err

    def updatePolicy(self):
        # Find reachable cell with minimal cost-to-go

        for j in range(0, self.sizeY):
            for i in range(0, self.sizeX):

                # List of potential actions
                u_candidates = self.allowed_actions[j][i]
                if len(u_candidates) > 0:

                    x = [i+self.qmin_discrete, j+self.qdmin_discrete]

                    # List of costs associated to each action
                    costs = []
                    
                    for u in u_candidates:

                        xnext = self.update_position(np.array([i+self.qmin, j+self.qdmin]), np.array([u]))
                        costs.append(self.cost_to_gomap[xnext[0]-self.qmin, xnext[1]-self.qdmin] + self.cost_function_action(np.array([u])))
                        
                    J_star = min(costs) # Optimal cost-to-go
                    self.policy[i][j] = u_candidates[costs.index(J_star)]



    
    def solveDiscreteproblem(self):
        self.sizeX = self.qmax_discrete - self.qmin_discrete
        self.sizeY = self.qdmax_discrete - self.qdmin_discrete

        if self.qmax_discrete >= 0 or self.qmin_discrete <= 0:
            self.sizeX += 1

        if self.qdmax_discrete >= 0 or self.qdmin_discrete <= 0:
            self.sizeY += 1
        
        # Initialize policy array
        self.policy = np.zeros((self.sizeX,self.sizeY))
        self.policy = np.array([
                        [min(cell) if cell else 0 for cell in row]
                        for row in self.allowed_actions
                        ]).T
        self.policy[self.goal[0]-self.qmin, self.goal[1]-self.qdmin] = 0
        
        self.cost_to_gomap = np.zeros((self.sizeX,self.sizeY))

        err = np.inf
        i = 0

        while i < self.maxiter and err > self.tolerance:
            err = self.evaluatePolicy()
            self.updatePolicy()
            i += 1

        print("iterations : ", i)
        print("error: ", err)

    def drawMap(self):
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(self.cost_to_gomap, cmap = "viridis", interpolation = "none")

        fig.colorbar(im, ax=ax)
        plt.ylabel("Position [m]")
        plt.xlabel("Velocity [m/s]")

        ticks_y = np.linspace(0, self.sizeX, 5)
        labels_y = np.linspace(self.qmin, self.qmax, 5)

        ticks_x = np.linspace(0, self.sizeY, 5)
        labels_x = np.linspace(self.qdmin, self.qdmax, 5)
        
        ax.set_xticks(ticks_x)
        ax.set_yticks(ticks_y)
        
        ax.set_xticklabels([f"{l:.0f}" for l in labels_x])
        ax.set_yticklabels([f"{l:.0f}" for l in labels_y])
        ax.set_title("Optimal cost-to-go")

        fig.tight_layout()
        plt.show()

    def drawPolicy(self):
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(self.policy, cmap = "viridis", interpolation = "none")

        fig.colorbar(im, ax=ax)
        plt.ylabel("Position [m]")
        plt.xlabel("Velocity [m/s]")

        ticks_y = np.linspace(0, self.sizeX, 5)
        labels_y = np.linspace(self.qmin, self.qmax, 5)

        ticks_x = np.linspace(0, self.sizeY, 5)
        labels_x = np.linspace(self.qdmin, self.qdmax, 5)
        
        ax.set_xticks(ticks_x)
        ax.set_yticks(ticks_y)
        
        ax.set_xticklabels([f"{l:.0f}" for l in labels_x])
        ax.set_yticklabels([f"{l:.0f}" for l in labels_y])
        ax.set_title("Optimal cost-to-go")

        fig.tight_layout()
        plt.show()