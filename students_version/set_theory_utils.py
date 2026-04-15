import numpy as np 

def minkowskiSum(P, Q, zero):
    # Compute Minkowski sum of 2D sets P and Q, given that both are 2D arrays of ones and zeros in spaces of the same location of zero and limits
    # P and Q must be numpy arrays of int64
    if np.shape(P) != np.shape(Q):
        raise ValueError("The shapes of P and Q are not equal.")
    else:
        x_vec = range(-int(zero[0]), np.shape(P)[1] -int(zero[0]))
        y_vec = range(-int(zero[1]), np.shape(P)[0] -int(zero[1]))
        mesh_x, mesh_y = np.meshgrid(x_vec, y_vec)
        
        print(mesh_x)
        print(mesh_y)

        addition = np.zeros(np.shape(P), dtype=np.int64)
        for i in range(np.shape(P)[1]):
            for j in range(np.shape(P)[0]):
                for k in range(np.shape(Q)[1]):
                    for l in range(np.shape(Q)[0]):
                        if P[j,i] == 1 and Q[l,k] == 1:
                            vP = [mesh_x[j,i], mesh_y[j,i]]
                            print("vP",vP)
                            vQ = [mesh_x[l,k], mesh_y[l,k]]
                            print("vQ",vQ)
                            addpoint = [zero[0] + vP[0] + vQ[0], zero[1] + vP[1] + vQ[1]]
                            print("addpoint",addpoint)
                            if (addpoint[0]<=np.shape(P)[1]) and (addpoint[1]<=np.shape(P)[0]) \
                            and (addpoint[0]>=0) and (addpoint[1]>=0):
                                addition[addpoint[1], addpoint[0]] = 1

        return addition
'''
def composition(P, zero, f):
    # Compute the composition of a set P and funtion f, with the cells in P being 1 and the others being zero
    # P must be a numpy array of int64 with only zeros and ones
    
    for 
'''
def precursorSet(S, zero, A, B, U):
    # Compute the precursor set of a set S, with the cells in S being 1 and the others being zero
    # S must be a numpy array of int64 with only zeros and ones
    pre_S = np.zeros(np.shape(S), dtype=np.int64)
    u_feasible = [[[]for i in range(np.shape(S)[1])] for j in range(np.shape(S)[0])]
    #print("size S: \n", np.shape(pre_S))

    for i1 in range(np.shape(S)[1]):
        for j1 in range(np.shape(S)[0]):
            
            pos = i1+zero[0]
            vel = j1+zero[2]
            #print("\n pos: ", pos)
            #print("vel: ", vel)
           
            if (i1+zero[0] >= -1) and (i1+zero[0] <= 1) and (j1+zero[2] >= -1) and (j1+zero[2] <= 1):
                print_mode = True
            else:
                print_mode = False
            
            print_mode = True
            
            for u in range(U[0], U[1]+1):
                pos_p1 = pos + vel*1
                vel_p1 = vel + u
                if print_mode:
                    pass
                    #print("i position:\n", [vel-zero[2], pos-zero[0]])
                    #print("input:\n", u)
                    #print("i+1 position:\n", [vel_p1-zero[2], pos_p1-zero[0]])
                
                if (pos_p1 <= zero[1]) and (pos_p1 >= zero[0]) and (vel_p1 <= zero[3]) and (vel_p1 >= zero[2]):
                    
                    if S[vel_p1-zero[2], pos_p1-zero[0]] == 1:
                        pre_S[j1, i1] = 1
                        u_feasible[j1][i1].append(u)
            
            """
            if S[j1,i1] == 1:
                for i2 in range(np.shape(S)[1]):
                    for j2 in range(np.shape(S)[0]):

                        #x_k = [i2+zero[0], j2+zero[2]]
                        if print_mode:
                            print("x_k: \n", x_k)

                        for u in range(U[0], U[1]):

                            x_kp1 = [A[0][0]*x_k[0] + A[0][1]*x_k[1] + B[0]*u, A[1][0]*x_k[0] + A[1][1]*x_k[1] + B[1]*u]
                            if print_mode:
                                print("x_kp1: \n", x_kp1)
                            
                            if (x_kp1[1] + zero[2] >= 0) and (x_kp1[0] + zero[0] >= 0)\
                            and (x_kp1[1] + zero[3] < np.shape(S)[0]) and (x_kp1[0] + zero[1] < np.shape(S)[1]):
                                if (S[x_kp1[1] + zero[2], x_kp1[0] + zero[0]] == 1):
                                    pre_S[j2,i2] = 1

                                    u_feasible[j2][i2].append(u) if u not in u_feasible[j2][i2] else u_feasible[j2][i2]

                                    #break
    
            """
            
    # sort u_feasible

    for i in range(np.shape(S)[1]):
        for j in range(np.shape(S)[0]):
            u_feasible[j][i] = sorted(u_feasible[j][i])

    return pre_S, u_feasible

def findFeasibleSet(S, zero, A, B, U):
    # Compute the feasible set of the system that must be driven to set S, with the cells in S being 1 and the others being zero
    # S must be a numpy array of int64 with only zeros and ones

    S_prev = np.ones(np.shape(S), dtype=np.int64)

    while (S_prev != S).any():

        S_prev = S
        S, u_feas = precursorSet(S, zero, A, B, U)
        #print("S: \n", S, "\n", "S_prev: \n", S_prev)
        
    return S, u_feas