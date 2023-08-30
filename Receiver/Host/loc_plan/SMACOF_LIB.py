import numpy as np
from .Config import SMACOF_MAX_ITER, SMACOF_CONVERGE
import matplotlib.pyplot as plt

def norm(vec):
    return np.sqrt(np.sum(vec**2))

## MSD use the SVD deompoition
def mds(d, dimensions = 2):
    """
    Multidimensional Scaling - Given a matrix of interpoint distances,
    find a set of low dimensional points that have similar interpoint
    distances.
    """

    (n,n) = d.shape
    E = (-0.5 * d**2)

    # Use mat to get column and row means to act as column and row means.
    Er = np.mat(np.mean(E,1))
    Es = np.mat(np.mean(E,0))

    # From Principles of Multivariate Analysis: A User's Perspective (page 107).
    F = np.array(E - np.transpose(Er) - Es + np.mean(E))

    [U, S, V] = np.linalg.svd(F)

    Y = U * np.sqrt(S)

    return (Y[:,0:dimensions], S)


## MSD use the iteration
def calculate_D(X):
    N = X.shape[0]
    D = np.zeros((N, N))

    for i in range(0, N):
        for j in range(i, N):
            if(i == j):
                D[i, i] = 0
            else:
                pointi = X[i, :]
                pointj = X[j, :]
                d_temp = norm(pointi - pointj)
                D[i,j] = d_temp
                D[j,i] = d_temp
    return D

def calculate_V(W):
    N = W.shape[0]
    V = np.zeros((N, N))

    for i in range(0, N ):
        for j in range(0, N):
            if(i != j ): 
                for k in range(0, N):
                    if(k != j): V[i, j] -= W[k, j]
    for j in range(0, N):
        for k in range(0, N):
            if(k != j): V[j, j] += V[k, j]
    return V


def calculate_V2(W):
    N = W.shape[0]
    V = np.zeros((N, N))

    for i in range(0, N ):
        for j in range(0, N):
            if(i != j ): 
                for k in range(0, N):
                    if(k != j): V[i, j] = -W[k, j]
    for j in range(0, N):
        for k in range(0, N):
            if(k != j): V[j, j] -= V[k, j]
    return V


def calculate_B(D, W, Z):
    N = D.shape[0]
    B = np.zeros((N, N))

    DZ = calculate_D(Z)

    for i in range(0, N ):
        for j in range(0, N):
            if(i != j ): 
                if DZ[i,j] == 0:
                    print("warining")
                    d_inv = 0
                else:
                    d_inv = D[i,j]/DZ[i,j]
                for k in range(0, N):
                    if(k != j):  B[i, j] += W[k, j]*d_inv

    for j in range(0, N):
        for k in range(0, N):
            if(k != j): B[j, j] -= B[k, j]
    
    return B

def calculate_B2(D, W, Z):
    N = D.shape[0]
    B = np.zeros((N, N))

    DZ = calculate_D(Z)

    for i in range(0, N ):
        for j in range(0, N):
            if(i != j ): 
                if DZ[i,j] == 0:
                    print("warining")
                    d_inv = 0
                else:
                    d_inv = D[i,j]/DZ[i,j]
                for k in range(0, N):
                    if(k != j):  B[i, j] = -W[k, j]*d_inv

    for j in range(0, N):
        for k in range(0, N):
            if(k != j): B[j, j] -= B[k, j]

    return B


def calculate_S(D, X, W):
    N = X.shape[0]
    DX = calculate_D(X)
    err = 0

    for i in range(0, N):
        for j in range(i+1, N):
            err += W[i, j] * (D[i, j] - DX[i, j])**2
    return err

def update_X(X_old, W, D):
    N = X_old.shape[0]
    Z = X_old
    V  = calculate_V2(W)
    #B = calculate_B(D, W, Z)
    B = calculate_B2(D, W, Z)
    #print(B)
    #print(B2)
    V_inv = np.linalg.pinv(V)
    X_new = np.matmul(V_inv, np.matmul(B, Z))
    return X_new


def update_X0(X_old, W, D):
    N = X_old.shape[0]
    Z = X_old
    V  = calculate_V(W)
    #B = calculate_B(D, W, Z)
    B = calculate_B(D, W, Z)
    #print(B)
    #print(B2)
    V_inv = np.linalg.pinv(V)
    X_new = np.matmul(V_inv, np.matmul(B, Z))
    return X_new

def update_X2(X_old, D):
    n_samples = D.shape[0]
    dis = calculate_D(X_old)
    dis[dis == 0] = 1e-6
    ratio = D / dis

    B = -ratio
    B[np.arange(len(B)), np.arange(len(B))] += ratio.sum(axis=1)
    print(B)
    X = 1.0 / n_samples * np.dot(B, X_old)
    return X


def update_X_anchor(X_old, W, D, N_unknown):
    N = X_old.shape[0]

    # print(W)
    V  = calculate_V2(W)
    V11 = V[0:N_unknown, 0:N_unknown]
    V12 = V[0:N_unknown, N_unknown:]
    
    B = calculate_B2(D, W, X_old)
    B11 = B[0:N_unknown, 0:N_unknown]
    B12 = B[0:N_unknown, N_unknown:]

    Z_u = X_old[0:N_unknown]
    Z_a = X_old[N_unknown:]
    # print(V11)
    V11_inv = np.linalg.pinv(V11) # + 1e-2*np.eye(V11.shape[0]))
    # print(V11_inv)
    M = np.matmul(B11, Z_u) +  np.matmul(B12, Z_a) - np.matmul(V12, Z_a)
    X_u =  np.matmul(V11_inv, M)

    X_new = np.concatenate([X_u, Z_a], axis = 0)
    return X_new


# def SNACOF_iter(X0, D, W, max_iter):
#     Errs = []
    
#     X_old = X0
#     for i in range(0, max_iter):
#         X_new = update_X(X_old, W, D)
#         X_new3 = update_X0(X_old, W, D)
#         print(X_new3)
#         print(X_new)
#         raise KeyboardInterrupt
#         err = calculate_S(D, X_new, W)
#         print(i, err)
#         Errs.append(err)
#         X_old = X_new
    
#     plt.figure(1)
#     plt.plot(Errs)
#     plt.show()

#     raise KeyboardInterrupt

#     return X_old

def SMACOF_iter_anchor(X0, D, W, N_unknown, max_iter, tolerance):
    Errs = []
    
    X_old = X0
    last_err = 10000
    err = calculate_S(D, X_old, W)
    print("Init error: ", err)
    # print(X0)
    for i in range(0, max_iter):
        X_new = update_X_anchor(X_old, W, D, N_unknown)
        err = calculate_S(D, X_new, W)
        # print(X_new)
        # print(err)
        # raise KeyboardInterrupt
        Errs.append(err)
        X_old = X_new
        if abs(last_err - err) < tolerance:
            break
        if (i < max_iter-1):
            #print("Warning! not converge for last")
            last_err = err
    print("Finish iteration at Epoch %d, with last err %.3f and err %.3f"%(i, last_err, err))
    
    # plt.figure()
    # plt.plot(Errs)
    # plt.show()
    
    return X_old


def localize_pairwise(Dis_matrix):
    return mds(Dis_matrix)

def localize_pairwise_anchors(Dis_matrix, init_pos, achor_pos, W = None, max_iter = SMACOF_MAX_ITER, tolerance= SMACOF_CONVERGE):
    '''
        
    '''
    N_total = Dis_matrix.shape[0] 
    N_achor = achor_pos.shape[0]  
    N_robot = init_pos.shape[0]

    assert(N_robot + N_achor == N_total)

    if W is None:
        W = np.ones((N_total, N_total))

    pos0 = np.concatenate([init_pos, achor_pos], axis = 0)

    pos_new = SMACOF_iter_anchor(pos0, Dis_matrix, W, N_robot, max_iter, tolerance)

    return pos_new
