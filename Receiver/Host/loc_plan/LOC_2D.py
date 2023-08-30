import numpy as np
import numpy.linalg 
import time
#from localization import Anchor
from .Config import SOUND_SPEED, DIS_BIAS, FS
from .Geometric import point, Anchor, Target, circle
from scipy.optimize import minimize


Method = 1
outlier_threshold = 1.1

################################################
def Norm(x, y, mode='2D'):
    if mode == '2D':
        return ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2) ** .5
    elif mode == '3D':
        return ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 + (x[2] - y[2]) ** 2) ** .5
    else:
        raise KeyboardInterrupt

def sum_error(x, c, r, mode):
    l = len(c)
    e = 0
    for i in range(l):
        e = e + (Norm(x, c[i].std(), mode=mode) - r[i]) ** 2/l
    return e


def lse(cA, ponit0, mode='2D'):
    l = len(cA)
    r = [w.r for w in cA]
    c = [w.c for w in cA]

    S = sum(r)
    W = [(S - w) / ((l - 1) * S) for w in r]
    if ponit0 is not None:
        p0 = point(ponit0[0], ponit0[1], ponit0[2])  # Initialized point
    else:
        p0 = point(0, 0, 0)
        for i in range(l):
            p0 = p0 + W[i] * c[i]

    if mode == '2D':
        x0 = np.array([p0.x, p0.y])
    elif mode == '3D':
        x0 = np.array([p0.x, p0.y, p0.z])
    else:
        raise KeyboardInterrupt
    # print("x0: ", x0)

    res = minimize(sum_error, x0, args=(c, r, mode), method='BFGS')
    ans = res.x
    err = sum_error(ans, c, r, mode)
    #print(ans, err)
    return point(ans), err


class Project:
    def __init__(self, mode='2D', solver='LSE'):
        self.mode = mode
        self.solver = solver
        self.AnchorDic = {}
        self.TargetDic = {}
        self.nt = 0

    def add_anchor(self, ID, loc):
        try:
            self.AnchorDic[ID]
            print(str(ID) + ':Anchor with same ID already exists')
            return
        except KeyError:
            a = Anchor(ID, point(loc))
            self.AnchorDic[ID] = a
        return a

    def add_target(self, ID=None):
        try:
            self.TargetDic[ID]
            print('Target with same ID already exists')
            return
        except:
            self.nt = self.nt + 1
            if ID:
                pass
            else:
                ID = 't' + str(self.nt)
            t = Target(ID)
            self.TargetDic[ID] = t
        return (t, ID)

    def solve(self, p0 = None, **kwargs):
        for tID in self.TargetDic.keys():
            tar = self.TargetDic[tID]
            cA = []
            for tup in tar.measures:
                landmark = tup[0]
                c = self.AnchorDic[landmark].loc
                d = tup[1]
                cA.append(circle(c, d))
            pos, err = lse(cA, p0, mode=self.mode)
            return pos, err

    def solve_with_remove(self, p0 = None, remove_id = 0,**kwargs):
        for tID in self.TargetDic.keys():
            tar = self.TargetDic[tID]
            cA = []
            for i, tup in enumerate(tar.measures):
                if i == remove_id:
                    continue
                landmark = tup[0]
                c = self.AnchorDic[landmark].loc
                d = tup[1]
                cA.append(circle(c, d))
            pos, err = lse(cA, p0, mode=self.mode)
            return pos, err
################################################

def norm(vec):
    return np.sqrt(np.sum(vec**2))


def average_without_outlier(sample_list):
    median_value = numpy.median(sample_list)
    values = []
    assert sample_list.shape[0] == 4
    for i in range(sample_list.shape[0]):
        if(abs(sample_list[i] - median_value)  < outlier_threshold):
            values.append(sample_list[i])

    print(values)
    if(len(values) == 0):
        return median_value
    else:
        return np.mean(values)

def extract_sample_diff(samples1, samples2):
    avg_sample1 = average_without_outlier(samples1)
    avg_sample2 = average_without_outlier(samples2)

    return avg_sample2 - avg_sample1


def extract_distance_single(samples1, samples2): 
    return np.abs(samples2 - samples1)/FS*SOUND_SPEED + DIS_BIAS

def extract_distance(samples1, samples2): 
    avg_sample1 = average_without_outlier(samples1)
    avg_sample2 = average_without_outlier(samples2)

    return abs(avg_sample2 - avg_sample1)/FS*SOUND_SPEED + DIS_BIAS

def convert_offset_to_dis(known_poses, sample_offsets, self_offfset):
    achor_num = sample_offsets.shape[0]
    dis = np.zeros((achor_num, 1))
    for i in range(0, achor_num):
        dis[i] = (sample_offsets[i] - self_offfset)/FS*SOUND_SPEED + DIS_BIAS
    return dis

def offset_to_dis(samples):
    return samples*SOUND_SPEED/FS

def dis_to_offset(dis):
    return np.round(dis/SOUND_SPEED*FS) + DIS_BIAS

'''
def local_2D_by_samples(known_poses, sample_offsets, self_offfset): 
    # known poses of achors - cm, size - (achor_num, 2)
    # sample offsets of each achor, size - (achor_num, 1)
    # offfset of self chirp, - float

    achor_num = sample_offsets.shape[0]
    assert (known_poses.shape[0] == sample_offsets.shape[0])

    dis = np.zeros((achor_num, 1))
    for i in range(0, achor_num):
        dis[i] = (sample_offsets[i] - self_offfset)/FS*SOUND_SPEED + DIS_BIAS
    
    #print(dis)

    A = np.zeros((achor_num, 2 + 1))
    b = np.zeros((achor_num, 1))
    for i in range(0, achor_num):
        x = known_poses[i, 0]
        y = known_poses[i, 1]
        s = dis[i]
        A[i ,:] = np.array([1, -2*x, -2*y])
        b[i] = s**2-x**2-y**2
    pos = np.matmul(np.linalg.pinv(A), b)
    return [pos[1, 0], pos[2, 0]]
'''

def local_2D_by_dis(known_poses, dis, p0 = None): 
    # dis size - anchor_num*1
    achor_num = dis.shape[0]
    #print(achor_num)
    #print(dis)
    assert (known_poses.shape[0] == dis.shape[0])

    if Method:
        P=Project(mode='2D',solver='LSE')
        for i in range(0, achor_num):
            P.add_anchor(str(i), known_poses[i,:])
        t,_=P.add_target()
        for i in range(0, achor_num):
            t.add_measure(str(i), dis[i, 0])
        pos, err = P.solve(p0)
        #print(t.loc.x, t.loc.y)
        #if (t.loc.y > -20):
        return np.array([[pos.x], [pos.y], [0]])
        #else:
        #    return np.array([[t.loc.x], [-20 -20 - t.loc.y], [0]])
    else:
        A = np.zeros((achor_num, 2 + 1))
        b = np.zeros((achor_num, 1))
        for i in range(0, achor_num):
            x = known_poses[i, 0]
            y = known_poses[i, 1]
            s = dis[i]
            A[i ,:] = np.array([1, -2*x, -2*y])
            b[i] = s**2-x**2-y**2
        pos = np.matmul(np.linalg.pinv(A), b)
        #print(pos)
        return np.array([[pos[1, 0]], [pos[2, 0] ], [0]])



def local_2D_by_dis_without_outlier(known_poses, dis, p0 = None): 
    # dis size - anchor_num*1
    achor_num = dis.shape[0]
    #print(achor_num)
    #print(dis)
    assert (known_poses.shape[0] == dis.shape[0])

    
    P=Project(mode='2D',solver='LSE')
    for i in range(0, achor_num):
        P.add_anchor(str(i), known_poses[i,:])

    t,_=P.add_target()
    for i in range(0, achor_num):
        t.add_measure(str(i), dis[i, 0])
    pos0, err0 = P.solve(p0)
    #print(pos0, err0)
    if achor_num <= 3 or err0 < 0.25:
        return np.array([[pos0.x], [pos0.y], [0]]), -1
    else:
        min_err = err0/10
        min_pos = None
        min_remove_id = -1
        for i in range(0, achor_num):
            pos, err = P.solve_with_remove(p0, i)
            print(i, pos, err)
            if err < min_err:
                min_pos = pos
                min_remove_id = i
                min_err = err
        if min_pos is None:
            return np.array([[pos0.x], [pos0.y], [0]]), min_remove_id
        else:
            print("outlier!!!! remove the measurement of anchor {}".format(min_remove_id), err0, min_err)
            return np.array([[min_pos.x], [min_pos.y], [0]]), min_remove_id


if __name__ == "__main__":
    d = extract_sample_diff(np.array([1,1,2,3]), np.array([4,4,4,6]))
    print(d)
    raise KeyboardInterrupt
    array_size = 15
    tolerance = 1e-4
    dest = [15, 100]
    points = np.array([[array_size, 0],  [-array_size, 0],  [0, -array_size], [dest[0], dest[1]]] )
    np.random.seed(1)

    offsets = np.zeros((3, 1))

    sample_err = np.random.randint(3, size=3) - 1
    print(sample_err)

    for i in range(0, 3):
        offsets[i] = round(norm(points[3, :] - points[i, :]) / SOUND_SPEED*FS) + sample_err[i]


    t0 = time.time()

    pred_loc = local_2D_by_samples(points[:3, :], offsets, 0)
    t1 = time.time()
    print(pred_loc)
    print(t1 - t0)
    