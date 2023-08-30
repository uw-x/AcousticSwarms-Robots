import numpy as np  
from scipy.fft import fft, ifft
from scipy.io import wavfile
from scipy import signal

import matplotlib.pyplot as plt
import json
from .Geometric import point, Anchor, Target, circle
from scipy.optimize import minimize


fs = 62500
speed = 344.0
bias = 0.018
begin_index = 320
f_begin = 15500
f_end = 29800
N_channel = 2048

delta_f = fs/N_channel
begin_i = round(f_begin/delta_f) + 1
end_i = round(f_end/delta_f) -1




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







def channel_estimation(seg, chirp):
    Y = fft(seg)
    X = fft(chirp)

    H = np.zeros_like(X)
    H[begin_i:end_i] = np.divide(Y[begin_i:end_i], X[begin_i:end_i]  )

    h = ifft(H)

    h = np.absolute(h)

    max_id = np.argmax(h)
    max_value = h[max_id]
    final_id = max_id


    for i in range(3, max_id):
        if( h[i] > h[i-1] and h[i] > h[i+1] and h[i] > h[i+2]  and h[i] > h[i-2] and  h[i] > max_value*0.42):
            final_id = i
            break

    # plt.figure()
    # plt.plot(h)
    # plt.scatter(max_id, max_value, marker='o', c="red")
    # plt.scatter(final_id, h[final_id], marker='x', c="green")

    # plt.show()
    return final_id

def plot_measure(ax, xy, radius, edgecolor='k', facecolor='w', **kwargs):
    """Plot a circle."""
    #print(radius)
    #print(xy)
    circle = plt.Circle(
        xy,
        radius=radius,
        fill=False,
        edgecolor=edgecolor,
        facecolor=facecolor,
        **kwargs)
    ax.add_patch(circle)

def plot_measurement(dis, mic_pos, pos_pred):
    num_mic = mic_pos.shape[0]


    fig = plt.figure()
    ax = fig.gca(
        aspect='equal',
        xlim=( 0, 5),
        ylim=( -4, 4)
    )

    plt.scatter(mic_pos[:, 0 ], mic_pos[:, 1], marker='o', color = "red")
    
    for j in range(num_mic):
        d = dis[j]

        plot_measure(ax, (mic_pos[j, 0], mic_pos[j, 1]), d)
    plt.scatter([pos_pred[0]], [pos_pred[1]],s = 50,  marker='^', color = "blue")
    plt.show()



chirp = np.loadtxt("loc_helpers/preamble_repeat_15_30_625.txt")
def get_speaker_pos(dat, mic_positions, height=0.2, plot=False):
    dis = []
    #print(dat.shape)
    for i in range(mic_positions.shape[0] + 1):
        if i == 7:
            begin_index = 320
        else:
            begin_index = 500
        y = dat[:, i*2] 

        seg = y[begin_index:begin_index + N_channel]
        final_id = channel_estimation(seg, chirp)
        dis.append(final_id + begin_index)

    dis = np.array(dis)
    dis = dis - dis[-1]
    dis = dis/fs * speed + bias

    h = height
    dis = np.sqrt(dis**2 -h**2) 

    #### begin to do 3d localization 

    p0 = [-1, 0, 0]
    p1 = [1, 0, 0]


    P=Project(mode='2D',solver='LSE')
    for i in range(0, mic_positions.shape[0]):
        P.add_anchor(str(i), mic_positions[i,:])

    t,_=P.add_target()
    for i in range(0, mic_positions.shape[0]):
        t.add_measure(str(i), dis[i])

    pos0, err0 = P.solve(p0)
    pos1, err1 = P.solve(p1)
    if err0 < err1:
        pos = pos0
        err = err0
        p00 = p0
    else:
        pos = pos1
        err = err1
        p00 = p1

    if err < 0.01:
        pos_pred = np.array([pos.x, pos.y])
    else:
        min_err = err/10
        min_pos = None
        min_remove_id = -1
        for i in range(0, mic_positions.shape[0]):
            pos, err = P.solve_with_remove(p00, i)
            if err < min_err:
                min_pos = pos
                min_remove_id = i
                min_err = err
        if min_pos is None:
            pos_pred = np.array([pos.x, pos.y])
        else:
            print("outlier!!!! remove the measurement of anchor {}".format(min_remove_id), err, min_err)
            pos_pred  = np.array([min_pos.x, min_pos.y])
            err = min_err
    
    # pos_pred = np.array([pos.x, pos.y])
    if plot:
        print('POS', pos_pred)
        plot_measurement(dis, mic_positions,pos_pred)
    return pos_pred

if __name__ == "__main__":
    chirp = np.loadtxt("sep/preamble_repeat_15_30_625.txt")
    print(chirp.shape)
    folder  = "datasets/Real World/RawReverberant/00000" 

    spk_num = 7
    mic_num = 8

    meta_file =  folder + '/metadata.json'

    with open(meta_file, 'r') as json_file:
        meta_data = json.load(json_file)

    mics = meta_data['mic']
    mic_positions = np.array([key['pos'] for key in mics])

    p0 = [-1, 0, 0]
    p1 = [1, 0, 0]

    for spk_id in range(0, spk_num):
        samplerate, dat = wavfile.read(folder + '/voice{:02d}_chirp0.wav'.format(spk_id)  )
        dis = []
        
        
        #print(dat.shape)
        for i in range(mic_num):
            if i == 7:
                begin_index = 320
            else:
                begin_index = 500
            y = dat[:, i*2] 

            seg = y[begin_index:begin_index + N_channel]
            final_id = channel_estimation(seg, chirp)
            dis.append(final_id + begin_index)
     
        dis = np.array(dis)
        dis = dis - dis[-1]
        dis = dis/fs * speed + bias
        h = 0.2
        dis = np.sqrt(dis**2 -h**2) 
        print(dis)
        #### begin to do 3d localization         


        P=Project(mode='2D',solver='LSE')
        for i in range(0, mic_positions.shape[0]):
            P.add_anchor(str(i), mic_positions[i,:])

        t,_=P.add_target()
        for i in range(0, mic_positions.shape[0]):
            t.add_measure(str(i), dis[i])
        pos0, err0 = P.solve(p0)
        pos1, err1 = P.solve(p1)
        if err0 < err1:
            pos = pos0
            err = err0
            p00 = p0
        else:
            pos = pos1
            err = err1
            p00 = p1

        pos_pred = None

        if err < 0.01:
            pos_pred = np.array([pos.x, pos.y])
        else:
            min_err = err/10
            min_pos = None
            min_remove_id = -1
            for i in range(0, mic_positions.shape[0]):
                pos, err = P.solve_with_remove(p00, i)
                if err < min_err:
                    min_pos = pos
                    min_remove_id = i
                    min_err = err
            if min_pos is None:
                pos_pred = np.array([pos.x, pos.y])
            else:
                print("outlier!!!! remove the measurement of anchor {}".format(min_remove_id), err, min_err)
                pos_pred  = np.array([min_pos.x, min_pos.y])
                err = min_err
        
        print(pos_pred, err)
        print("-"*10)
        plot_measurement(dis, mic_positions,pos_pred)