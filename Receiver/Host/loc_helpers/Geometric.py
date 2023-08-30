import math as math
import numpy as num

res = 0.0001
s_res = math.pi / 180.0

class geoError(Exception):
    def __init__(self, value):
        self.tag = value

    def __str__(self):
        return repr(self.tag)

class point:
    def __init__(self, *argv):
        l = len(argv)
        if l == 1:
            self.dim = len(argv[0])
        else:
            self.dim = l
        if l == 1:
            self.x = argv[0][0]
            self.y = argv[0][1]
            try:
                self.z = argv[0][2]
            except IndexError:
                self.z = 0.0
        else:
            if l == 2:
                z = 0
            elif l == 3:
                z = argv[2]
            else:
                raise geoError('Input')
            self.x = float(argv[0])
            self.y = float(argv[1])
            self.z = z

    def __str__(self):
        return 'p(' + str(self.x) + ',' + str(self.y) + ',' + str(self.z) + ')'

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def __sub__(self, other):
        if isinstance(other, point):
            tx = self.x - other.x
            ty = self.y - other.y
            tz = self.z - other.z
        else:
            tx = self.x - other.dx
            ty = self.y - other.dy
            tz = self.z - other.dz
        return point(tx, ty, tz)

    def __add__(self, other):
        if isinstance(other, point):
            tx = self.x + other.x
            ty = self.y + other.y
            tz = self.z + other.z
        else:
            tx = self.x + other.dx
            ty = self.y + other.dy
            tz = self.z + other.dz
        return point(tx, ty, tz)

    def __mul__(self, other):
        return point(other * self.x, other * self.y, other * self.z)

    def __rmul__(self, other):
        return point(other * self.x, other * self.y, other * self.z)

    def __div__(self, other):
        return point(self.x / other, self.y / other, self.z / other)

    def __neg__(self):
        x = -self.x
        y = -self.y
        z = -self.z
        return point(x, y, z)

    def area(self):
        return 0.0

    def dist(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def std(self):
        if self.dim == 2:
            return [self.x, self.y]
        return [self.x, self.y, self.z]

    def c2s(self):
        R = self.dist(point(0, 0, 0))
        lg = math.atan(self.y / self.x)
        lat = acos(self.z / R)
        return (lg, lat, R)

    # ~ def transform(self,p1,p2):
    # ~ if isinstance(p2,point):
    # ~ v=vec(p1,p2)
    # ~ rot=v.angle()
    # ~ return self.transform(p1,rot)
    # ~ else:
    # ~ temp=self-p1
    # ~ rot=p2
    # ~ px=math.cos(rot)*temp.x+math.sin(rot)*temp.y
    # ~ py=-math.sin(rot)*temp.x+math.cos(rot)*temp.y
    # ~ return point(px,py)
    def transform(self, p, rot):
        px = math.cos(rot) * self.x + math.sin(rot) * self.y
        py = -math.sin(rot) * self.x + math.cos(rot) * self.y
        p_t = point(px, py)
        return p_t - p

    def rot(self, a):
        px = math.cos(a) * self.x - math.sin(a) * self.y
        py = math.sin(a) * self.x + math.cos(a) * self.y
        return point(px, py)

    def angle(self, p):
        v = vec(self, p)
        return v.angle()

class Anchor:
    def __init__(self, ID, loc):
        self.loc = loc
        self.ID = str(ID)

    def __str__(self):
        return 'Anchor ' + self.ID + ' @ ' + self.loc.__str__()

class Target:
    def __init__(self, ID):
        self.loc = None
        self.ID = str(ID)
        self.measures = []

    def __str__(self):
        if self.loc is None:
            return 'Target ' + self.ID
        else:
            return 'Target ' + self.ID + ' @ Real Location:' + self.loc.__str__()

    def add_measure(self, a, d):
        self.measures.append((a, d))


class circle:
    def __init__(self, p, r):
        self.c = p
        self.r = float(r)

    def __str__(self):
        return 'Circle[' + self.c.__str__() + ',' + str(self.r) + ']'

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def touch(self, o, fg=0):
        d = self.c.dist(o.c)
        if fg == 0:
            met = self.r + o.r
        else:
            met = 180.0 * (self.r + o.r) / (E.R * math.pi)
        if d < met + res:
            return True
        else:
            return False

    def side(self, p):
        d = p.dist(self.c)
        r = self.r
        if d < r:
            return 1
        elif d < r + res:
            return 0
        else:
            return -1

    def to_poly(self, step):
        alpha = 0
        pll = []
        while alpha < (2 * math.pi):
            x = self.c + point(self.r * math.cos(alpha), self.r * math.sin(alpha))
            pll.append(x)
            alpha = alpha + step
        pll[-1] = pll[0]
        return pll