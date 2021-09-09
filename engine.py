import numpy as np


class Map:
    segments = None

    def addSeg(self, seg):
        if self.segments is None:
            self.segments = seg
        else:
            self.segments = np.concatenate((self.segments, seg), axis=0)

    def addTri(self, p1, p2, p3):
        self.addSeg(np.array([[p1, p2]]))
        self.addSeg(np.array([[p2, p3]]))
        self.addSeg(np.array([[p3, p1]]))

    def addRect(self, bl, tr):
        br = [tr[0], bl[1]]
        tl = [bl[0], tr[1]]

        self.addSeg(np.array([[bl, br]]))
        self.addSeg(np.array([[br, tr]]))
        self.addSeg(np.array([[tr, tl]]))
        self.addSeg(np.array([[tl, bl]]))

    def __init__(self):
        self.addRect([2000.0, 4000.0], [3000.0, 5000.0])
        self.addRect([5000.0, 6000.0], [6000.0, 8500.0])
        self.addRect([5000.0, 9500.0], [6000.0, 10000.0])
        self.addTri([7000.0, 5500.0], [7000.0, 7000.0], [8500.0, 7000.0])
        self.addTri([8000.0, 5500.0], [8500.0, 6000.0], [8500.0, 5500.0])

        for i in range(5600, 8600, 500):
            for j in range(600, 4100, 500):
                self.addRect([i, j], [i + 250, j + 250])


def isCounterClockwise(A, B, C):
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)


# Return true if line segments ray and seg intersect
def doesIntersect(ray, seg):
    return isCounterClockwise(ray[0, 0:2], seg[0, 0:2], seg[1, 0:2]) != isCounterClockwise(ray[1, 0:2], seg[0, 0:2],
                                                                                           seg[a,
                                                                                           0:2]) and isCounterClockwise(
        ray[0, 0:2], ray[1, 0:2], seg[0, 0:2]) != isCounterClockwise(ray[0, 0:2], ray[1, 0:2], seg[1, 0:2])


def getIntersect(ray, seg):
    p = ray[0, 0:2]
    r = (ray[1, 0:2]) - p
    q = seg[0, 0:2]
    s = (seg[1, 0:2]) - q

    if np.cross(r, s) == 0 and np.cross((q - p), r) == 0:
        # Segments are Collinear
        t0 = np.dot((q - p), r) / np.dot(r, r)
        t1 = t0 + np.dot(s, r) / np.dot(r, r)

        # Check for overlap
        if 0 <= t0 <= 1 or 0 <= t1 <= 1 or (t0 <= 0 and t1 >= 1):
            # Collinear segments overlap
            p_dot = np.dot(p, p)
            q_dot = np.dot(q, q)
            r_dot = np.dot(r, r)
            s_dot = np.dot(s, s)

            # TODO: do this

            # if q <= p <= r or r <= p <= q:
            #    return p
            # if p <= q <= r or r <= q <= p:
            #    return q
            # Else p <= s <= r
            # return s
        return None

    if np.cross(r, s) == 0 and np.cross((q - p), r) != 0:
        # Segments are parallel
        return None

    t = np.cross((q - p), s) / np.cross(r, s)
    u = np.cross((q - p), r) / np.cross(r, s)
    if np.cross(r, s) != 0 and 0 <= t <= 1 and 0 <= u <= 1:
        hitPoint = p + t * r
        return RayHit(np.linalg.norm(hitPoint - p), hitPoint)

    # Otherwise the segments do not intersect
    return None


class RayHit:
    dist = 0
    pos = np.zeros(2)

    def __init__(self, dist, pos):
        self.dist = dist
        self.pos = pos


def raytrace(pos, rot, maxDist, map):
    u = rot / (rot ** 2).sum() ** 0.5
    ray = np.array([pos, (u * maxDist) + pos])

    z = maxDist
    hit = None

    segSize = map.segments.shape[0]
    for i in range(0, segSize):
        seg = map.segments[i, :, :]
        temp = getIntersect(ray, seg)
        if temp is not None:
            if temp.dist < z:
                z = temp.dist
                hit = temp

    # if hit is None:
    #    return RayHit(maxDist, ray[1, 0:2])

    return hit

    # dTime = 0.01
    # dist = 0
    # test = pos

    # u = rot / (rot**2).sum()**0.5
    # u_dt = u * dTime

    # while(True):
    #    #if pos + (u * dist) colides with map
    #    dist += dTime

    # return RayHit

class Data:
    timestamp = None
    lidar = None
    odometry = None

    def __init__(self, timestamp, lidar, odometry):
        self.timestamp = timestamp
        self.lidar = lidar
        self.odometry = odometry


def save_data(datadir, dataset, dataList):
    filename = '%s/%s.dat' % (datadir, dataset)
    print('Saving data to %s...' % filename)

    fd = open(filename, 'w')

    for data in dataList:
        s = str(int(data.timestamp))

        if data.odometry is not None:
            s += " 0 " + str(int(data.odometry.pos[0])) + " " + str(int(data.odometry.pos[1]))
        else:
            s += " 0 0 0"

        for j in range(0, 20):
            s += " 0"

        if data.lidar is not None:
            s += " " + str(int(data.lidar[0])) + " " + str(int(data.lidar[1]))

        fd.write(s + "\n")

    fd.close()
