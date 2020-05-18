from sklearn.cluster import DBSCAN
import numpy as np

def get_test_img():
    im = np.zeros([500, 500], dtype=int)
    im[1,1:10] = 255
    im[2,2] = 255
    im[4,35] = 255
    im[42,7] = 255
    im[48, 40] = 255
    im[25,1] = 255
    im[22,26] = 255
    im[23,25] = 255
    im[24,24] = 255
    im[29,29] = 255

    im[455,455] = 255
    im[451,456] = 255
    return im

class Fdsa():
    def __init__(self, a):
        self.b = a

    @property
    def b(self):
        pass

    @b.setter
    def b(self, a):
        if a=='a':
            self.b = a
        else:
            self.b = 's'

f = Fdsa('a')

timg = get_test_img()
p = np.where(timg==255)
p = np.hstack([np.array([n]).T for n in p])
print(p)

db = DBSCAN(eps=10, min_samples=1).fit(p)
print(db.labels_)