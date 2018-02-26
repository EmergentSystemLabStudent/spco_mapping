#! /usr/bin/env python

# made by       Yuki Katsumata   2018.1.15
# edited by     Ryo Ozaki        2018.2.7

from __init__ import *

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Point

from scipy.stats import multinomial, dirichlet

# from PIL import Image
import copy

def counting(datas, dim):
    cnt = np.zeros(dim, dtype=int)
    if datas is None:
        return cnt
    for i in range(dim):
        cnt[i] = (datas == i).sum()
    return cnt

class LearnSpCo(object):

    def do_mkdir(self):

        p = subprocess.Popen("rm -rf " + DATASET_FOLDER + TRIALNAME, shell=True)
        sleep(1)
        p = subprocess.Popen("mkdir -p " + DATASET_FOLDER + TRIALNAME + "/", shell=True)
        p = subprocess.Popen("cp __init__.py " + DATASET_FOLDER + TRIALNAME + "/__init__.py", shell=True)

        p = subprocess.Popen("mkdir -p " + DATASET_FOLDER + TRIALNAME + "/w/", shell=True)
        p = subprocess.Popen("mkdir -p " + DATASET_FOLDER + TRIALNAME + "/theta/", shell=True)
        p = subprocess.Popen("mkdir -p " + DATASET_FOLDER + TRIALNAME + "/pi/", shell=True)
        print "mkdir "+ DATASET_FOLDER + TRIALNAME


    def save_colormap(self):

        hoge = copy.deepcopy(self.C)
        self.map.data = [e for hoge in hoge for e in hoge]
        self.pub_colorMap.publish(self.map)
        print "saved color_map"


    def save_parameter(self):

        count05 = '%05d' % self.iteration_count

        fp2 = open(DATASET_FOLDER + TRIALNAME + "/w/" + count05 +'.csv','w')
        for i in xrange(L):
            if i > 0:
                fp2.write("\n")
            fp2.write(','.join(map(str, self.w[i])))
        fp2.close()

        fp3 = open(DATASET_FOLDER + TRIALNAME + "/theta/" + count05 +'.csv','w')
        for i in xrange(L):
            if i > 0:
                fp3.write("\n")
            fp3.write(','.join(map(str, self.theta[i])))
        fp3.close()

        np.savetxt(DATASET_FOLDER + TRIALNAME + "/pi/" + count05 +'.csv', self.pi, delimiter=",")

        if self.iteration_count == ITERATION - 1:
            np.savetxt(DATASET_FOLDER + TRIALNAME + "/C.csv", self.C, delimiter=",")

        print "saved parameter " + count05


    def read_dataset(self):

        print "start read_dataset"

        self.Ft = np.loadtxt(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/Ft.csv", delimiter=",")
        self.Xt_x = np.loadtxt(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/Xtx.csv", delimiter=",")
        self.Xt_y = np.loadtxt(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/Xty.csv", delimiter=",")
        self.St_Xt_x = np.loadtxt(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/StXtx.csv", delimiter=",")
        self.St_Xt_y = np.loadtxt(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/StXty.csv", delimiter=",")

        for line in open(DATASET_FOLDER + "DATASET/" + DATASET_NAME + "/St.csv", 'r'):
            self.St = line[:].split(',')

        for x in xrange(len(self.St)):
            if self.St[x] not in self.St_list:
                self.St_list.append(self.St[x])


    def map_callback(self, hoge):

        self.map = hoge
        self.map_flag = False
        print "get map"


    def get_map_callback(self, hoge):

        if self.map_flag: return

        self.initC = np.array([self.map.data[i:i+self.map.info.width] for i in range(0, len(self.map.data), self.map.info.width)])
        self.initC[self.initC == 100] = -2
        self.map_flag = True
        print "map OK"


    def learning(self):

        print "start learning."

        self.theta = np.ones((L, CNN_SIZE)) / CNN_SIZE
        self.w = np.ones((L, len(self.St_list))) / L
        self.pi = np.ones(L) / L

        St_Xt_i = (self.St_Xt_x - self.map.info.origin.position.x) / self.map.info.resolution
        St_Xt_i = St_Xt_i.astype(int)
        St_Xt_j = (self.St_Xt_y - self.map.info.origin.position.y) / self.map.info.resolution
        St_Xt_j = St_Xt_j.astype(int)

        St_BoW = {}
        for k,(j,i)in enumerate(zip(St_Xt_j,St_Xt_i)):
            if (j,i) not in St_BoW:
                St_BoW[(j,i)] = np.zeros(len(self.St_list),dtype=int)
            St_BoW[(j,i)][self.St_list.index(self.St[k])] += 1

        Ft_Xt_i = (self.Xt_x - self.map.info.origin.position.x) / self.map.info.resolution
        Ft_Xt_i = Ft_Xt_i.astype(int)
        Ft_Xt_j = (self.Xt_y - self.map.info.origin.position.y) / self.map.info.resolution
        Ft_Xt_j = Ft_Xt_j.astype(int)

        Ft_BoW = {}
        for k,(j,i)in enumerate(zip(Ft_Xt_j,Ft_Xt_i)):
            if (j,i) not in Ft_BoW:
                Ft_BoW[(j,i)] = np.zeros(CNN_SIZE,dtype=int)
            Ft_BoW[(j,i)] += (self.Ft[k] * 100).astype(int)

        mrf = np.zeros(L,dtype=int)

        pSt = np.empty(L)
        pFt = np.empty(L)
        where_map = np.where(self.C >= 0)

        for y in xrange(ITERATION):

            for j,i in zip(where_map[0],where_map[1]):

                mrf[:] = 0
                pSt[:] = 0.0
                if (j,i) in St_BoW:
                    temp = St_BoW[(j,i)] * np.log(self.w)
                    temp[np.isnan(temp)] = 0.0
                    np.sum(temp, axis=1, out=pSt)

                pFt[:] = 0.0
                if (j,i) in Ft_BoW:
                    temp = Ft_BoW[(j,i)] * np.log(self.theta)
                    temp[np.isnan(temp)] = 0.0
                    np.sum(temp, axis=1, out=pFt)

                if self.C[j-1,i] > 0:
                    mrf[self.C[j-1,i] - 1] += 1
                if self.C[j+1,i] > 0:
                    mrf[self.C[j+1,i] - 1] += 1
                if self.C[j,i-1] > 0:
                    mrf[self.C[j,i-1] - 1] += 1
                if self.C[j,i+1] > 0:
                    mrf[self.C[j,i+1] - 1] += 1

                temp = mrf * GAMMA + pSt + pFt + np.log(self.pi)
                temp -= temp.max()
                temp = np.exp(temp)
                temp = temp / temp.sum()

                self.C[j,i] = np.argmax(np.random.multinomial(1, temp, size=1)) + 1

            num_C = counting(self.C, L + 1)[1:]
            self.pi = dirichlet.rvs(num_C + ALPHA / L)[0]

            for l in xrange(L):

                temp = np.sum([Ft_BoW[(j,i)] for j,i in Ft_BoW if self.C[j,i] == l + 1],axis=0)
                if temp.ndim == 0:
                    temp = np.zeros(CNN_SIZE)
                self.theta[l] = dirichlet.rvs(temp + CHI)[0]

                temp = np.sum([St_BoW[(j,i)] for j,i in St_BoW if self.C[j,i] == l + 1],axis=0)
                if temp.ndim == 0:
                    temp = np.zeros(len(self.St_list))
                self.w[l] = dirichlet.rvs(temp + BETA)[0]

            self.save_colormap()
            self.save_parameter()

            print y
            self.iteration_count += 1

        print "finish learning"

    def execute(self, subs):

        self.do_mkdir()

        self.C = copy.deepcopy(self.initC)
        self.iteration_count = 0
        self.St_list = []
        self.read_dataset()

        self.learning()


    def __init__(self):

        self.pub_colorMap = rospy.Publisher("/spco/color_map", OccupancyGrid, queue_size=1)
        rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber("/spco/start_learning", String, self.get_map_callback, queue_size=1)
        rospy.Subscriber("/spco/start_learning", String, self.execute, queue_size=1)

        self.map_flag = False


if __name__ == '__main__':

    rospy.init_node('LearnSpCo', anonymous=True)
    hoge = LearnSpCo()
    rospy.spin()
