import numpy as np
import cv2
import os

class PNGReader:
    def __init__(self, path2trainfile, path2train, path2valfile, path2val):
        self.train_file = path2trainfile
        self.val_file = path2valfile
        self.train_root = path2train
        self.val_root = path2val
        self.load_data()
        self.compute_mean()

    def load_data(self):
        # LOAD TRAIN
        x_path, self.y_train = self.read_file(self.train_file)
        self.x_train = self.load_x(x_path, self.train_root)
        self.train_size = self.x_train.shape[0]
        # LOAD TEST
        x_path, self.y_val = self.read_file(self.val_file)
        self.x_val = self.load_x(x_path, self.val_root)
        self.val_size = self.x_val.shape[0]
        
    def read_file(self, filepath):
        x_path = []
        y = []
        with open(filepath) as f:
            line = f.readline()
            while line:
                data = line.strip().split(' ')
                x_path.append(data[0])
                y.append(int(data[1]))
                line = f.readline()

        return x_path, np.array(y)

    def load_x(self, path_list, root):
        x = []
        for path in path_list:
            path = os.path.join(root, path)
            raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            sx, sy = raw.shape[0:2]
            x.append(cv2.resize(raw,(0,0),fx=32.0/sx,fy=32.0/sy, interpolation = cv2.INTER_AREA))
        return np.array(x)

    def compute_mean(self):
        self.mean = np.mean(np.mean(np.mean(self.x_train, axis = 0), axis = 0),axis = 0)
        self.std = np.std(np.std(np.std(self.x_train, axis = 0), axis = 0), axis = 0)
