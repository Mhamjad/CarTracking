# coding: utf-8

import os
import sys
import time

import torch
from torch.autograd import Variable
import numpy as np
import cv2


if torch.cuda.is_available():
    torch.set_default_tensor_type('torch.cuda.FloatTensor')

from Detector.bbox import BBox
from Detector.undistort import undistort
from Detector.ssd import build_ssd
# import config


class Preprocessor(object):
    def preprocess(self, frame):

        # rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        x = cv2.resize(frame, (300, 300))
        x = x[:, :, ::-1].copy()
        #x = undistort(x)
        x = x.astype(np.float32)
        x -= (113.2176, 114.6976, 112.6735)
        x = torch.from_numpy(x).permute(2, 0, 1)

        # wrap tensor in Variable
        xx = Variable(x.unsqueeze(0))

        if torch.cuda.is_available():
            xx = xx.cuda()
            print('Using GPU!')

        return xx


class Postprocessor(object):
    def postprocess(self, detections, frame):
        threshold = 0.3
        scale = torch.Tensor(frame.shape[1::-1]).repeat(2)

        boxes = []
        got_boxes = False

        for i in range(detections.size(1)):
            j = 0
            while j < detections.size(2) and detections[0, i, j, 0] >= threshold:

                pt = (detections[0, i, j, 1:] * scale).cpu().numpy()
                bb = BBox(pt[0], pt[1], pt[2], pt[3])
                if not (bb.x2 < 728 and bb.y2 < 72):
                    boxes.append(bb)
                    got_boxes = True

                j += 1
        if got_boxes:
            pass

        return got_boxes, boxes


class Detector(object):
    def __init__(self, weights_file):

        num_classes = 2
        self.net = build_ssd('test', 300, num_classes)
        self.net.load_weights(weights_file)

        self.pre_processor = Preprocessor()
        self.post_processor = Postprocessor()

    def detect(self, frame):
        start_time = time.time()
        f = self.pre_processor.preprocess(frame)
        # print('Preprocess time: ', time.time() - start_time)
        start_time = time.time()

        y = self.net(f)
        # print('Network forward time: ', time.time() - start_time)
        start_time = time.time()
        got_bbs, bboxes = self.post_processor.postprocess(y.data, frame)
        # print('Postprocess time: ', time.time() - start_time)

        return got_bbs, bboxes
