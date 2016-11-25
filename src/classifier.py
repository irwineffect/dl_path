#!/usr/bin/env python
import os
import glob
import cv2
import lmdb
import numpy as np

os.environ['GLOG_minloglevel'] = '2' 
import caffe
from caffe.proto import caffe_pb2


class Classifier:
    def __init__(self):
        caffe.set_mode_gpu() 

        #Size of images
        self.IMAGE_WIDTH = 224
        self.IMAGE_HEIGHT = 224

        '''
        Reading mean image, caffe model and its weights 
        '''
        #Read mean image
        mean_blob = caffe_pb2.BlobProto()
        with open('/home/james/external/trained_models/lrf_hallway_full/mean.binaryproto') as f:
            mean_blob.ParseFromString(f.read())
        mean_array = np.asarray(mean_blob.data, dtype=np.float32).reshape(
            (mean_blob.channels, mean_blob.height, mean_blob.width))


        #Read model architecture and trained model's weights
        self.net = caffe.Net('/home/james/external/trained_models/lrf_hallway_full/caffenet_deploy.prototxt',
                        '/home/james/external/trained_models/lrf_hallway_full/lrf_hallway_full.caffemodel',
                        caffe.TEST)

        #Define image transformers
        self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
        self.transformer.set_mean('data', mean_array)
        self.transformer.set_transpose('data', (2,0,1))


    def transform_img(self, img, img_width, img_height):

        #Histogram Equalization
        img[:, :, 0] = cv2.equalizeHist(img[:, :, 0])
        img[:, :, 1] = cv2.equalizeHist(img[:, :, 1])
        img[:, :, 2] = cv2.equalizeHist(img[:, :, 2])

        #Image Resizing
        img = cv2.resize(img, (img_width, img_height), interpolation = cv2.INTER_CUBIC)

        return img


    def classify(self, img):
        img = self.transform_img(img, img_width=self.IMAGE_WIDTH, img_height=self.IMAGE_HEIGHT)
        self.net.blobs['data'].data[...] = self.transformer.preprocess('data', img)
        out = self.net.forward()
        [pred_probas] = out['prob']
        return pred_probas

