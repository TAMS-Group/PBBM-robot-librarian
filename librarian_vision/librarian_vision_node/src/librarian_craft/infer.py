# -*- coding: utf-8 -*-
import time

import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable

import cv2
import numpy as np

from collections import OrderedDict
from .craft import CRAFT
from PIL import Image


def copyStateDict(state_dict):
    if list(state_dict.keys())[0].startswith("module"):
        start_idx = 1
    else:
        start_idx = 0
    new_state_dict = OrderedDict()
    for k, v in state_dict.items():
        name = ".".join(k.split(".")[start_idx:])
        new_state_dict[name] = v
    return new_state_dict


def str2bool(v):
    return v.lower() in ("yes", "y", "true", "t", "1")


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class CraftInfer:
    def __init__(self, craft_utils, weights, resolution=1280, text_threshold=0.7, link_threshold=0.4):
        self.craft_utils = craft_utils
        self.args = Namespace(canvas_size=resolution, cuda=True, link_threshold=link_threshold,
                              low_text=0.4, mag_ratio=1.5, poly=False, refine=False,
                              # refiner_model='weights/craft_refiner_CTW1500.pth',
                              show_time=False, text_threshold=text_threshold,
                              trained_model=weights)
        self.craft = CRAFT()

        if self.args.cuda:
            self.craft.load_state_dict(copyStateDict(
                torch.load(self.args.trained_model)))
        else:
            self.craft.load_state_dict(copyStateDict(
                torch.load(self.args.trained_model, map_location='cpu')))

        if self.args.cuda:
            self.craft = self.craft.cuda()
            self.craft = torch.nn.DataParallel(self.craft)
            cudnn.benchmark = False

        self.craft.eval()

    def infer(self, img_raw):
        t = time.time()
        img = self.load_image(img_raw)
        bboxes, polys, score_text = self.test_net(
            img, self.args.text_threshold, self.args.link_threshold, self.args.low_text, self.args.cuda, self.args.poly, None)
        # print("elapsed time : {}s".format(time.time() - t))
        return bboxes, polys, score_text

    def load_image(self, img):
        if img.shape[0] == 2:
            img = img[0]
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        if img.shape[2] == 4:
            img = img[:, :, :3]
        img = np.array(img)
        return img

    def normalizeMeanVariance(self, in_img, mean=(0.485, 0.456, 0.406), variance=(0.229, 0.224, 0.225)):
        # should be RGB order
        img = in_img.copy().astype(np.float32)

        img -= np.array([mean[0] * 255.0, mean[1] * 255.0,
                        mean[2] * 255.0], dtype=np.float32)
        img /= np.array([variance[0] * 255.0, variance[1] *
                        255.0, variance[2] * 255.0], dtype=np.float32)
        return img

    def denormalizeMeanVariance(self, in_img, mean=(0.485, 0.456, 0.406), variance=(0.229, 0.224, 0.225)):
        # should be RGB order
        img = in_img.copy()
        img *= variance
        img += mean
        img *= 255.0
        img = np.clip(img, 0, 255).astype(np.uint8)
        return img

    def resize_aspect_ratio(self, img, square_size, interpolation, mag_ratio=1):
        height, width, channel = img.shape

        # magnify image size
        target_size = mag_ratio * max(height, width)

        # set original image size
        if target_size > square_size:
            target_size = square_size

        ratio = target_size / max(height, width)

        target_h, target_w = int(height * ratio), int(width * ratio)
        proc = cv2.resize(img, (target_w, target_h),
                          interpolation=interpolation)

        # make canvas and paste image
        target_h32, target_w32 = target_h, target_w
        if target_h % 32 != 0:
            target_h32 = target_h + (32 - target_h % 32)
        if target_w % 32 != 0:
            target_w32 = target_w + (32 - target_w % 32)
        resized = np.zeros((target_h32, target_w32, channel), dtype=np.float32)
        resized[0:target_h, 0:target_w, :] = proc
        target_h, target_w = target_h32, target_w32

        size_heatmap = (int(target_w/2), int(target_h/2))

        return resized, ratio, size_heatmap

    def cvt2HeatmapImg(self, img):
        img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        return img

    def test_net(self, image, text_threshold, link_threshold, low_text, cuda, poly, refine_net=None):

        # resize
        img_resized, target_ratio, size_heatmap = self.resize_aspect_ratio(
            image, self.args.canvas_size, interpolation=cv2.INTER_LINEAR, mag_ratio=self.args.mag_ratio)
        ratio_h = ratio_w = 1 / target_ratio

        # preprocessing
        x = self.normalizeMeanVariance(img_resized)
        x = torch.from_numpy(x).permute(2, 0, 1)    # [h, w, c] to [c, h, w]
        x = Variable(x.unsqueeze(0))                # [c, h, w] to [b, c, h, w]
        if cuda:
            x = x.cuda()

        t0 = time.time()
        # forward pass
        with torch.no_grad():
            y, feature = self.craft(x)

        # make score and link map
        score_text = y[0, :, :, 0].cpu().data.numpy()
        score_link = y[0, :, :, 1].cpu().data.numpy()

        # refine link
        if refine_net is not None:
            with torch.no_grad():
                y_refiner = refine_net(y, feature)
            score_link = y_refiner[0, :, :, 0].cpu().data.numpy()

        t0 = time.time() - t0
        t1 = time.time()
        # Post-processing
        boxes, polys = self.craft_utils.getDetBoxes(
            score_text, score_link, text_threshold, link_threshold, low_text, poly)

        # coordinate adjustment
        boxes = self.craft_utils.adjustResultCoordinates(
            boxes, ratio_w, ratio_h)
        polys = self.craft_utils.adjustResultCoordinates(
            polys, ratio_w, ratio_h)
        for k in range(len(polys)):
            if polys[k] is None:
                polys[k] = boxes[k]

        t1 = time.time() - t1

        # render results (optional)
        render_img = score_text.copy()
        render_img = np.hstack((render_img, score_link))
        render_img = (np.clip(render_img, 0, 1) * 255).astype(np.uint8)
        render_img = Image.fromarray(render_img)
        # ret_score_text = self.cvt2HeatmapImg(render_img)

        if self.args.show_time:
            print("\ninfer/postproc time : {:.3f}/{:.3f}".format(t0, t1))

        return boxes, polys, render_img
