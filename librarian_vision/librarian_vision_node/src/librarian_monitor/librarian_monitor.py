#!/usr/bin/env python3

"""
This file contains most of the functions to create images for monitoring.
"""


import glob
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
from librarian_resources.msg import (Book, BookRecognition, BookRecognitions,
                                     Books, TextRecognition, TextRecognitions, ShelfImages)
from PIL import Image, ImageDraw
from sensor_msgs.msg import Image as ImageMsg
import matplotlib
import pathlib

import rospy

import re

bridge = CvBridge()


def load_img(msg: ImageMsg):
    im = bridge.imgmsg_to_cv2(msg)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return Image.fromarray(im)


def read_spines(spines_dir):
    spine_files = glob.glob(f"{spines_dir}/*")
    spine_files = list(
        map(lambda s: [int(re.findall("(?<=id_)\d+", s)[0]), s], spine_files))
    spine_files.sort(key=lambda x: x[0])
    return [Image.open(spine) for _, spine in spine_files]


def monitor_text_recognitions(text_recs: TextRecognitions, dir=str(time.time())):
    im = load_img(text_recs.source)

    recs = [load_img(rec.source) for rec in text_recs.recognitions]
    max_w = np.max([im.width for im in recs])
    max_h = max(20, np.max([im.height for im in recs]))

    im = Image.new("RGB", (max_w + 640, max_h * len(recs)), "white")
    draw = ImageDraw.ImageDraw(im)

    rec: TextRecognition
    for i, (rec, rec_im) in enumerate(zip(text_recs.recognitions, recs)):
        im.paste(rec_im, (0, i * max_h))
        draw.text((max_w + 10,  i * max_h), rec.text.data, "black")
        draw.text((max_w + 600,  i * max_h), str(rec.confidence), "black")
        draw.line(((0, i*max_h), (im.width, i*max_h)), "grey")
    return im


def monitor_book_recognitions(recs: BookRecognitions, dir=str(time.time())):
    rec: BookRecognition = recs.recognitions[0]
    rec.sources
    spines_max_w = np.max([rec.spine.width for rec in recs.recognitions])
    spines_max_h = np.max([rec.spine.height for rec in recs.recognitions])

    text_max_w = np.max([np.max([0, *[src.width for src in rec.sources]])
                        for rec in recs.recognitions])
    text_max_h = np.max([np.max([0, *[src.height for src in rec.sources]])
                        for rec in recs.recognitions])
    text_max_num = np.max([len(rec.sources) for rec in recs.recognitions])

    max_w = np.max([spines_max_w, text_max_w])

    height = spines_max_h + text_max_h * text_max_num + 40 + 50
    width = max_w * len(recs.recognitions)
    im = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.ImageDraw(im)

    bar_width = int(max_w / len(recs.recognitions[0].hue_histogram))

    for i, rec in enumerate(recs.recognitions):
        im.paste(load_img(rec.spine), (max_w * i, 0))

        for k, text in enumerate(rec.sources):
            im.paste(load_img(text), (max_w * i, spines_max_h + k * text_max_h))

        width_str = f"w: {rec.spine_size[0]*100:.1f}cm"
        height_str = f"h: {rec.spine_size[1]*100:.1f}cm"
        draw.text((max_w * i + 2, height - 40 - 50), width_str, "black")
        draw.text((max_w * i + 2, height - 20 - 50), height_str, "black")
        draw.line(((max_w * i, 0), (max_w * i, height)), "grey")

        for k, hue in enumerate(rec.hue_histogram):
            val = hue * 50
            pos = max_w * i + bar_width * k
            draw.rectangle((pos, height - val, pos+bar_width, height), "blue")

    draw.line(((0, spines_max_h), (width, spines_max_h)), "grey")
    draw.line(((0, height - 90), (width, height - 90)), "grey")
    draw.line(((0, height - 50), (width, height - 50)), "grey")

    return im


def monitor_books(books: Books, spines_dir):
    recs = [b.recognition for b in books.books]
    recs_im = monitor_book_recognitions(BookRecognitions(recognitions=recs))
    col_width = int(recs_im.width / len(recs))

    spines_raw = read_spines(spines_dir)
    spines = []
    for spine in spines_raw:
        if spine.width > col_width:
            h = spine.height * (col_width / spine.width)
            s = spine.resize((col_width, int(h)))
        else:
            s = spine
        spines.append(s)
    spines_max_h = np.max([s.height for s in spines])

    im = Image.new(
        "RGB", (recs_im.width, recs_im.height + spines_max_h), "white")
    im.paste(recs_im, (0, 0))

    book: Book
    for i, book in enumerate(books.books):
        id = book.database_id
        im.paste(spines[id - 1], (i * col_width, recs_im.height))

    return im


def monitor_matrix(book_recs: BookRecognitions, spines_dir, values, label, cmap='viridis'):
    col_size = 30
    row_size = 30
    cmap_f = matplotlib.cm.get_cmap(cmap)

    spines_raw = read_spines(spines_dir)
    spines = []
    for spine in spines_raw:
        if spine.width > col_size:
            h = spine.height * (col_size / spine.width)
            s = spine.resize((col_size, int(h)))
        else:
            s = spine
        spines.append(s)
    spines_max_h = np.max([s.height for s in spines])

    detects = []
    for rec in book_recs.recognitions:
        s = load_img(rec.spine)
        s = s.rotate(90, expand=True)

        if s.height > row_size:
            w = s.width * (row_size / s.height)
            s = s.resize((int(w), row_size))
        detects.append(s)

    detect_max_w = np.max([d.width for d in detects])

    width = detect_max_w + np.shape(values)[1] * col_size
    height = spines_max_h + np.shape(values)[0] * row_size

    im = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.ImageDraw(im)

    values_max = np.max(values)
    values_min = np.min(values)
    values_norm = (values - values_min) / (values_max - values_min)

    # create a mapping to sort the books according to the best values
    # 1. find highest value in all rows
    # 2. append row containing highset value to sorted values
    # 3. append the book index to create the new mapping
    values_norm_remaining_select = np.array(values_norm)
    values_norm_remaining = np.array(values_norm)
    book_id_remaining = list(range(np.shape(values)[1]))
    book_id_mapping = []
    book_spine_remaining = list(range(np.shape(values)[0]))
    book_spine_mapping = []
    for i in range(values_norm.shape[0]):
        row, col = np.unravel_index(
            np.argmax(values_norm_remaining_select), values_norm_remaining_select.shape)
        book_spine_mapping.append(book_spine_remaining[row])
        book_id_mapping.append(book_id_remaining[col])
        # delete book id
        book_id_remaining = np.delete(book_id_remaining, col)
        book_spine_remaining = np.delete(book_spine_remaining, row)
        # delete the row
        values_norm_remaining_select = np.delete(values_norm_remaining_select, row, axis=0)
        values_norm_remaining = np.delete(values_norm_remaining, row, axis=0)
        # delete the column
        values_norm_remaining_select = np.delete(values_norm_remaining_select, col, axis=1)
    book_id_mapping.extend(book_id_remaining)

    for i, det_i in enumerate(range(len(detects))):
        det = detects[book_spine_mapping[det_i]]
        im.paste(det, (detect_max_w - det.width, spines_max_h + i * row_size))
        draw.line(((0, spines_max_h + i * row_size),
                  (width, spines_max_h + i * row_size)), "grey")
    
    for i, sp_i in enumerate(range(len(spines))):
        sp = spines[book_id_mapping[sp_i]]
        im.paste(sp, (detect_max_w + i * col_size, spines_max_h - sp.height))
        draw.line(((detect_max_w + i * col_size, 0),
                  (detect_max_w + i * col_size, height)), "grey")

    for row in range(np.shape(values)[0]):
        for col in range(np.shape(values)[1]):
            value = values_norm[book_spine_mapping[row]][book_id_mapping[col]]
            # color = (int((1 - value) * 255), int(value * 255), 0)
            color = tuple(np.int32(np.array(cmap_f(value)[:3]) * 255))
            pos1 = (detect_max_w + col * col_size + 1,
                    spines_max_h + row * row_size + 1)
            pos2 = (pos1[0] + row_size - 2, pos1[1] + col_size - 2)
            draw.rectangle((pos1, pos2), color)

    draw.text((10, 10), "Score matrix", "grey")
    draw.text((10, 25), label, "black")

    return im

num_samples = 0
def save_samples(book_recs: BookRecognitions, values, outdir):
    global num_samples
    if num_samples >= 10:
        return
    
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    out = f"{outdir}/{stamp}"
    pathlib.Path(out).mkdir(parents=True, exist_ok=True)
    size = 50

    detects = []
    for rec in book_recs.recognitions:
        s = load_img(rec.spine)
        s = s.rotate(90, expand=True)

        if s.height > size:
            w = s.width * (size / s.height)
            s = s.resize((int(w), size))
        detects.append(s)

    detect_max_w = np.max([d.width for d in detects])

    width = detect_max_w
    height = np.shape(values)[0] * size

    im = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.ImageDraw(im)

    for i, det in enumerate(detects):
        im.paste(det, (detect_max_w - det.width, i * size))
        draw.line(((0, i * size),
                    (width,  i * size)), "grey")
    
    im.save(f"{out}/detections.jpg")
    np.savetxt(f"{out}/values.txt", values)
    rospy.loginfo(f"Sample data {num_samples + 1}/10 save to {out}")
    num_samples += 1


def shelf_images_to_monitor_msg(imgs):
    """ Pastes both corrected images into one and returns it in an image message. """
    maxwidth = 0
    total_height = 0
    for im in imgs:
        maxwidth = maxwidth if im.shape[1] < maxwidth else im.shape[1]
        total_height += im.shape[0]

    pad = 10
    combined = np.ones(
        (total_height + pad * (len(imgs) + 1), maxwidth + pad * 2, imgs[0].shape[2])) * 255

    height_acc = pad
    for im in imgs:
        combined[height_acc:height_acc+im.shape[0],
                    pad:pad+im.shape[1]] = im
        height_acc += im.shape[0] + pad

    return bridge.cv2_to_imgmsg(np.uint8(combined))


def draw_polys_on_source(text_polys, spine_polys, msg: ShelfImages):
    """ Draw the given text/spine polygons on the original image and returns it."""
    image = bridge.imgmsg_to_cv2(msg.source, 'passthrough')
    cv2.polylines(image, np.int32(text_polys), True, (255, 0, 255, 255), 5)
    cv2.polylines(image, np.int32(spine_polys), True, (0, 255, 0, 255), 5)
    monitor_msg = bridge.cv2_to_imgmsg(np.uint8(image))
    monitor_msg.header = msg.header
    return monitor_msg


def draw_heatmaps(images, heatmaps):
    imgs = []
    for im, hm in zip(images, heatmaps):
        hm = cv2.applyColorMap(np.array(hm), cv2.COLORMAP_HOT)
        im = cv2.resize(
            im, (hm.shape[1], hm.shape[0]), interpolation=cv2.INTER_LINEAR)
        imgs.append(im[:, :, :3] * 0.5 + hm * 0.5)

    maxwidth = 0
    total_height = 0
    for im in imgs:
        maxwidth = maxwidth if im.shape[1] < maxwidth else im.shape[1]
        total_height += im.shape[0]

    pad = 10
    combined = np.ones(
        (total_height + pad * (len(imgs) + 1), maxwidth + pad * 2, imgs[0].shape[2])) * 255
    height_acc = pad
    for im in imgs:
        combined[height_acc:height_acc+im.shape[0],
                    pad:pad+im.shape[1]] = im
        height_acc += im.shape[0] + pad

    return bridge.cv2_to_imgmsg(np.uint8(combined))
