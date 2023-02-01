#!/usr/bin/env python3

import cv2
import numpy as np
import pandas as pd
import rospkg
import rospy
import torch
from cv_bridge import CvBridge
from fuzzywuzzy.fuzz import partial_ratio
from librarian_craft import craft_utils as cu
from librarian_craft.infer import CraftInfer
from librarian_dtrb.dtrb_model import DTRB
from librarian_resources.srv import (BookCoverInspect, BookCoverInspectRequest,
                                     BookCoverInspectResponse)
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BookCoverInspectionNode:
    """ This node is used to inspect the book when it is held in front of the camera.
        It does so by offering a service which directly answers with the database id
        using text matching.
    """
    def __init__(self) -> None:
        self.bridge = CvBridge()
        rospy.Service("/librarian/book/inspect", BookCoverInspect,
                      self.handleBookInspectRequest)

        # Load book databse
        self.res_path = rospkg.RosPack().get_path("librarian_resources")
        self.book_db = pd.read_csv(f"{self.res_path}/books.csv")

        self.vision_path = rospkg.RosPack().get_path('librarian_vision_node')
        # vanilla yolo network used to detect the focused book
        self.yolo = torch.hub.load(
            'ultralytics/yolov5', 'yolov5s', pretrained=True)

        # CRAFT network to detect text so we can recognize it
        craft_weights = f"{self.vision_path}/src/librarian_craft/craft_mlt_25k.pth"
        self.craft = CraftInfer(cu, craft_weights, text_threshold=0.5)

        # DTRB network used to recognize text
        dtrb_weights = f"{self.vision_path}/src/librarian_dtrb/TPS-ResNet-BiLSTM-Attn.pth"
        self.dtrb = DTRB(dtrb_weights)

        rospy.spin()

    def handleBookInspectRequest(self, req: BookCoverInspectRequest):
        img: np.ndarray
        msg = rospy.wait_for_message("/librarian/input_image", Image)
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Detect texts in original orientation
        crop1 = self.crop_to_book_detection(img)
        detections1 = self.get_text_detections(crop1)
        texts1, confidence1 = self.read_text(detections1)

        # Detect texts in 180° rotated orientation
        rotated = cv2.rotate(img, cv2.ROTATE_180)
        crop2 = self.crop_to_book_detection(rotated)
        detections2 = self.get_text_detections(crop2)
        texts2, confidence2 = self.read_text(detections2)

        # Pick the best matching book from the database and prepare the response
        texts = texts1 if confidence1 >= confidence2 else texts2
        res = BookCoverInspectResponse()
        res.id, res.score = self.get_book_id(texts)
        res.confidence = np.max([confidence1, confidence2])
        res.title = String(self.book_db["title"].tolist()[res.id - 1])
        res.text = String(" ".join(texts))
        return res

    def crop_to_book_detection(self, img):
        # yolo forward pass
        det = self.yolo([img])
        # get the bounding boxes from yolo detections
        bboxes = det.pandas().xyxy[0].to_numpy().tolist()
        # filter for only book detections
        books = list(filter(lambda x: x[6] == "book", bboxes))
        # compute the pixel area for each book detection
        book_areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in books]
        # find the biggest detected book
        book = books[np.argmax(book_areas)]
        # crop the image so that only the biggest book is visible
        return img[int(book[1]):int(book[3]), int(book[0]):int(book[2])]

    def get_text_detections(self, img):
        _, craft_polys, _ = self.craft.infer(img)
        text_detections = []
        for poly in craft_polys:
            # Calculate the new imgs width and height based on the polygon size
            width = int(np.linalg.norm(poly[0][0] - poly[1][0]))
            height = int(np.linalg.norm(poly[0][1] - poly[2][1]))
            target_rect = np.float32(
                [[0, 0], [width, 0], [width, height], [0, height]])

            # Transform the image based on the polygin so that only the book
            # remains in the new image
            pm = cv2.getPerspectiveTransform(np.float32(poly), target_rect)
            newim = cv2.warpPerspective(img, pm, (width, height))
            text_detections.append(newim)
        return text_detections

    def get_book_id(self, texts):
        # for each database entry, calculate a score
        scores = []

        for title in self.book_db["title"].tolist():
            # for each token for each recognized text, store the partial match
            tokens = title.split(" ")
            score_matrix = []
            for text in texts:
                score_row = []
                for token in tokens:
                    score = partial_ratio(text, token)
                    score_row.append(score)
                score_matrix.append(score_row)

            # Ideally, the same book title should have a perfect score in each row.
            #
            #      the  cool  book
            # cool  20   100    80
            # the  100    20    10
            # book  10    80   100
            #
            # Compute the sum of max scores over either rows or columns depending
            # on the size. If the title is longer than recognized books, pick max
            # over rows. If the title has less words, pick max over columns.
            score_matrix = np.array(score_matrix) if len(
                tokens) >= len(texts) else np.transpose(score_matrix)
            score = np.sum(np.max(score_matrix, axis=1))
            scores.append(score)

        return np.argmax(scores) + 1, np.max(scores)

    def read_text(self, detections):
        results = self.dtrb.infer(detections)
        confidence = np.mean([r[1] for r in results])
        return [r[0] for r in results if r[1] > .5], confidence


if __name__ == '__main__':
    rospy.init_node("book_cover_inspection_node")
    BookCoverInspectionNode()
