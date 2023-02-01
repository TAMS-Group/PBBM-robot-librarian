#!/usr/bin/env python3

import os
import time

import cv2
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from librarian_resources.msg import (BookDetections, TextRecognition,
                                     TextRecognitions)
from rospy import Publisher, Subscriber, loginfo
from std_msgs.msg import String

# Swutch between google lens text recognition and DTRB
# (google lens requires an api key and sufficient amount of 'credits')
use_google = False
if use_google:
    from google.cloud import vision
else:
    from librarian_dtrb.dtrb_model import DTRB


class TextRecognizeNode:
    """ The text recognition node uses the detected text polygons
        to attempt to read the text. This text can be used for classification.
    """

    def __init__(self) -> None:
        rospy.init_node('text_recognize_node', anonymous=True)

        # Subscribe to the book detections containing polygons for each
        # detected (but not recognized yet) piece of text
        Subscriber('/librarian/book/detections', BookDetections,
                   self.receive_detections, queue_size=1)
        self.bridge = CvBridge()
        self.pack_dir = rospkg.RosPack().get_path('librarian_vision_node')

        self.use_google = use_google
        if self.use_google:
            # Configure google cloud service
            creds_path = f"{self.pack_dir}/creds.json"
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = creds_path
            self.client = vision.ImageAnnotatorClient()
        else:
            # Ensure the weights are present as they are ignored by git
            weights = f"{self.pack_dir}/src/librarian_dtrb/TPS-ResNet-BiLSTM-Attn.pth"
            # Use DTRB for recognition
            self.dtrb = DTRB(weights)

        # Publish book detections enriched with text recognitions
        self.pub_recognitions = Publisher(
            '/librarian/text_recognitions', TextRecognitions, queue_size=1)

        loginfo('text_recognition_node ready')
        rospy.spin()

    def receive_detections(self, msg: BookDetections):
        """ Created one image for each text recognition using the provided
            polygon and runs text recognition with either google cloud or DTRB.
            The results are then published for further processing. """

        text_polys = msg.text_polys

        # Reconstruct polygons
        edge1 = list(zip(text_polys.polys_x1, text_polys.polys_y1))
        edge2 = list(zip(text_polys.polys_x2, text_polys.polys_y2))
        edge3 = list(zip(text_polys.polys_x3, text_polys.polys_y3))
        edge4 = list(zip(text_polys.polys_x4, text_polys.polys_y4))
        polys = list(zip(edge1, edge2, edge3, edge4))

        image = self.bridge.imgmsg_to_cv2(msg.source, 'passthrough')
        infer_images = []
        cropped_imgs = []

        for poly in polys:
            # Get the image size based on the poly size
            width = int(np.linalg.norm(poly[0][0] - poly[1][0]))
            height = int(np.linalg.norm(poly[0][1] - poly[2][1]))
            target_rect = np.float32(
                [[0, 0], [width, 0], [width, height], [0, height]])

            # Apply a perspective transform so only the text is depicted in the image
            pm = cv2.getPerspectiveTransform(np.float32(poly), target_rect)
            newim = cv2.warpPerspective(image, pm, (width, height))

            # rotate if height larger than width so the text is horizontal
            if newim.shape[0] > newim.shape[1]:
                newim = cv2.rotate(newim, cv2.ROTATE_90_CLOCKWISE)

            # Also infer a 180 rotated version in case the text is upside down
            newimflip = cv2.rotate(newim, cv2.ROTATE_180)
            infer_images.extend([newim, newimflip])

            # prepare message form cropped imagae to attach it to a TextRecognition
            cropped = self.bridge.cv2_to_imgmsg(newim)
            cropped.header = msg.source.header
            cropped_imgs.append(cropped)

        best = []
        results = []
        # Run text recognition with either google cloud or DTRB
        if self.use_google:
            for infer_image in infer_images:
                best_text, confident = self.detect_text_google(infer_image)
                results.append((best_text, confident))
            pairs = list(zip(*[iter(results)]*2))
            # pick from both the one with higher confidence
            best = [p[0] if p[0][1] >= p[1][1] else p[1] for p in pairs]
            print(f"best_result:{best}")

        else:
            # results contain (string, confidence) tuples where
            results = self.dtrb.infer(infer_images)

            # chunks results in pairs so [(string1, confidence1), (string2, confidence2)]
            # where both belong to the same text except one was rotated 180°
            pairs = list(zip(*[iter(results)]*2))

            # pick from both the one with higher confidence
            best = [p[0] if p[0][1] >= p[1][1] else p[1] for p in pairs]

        # Prepare the message
        recognitions = TextRecognitions(
            header=msg.header, spine_polys=msg.spine_polys)
        recognitions.source = msg.source

        for (text, confidence), poly, source in zip(best, polys, cropped_imgs):
            recognition = TextRecognition()
            recognition.text = String(data=text)
            recognition.confidence = confidence
            recognition.source = source
            # flatten polygon for message transport
            poly = poly[0] + poly[1] + poly[2] + poly[3]
            recognition.polygon = poly
            recognitions.recognitions.append(recognition)

        self.pub_recognitions.publish(recognitions)

    def detect_text_google(self, content):
        """ Runs google cloud text recognition on the provided content. """

        success, encoded_image = cv2.imencode('.png', content)
        image = vision.Image(content=encoded_image.tobytes())
        response = self.client.text_detection(image=image)
        # print(f"response:{response}")
        if response.error.message:
            raise Exception(
                '{}\nFor more info on error messages, check: '
                'https://cloud.google.com/apis/design/errors'.format(
                    response.error.message))

        # Get the whole sentence
        if len(response.text_annotations) == 0:
            best_text = None
            # print(f"best_text:{best_text}")
            return None, 0
        else:
            best_text = response.text_annotations[0].description
            # print(f"best_text:{best_text}")
            return best_text, 1


if __name__ == '__main__':
    TextRecognizeNode()
