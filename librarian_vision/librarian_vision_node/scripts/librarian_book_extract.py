#!/usr/bin/env python3

from math import isfinite

import cv2
import Levenshtein as L
import matplotlib.path as mlpath
import numpy as np
import pandas as pd
import rospy
from cv_bridge import CvBridge
from fuzzywuzzy import fuzz
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Quaternion
from image_geometry import PinholeCameraModel
from librarian_monitor.librarian_monitor import (monitor_book_recognitions,
                                                 monitor_books, monitor_matrix,
                                                 monitor_text_recognitions,
                                                 read_spines, save_samples)
from librarian_resources.msg import (Book, BookRecognition, BookRecognitions,
                                     Books, TextRecognitions)
from librarian_resources.srv import *
from librarian_vision_node.srv import *
from PIL import Image as ImagePIL
from rospkg import RosPack
from rospy import Publisher, Subscriber, loginfo
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Empty, String
from tf import TransformBroadcaster, TransformListener
from tf.transformations import quaternion_inverse, quaternion_matrix
import torch
from transformers import CLIPProcessor, CLIPModel


def linePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
    """ Stolen from https://rosettacode.org/wiki/Find_the_intersection_of_a_line_with_a_plane#Python """
    ndotu = planeNormal.dot(rayDirection)
    if abs(ndotu) < epsilon:
        raise RuntimeError("no intersection or line is within plane")
    w = rayPoint - planePoint
    si = -planeNormal.dot(w) / ndotu
    Psi = w + si * rayDirection + planePoint
    return Psi


class BookExtract:
    """ This node uses the spine detections + text recognitions top classify each detection.
        It uses a hand-crafted feature vector for classification. The algorithm consideres that
        each book could only occur once in the scene (so there can be no id assigned to two 
        detections simultaniously). Furthermore, the pose for each spine detection will be 
        computed using the point cloud service (librarian_pc_process).
    """

    def __init__(self):
        rospy.init_node('librarian_vision_node')
        rospy.wait_for_service('librarian_pc_process')

        # This service gives us the average position of all given points defined by their indices.
        self.process_pc = rospy.ServiceProxy(
            'librarian_pc_process', LibrarianPcProcess)

        self.vision_node_on = True
        self.perception_service = rospy.Service('toggle_vision_node_service', ToggleVisionNode,
                                                self.toggle_vision_node)


        self.tfl = TransformListener()
        self.tfb = TransformBroadcaster()
        self.bridge = CvBridge()

        # Our input, spine detections + text recognitions
        self.recognitions_sub = Subscriber('/librarian/text_recognitions', TextRecognitions,
                                           lambda x: self.receive_recognitions(x), queue_size=2)

        # A pose array to monitor the found poses coming from the point cloud service
        self.detected_book_poses = Publisher(
            "/librarian/detected_book_poses", PoseArray, queue_size=5)

        # The final books this node publishs, it contains each spine detection along with a database id.
        self.books_pub = Publisher(
            "/librarian/books", Books, queue_size=5)

        self.env_reset_pub = Publisher(
            "/librarian/reset_env", Empty, queue_size=1)

        # An image showing which spine detection was classified as which book.
        self.monitor_books_pub = Publisher(
            "/librarian/monitor/books/compressed", CompressedImage, queue_size=1)

        # A matrix showing (different) scores for each spine detection with each book from the database.
        # The more green the cell is, the higher the score. This topic can be used to look at the
        # quality of different features or the overall score. The feature that is suppose to be
        # monitored can be defined in the 'match_books' function.
        self.matrix_pub = Publisher(
            "/librarian/monitor/matrix/compressed", CompressedImage, queue_size=1)

        # Get pinhole camera to calculate 3d pose of spine edges
        cam_msg = rospy.wait_for_message("/librarian/camera_info", CameraInfo)
        self.pinhole = PinholeCameraModel()
        self.pinhole.fromCameraInfo(cam_msg)

        # get the cvs file from librarian common
        path = RosPack().get_path("librarian_resources")
        self.book_db = pd.read_csv(f"{path}/books.csv")
        self.spines_dir = f"{path}/spines"
        self.spine_imgs = read_spines(self.spines_dir)
        self.spines_db = [cv2.cvtColor(
            np.array(s), cv2.COLOR_RGB2BGR) for s in self.spine_imgs]
        self.data_collect_dir = path + "/collect_data"

        # Compute the histograms (features for classification) for each book in the database.
        self.init_db_histograms()

        # setup SIFT matching
        self.sift = cv2.SIFT_create()
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)   # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.sift_des_db = [self.sift.detectAndCompute(
            s, None)[1] for s in self.spines_db]
        
        self.im_enc_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.im_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.spine_encodings = [self.get_embedding(im) for im in self.spine_imgs]
        rospy.loginfo("Book extract ready")
        rospy.spin()

    def receive_recognitions(self, msg: TextRecognitions):
        # check if perception is allowed
        if not self.vision_node_on:
            return
        poses = []  # centers of spines
        offsets = []  # offset of the center of the spine towards the book shelf

        shelf = self.tfl.lookupTransform(
            msg.header.frame_id, 'Shelf', rospy.Time(0))
        upper_shelf = self.tfl.lookupTransform(
            msg.header.frame_id, 'shelf_1_0_0', rospy.Time(0))
        lower_shelf = self.tfl.lookupTransform(
            msg.header.frame_id, 'shelf_0_0_0', rospy.Time(0))

        # compute the normal of the shelf facing the camera
        shelf_rot_mat = quaternion_matrix(
            quaternion_inverse(np.array(shelf[1])))
        shelf_normal = np.dot(np.array([0, 0, 1, 0]), shelf_rot_mat)[:3]

        # compute the normal of the upper/lower shelf facing the sky
        upper_lower_shelf_rot_mat = quaternion_matrix(
            quaternion_inverse(np.array(shelf[1])))
        upper_lower_shelf_normal = np.dot(
            np.array([0, 1, 0, 0]), upper_lower_shelf_rot_mat)[:3]

        # Reconstruct spine polygons
        spines = msg.spine_polys
        edge1 = list(zip(spines.polys_x1, spines.polys_y1))
        edge2 = list(zip(spines.polys_x2, spines.polys_y2))
        edge3 = list(zip(spines.polys_x3, spines.polys_y3))
        edge4 = list(zip(spines.polys_x4, spines.polys_y4))
        spine_polys = list(zip(edge1, edge2, edge3, edge4))

        # list of all approximated spine sizes
        spine_sizes = []

        for poly in spine_polys:
            # 1. Approximate the size in metres of each spine detection

            # Compute the rays of each spine edge so we can calucalte
            # the 3d plane using plane/line intersection with the plane from the shelf
            rays = list(map(self.pinhole.projectPixelTo3dRay, poly[:3]))
            points = []
            # Now compute the 3d positions of the vertices
            for ray in rays:
                p = linePlaneCollision(shelf_normal, np.array(
                    shelf[0]), np.array(ray), np.array([0, 0, 0]))
                points.append(np.array(p))

            # approx spine size in metres
            width = np.linalg.norm(points[1] - points[0])
            height = np.linalg.norm(points[1] - points[2])
            size = [width, height]
            spine_sizes.append(size)

            # 2. compute the 3D pose for each spine detection.

            # Get each pixel by coordinate that is inside the spine polygon
            path = mlpath.Path(np.array(poly))
            path_bbox = path.get_extents()
            x_indices = []
            y_indices = []
            for x in range(int(path_bbox.x0), int(path_bbox.x1)):
                for y in range(int(path_bbox.y0), int(path_bbox.y1)):
                    if path.contains_point((x, y)):
                        x_indices.append(int(x))
                        y_indices.append(int(y))

            # Get the average position of points from the point cloud
            # given the 2D pixel coordinates
            pose = self.process_pc(
                xIndices=x_indices, yIndices=y_indices, width=msg.source.width).pose
            pose.orientation.x = shelf[1][0]
            pose.orientation.y = shelf[1][1]
            pose.orientation.z = shelf[1][2]
            pose.orientation.w = shelf[1][3]

            # Filter out nan and compute the offset towards the shelf board so we can
            # Place the books collision box directly on top of the board
            if isfinite(pose.position.x) and isfinite(pose.position.y) and isfinite(pose.position.z):
                stamped = PoseStamped(header=msg.header, pose=pose)
                poses.append(stamped)
                # calculate the offset from the center of the spine the the shelf
                pos = np.array(
                    [pose.position.x, pose.position.y, pose.position.z])
                p_upper = linePlaneCollision(upper_lower_shelf_normal, np.array(
                    upper_shelf[0]), upper_lower_shelf_normal, pos)
                p_lower = linePlaneCollision(upper_lower_shelf_normal, np.array(
                    lower_shelf[0]), upper_lower_shelf_normal, pos)
                offset = np.min([np.linalg.norm(p_upper - pos),
                                 np.linalg.norm(p_lower - pos)])
                offsets.append(offset)
            else:
                poses.append(None)
                offsets.append(None)

        # Monitor the detected poses
        poses_msg = PoseArray()
        poses_msg.header = msg.header
        poses_msg.poses = list(map(lambda x: x.pose, poses))
        self.detected_book_poses.publish(poses_msg)

        # Assign each text recognition to a spine detection and bundle the
        # information to a BookRecognition (yet without database id)
        book_recs = self.extract_books(
            msg, poses, spine_polys, spine_sizes, offsets)
        
        if not book_recs:
            rospy.logwarn("No book recognitions for the current image!")
            return

        # give each book recognition a database id
        books = self.match_books(msg.header, book_recs)
        self.books_pub.publish(books)

        # monitor results with cool images
        self.publish_monitor(monitor_books(
            books, self.spines_dir), self.monitor_books_pub)

    def point(self, p):
        return Point(x=p[0], y=p[1], z=p[2])

    def quat(self, q):
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def cosine_sim(self, a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    def match_books(self, header, book_recs: BookRecognitions) -> Books:
        title_list = self.book_db["title"].tolist()
        num_in_db = len(title_list)
        heights = np.array(self.book_db["height"].tolist())
        depths = np.array(self.book_db["depth"].tolist())
        ratios = heights / depths
        areas = heights * depths
        scores = []

        # All the features as matrix (detection x book_db)
        all_text_scores = []
        all_hue_scores = []
        all_sat_scores = []
        all_val_scores = []
        all_ratio_scores = []
        all_area_scores = []
        all_sift_scores = []

        all_enc_scores = []

        book_rec: BookRecognition
        for book_rec in book_recs.recognitions:

            # text match: check for each text the partial result and take the sum.
            # the score is between 0 and 100 so divide by 100 * num_texts to normalize
            text_recognitions = list(filter(lambda x: len(
                x) > 0, map(lambda x: x.data, book_rec.text)))
            text_scores = []
            for title in title_list:
                sum = 0
                for one_text in text_recognitions:
                    # sum += fuzz.partial_ratio(title, one_text)
                    sum += 1 / (L.distance(title, one_text) + 1)
                # append score for each database entry
                text_scores.append((sum / (len(text_recognitions) + 1)) / 100)
            text_scores = np.array(text_scores)
            if np.sum(text_scores) > 0:
                text_scores /= np.sum(text_scores)

            # HSV match
            # Each channel gets its score using cosine similarity over the histogram
            spine = self.bridge.imgmsg_to_cv2(book_rec.spine)
            spine_hsv = cv2.cvtColor(spine, cv2.COLOR_BGR2HSV)
            spine_rgb = cv2.cvtColor(spine, cv2.COLOR_BGR2RGB)

            hue_score = []
            sat_score = []
            val_score = []

            hist, _ = np.histogram(spine_hsv[:, :, 0], bins=20)
            hist = hist / np.linalg.norm(hist)
            hist_score = list(
                map(lambda x: (self.cosine_sim(x, hist) + 1) / 2, self.hue_hists))
            hist_score = np.array(hist_score) / np.sum(hist_score)
            hue_score = hist_score

            hist, _ = np.histogram(spine_hsv[:, :, 1], bins=20)
            hist = hist / np.linalg.norm(hist)
            hist_score = list(
                map(lambda x: (self.cosine_sim(x, hist) + 1) / 2, self.sat_hists))
            hist_score = np.array(hist_score) / np.sum(hist_score)
            sat_score = hist_score

            hist, _ = np.histogram(spine_hsv[:, :, 2], bins=20)
            hist = hist / np.linalg.norm(hist)
            hist_score = list(
                map(lambda x: (self.cosine_sim(x, hist) + 1) / 2, self.val_hists))
            hist_score = np.array(hist_score) / np.sum(hist_score)
            val_score = hist_score

            # shape match with ratio and area
            height = max(book_rec.spine_size)
            depth = min(book_rec.spine_size)
            ratio = height / depth
            area = height * depth

            ratio_dists = list(map(lambda r: np.linalg.norm(r-ratio), ratios))
            ratio_score = 1 - (ratio_dists / np.linalg.norm(ratio_dists))
            ratio_score /= np.sum(ratio_score)

            area_dists = list(map(lambda a: np.linalg.norm(a-area), areas))
            area_score = 1 - (area_dists / np.linalg.norm(area_dists))
            area_score /= np.sum(area_score)

            # SIFT match
            sift_score = []
            _, des = self.sift.detectAndCompute(spine, None)
            for des_db in self.sift_des_db:
                matches = self.flann.knnMatch(des, des_db, k=2)
                # Need to draw only good matches, so create a mask
                matchesMask = [[0, 0] for i in range(len(matches))]
                match_score = 0
                for i, (m, n) in enumerate(matches):
                    if m.distance < 0.7*n.distance:
                        matchesMask[i] = [1, 0]
                        match_score = match_score + 1
                sift_score.append(match_score)
            sift_score = np.array(sift_score) / (np.sum(sift_score) + 1)

            # image encoding similarity score
            enc_scores = []
            rec_enc = self.get_embedding(spine_rgb)
            for db_enc in self.spine_encodings:
                enc_scores.append(self.cosine_similarity(rec_enc, db_enc))

            # Append individual scores for monitoring
            all_hue_scores.append(hue_score)
            all_sat_scores.append(sat_score)
            all_val_scores.append(val_score)
            all_text_scores.append(text_scores)
            all_ratio_scores.append(ratio_score)
            all_area_scores.append(area_score)
            all_sift_scores.append(sift_score)
            all_enc_scores.append(enc_scores)
            
            # compute final score
            # scores.append(hue_score + 0.2 * sat_score)
            scores.append(enc_scores)

        try:
            # Monitoring the quality of features. Uncomment the scores you weant to inspect.
            #
            # m = monitor_matrix(book_recs, self.spines_dir, all_text_scores, "text")
            # m = monitor_matrix(book_recs, self.spines_dir, all_hue_scores, "hue")
            # m = monitor_matrix(book_recs, self.spines_dir, all_sat_scores, "sat")
            # m = monitor_matrix(book_recs, self.spines_dir, all_val_scores, "val")
            # m = monitor_matrix(book_recs, self.spines_dir, all_ratio_scores, "ratio")
            # m  monitor_matrix(book_recs, self.spines_dir, all_area_scores, "area")
            # m = monitor_matrix(book_recs, self.spines_dir, all_enc_scores, "Encodings")
            m = monitor_matrix(book_recs, self.spines_dir, scores, "Scores")
            self.publish_monitor(m, self.matrix_pub)
        except:
            rospy.logwarn("Could not construct monitor matix")
        
        save_samples(book_recs, scores, self.data_collect_dir)

        # The score matrix now is used to assign each spine detection one datbase id
        scores_left = np.array(scores)  # the list of scores left
        indices = np.array([i + 1 for i in range(num_in_db)])  # the db indices
        match_ids = []
        max_scores = []
        recs_left = list(range(len(book_recs.recognitions)))
        recs_index = []

        # Greedily assign one book id to each spine detection given the scores.
        # 1. Find the highest from scores_left
        # 2. remove the score row in scores_left and scores_rows
        # 3. Assign the book id to the spine with the currently highest score
        # 4. remove the database column so the book cant be selected twice
        for _ in range(min(num_in_db, len(scores))):
            max_score = np.max(scores_left)
            row, col = np.unravel_index(
                np.argmax(scores_left), scores_left.shape)
            index = indices[col]
            # delete the row
            scores_left = np.delete(scores_left, row, axis=0)
            # delete the column
            scores_left = np.delete(scores_left, col, axis=1)
            indices = np.delete(indices, col)
            match_ids.append(index)
            max_scores.append(max_score)
            # remove the recognition index
            recs_index.append(recs_left[row])
            recs_left = np.delete(recs_left, row)

        # Create the message
        books = Books(header=header)
        rec: BookRecognition
        for rec_idx, score, id in zip(recs_index, max_scores, match_ids):
            rec = book_recs.recognitions[rec_idx]
            book = Book()
            book.recognition = rec
            book.database_id = id
            book.confidence = score
            t = self.book_db.loc[self.book_db["id"] == id].iloc[0]["title"]
            book.title = String(t)
            books.books.append(book)

        return books

    def extract_books(self, msg: TextRecognitions, poses, spine_polys, spine_sizes, offsets):
        """ Creates BookRecognitions given the text recognitions and spine information. """

        recognitions = BookRecognitions(header=msg.header)
        recognitions.source = msg.source

        img = self.bridge.imgmsg_to_cv2(msg.source)

        for pose, spine_poly, spine_size, offset in zip(poses, spine_polys, spine_sizes, offsets):
            current_recognition = BookRecognition()
            p = spine_poly
            flattened_poly = p[0] + p[1] + p[2] + p[3]
            current_recognition.polygon = flattened_poly
            spine = self.crop_spine_image(img, spine_poly)
            current_recognition.spine = self.bridge.cv2_to_imgmsg(spine)
            current_recognition.spine.header = msg.header
            current_recognition.hue_histogram = self.get_hue_histogram(
                spine)  # deprecated
            current_recognition.spine_size = spine_size
            current_recognition.text = []  # text fragments
            current_recognition.confidences = []
            current_recognition.sources = []
            current_recognition.offset = offset
            current_recognition.pose = pose

            # Assign the text recognitions to the current recognition
            for text_recognition in msg.recognitions:
                text_polys = list(zip(*[iter(text_recognition.polygon)]*2))

                if (self.tBelongs2s(text_polys, spine_poly)):
                    current_recognition.text.append(text_recognition.text)
                    current_recognition.confidences.append(
                        text_recognition.confidence)
                    current_recognition.sources.append(text_recognition.source)

            recognitions.recognitions.append(current_recognition)

        return recognitions

    def crop_spine_image(self, src, poly):
        """ Crops out the spine using the source image and the spine's polygon. """
        self.temp = self.temp + 1 if hasattr(self, 'temp') else 0
        width = int(np.linalg.norm(poly[0][0] - poly[1][0]))
        height = int(np.linalg.norm(poly[0][1] - poly[2][1]))
        target_rect = np.float32(
            [[0, 0], [width, 0], [width, height], [0, height]])
        pm = cv2.getPerspectiveTransform(np.float32(poly), target_rect)
        newim = cv2.warpPerspective(src, pm, (width, height))
        return newim

    def get_hue_histogram(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hist, _ = np.histogram(hsv[:, :, 0], bins=20)
        return (hist / np.linalg.norm(hist)).tolist()

    def init_db_histograms(self):
        self.hue_hists = []
        self.sat_hists = []
        self.val_hists = []
        for spine in self.spines_db:
            hsv = cv2.cvtColor(spine, cv2.COLOR_BGR2HSV)
            for i, array in enumerate([self.hue_hists, self.sat_hists, self.val_hists]):
                hist, _ = np.histogram(hsv[:, :, i], bins=20)
                hist = hist / np.linalg.norm(hist)
                array.append(hist)

    def publish_monitor(self, img: ImagePIL.Image, pub: Publisher):
        img = cv2.cvtColor(np.uint8(img), cv2.COLOR_RGB2BGR)
        compr = self.bridge.cv2_to_compressed_imgmsg(img)
        pub.publish(compr)
    

    def get_embedding(self, image):
        """ Turns the given image into a ambedding vector of length 512. """
        mapping = self.im_processor.feature_extractor(image)
        tensor_p_values = torch.Tensor(mapping['pixel_values'])
        img_pixel = {"pixel_values": tensor_p_values}
        with torch.no_grad():
            query_embeddings = self.im_enc_model.get_image_features(**img_pixel)
            return query_embeddings

    def cosine_similarity(self, emb_one, emb_two):
        """Computes cosine similarity between two vectors."""
        return torch.nn.functional.cosine_similarity(emb_one, emb_two).numpy()[0]


    def tBelongs2s(self, t_poly, s_poly):
        '''
        t_poly: [(p1),(p2),(p3),(p4)]
        s_poly: [(p1),(p2),(p3),(p4)]
        '''
        offset = 10

        s_min_x = s_poly[0][0] - offset
        s_min_y = s_poly[0][1] - offset
        s_max_x = s_poly[2][0] + offset
        s_max_y = s_poly[2][1] + offset

        t1_x = t_poly[0][0]
        t1_y = t_poly[0][1]
        t3_x = t_poly[2][0]
        t3_y = t_poly[2][1]

        if t1_x >= s_min_x and t1_x <= s_max_x and \
            t1_y >= s_min_y and t1_y <= s_max_y and \
                t3_x >= s_min_x and t3_x <= s_max_x and \
            t3_y >= s_min_y and t3_y <= s_max_y:
            return True

        return False

    def toggle_vision_node(self, flag):
        rospy.loginfo("setting allow perception {}".format(flag.command))
        self.vision_node_on = flag.on
        if not flag.on:
            self.env_reset_pub.publish(Empty)
        resp = ToggleVisionNodeResponse(0)
        return resp


if __name__ == '__main__':
    node = None
    try:
        node = BookExtract()
    except Exception as e:
        loginfo(e)
