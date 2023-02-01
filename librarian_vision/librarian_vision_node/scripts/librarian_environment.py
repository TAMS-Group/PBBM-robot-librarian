#!/usr/bin/env python3

import numpy as np
import pandas as pd
import rospkg
import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose, Point, PoseStamped, TransformStamped
from librarian_resources.msg import Book, BookRecognition, Books, Environment, VisionControl
from librarian_resources.srv import EnvironmentService
from moveit_commander import PlanningSceneInterface
from rospy import Header, loginfo
from tf.transformations import quaternion_inverse, quaternion_matrix, quaternion_about_axis, quaternion_multiply
from visualization_msgs.msg import Marker, MarkerArray
import time
from pyclustering.cluster.kmeans import kmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
import pathlib
import cv2
from cv_bridge import CvBridge


class BookVector:
    """ This class represents all the important information we need for a book.
        It is used for single observations as well as the accumulated version. """
    
    def __init__(self, id: int, pose: Pose, width: float, height: float, offset: float, rec: BookRecognition, conf: float):
        self.id = id
        self.pose = pose
        self.width = width
        self.height = height
        self.depth = height
        self.offset = offset
        self.rec = rec
        self.confidence = conf
        self.books = []

    def pos(self):
        return [self.pose.position.x, self.pose.position.y, self.pose.position.z]

    def __repr__(self) -> str:
        pos = self.pose.position
        pos_format = f"[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]"
        return f"\nBV[ id: {self.id:02}, pos: {pos_format}, w: {self.width:.3f}, h: {self.height:.3f}]"


class LibrarianEnvironment:
    """ The environment receives multiple book observations to accumulate them into one with less noise.
        Using k-means, the individual observations are clustered spatially into books.
        Then, greedily assign the most likely id to a cluster and remove it from possible ids so each
        id only gets assigned once.
    """

    def __init__(self) -> None:
        self.enabled = True
        rospy.init_node("librarian_environment", anonymous=True)

        # Subscribe to book observations to accumulate the,
        rospy.Subscriber("/librarian/books",
                         Books, self.receive_books)
        
        rospy.Subscriber("/librarian/reset_env",  # additionaly set the dict to 0 when the service sends the reset message
                         Empty, self.reset_env_dict)
        
        # Turns on/off the vision pipeline. It hinders this node from publishing environemnts
        # (even if new observation arrive).
        rospy.Subscriber("/librarian/control/vision",
                         VisionControl, self.receive_control)

        self.scene = PlanningSceneInterface(synchronous=True)
        self.environment_pub = rospy.Publisher(
            "/librarian/environment", Environment, queue_size=1)
        self.marker_pub = rospy.Publisher(
            '/librarian/marker_array', MarkerArray, queue_size=1)

        # get the cvs file from librarian common
        resources = rospkg.RosPack().get_path("librarian_resources")
        self.book_db = pd.read_csv(f"{resources}/books.csv")
        self.book_confidences = np.zeros(len(self.book_db))
        self.environment_service = rospy.Service(
            'environment_service',  EnvironmentService,  self.make_env_msg)

        self.data_out = f"{resources}/data_accumulated"
        self.bridge = CvBridge()
        self.num_collected = 0

        self.book_vector_sequence = []
        self.num_samples = 10  # the number of messages we compute the environment on
        self.cluster_threshold = 4  # at least two samples in one cluster to be approaved
        rospy.spin()

    def receive_control(self, msg: VisionControl):
        self.enabled = msg.enabled
        if self.enabled:
            # reset observations once the pipeline is enabled again
            self.book_vector_sequence = []

    def receive_books(self, msg: Books):
        """ When receiving book recognitions we transform each book to a BookVector and append all
            to the book_vector_sequence list for later processing.
        """

        if not self.enabled:
            return

        book_vectors = []
        book: Book
        for book in msg.books:
            rec: BookRecognition = book.recognition
            vector = BookVector(
                book.database_id, rec.pose.pose, rec.spine_size[0], rec.spine_size[1], rec.offset, rec, book.confidence)
            book_vectors.append(vector)
        self.book_vector_sequence.append(book_vectors)
        loginfo(f"Books sequence length: {len(self.book_vector_sequence)}")

        # When the threshold is exceeded, we compute the environment and publish the marker/collision objects
        if len(self.book_vector_sequence) >= self.num_samples:
            env = self.prepare_env()
            self.publish_env(env, msg.header)
            self.publish_marker(env, msg.header)
            self.book_vector_sequence = []

    def prepare_env(self):
        """ Computes the environment based on a sequence of book detections. """

        # Determine the max number of detected books in one time step
        num_centroids = np.max(
            list(map(lambda x: len(x), self.book_vector_sequence)))

        # flatten the positions / book vectors
        seqf = []
        booksf = []
        for s in self.book_vector_sequence:
            seqf.extend(list(map(lambda x: x.pos(), s)))
            booksf.extend(s)

        # initialize the centroids
        centroids = kmeans_plusplus_initializer(
            seqf, num_centroids).initialize()
        # start kmeans
        instance = kmeans(seqf, centroids)
        instance.process()
        clusters = instance.get_clusters()

        # purge too small cluster
        clusters = list(filter(lambda x: len(
            x) >= self.cluster_threshold, clusters))

        # create a combined bookVector from each cluster
        bookVecs = self.combine_cluster_to_books(clusters, booksf)
        self.collect_data(bookVecs)

        # Edit the pose so the book 'stands' on the shelf and not intersects it
        book: BookVector
        for book in bookVecs:
            row = self.book_db[self.book_db["id"] == book.id]
            height = row["height"].values[0]

            # add small extra offset to depth to avoid crashing the books into the shelf
            book.depth = row["width"].values[0] + 0.02  

            # rotate the book to match it with the shelf
            q = book.pose.orientation
            mat = quaternion_matrix(quaternion_inverse(
                np.array([q.x, q.y, q.z, q.w])))

            # The book should 'stand' on the shelf board so apply this offset.
            # It is the difference between the measured / approximanted height and the database height.
            height_offset = (book.height - height) / 2

            # The offset now consideres the correct height + 5mm offset to not collide with the shelf
            offset_v = np.array(
                [0, (book.height / 2) - book.offset - height_offset + 0.005, -book.depth / 2, 0])
            offset = np.dot(offset_v, mat)
            book.pose.position.x += offset[0]
            book.pose.position.y += offset[1]
            book.pose.position.z += offset[2]

        return bookVecs

    def collect_data(self, books):
        if self.num_collected >= 10:
            return
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        out = f"{self.data_out}/{stamp}"

        book: BookVector
        for book in books:
            outdir = f"{out}/{book.id}"
            pathlib.Path(outdir).mkdir(parents=True, exist_ok=True)
            for i, bookvec in enumerate(book.books):
                im = self.bridge.imgmsg_to_cv2(bookvec.rec.spine)
                cv2.imwrite(f"{outdir}/{i}.jpg", im)
        
        self.num_collected += 1
        rospy.loginfo(f"Collected environement data {self.num_collected}/10 in {out}")


    def combine_cluster_to_books(self, clusters, booksf):
        bookVecs = []

        # determine id using max number of same ids
        cluster_with_ids = []
        for c in clusters:
            books = list(map(lambda i: booksf[i], c))
            ids = {}
            b: BookVector
            for b in books:
                ids[b.id] = ids.setdefault(b.id, 0) + 1
            cluster_with_ids.append([c, ids])

        # assign each cluster the most likely id
        cluster_with_id = []
        for _ in range(len(cluster_with_ids)):
            # find biggest number of ids that haven't beed assigned yet
            max_values = [max(id.values()) for _, id in cluster_with_ids]
            if max(max_values) == -1:
                continue
            index = np.argmax(max_values)
            cluster = cluster_with_ids[index]
            cluster_with_ids.remove(cluster)
            value_index = np.argmax(list(cluster[1].values()))
            id = list(cluster[1].keys())[value_index]
            cluster_with_id.append([cluster[0], id])
            # remove the id from remaining cluster so it cannot be assigned again
            ids: dict
            for _, ids in cluster_with_ids:
                ids[id] = -1

        # Turn a cluster inro a single book
        for c, id in cluster_with_id:
            books = list(map(lambda i: booksf[i], c))

            # average the position (orientation should be all the same)
            positions = np.array(list(map(lambda x: x.pos(), books)))
            mean = np.mean(positions.T, axis=1)
            pose = Pose(position=Point(
                x=mean[0], y=mean[1], z=mean[2]), orientation=books[0].pose.orientation)
            # average the estimated width/height/offset as well
            width = np.mean(list(map(lambda x: x.width, books)))
            height = np.mean(list(map(lambda x: x.height, books)))
            offset = np.mean(list(map(lambda x: x.offset, books)))

            # The confidence is the sum of condifences where the id matched 
            # divided by length of cluster to avoid over-confidence on large cluster
            conf = np.sum([b.confidence for b in books if b.id == id])
            conf /= len(books)

            book = BookVector(id, pose, width,
                              height, offset, books[0].rec, conf)
            book.books = books
            bookVecs.append(book)
        return bookVecs

    def publish_marker(self, books, header: Header):
        """ Published the given list of bookVectors based on the properties on 
            the corresponding database entry for each book. """
        self.clear_books()
        marker_array = MarkerArray()
        marker_array.markers = []

        book: BookVector
        for book in books:
            row = self.book_db[self.book_db["id"] == book.id]
            height = row["height"].values[0]
            width = row["depth"].values[0]

            pose = PoseStamped(pose=book.pose)
            pose.header.frame_id = header.frame_id  # "/base_footprint"

            o = pose.pose.orientation
            q = np.array([o.x, o.y, o.z, o.w])
            rot1 = quaternion_about_axis(-np.pi / 2, (1, 0, 0))
            rot2 = quaternion_about_axis(np.pi / 2, (0, 0, 1))
            q = quaternion_multiply(q, rot1)
            q = quaternion_multiply(q, rot2)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            self.scene.add_box(
                f"book-{book.id}", pose, (book.depth, width - 0.0006, height))

            marker = self.create_marker(
                book, pose.pose, (width * 0.5, height, book.depth), header)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_env(self, books, header: Header):
        book_msgs = []
        book: BookVector
        for book in books:
            msg = Book()
            msg.confidence = book.confidence
            msg.database_id = book.id
            msg.recognition = book.rec
            row = self.book_db[self.book_db["id"] == book.id]
            msg.title = String(data=row["title"].values[0])
            book_msgs.append(msg)

        env = Environment()
        env.books = book_msgs
        env.header = header
        self.environment_pub.publish(env)

    def create_marker(self, book: BookVector, pose: Pose, size, header):
        marker = Marker(header=header)
        marker.header.stamp = rospy.Time(time.time())
        marker.id = book.id
        marker.ns = 'books'
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.lifetime = rospy.Duration(10)
        marker.pose = pose
        marker.pose.position.y *= 0.87
        marker.pose.position.z *= 0.82
        marker.text = str(book.id)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker

    def clear_books(self):
        all_names = self.scene.get_known_object_names()
        book_names = list(filter(lambda x: x.startswith("book-"), all_names))
        for name in book_names:
            self.scene.remove_world_object(name)

    def reset_env_dict(self):
        self.env_dict = {}

    def make_env_msg(self):
        rospy.loginfo("updating env repr")
        env_msg = Environment()
        env_msg.books = [v["msg"] for _, v in self.env_dict.items()]
        env_msg.header.stamp = rospy.Time.now()
        return env_msg


if __name__ == '__main__':
    LibrarianEnvironment()
