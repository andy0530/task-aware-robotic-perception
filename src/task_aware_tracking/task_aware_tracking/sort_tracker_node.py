import json
import math
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# -------------------------
# Geometry helpers
# -------------------------
def iou_xyxy(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0.0, inter_x2 - inter_x1)
    ih = max(0.0, inter_y2 - inter_y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


def xyxy_to_uvsr(b):
    x1, y1, x2, y2 = b
    w = max(1e-6, x2 - x1)
    h = max(1e-6, y2 - y1)
    u = x1 + w / 2.0
    v = y1 + h / 2.0
    s = w * h
    r = w / h
    return u, v, s, r


def uvsr_to_xyxy(u, v, s, r):
    # s = w*h, r = w/h => w = sqrt(s*r), h = sqrt(s/r)
    w = math.sqrt(max(1e-6, s * r))
    h = math.sqrt(max(1e-6, s / max(1e-6, r)))
    x1 = u - w / 2.0
    y1 = v - h / 2.0
    x2 = u + w / 2.0
    y2 = v + h / 2.0
    return [float(x1), float(y1), float(x2), float(y2)]


# -------------------------
# Hungarian Algorithm
# Minimizes cost matrix
# Complexity O(n^3)
# -------------------------
def hungarian(cost: List[List[float]]) -> List[Tuple[int, int]]:
    """
    Returns list of (row, col) assignments minimizing total cost.
    Works on rectangular matrix by padding to square.
    """
    n_rows = len(cost)
    n_cols = len(cost[0]) if n_rows > 0 else 0
    n = max(n_rows, n_cols)

    # Pad to square
    big = 1e6
    C = [[big] * n for _ in range(n)]
    for i in range(n_rows):
        for j in range(n_cols):
            C[i][j] = cost[i][j]

    # Step 1: subtract row minima
    for i in range(n):
        m = min(C[i])
        for j in range(n):
            C[i][j] -= m

    # Step 2: subtract col minima
    for j in range(n):
        m = min(C[i][j] for i in range(n))
        for i in range(n):
            C[i][j] -= m

    # Helpers for starring/priming zeros
    starred = [[False]*n for _ in range(n)]
    primed = [[False]*n for _ in range(n)]
    row_covered = [False]*n
    col_covered = [False]*n

    def find_zero():
        for i in range(n):
            if row_covered[i]:
                continue
            for j in range(n):
                if col_covered[j]:
                    continue
                if abs(C[i][j]) < 1e-12:
                    return i, j
        return None

    def star_in_row(i):
        for j in range(n):
            if starred[i][j]:
                return j
        return None

    def star_in_col(j):
        for i in range(n):
            if starred[i][j]:
                return i
        return None

    def prime_in_row(i):
        for j in range(n):
            if primed[i][j]:
                return j
        return None

    # Step 3: star zeros greedily
    for i in range(n):
        for j in range(n):
            if abs(C[i][j]) < 1e-12 and (not any(starred[i])) and (not any(starred[k][j] for k in range(n))):
                starred[i][j] = True

    # Cover columns containing a starred zero
    for j in range(n):
        if any(starred[i][j] for i in range(n)):
            col_covered[j] = True

    def covered_cols_count():
        return sum(1 for x in col_covered if x)

    # Main loop
    while covered_cols_count() < n:
        z = find_zero()
        while z is None:
            # Step 6: adjust matrix (create new zeros)
            min_uncovered = big
            for i in range(n):
                if row_covered[i]:
                    continue
                for j in range(n):
                    if col_covered[j]:
                        continue
                    min_uncovered = min(min_uncovered, C[i][j])
            if min_uncovered == big:
                min_uncovered = 0.0
            for i in range(n):
                for j in range(n):
                    if row_covered[i]:
                        C[i][j] += min_uncovered
                    if not col_covered[j]:
                        C[i][j] -= min_uncovered
            z = find_zero()

        i, j = z
        primed[i][j] = True
        star_j = star_in_row(i)
        if star_j is not None:
            # cover this row and uncover the starred column
            row_covered[i] = True
            col_covered[star_j] = False
        else:
            # Step 5: augmenting path
            path = [(i, j)]
            col = j
            while True:
                star_i = star_in_col(col)
                if star_i is None:
                    break
                path.append((star_i, col))
                prime_j = prime_in_row(star_i)
                path.append((star_i, prime_j))
                col = prime_j

            # flip stars along the path
            for (pi, pj) in path:
                starred[pi][pj] = not starred[pi][pj]
            # clear covers and primes
            row_covered = [False]*n
            col_covered = [False]*n
            primed = [[False]*n for _ in range(n)]
            # cover columns with starred zeros
            for jj in range(n):
                if any(starred[ii][jj] for ii in range(n)):
                    col_covered[jj] = True

    # Extract assignments for original size
    out = []
    for i in range(n_rows):
        for j in range(n_cols):
            if starred[i][j]:
                out.append((i, j))
    return out


# -------------------------
# Kalman Filter for SORT
# state: [u, v, s, r, du, dv, ds]
# meas:  [u, v, s, r]
# -------------------------
class Kalman:
    def __init__(self):
        # state dim = 7, meas dim = 4
        self.x = [0.0]*7

        # covariance (P), process noise (Q), measurement noise (R)
        self.P = [[0.0]*7 for _ in range(7)]
        self.Q = [[0.0]*7 for _ in range(7)]
        self.R = [[0.0]*4 for _ in range(4)]

        # Initialize with reasonable defaults
        for i in range(7):
            self.P[i][i] = 10.0
            self.Q[i][i] = 1e-2
        for i in range(4):
            self.R[i][i] = 1.0

        # F matrix (constant velocity model on u,v,s)
        # r assumed constant
        self.F = [[0.0]*7 for _ in range(7)]
        for i in range(7):
            self.F[i][i] = 1.0
        self.F[0][4] = 1.0  # u += du
        self.F[1][5] = 1.0  # v += dv
        self.F[2][6] = 1.0  # s += ds

        # H matrix: maps state -> measurement [u,v,s,r]
        self.H = [[0.0]*7 for _ in range(4)]
        self.H[0][0] = 1.0
        self.H[1][1] = 1.0
        self.H[2][2] = 1.0
        self.H[3][3] = 1.0

    @staticmethod
    def matmul(A, B):
        n = len(A)
        m = len(B[0])
        k = len(B)
        out = [[0.0]*m for _ in range(n)]
        for i in range(n):
            for j in range(m):
                s = 0.0
                for t in range(k):
                    s += A[i][t]*B[t][j]
                out[i][j] = s
        return out

    @staticmethod
    def matadd(A, B):
        return [[A[i][j]+B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

    @staticmethod
    def matsub(A, B):
        return [[A[i][j]-B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

    @staticmethod
    def transpose(A):
        return list(map(list, zip(*A)))

    @staticmethod
    def inv4(M):
        # Invert 4x4 with Gauss-Jordan
        n = 4
        A = [[float(M[i][j]) for j in range(n)] for i in range(n)]
        I = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
        for i in range(n):
            # pivot
            pivot = A[i][i]
            if abs(pivot) < 1e-12:
                # find row to swap
                for r in range(i+1, n):
                    if abs(A[r][i]) > 1e-12:
                        A[i], A[r] = A[r], A[i]
                        I[i], I[r] = I[r], I[i]
                        pivot = A[i][i]
                        break
            pivot = A[i][i]
            if abs(pivot) < 1e-12:
                # singular -> return identity-ish fallback
                return [[1.0 if a == b else 0.0 for b in range(n)] for a in range(n)]
            # normalize row
            invp = 1.0 / pivot
            for j in range(n):
                A[i][j] *= invp
                I[i][j] *= invp
            # eliminate
            for r in range(n):
                if r == i:
                    continue
                f = A[r][i]
                for j in range(n):
                    A[r][j] -= f * A[i][j]
                    I[r][j] -= f * I[i][j]
        return I

    def predict(self):
        # x = F x
        x_col = [[v] for v in self.x]
        x_new = self.matmul(self.F, x_col)
        self.x = [x_new[i][0] for i in range(7)]

        # P = F P F^T + Q
        FP = self.matmul(self.F, self.P)
        FPFt = self.matmul(FP, self.transpose(self.F))
        self.P = self.matadd(FPFt, self.Q)

    def update(self, z_uvsr):
        # z: 4x1
        z = [[z_uvsr[0]], [z_uvsr[1]], [z_uvsr[2]], [z_uvsr[3]]]
        x_col = [[v] for v in self.x]

        # y = z - Hx
        Hx = self.matmul(self.H, x_col)
        y = self.matsub(z, Hx)

        # S = HPH^T + R
        HP = self.matmul(self.H, self.P)
        HPHt = self.matmul(HP, self.transpose(self.H))
        S = self.matadd(HPHt, self.R)

        # K = P H^T S^-1
        PHt = self.matmul(self.P, self.transpose(self.H))
        S_inv = self.inv4(S)
        K = self.matmul(PHt, S_inv)  # 7x4

        # x = x + K y
        Ky = self.matmul(K, y)  # 7x1
        self.x = [self.x[i] + Ky[i][0] for i in range(7)]

        # P = (I - K H) P
        I = [[1.0 if i == j else 0.0 for j in range(7)] for i in range(7)]
        KH = self.matmul(K, self.H)  # 7x7
        I_KH = self.matsub(I, KH)
        self.P = self.matmul(I_KH, self.P)


@dataclass
class Detection:
    bbox: List[float]  # xyxy
    conf: float
    cls_id: int
    cls_name: str


class Track:
    _next_id = 1

    def __init__(self, det: Detection):
        self.id = Track._next_id
        Track._next_id += 1

        u, v, s, r = xyxy_to_uvsr(det.bbox)
        self.kf = Kalman()
        self.kf.x[0] = u
        self.kf.x[1] = v
        self.kf.x[2] = s
        self.kf.x[3] = r
        # velocities start at 0

        self.cls_name = det.cls_name
        self.cls_id = det.cls_id
        self.conf = det.conf

        self.hits = 1
        self.age = 1
        self.time_since_update = 0

    def predict(self):
        self.kf.predict()
        self.age += 1
        self.time_since_update += 1

    def update(self, det: Detection):
        u, v, s, r = xyxy_to_uvsr(det.bbox)
        self.kf.update((u, v, s, r))
        self.time_since_update = 0
        self.hits += 1
        # keep latest class/conf
        self.cls_name = det.cls_name
        self.cls_id = det.cls_id
        self.conf = det.conf

    def bbox(self) -> List[float]:
        u, v, s, r = self.kf.x[0], self.kf.x[1], self.kf.x[2], self.kf.x[3]
        return uvsr_to_xyxy(u, v, s, r)


class SortTrackerNode(Node):
    def __init__(self):
        super().__init__("sort_tracker")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("tracks_topic", "/perception/tracks")
        self.declare_parameter("debug_topic", "/perception/tracked_debug_image")
        self.declare_parameter("iou_thresh", 0.3)
        self.declare_parameter("max_age", 10)   # frames allowed without update
        self.declare_parameter("min_hits", 1)   # frames before reporting (keep 1 for now)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.tracks_topic = self.get_parameter("tracks_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value

        self.iou_thresh = float(self.get_parameter("iou_thresh").value)
        self.max_age = int(self.get_parameter("max_age").value)
        self.min_hits = int(self.get_parameter("min_hits").value)

        self.bridge = CvBridge()
        self.latest_image: Optional[Image] = None

        self.tracks: List[Track] = []

        self.sub_det = self.create_subscription(String, self.detections_topic, self.on_detections, 10)
        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.pub_tracks = self.create_publisher(String, self.tracks_topic, 10)
        self.pub_debug = self.create_publisher(Image, self.debug_topic, 10)

        self.get_logger().info(
            f"SORT tracker started.\n"
            f"  detections: {self.detections_topic}\n"
            f"  image:      {self.image_topic}\n"
            f"  outputs:    {self.tracks_topic}, {self.debug_topic}\n"
            f"  iou_thresh={self.iou_thresh} max_age={self.max_age} min_hits={self.min_hits}"
        )

    def on_image(self, msg: Image):
        self.latest_image = msg

    def on_detections(self, msg: String):
        t0 = time.perf_counter()

        # Parse YOLO JSON
        try:
            data = json.loads(msg.data)
            dets_raw = data.get("detections", [])
            stamp = data.get("stamp", None)
            frame_id = data.get("frame_id", "")
        except Exception as e:
            self.get_logger().error(f"Failed to parse detections JSON: {e}")
            return

        detections: List[Detection] = []
        for d in dets_raw:
            try:
                bbox = d["bbox_xyxy"]
                detections.append(
                    Detection(
                        bbox=[float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])],
                        conf=float(d.get("conf", 0.0)),
                        cls_id=int(d.get("class_id", -1)),
                        cls_name=str(d.get("class_name", "unknown")),
                    )
                )
            except Exception:
                continue

        # 1) Predict all tracks forward
        for trk in self.tracks:
            trk.predict()

        # 2) Build IoU cost matrix between predicted tracks and detections
        cost = []
        if self.tracks and detections:
            for trk in self.tracks:
                trk_box = trk.bbox()
                row = []
                for det in detections:
                    row.append(1.0 - iou_xyxy(trk_box, det.bbox))
                cost.append(row)

        matches = []
        unmatched_tracks = list(range(len(self.tracks)))
        unmatched_dets = list(range(len(detections)))

        # 3) Hungarian assignment
        if cost:
            assignments = hungarian(cost)  # list of (track_idx, det_idx)
            for ti, di in assignments:
                # validate with IoU threshold
                trk_box = self.tracks[ti].bbox()
                iou = iou_xyxy(trk_box, detections[di].bbox)
                if iou >= self.iou_thresh:
                    matches.append((ti, di))

            matched_t = {ti for ti, _ in matches}
            matched_d = {di for _, di in matches}
            unmatched_tracks = [i for i in range(len(self.tracks)) if i not in matched_t]
            unmatched_dets = [j for j in range(len(detections)) if j not in matched_d]

        # 4) Update matched tracks
        for ti, di in matches:
            self.tracks[ti].update(detections[di])

        # 5) Create new tracks for unmatched detections
        for di in unmatched_dets:
            self.tracks.append(Track(detections[di]))

        # 6) Prune dead tracks
        alive = []
        for trk in self.tracks:
            if trk.time_since_update <= self.max_age:
                alive.append(trk)
        self.tracks = alive

        # 7) Publish tracks JSON
        out_tracks = []
        for trk in self.tracks:
            if trk.hits < self.min_hits:
                continue
            out_tracks.append({
                "track_id": trk.id,
                "bbox_xyxy": trk.bbox(),
                "class_id": trk.cls_id,
                "class_name": trk.cls_name,
                "conf": trk.conf,
                "hits": trk.hits,
                "age": trk.age,
                "time_since_update": trk.time_since_update,
            })

        payload = {
            "stamp": stamp,
            "frame_id": frame_id,
            "num_tracks": len(out_tracks),
            "tracks": out_tracks,
            "tracker": {
                "type": "SORT",
                "iou_thresh": self.iou_thresh,
                "max_age": self.max_age,
                "min_hits": self.min_hits,
            },
            "runtime_ms": (time.perf_counter() - t0) * 1000.0,
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.pub_tracks.publish(msg_out)

        # 8) Publish overlay image (if we have a recent image)
        if self.latest_image is not None:
            try:
                import cv2
                cv_rgb = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="rgb8")
                cv_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)

                for trk in out_tracks:
                    x1, y1, x2, y2 = [int(v) for v in trk["bbox_xyxy"]]
                    tid = trk["track_id"]
                    label = f"ID:{tid} {trk['class_name']} {trk['conf']:.2f}"
                    cv2.rectangle(cv_bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(cv_bgr, label, (x1, max(0, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                out_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
                out_msg = self.bridge.cv2_to_imgmsg(out_rgb, encoding="rgb8")
                out_msg.header = self.latest_image.header
                self.pub_debug.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f"Overlay publish failed: {e}")


def main():
    rclpy.init()
    node = SortTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
