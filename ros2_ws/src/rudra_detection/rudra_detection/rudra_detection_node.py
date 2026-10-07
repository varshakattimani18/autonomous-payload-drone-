#!/usr/bin/env python3

import ctypes
import json

import cv2
import numpy as np
import tensorrt as trt

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String


# ============================================================
# CONFIGURATION
# ============================================================

ENGINE_PATH = "/home/orin/rfdetr_export/rfdetr-medium.engine"

TARGET_SIZE = (576, 576)

TARGET_CLASS_ID = 0
CONF_THRESHOLD = 0.50


# ============================================================
# CUDA RUNTIME
# ============================================================

libcudart = ctypes.CDLL("libcudart.so")

libcudart.cudaMalloc.argtypes = [
    ctypes.POINTER(ctypes.c_void_p),
    ctypes.c_size_t
]
libcudart.cudaMalloc.restype = ctypes.c_int

libcudart.cudaFree.argtypes = [
    ctypes.c_void_p
]
libcudart.cudaFree.restype = ctypes.c_int

libcudart.cudaMemcpyAsync.argtypes = [
    ctypes.c_void_p,
    ctypes.c_void_p,
    ctypes.c_size_t,
    ctypes.c_int,
    ctypes.c_void_p
]
libcudart.cudaMemcpyAsync.restype = ctypes.c_int

libcudart.cudaStreamCreate.argtypes = [
    ctypes.POINTER(ctypes.c_void_p)
]
libcudart.cudaStreamCreate.restype = ctypes.c_int

libcudart.cudaStreamSynchronize.argtypes = [
    ctypes.c_void_p
]
libcudart.cudaStreamSynchronize.restype = ctypes.c_int

libcudart.cudaEventCreate.argtypes = [
    ctypes.POINTER(ctypes.c_void_p)
]
libcudart.cudaEventCreate.restype = ctypes.c_int

libcudart.cudaEventCreate.restype = ctypes.c_int

libcudart.cudaEventRecord.argtypes = [
    ctypes.c_void_p,
    ctypes.c_void_p
]
libcudart.cudaEventRecord.restype = ctypes.c_int

libcudart.cudaEventElapsedTime.argtypes = [
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_void_p,
    ctypes.c_void_p
]
libcudart.cudaEventElapsedTime.restype = ctypes.c_int

libcudart.cudaEventDestroy.argtypes = [
    ctypes.c_void_p
]
libcudart.cudaEventDestroy.restype = ctypes.c_int


CUDA_HOST_TO_DEVICE = 1
CUDA_DEVICE_TO_HOST = 2


def check_cuda(status):
    if status != 0:
        raise RuntimeError(f"CUDA Runtime Error: {status}")


# ============================================================
# TENSORRT MODEL
# ============================================================

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


class TRTModel:

    def __init__(self, engine_path):

        print("[TRT] Loading TensorRT engine:")
        print(f"[TRT] {engine_path}")

        self.runtime = trt.Runtime(TRT_LOGGER)

        with open(engine_path, "rb") as f:
            engine_data = f.read()

        self.engine = self.runtime.deserialize_cuda_engine(
            engine_data
        )

        if self.engine is None:
            raise RuntimeError(
                "Failed to load TensorRT engine"
            )

        self.context = self.engine.create_execution_context()

        if self.context is None:
            raise RuntimeError(
                "Failed to create TensorRT context"
            )

        self.stream = ctypes.c_void_p()

        check_cuda(
            libcudart.cudaStreamCreate(
                ctypes.byref(self.stream)
            )
        )

        self.input_names = []
        self.output_names = []
        self.buffers = {}

        # ----------------------------------------------------
        # FIND INPUT / OUTPUT TENSORS
        # ----------------------------------------------------

        for i in range(self.engine.num_io_tensors):

            name = self.engine.get_tensor_name(i)

            shape = tuple(
                self.engine.get_tensor_shape(name)
            )

            dtype = trt.nptype(
                self.engine.get_tensor_dtype(name)
            )

            mode = self.engine.get_tensor_mode(name)

            if mode == trt.TensorIOMode.INPUT:

                self.input_names.append(name)

            else:

                self.output_names.append(name)

        if not self.input_names:
            raise RuntimeError(
                "TensorRT engine has no input tensor"
            )

        self.input_name = self.input_names[0]

        # ----------------------------------------------------
        # INPUT SHAPE
        # ----------------------------------------------------

        input_shape = tuple(
            self.engine.get_tensor_shape(
                self.input_name
            )
        )

        if any(x < 0 for x in input_shape):

            self.context.set_input_shape(
                self.input_name,
                (
                    1,
                    3,
                    TARGET_SIZE[1],
                    TARGET_SIZE[0]
                )
            )

        self.input_shape = tuple(
            self.context.get_tensor_shape(
                self.input_name
            )
        )

        self.input_dtype = trt.nptype(
            self.engine.get_tensor_dtype(
                self.input_name
            )
        )

        # ----------------------------------------------------
        # INPUT BUFFER
        # ----------------------------------------------------

        input_elements = int(
            np.prod(self.input_shape)
        )

        input_bytes = (
            input_elements
            * np.dtype(self.input_dtype).itemsize
        )

        self.input_host = np.empty(
            input_elements,
            dtype=self.input_dtype
        )

        self.input_device = ctypes.c_void_p()

        check_cuda(
            libcudart.cudaMalloc(
                ctypes.byref(self.input_device),
                input_bytes
            )
        )

        # ----------------------------------------------------
        # OUTPUT BUFFERS
        # ----------------------------------------------------

        for name in self.output_names:

            shape = tuple(
                self.context.get_tensor_shape(name)
            )

            dtype = trt.nptype(
                self.engine.get_tensor_dtype(name)
            )

            nbytes = (
                int(np.prod(shape))
                * np.dtype(dtype).itemsize
            )

            device_ptr = ctypes.c_void_p()

            check_cuda(
                libcudart.cudaMalloc(
                    ctypes.byref(device_ptr),
                    nbytes
                )
            )

            self.buffers[name] = {
                "shape": shape,
                "dtype": dtype,
                "bytes": nbytes,
                "device": device_ptr,
            }

        # ----------------------------------------------------
        # CONNECT TENSOR ADDRESSES
        # ----------------------------------------------------

        self.context.set_tensor_address(
            self.input_name,
            self.input_device.value
        )

        for name in self.output_names:

            self.context.set_tensor_address(
                name,
                self.buffers[name]["device"].value
            )

        print("[TRT] Input:", self.input_name)
        print("[TRT] Input shape:", self.input_shape)
        print("[TRT] Input dtype:", self.input_dtype)
        print("[TRT] Outputs:", self.output_names)

    # --------------------------------------------------------
    # INFERENCE
    # --------------------------------------------------------

    def infer(self, input_tensor):

        np.copyto(
            self.input_host,
            input_tensor.ravel()
        )

        start_event = ctypes.c_void_p()
        end_event = ctypes.c_void_p()

        check_cuda(
            libcudart.cudaEventCreate(
                ctypes.byref(start_event)
            )
        )

        check_cuda(
            libcudart.cudaEventCreate(
                ctypes.byref(end_event)
            )
        )

        # GPU start timestamp

        check_cuda(
            libcudart.cudaEventRecord(
                start_event,
                self.stream
            )
        )

        # ----------------------------------------------------
        # HOST -> DEVICE
        # ----------------------------------------------------

        check_cuda(
            libcudart.cudaMemcpyAsync(
                self.input_device,
                self.input_host.ctypes.data,
                self.input_host.nbytes,
                CUDA_HOST_TO_DEVICE,
                self.stream
            )
        )

        # ----------------------------------------------------
        # TENSORRT EXECUTION
        # ----------------------------------------------------

        success = self.context.execute_async_v3(
            stream_handle=self.stream.value
        )

        if not success:
            raise RuntimeError(
                "TensorRT execute_async_v3 failed"
            )

        # ----------------------------------------------------
        # DEVICE -> HOST
        # ----------------------------------------------------

        outputs = {}

        for name in self.output_names:

            info = self.buffers[name]

            output = np.empty(
                info["shape"],
                dtype=info["dtype"]
            )

            check_cuda(
                libcudart.cudaMemcpyAsync(
                    output.ctypes.data,
                    info["device"],
                    info["bytes"],
                    CUDA_DEVICE_TO_HOST,
                    self.stream
                )
            )

            outputs[name] = output

        check_cuda(
            libcudart.cudaEventRecord(
                end_event,
                self.stream
            )
        )

        check_cuda(
            libcudart.cudaStreamSynchronize(
                self.stream
            )
        )

        elapsed_ms = ctypes.c_float()

        check_cuda(
            libcudart.cudaEventElapsedTime(
                ctypes.byref(elapsed_ms),
                start_event,
                end_event
            )
        )

        libcudart.cudaEventDestroy(
            start_event
        )

        libcudart.cudaEventDestroy(
            end_event
        )

        return outputs, float(
            elapsed_ms.value
        )

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------

    def close(self):

        try:
            if self.input_device:
                libcudart.cudaFree(
                    self.input_device
                )
        except Exception:
            pass

        for info in self.buffers.values():

            try:
                libcudart.cudaFree(
                    info["device"]
                )
            except Exception:
                pass


# ============================================================
# PREPROCESS
# ============================================================

MEAN = np.array(
    [0.485, 0.456, 0.406],
    dtype=np.float32
)

STD = np.array(
    [0.229, 0.224, 0.225],
    dtype=np.float32
)


def preprocess(frame):

    # BGR -> RGB

    rgb = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB
    )

    # Resize

    resized = cv2.resize(
        rgb,
        TARGET_SIZE,
        interpolation=cv2.INTER_LINEAR
    )

    # float32 / 255

    tensor = (
        resized.astype(np.float32)
        / 255.0
    )

    # Normalize

    tensor = (
        tensor - MEAN
    ) / STD

    # HWC -> CHW

    tensor = np.transpose(
        tensor,
        (2, 0, 1)
    )

    # Add batch dimension

    tensor = np.expand_dims(
        tensor,
        axis=0
    )

    return np.ascontiguousarray(
        tensor
    )


# ============================================================
# POSTPROCESS
# ============================================================

def sigmoid(x):

    x = np.clip(
        x,
        -80,
        80
    )

    return 1.0 / (
        1.0 + np.exp(-x)
    )


def box_to_xyxy(
    box,
    frame_width,
    frame_height
):

    # Normalized coordinates

    if np.max(box) <= 1.05:

        # cx, cy, w, h

        if (
            box[2] < box[0]
            or box[3] < box[1]
        ):

            cx, cy, w, h = box

            return (
                int(
                    (cx - w / 2)
                    * frame_width
                ),
                int(
                    (cy - h / 2)
                    * frame_height
                ),
                int(
                    (cx + w / 2)
                    * frame_width
                ),
                int(
                    (cy + h / 2)
                    * frame_height
                )
            )

        return (
            int(box[0] * frame_width),
            int(box[1] * frame_height),
            int(box[2] * frame_width),
            int(box[3] * frame_height)
        )

    # Pixel coordinates relative to
    # the 576x576 TensorRT input

    sx = (
        frame_width
        / TARGET_SIZE[0]
    )

    sy = (
        frame_height
        / TARGET_SIZE[1]
    )

    return (
        int(box[0] * sx),
        int(box[1] * sy),
        int(box[2] * sx),
        int(box[3] * sy)
    )


def postprocess(
    outputs,
    frame_width,
    frame_height
):

    boxes_out = outputs["dets"][0]

    scores_out = outputs["labels"][0]

    # Preserve original score handling

    if (
        np.max(scores_out) > 1.0
        or np.min(scores_out) < 0.0
    ):

        scores = sigmoid(
            scores_out
        )

    else:

        scores = scores_out

    class_scores = scores[
        :,
        TARGET_CLASS_ID
    ]

    valid_indices = np.where(
        class_scores >= CONF_THRESHOLD
    )[0]

    detections = []

    for idx in valid_indices:

        score = float(
            class_scores[idx]
        )

        box = boxes_out[idx].astype(
            np.float32
        )

        x1, y1, x2, y2 = box_to_xyxy(
            box,
            frame_width,
            frame_height
        )

        # Clamp

        x1 = max(
            0,
            min(x1, frame_width - 1)
        )

        y1 = max(
            0,
            min(y1, frame_height - 1)
        )

        x2 = max(
            0,
            min(x2, frame_width - 1)
        )

        y2 = max(
            0,
            min(y2, frame_height - 1)
        )

        if x2 <= x1 or y2 <= y1:
            continue

        detections.append(
            {
                "class_id": TARGET_CLASS_ID,
                "confidence": score,
                "bbox": [
                    x1,
                    y1,
                    x2,
                    y2
                ],
                "u": (
                    x1 + x2
                ) / 2.0,
                "v": (
                    y1 + y2
                ) / 2.0,
                "geo": None,
                "person_no": None,
                "is_new": False,
                "sent": False,
            }
        )

    return detections


# ============================================================
# ROS 2 NODE
# ============================================================

class RudraDetectionNode(Node):

    def __init__(self):

        super().__init__(
            "rudra_detection_node"
        )

        self.get_logger().info(
            "Starting Rudra RF-DETR detection node"
        )

        self.get_logger().info(
            f"Engine: {ENGINE_PATH}"
        )

        self.get_logger().info(
            f"Input: {TARGET_SIZE}"
        )

        self.get_logger().info(
            f"Class: {TARGET_CLASS_ID}"
        )

        self.get_logger().info(
            f"Confidence: {CONF_THRESHOLD}"
        )

        # ----------------------------------------------------
        # LOAD MODEL
        # ----------------------------------------------------

        self.model = TRTModel(
            ENGINE_PATH
        )

        # ----------------------------------------------------
        # WARMUP
        # ----------------------------------------------------

        self.get_logger().info(
            "Warming up TensorRT..."
        )

        dummy = np.zeros(
            (
                1,
                3,
                TARGET_SIZE[1],
                TARGET_SIZE[0]
            ),
            dtype=self.model.input_dtype
        )

        for _ in range(3):

            self.model.infer(
                dummy
            )

        self.get_logger().info(
            "TensorRT warmup complete"
        )

        # ----------------------------------------------------
        # ROS SUBSCRIBER
        # ----------------------------------------------------

        self.image_sub = self.create_subscription(
            Image,
            "/rudra/camera/image_raw",
            self.image_callback,
            10
        )

        # ----------------------------------------------------
        # ROS PUBLISHER
        # ----------------------------------------------------

        self.detection_pub = self.create_publisher(
            String,
            "/rudra/detections",
            10
        )

        self.frame_count = 0
        self.detection_count = 0

        self.get_logger().info(
            "Subscribed: /rudra/camera/image_raw"
        )

        self.get_logger().info(
            "Publishing: /rudra/detections"
        )

    # --------------------------------------------------------
    # IMAGE CALLBACK
    # --------------------------------------------------------

    def image_callback(self, msg):

        try:

            # ------------------------------------------------
            # ROS IMAGE -> NUMPY
            # ------------------------------------------------

            if msg.encoding != "bgr8":

                self.get_logger().warn(
                    f"Unexpected image encoding: "
                    f"{msg.encoding}"
                )

                return

            frame = np.frombuffer(
                msg.data,
                dtype=np.uint8
            ).reshape(
                (
                    msg.height,
                    msg.width,
                    3
                )
            )

            frame = np.ascontiguousarray(
                frame
            )

            self.frame_count += 1

            # ------------------------------------------------
            # PREPROCESS
            # ------------------------------------------------

            input_tensor = preprocess(
                frame
            )

            # ------------------------------------------------
            # INFERENCE
            # ------------------------------------------------

            outputs, inference_ms = (
                self.model.infer(
                    input_tensor
                )
            )

            # ------------------------------------------------
            # POSTPROCESS
            # ------------------------------------------------

            detections = postprocess(
                outputs,
                msg.width,
                msg.height
            )

            self.detection_count += len(
                detections
            )

            # ------------------------------------------------
            # ROS MESSAGE
            # ------------------------------------------------

            result = {
                "frame_id": msg.header.frame_id,
                "stamp_sec": (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec
                    / 1e9
                ),
                "frame_width": msg.width,
                "frame_height": msg.height,
                "inference_ms": inference_ms,
                "count": len(detections),
                "detections": detections,
            }

            ros_msg = String()

            ros_msg.data = json.dumps(
                result
            )

            self.detection_pub.publish(
                ros_msg
            )

            # ------------------------------------------------
            # LOG
            # ------------------------------------------------

            self.get_logger().info(
                f"Frame={self.frame_count} | "
                f"detections={len(detections)} | "
                f"TensorRT={inference_ms:.2f} ms"
            )

        except Exception as e:

            self.get_logger().error(
                f"Detection error: {e}"
            )

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------

    def destroy_node(self):

        try:

            self.get_logger().info(
                "Releasing TensorRT resources..."
            )

            self.model.close()

        except Exception as e:

            print(
                f"[TRT] Cleanup error: {e}"
            )

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = RudraDetectionNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f"[FATAL] {e}"
        )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
