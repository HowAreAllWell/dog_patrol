"""TensorRT YOLO face detector used by the production face verifier.

Adapted from the standalone face pipeline (``trt_infer.py``). TensorRT and
PyCUDA are imported lazily so that ROS/unit-test environments without a GPU can
still import and exercise the non-GPU parts of this package.
"""

from __future__ import annotations

import numpy as np

_PREPROC_CUDA_CODE = r"""
__global__ void preprocess_kernel(
    const unsigned char* __restrict__ src,
    int src_h, int src_w,
    float* __restrict__ dst,
    int dst_h, int dst_w,
    float scale, float pad_x, float pad_y,
    int nw, int nh
) {
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    float pad_val = 114.0f / 255.0f;

    if (dx < pad_x || dx >= pad_x + nw || dy < pad_y || dy >= pad_y + nh) {
        dst[dy * dst_w + dx]                    = pad_val;
        dst[dst_h * dst_w + dy * dst_w + dx]    = pad_val;
        dst[2 * dst_h * dst_w + dy * dst_w + dx]= pad_val;
        return;
    }

    float rx = dx - pad_x;
    float ry = dy - pad_y;
    float sx = (rx + 0.5f) * (float)src_w / (float)nw - 0.5f;
    float sy = (ry + 0.5f) * (float)src_h / (float)nh - 0.5f;

    sx = max(0.0f, min(sx, (float)src_w - 1.0f));
    sy = max(0.0f, min(sy, (float)src_h - 1.0f));

    int x0 = (int)sx, y0 = (int)sy;
    int x1 = min(x0 + 1, src_w - 1);
    int y1 = min(y0 + 1, src_h - 1);
    float fx = sx - x0, fy = sy - y0;

    float w00 = (1.0f - fy) * (1.0f - fx);
    float w01 = (1.0f - fy) * fx;
    float w10 = fy * (1.0f - fx);
    float w11 = fy * fx;

    int stride = src_w * 3;
#define AT(y,x,c) src[(y)*stride + (x)*3 + (c)]

    float b = w00*AT(y0,x0,0) + w01*AT(y0,x1,0) + w10*AT(y1,x0,0) + w11*AT(y1,x1,0);
    float g = w00*AT(y0,x0,1) + w01*AT(y0,x1,1) + w10*AT(y1,x0,1) + w11*AT(y1,x1,1);
    float r = w00*AT(y0,x0,2) + w01*AT(y0,x1,2) + w10*AT(y1,x0,2) + w11*AT(y1,x1,2);
#undef AT

    float inv = 1.0f / 255.0f;
    dst[dy * dst_w + dx]                    = r * inv;
    dst[dst_h * dst_w + dy * dst_w + dx]    = g * inv;
    dst[2 * dst_h * dst_w + dy * dst_w + dx]= b * inv;
}
"""


def ensure_cuda_context():
    """Create and push a PyCUDA context for the calling thread, if none is current.

    PyCUDA contexts are thread-local: CUDA/TensorRT calls from a thread that
    never created a context fail with "invalid device context". The face
    provider runs inference on its own worker thread, so it calls this there
    before the first build/use of the GPU verifier. Returns the pushed context
    for later cleanup, or ``None`` when the current thread already has one or
    when no CUDA runtime is available (e.g. CI).
    """
    try:
        import pycuda.driver as cuda
    except (ImportError, OSError):
        return None
    try:
        cuda.init()
        if cuda.Context.get_current() is not None:
            return None
        return cuda.Device(0).make_context()
    except cuda.Error:
        return None


def scale_back_boxes(boxes, pad_info):
    scale, dw, dh, w0, h0 = pad_info
    boxes = boxes.copy()
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dw) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dh) / scale
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w0)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h0)
    return boxes


def scale_back_kpts(kpts, pad_info):
    scale, dw, dh, w0, h0 = pad_info
    kpts = kpts.copy()
    kpts[:, 0] = (kpts[:, 0] - dw) / scale
    kpts[:, 1] = (kpts[:, 1] - dh) / scale
    kpts[:, 0] = kpts[:, 0].clip(0, w0)
    kpts[:, 1] = kpts[:, 1].clip(0, h0)
    return kpts


class TRTInference:
    """TensorRT YOLO detector; only the ``face`` class is kept for output."""

    KEEP_CLS = {1}

    def __init__(self, engine_path, conf_thres=0.25, iou_thres=0.5):
        import tensorrt as trt

        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self._trt_logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(self._trt_logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = None
        self._raw_dev_buf = None
        self._last_raw_size = 0
        self._preproc_module = None
        self._preproc_kernel = None
        self._setup_bindings()

    def _setup_bindings(self):
        import pycuda.driver as cuda

        self.stream = cuda.Stream()
        self.bindings = []
        self.inputs = []
        self.outputs = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = self._trt_nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)
            size = int(np.prod(shape))
            host = cuda.pagelocked_empty(size, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.bindings.append(int(device))
            if self.engine.get_tensor_mode(name) == self._trt_tensor_input_mode():
                self.inputs.append(
                    {"name": name, "host": host, "device": device, "shape": shape}
                )
            else:
                self.outputs.append(
                    {"name": name, "host": host, "device": device, "shape": shape}
                )

    @staticmethod
    def _trt_nptype(dtype):
        import tensorrt as trt

        return trt.nptype(dtype)

    @staticmethod
    def _trt_tensor_input_mode():
        import tensorrt as trt

        return trt.TensorIOMode.INPUT

    @staticmethod
    def nms(boxes, scores, iou_thresh=0.5):
        order = np.argsort(-scores)
        keep = []
        while len(order) > 0:
            i = order[0]
            keep.append(i)
            if len(order) == 1:
                break
            b1 = boxes[i]
            xx1 = np.maximum(b1[0], boxes[order[1:], 0])
            yy1 = np.maximum(b1[1], boxes[order[1:], 1])
            xx2 = np.minimum(b1[2], boxes[order[1:], 2])
            yy2 = np.minimum(b1[3], boxes[order[1:], 3])
            w = np.maximum(0, xx2 - xx1)
            h = np.maximum(0, yy2 - yy1)
            inter = w * h
            area1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
            area2 = (
                (boxes[order[1:], 2] - boxes[order[1:], 0])
                * (boxes[order[1:], 3] - boxes[order[1:], 1])
            )
            iou = inter / (area1 + area2 - inter + 1e-7)
            order = order[1:][iou <= iou_thresh]
        return np.array(keep)

    def postprocess(self, raw_out, pad_info):
        cls_names = ["person", "face", "vehicle", "animal", "weapon"]
        attr_names = [
            "male", "female", "child", "teen", "adult", "senior",
            "hat", "mask", "glasses", "facial_hair", "long_hair", "bald",
            "bag", "phone", "uniform", "long_sleeves", "shorts", "coat",
            "hoodie", "bright", "hi_vis", "pattern", "logo", "heavy_build",
            "lying", "threatening", "running", "fighting", "behind_camera",
            "weapon", "weapon_hands", "tattoos", "smoking", "top_white",
            "top_black", "top_blue", "top_green", "top_red", "top_orange",
            "bottom_white", "bottom_black", "bottom_blue", "bottom_green",
            "bottom_red", "bottom_orange",
        ]
        output_key = list(raw_out.keys())[0]
        dets = raw_out[output_key][0]
        scores = dets[:, 4]
        cls_ids = dets[:, 5].astype(int)
        mask = (scores > self.conf_thres) & np.isin(cls_ids, list(self.KEEP_CLS))
        dets = dets[mask]
        scores = scores[mask]
        cls_ids = cls_ids[mask]

        if len(dets) == 0:
            return []

        keep = self.nms(dets[:, :4], scores, self.iou_thres)
        dets = dets[keep]
        scores = scores[keep]
        cls_ids = cls_ids[keep]

        boxes_640 = dets[:, :4]
        attrs_raw = 1.0 / (1.0 + np.exp(-dets[:, 6:56]))
        kpts_raw = dets[:, 56:]

        boxes = scale_back_boxes(boxes_640, pad_info)

        results = []
        for j in range(len(dets)):
            kpts_j = np.zeros((22, 3))
            n_kpt_vals = kpts_raw.shape[1]
            n_kpts = n_kpt_vals // 3
            for k in range(min(n_kpts, 22)):
                kpts_j[k, 0] = kpts_raw[j, k * 3]
                kpts_j[k, 1] = kpts_raw[j, k * 3 + 1]
                kpts_j[k, 2] = kpts_raw[j, k * 3 + 2]
            kpts_j = scale_back_kpts(kpts_j[:n_kpts], pad_info)
            attrs_j = {}
            for ai in range(len(attr_names)):
                if attrs_raw[j, ai] > 0.5:
                    attrs_j[attr_names[ai]] = round(float(attrs_raw[j, ai]), 3)
            results.append(
                {
                    "class": cls_names[cls_ids[j]],
                    "conf": round(float(scores[j]), 3),
                    "box": [round(float(x), 1) for x in boxes[j]],
                    "kpts": kpts_j.tolist(),
                    "visible_kpts": int((kpts_j[:, 2] > 0.01).sum()),
                    "attrs": attrs_j,
                    "fiqa": round(float(attrs_raw[j].mean()), 3),
                }
            )
        return results

    def _ensure_gpu_preproc(self):
        if self._preproc_module is not None:
            return
        from pycuda.compiler import SourceModule

        self._preproc_module = SourceModule(_PREPROC_CUDA_CODE)
        self._preproc_kernel = self._preproc_module.get_function(
            "preprocess_kernel"
        )

    def _gpu_preproc(self, frame_h, frame_w):
        target = 640
        scale = min(target / frame_h, target / frame_w)
        nw = int(frame_w * scale)
        nh = int(frame_h * scale)
        dw = (target - nw) // 2
        dh = (target - nh) // 2
        pad_info = (scale, dw, dh, frame_w, frame_h)

        inp_shape = self.inputs[0]["shape"]
        if inp_shape[-1] != target or inp_shape[-2] != target:
            raise ValueError(
                f"GPU preproc only supports {target}x{target} input, got {inp_shape}"
            )

        inp_dev = self.inputs[0]["device"]
        self._preproc_kernel(
            self._raw_dev_buf, np.int32(frame_h), np.int32(frame_w),
            inp_dev, np.int32(target), np.int32(target),
            np.float32(scale), np.float32(dw), np.float32(dh),
            np.int32(nw), np.int32(nh),
            grid=((target - 1 + 32) // 32, (target - 1 + 32) // 32, 1),
            block=(32, 32, 1),
            stream=self.stream,
        )
        return pad_info

    def infer_frame(self, frame, target_size=640):
        import pycuda.driver as cuda

        h0, w0 = frame.shape[:2]
        raw_bytes = frame.nbytes
        if self._raw_dev_buf is None or raw_bytes > self._last_raw_size:
            if self._raw_dev_buf is not None:
                self._raw_dev_buf.free()
            self._raw_dev_buf = cuda.mem_alloc(raw_bytes)
            self._last_raw_size = raw_bytes
        cuda.memcpy_htod_async(self._raw_dev_buf, frame, self.stream)
        pad_info = self._gpu_preproc(h0, w0)
        inp = self.inputs[0]
        self.context.set_tensor_address(inp["name"], int(inp["device"]))
        for out in self.outputs:
            self.context.set_tensor_address(out["name"], int(out["device"]))
        self.context.execute_async_v3(self.stream.handle)
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
        self.stream.synchronize()
        results = {}
        for out in self.outputs:
            results[out["name"]] = out["host"].reshape(out["shape"])
        return results, pad_info

    def close(self):
        if self._raw_dev_buf is not None:
            self._raw_dev_buf.free()
            self._raw_dev_buf = None
        for buf in self.inputs:
            buf["host"].base.free()
            buf["device"].free()
        for buf in self.outputs:
            buf["host"].base.free()
            buf["device"].free()
        if self.stream is not None:
            self.stream.synchronize()
            del self.stream
            self.stream = None
        self._preproc_module = None
        self._preproc_kernel = None
        del self.context
        del self.engine
