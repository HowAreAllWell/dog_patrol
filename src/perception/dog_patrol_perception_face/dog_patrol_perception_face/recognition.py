"""Face recognition embedding model (adapted from ``yolo-rec.py`` ``RecModel``)."""

from __future__ import annotations

import numpy as np


class RecModel:
    """TensorRT face embedding model (sface family, 128-d output).

    TensorRT and PyCUDA are imported lazily so the package can be imported in
    environments without a GPU.
    """

    def __init__(self, engine_path):
        import tensorrt as trt
        import pycuda.driver as cuda

        self.name = str(engine_path)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(
                trt.Logger(trt.Logger.WARNING)
            ).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        for i in range(self.engine.num_io_tensors):
            tname = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(tname))
            dtype = self.engine.get_tensor_dtype(tname)
            if self.engine.get_tensor_mode(tname) == trt.TensorIOMode.INPUT:
                self.inp_name = tname
                self.inp_shape = shape
                self.inp_dtype = trt.nptype(dtype)
            else:
                self.out_name = tname
                self.out_shape = shape
                self.out_dtype = trt.nptype(dtype)
        self.fixed_inp = tuple(1 if d in (0, -1) else d for d in self.inp_shape)
        self.fixed_out = tuple(1 if d in (0, -1) else d for d in self.out_shape)
        if self.fixed_inp != self.inp_shape:
            self.context.set_input_shape(self.inp_name, self.fixed_inp)
        self.is_nhwc = self.fixed_inp[-1] == 3
        self.use_raw = "sface" in self.name
        self.inp_host = cuda.pagelocked_empty(self.fixed_inp, self.inp_dtype)
        self.out_host = cuda.pagelocked_empty(self.fixed_out, self.out_dtype)

    def infer(self, bgr_112):
        if self.use_raw:
            blob = bgr_112.astype(np.float32)
            if not self.is_nhwc:
                blob = blob.transpose(2, 0, 1)[None, ...]
            else:
                blob = blob[None, ...]
        else:
            import cv2

            rgb = cv2.cvtColor(bgr_112, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            if self.is_nhwc:
                blob = rgb[None, ...]
            else:
                blob = rgb.transpose(2, 0, 1)[None, ...]
        np.copyto(self.inp_host, blob.astype(self.inp_dtype))
        self.context.set_tensor_address(self.inp_name, int(self.inp_host.ctypes.data))
        self.context.set_tensor_address(self.out_name, int(self.out_host.ctypes.data))
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        self.stream.synchronize()
        emb = self.out_host.copy().flatten()
        n = np.linalg.norm(emb)
        return emb / n if n > 0 else emb

    def close(self):
        if getattr(self, "inp_host", None) is not None:
            self.inp_host.base.free()
            self.inp_host = None
        if getattr(self, "out_host", None) is not None:
            self.out_host.base.free()
            self.out_host = None
        self.stream.synchronize()
        del self.stream
        del self.context
        del self.engine
