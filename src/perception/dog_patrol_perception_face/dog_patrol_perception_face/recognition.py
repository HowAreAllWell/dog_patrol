"""TensorRT face embedding models used by the production face provider."""

from __future__ import annotations

import numpy as np


SUPPORTED_RECOGNITION_MODELS = frozenset({"sface", "mobilefacenet_arcface"})


def prepare_recognition_input(bgr_image, input_shape, model: str) -> np.ndarray:
    """Prepare one aligned BGR face for the configured TensorRT model.

    Model preprocessing is explicit because an engine filename does not encode
    its color order or normalization contract.  ``sface`` retains the legacy
    raw-BGR behavior.  InsightFace MobileFaceNet uses RGB values in [-1, 1].
    """

    if model not in SUPPORTED_RECOGNITION_MODELS:
        raise ValueError(f"unsupported recognition model: {model!r}")
    shape = tuple(int(value) for value in input_shape)
    if len(shape) != 4 or shape[0] != 1:
        raise ValueError(f"unsupported recognition engine input shape: {shape}")
    is_nhwc = shape[-1] == 3
    expected_image_shape = shape[1:3] + (3,) if is_nhwc else shape[2:4] + (3,)
    image = np.asarray(bgr_image)
    if image.shape != expected_image_shape:
        raise ValueError(
            f"aligned face shape {image.shape} does not match engine input "
            f"{expected_image_shape}"
        )

    if model == "sface":
        prepared = image.astype(np.float32)
    else:
        prepared = image[..., ::-1].astype(np.float32) / 127.5 - 1.0
    if not is_nhwc:
        prepared = prepared.transpose(2, 0, 1)
    return np.ascontiguousarray(prepared[None, ...])


class RecModel:
    """TensorRT face embedding model with an explicit preprocessing contract.

    TensorRT and PyCUDA are imported lazily so the package can be imported in
    environments without a GPU.
    """

    def __init__(
        self,
        engine_path,
        *,
        model: str = "sface",
        expected_embedding_dim: int | None = None,
    ):
        import tensorrt as trt
        import pycuda.driver as cuda

        self.name = str(engine_path)
        if model not in SUPPORTED_RECOGNITION_MODELS:
            raise ValueError(f"unsupported recognition model: {model!r}")
        self.model = model
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(
                trt.Logger(trt.Logger.WARNING)
            ).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise ValueError(f"recognition engine cannot be loaded: {engine_path}")
        self.context = self.engine.create_execution_context()
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
        if self.fixed_inp != self.inp_shape:
            self.context.set_input_shape(self.inp_name, self.fixed_inp)
        self.fixed_out = tuple(self.context.get_tensor_shape(self.out_name))
        if any(d <= 0 for d in self.fixed_out):
            del self.context
            del self.engine
            raise ValueError(f"recognition engine output shape is unresolved: {self.fixed_out}")
        self.embedding_dim = int(np.prod(self.fixed_out))
        if (
            expected_embedding_dim is not None
            and self.embedding_dim != int(expected_embedding_dim)
        ):
            del self.context
            del self.engine
            raise ValueError(
                "recognition engine output dimension "
                f"{self.embedding_dim} does not match expected "
                f"{int(expected_embedding_dim)}"
            )
        self.stream = cuda.Stream()
        self.inp_host = cuda.pagelocked_empty(self.fixed_inp, self.inp_dtype)
        self.out_host = cuda.pagelocked_empty(self.fixed_out, self.out_dtype)
        self.inp_device = cuda.mem_alloc(self.inp_host.nbytes)
        self.out_device = cuda.mem_alloc(self.out_host.nbytes)

    def infer(self, bgr_112):
        blob = prepare_recognition_input(bgr_112, self.fixed_inp, self.model)
        np.copyto(self.inp_host, blob.astype(self.inp_dtype))
        import pycuda.driver as cuda

        cuda.memcpy_htod_async(self.inp_device, self.inp_host, self.stream)
        self.context.set_tensor_address(self.inp_name, int(self.inp_device))
        self.context.set_tensor_address(self.out_name, int(self.out_device))
        if not self.context.execute_async_v3(stream_handle=self.stream.handle):
            raise RuntimeError("TensorRT recognition inference failed")
        cuda.memcpy_dtoh_async(self.out_host, self.out_device, self.stream)
        self.stream.synchronize()
        emb = self.out_host.copy().flatten()
        if emb.size != self.embedding_dim or not np.isfinite(emb).all():
            raise RuntimeError(
                f"recognition returned an invalid embedding shape={emb.shape}"
            )
        n = np.linalg.norm(emb)
        return emb / n if n > 0 else emb

    def close(self):
        if getattr(self, "stream", None) is not None:
            self.stream.synchronize()
        if getattr(self, "inp_device", None) is not None:
            self.inp_device.free()
            self.inp_device = None
        if getattr(self, "out_device", None) is not None:
            self.out_device.free()
            self.out_device = None
        if getattr(self, "inp_host", None) is not None:
            self.inp_host.base.free()
            self.inp_host = None
        if getattr(self, "out_host", None) is not None:
            self.out_host.base.free()
            self.out_host = None
        del self.stream
        del self.context
        del self.engine
