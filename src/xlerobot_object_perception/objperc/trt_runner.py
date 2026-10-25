"""Run a TensorRT engine from Python with only `tensorrt` and `cuda-python` (no torch, no pycuda) so it works in the Jetson's
system Python. Inputs and outputs are numpy arrays; shapes may be dynamic (set per call)."""
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

_LOGGER = trt.Logger(trt.Logger.WARNING)


def _check(r):
    err = r[0]
    if int(err) != 0:
        raise RuntimeError(f"CUDA error {err}")
    return r[1] if len(r) == 2 else r[1:]


class TrtEngine:
    def __init__(self, path):
        with open(path, "rb") as f:
            self.engine = trt.Runtime(_LOGGER).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = _check(cudart.cudaStreamCreate())
        self.names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inputs = [n for n in self.names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        self.outputs = [n for n in self.names if n not in self.inputs]
        self._buf = {}                                  # name -> (device ptr, bytes)

    def _dev(self, name, nbytes):
        ptr, size = self._buf.get(name, (None, 0))
        if size < nbytes:
            if ptr:
                cudart.cudaFree(ptr)
            ptr = _check(cudart.cudaMalloc(nbytes))
            self._buf[name] = (ptr, nbytes)
        return ptr

    def __call__(self, **arrays):
        keep = {}
        for n in self.inputs:
            a = np.ascontiguousarray(arrays[n], dtype=trt.nptype(self.engine.get_tensor_dtype(n)))
            self.ctx.set_input_shape(n, tuple(a.shape))
            p = self._dev(n, a.nbytes)
            _check(cudart.cudaMemcpyAsync(p, a.ctypes.data, a.nbytes, cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
            self.ctx.set_tensor_address(n, int(p))
            keep[n] = a
        outs = {}
        for n in self.outputs:
            shape = tuple(self.ctx.get_tensor_shape(n))
            dt = trt.nptype(self.engine.get_tensor_dtype(n))
            o = np.empty(shape, dt)
            self.ctx.set_tensor_address(n, int(self._dev(n, o.nbytes)))
            outs[n] = o
        if not self.ctx.execute_async_v3(self.stream):
            raise RuntimeError("TensorRT execution failed")
        for n, o in outs.items():
            _check(cudart.cudaMemcpyAsync(o.ctypes.data, self._buf[n][0], o.nbytes, cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream))
        _check(cudart.cudaStreamSynchronize(self.stream))
        return outs
