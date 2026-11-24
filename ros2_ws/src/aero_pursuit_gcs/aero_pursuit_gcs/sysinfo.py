"""CPU, memory and GPU usage of the simulation machine."""

import ctypes
import glob
import os


class _Nvml:
    """NVIDIA GPUs through libnvidia-ml (driver library; no nvidia-smi or pip package needed)."""

    class _Util(ctypes.Structure):
        _fields_ = [('gpu', ctypes.c_uint), ('memory', ctypes.c_uint)]

    class _Mem(ctypes.Structure):
        _fields_ = [('total', ctypes.c_ulonglong), ('free', ctypes.c_ulonglong),
                    ('used', ctypes.c_ulonglong)]

    def __init__(self):
        self._lib = ctypes.CDLL('libnvidia-ml.so.1')
        if self._lib.nvmlInit_v2() != 0:
            raise OSError('nvmlInit failed')
        count = ctypes.c_uint()
        if self._lib.nvmlDeviceGetCount_v2(ctypes.byref(count)) != 0 or count.value == 0:
            raise OSError('no NVIDIA GPU')
        self._devs = []
        for i in range(count.value):
            h = ctypes.c_void_p()
            if self._lib.nvmlDeviceGetHandleByIndex_v2(i, ctypes.byref(h)) == 0:
                self._devs.append(h)

    def sample(self):
        """Busiest GPU [%] and VRAM summed over all GPUs [bytes]."""
        busy, used, total = None, 0, 0
        for h in self._devs:
            u, m = self._Util(), self._Mem()
            if self._lib.nvmlDeviceGetUtilizationRates(h, ctypes.byref(u)) == 0:
                busy = max(busy or 0, u.gpu)
            if self._lib.nvmlDeviceGetMemoryInfo(h, ctypes.byref(m)) == 0:
                used += m.used
                total += m.total
        return busy, used, total


class SysInfo:
    def __init__(self):
        self._prev = None
        self._gpu = self._find_gpu()
        self._nvml = None
        if self._gpu is None:
            try:
                self._nvml = _Nvml()
            except (OSError, AttributeError):
                self._nvml = None

    @staticmethod
    def _find_gpu():
        # amdgpu exposes gpu_busy_percent; NVIDIA goes through NVML (_Nvml).
        for dev in sorted(glob.glob('/sys/class/drm/card*/device')):
            if os.path.exists(os.path.join(dev, 'gpu_busy_percent')):
                return dev
        return None

    @staticmethod
    def _read(path, cast=int):
        try:
            with open(path) as f:
                return cast(f.read().strip())
        except (OSError, ValueError):
            return None

    def sample(self):
        with open('/proc/stat') as f:
            fields = [int(x) for x in f.readline().split()[1:]]
        idle = fields[3] + fields[4]
        total = sum(fields[:8])
        cpu = None
        if self._prev:
            dt = total - self._prev[0]
            cpu = 100.0 * (1 - (idle - self._prev[1]) / dt) if dt > 0 else None
        self._prev = (total, idle)

        mem = {}
        with open('/proc/meminfo') as f:
            for line in f:
                k, v = line.split(':', 1)
                mem[k] = int(v.split()[0])
        out = {
            'op': 'sys',
            'cpu': cpu,
            'cores': os.cpu_count(),
            'load': os.getloadavg()[0],
            'mem_used_gb': (mem['MemTotal'] - mem['MemAvailable']) / 2**20,
            'mem_total_gb': mem['MemTotal'] / 2**20,
        }
        if self._gpu:
            out['gpu'] = self._read(os.path.join(self._gpu, 'gpu_busy_percent'))
            used = self._read(os.path.join(self._gpu, 'mem_info_vram_used'))
            total_vram = self._read(os.path.join(self._gpu, 'mem_info_vram_total'))
            if used is not None and total_vram:
                out['vram_used_gb'] = used / 2**30
                out['vram_total_gb'] = total_vram / 2**30
        elif self._nvml:
            busy, used, total_vram = self._nvml.sample()
            if busy is not None:
                out['gpu'] = busy
            if total_vram:
                out['vram_used_gb'] = used / 2**30
                out['vram_total_gb'] = total_vram / 2**30
        return out
