import sys
import numpy as np

def sizeof_fmt(num, suffix="B"):
    """Convert bytes to human-readable format."""
    for unit in ["","KB","MB","GB","TB"]:
        if num < 1024.0:
            return f"{num:.2f} {unit}"
        num /= 1024.0
    return f"{num:.2f} PB"

def get_total_size(obj, verbose=True):
    """Recursively finds the memory footprint of a Python object."""
    if isinstance(obj, np.ndarray):
        size = obj.nbytes
        if verbose:
            print(f"Array of shape {obj.shape}, dtype={obj.dtype}: {sizeof_fmt(size)}")
        return size
    elif isinstance(obj, (list, tuple, set)):
        size = sum(get_total_size(i, verbose) for i in obj)
        if verbose:
            print(f"{type(obj)} total size: {sizeof_fmt(size)}")
        return size
    elif isinstance(obj, dict):
        size = sum(get_total_size(k, verbose) + get_total_size(v, verbose) for k, v in obj.items())
        if verbose:
            print(f"dict total size: {sizeof_fmt(size)}")
        return size
    else:
        size = sys.getsizeof(obj)
        if verbose:
            print(f"{type(obj)}: {sizeof_fmt(size)}")
        return size
