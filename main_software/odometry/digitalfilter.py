"""Implements a real time filter.
Based off https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
"""

import numpy as np


class LiveFilter:
    """Base class for live filters."""

    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")


from collections import deque


class LiveLFilter(LiveFilter):
    def __init__(self, b, a):
        """Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a) - 1)

    def _process(self, x):
        """Filter incoming data with standard difference equations."""
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


import scipy.signal


def create_filter(
    order: int, critical_freq: float, sampling_rate: float
) -> LiveLFilter:
    """Creates a low pass filter.

    Args:
        order (int): The order.
        critical_freq (float): The cutoff frequency.
        sampling_rate (float): The sampling rate.
    """
    b, a = scipy.signal.iirfilter(
        order, Wn=critical_freq, fs=sampling_rate, btype="low", ftype="butter"
    )

    return LiveLFilter(b, a)
