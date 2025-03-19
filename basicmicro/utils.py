"""
Utility functions for the Basicmicro package.
"""

import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)


def initialize_crc_table(polynomial: int = 0x1021) -> List[int]:
    """Initialize a CRC lookup table for faster CRC calculations.
    
    Args:
        polynomial: The CRC polynomial to use (default: 0x1021)
        
    Returns:
        List[int]: The pre-computed CRC table
    """
    table = [0] * 256
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ polynomial) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            table[i] = crc
    return table


def calc_mixed(fb: int, lr: int) -> Tuple[int, int]:
    """Utility function for calculating mixed mode values.
    
    Args:
        fb: Forward/backward value
        lr: Left/right value
    
    Returns:
        Tuple[int, int]: Tuple of mixed mode values (out0, out1)
    """
    # Calculate mixing
    if (lr ^ fb) < 0:  # Signs are different?
        if abs(lr) > abs(fb):
            out1 = -lr
        else:
            out1 = fb
        out0 = fb + lr
    else:
        if abs(fb) > abs(lr):
            out0 = fb
        else:
            out0 = lr
        out1 = fb - lr
    
    return (out0, out1)