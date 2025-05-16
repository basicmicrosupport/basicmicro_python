"""
Custom exceptions for the Basicmicro package.
"""

class CommunicationError(BasicmicroError):
    """Exception raised for errors in the communication with the controller."""
    pass


class PacketTimeoutError(TimeoutError):
    """Exception raised when a packet transmission times out."""
    pass
