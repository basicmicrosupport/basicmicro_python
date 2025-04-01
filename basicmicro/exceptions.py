"""
Custom exceptions for the Basicmicro package.
"""

class BasicmicroError(Exception):
    """Base exception for all Basicmicro-related errors."""
    pass


class CommunicationError(BasicmicroError):
    """Exception raised for errors in the communication with the controller."""
    pass


class CommandError(BasicmicroError):
    """Exception raised when a command is invalid or cannot be executed."""
    pass


class ResponseError(BasicmicroError):
    """Exception raised when a response from the controller is invalid."""
    pass


class ChecksumError(ResponseError):
    """Exception raised when a checksum from the controller is invalid."""
    pass


class TimeoutError(CommunicationError):
    """Exception raised when a timeout occurs during communication."""
    pass