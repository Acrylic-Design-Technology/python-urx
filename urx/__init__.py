"""
Python library to control an UR robot through its TCP/IP interface
"""

import logging

from urx.urrobot import RobotException, URRobot  # noqa

__version__ = "0.11.0"

# Add NullHandler by default to prevent "No handler found" warnings
# Users can configure logging themselves or use setup_logging()
logging.getLogger('URX Logger').addHandler(logging.NullHandler())


def setup_logging(level=logging.INFO):
    """
    Configure logging for the URX library.
    
    This is a convenience function to quickly enable logging output.
    For more control, you can configure the 'URX Logger' directly using
    the standard Python logging module.
    
    Args:
        level: The logging level (default: logging.INFO).
               Common values: logging.DEBUG, logging.INFO, logging.WARNING, logging.ERROR
    
    Returns:
        The configured logger instance.
    
    Example:
        >>> import urx
        >>> urx.setup_logging()  # Enable INFO level logging
        >>> robot = urx.Robot("192.168.1.1")  # Logs will now be visible
        
        >>> import logging
        >>> urx.setup_logging(level=logging.DEBUG)  # Enable DEBUG level logging
    """
    logger = logging.getLogger('URX Logger')
    
    # Remove any existing handlers to avoid duplicates
    logger.handlers.clear()
    
    # Create console handler with formatting
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    handler.setFormatter(formatter)
    
    # Configure logger
    logger.addHandler(handler)
    logger.setLevel(level)
    
    return logger


try:
    from urx.robot import Robot
except ImportError as ex:
    print("Exception while importing math3d base robot, disabling use of matrices", ex)
    Robot = URRobot