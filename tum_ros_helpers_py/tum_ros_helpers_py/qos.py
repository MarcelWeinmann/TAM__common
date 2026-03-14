# Copyright 2025
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def get_qos() -> QoSProfile:
    """
    Returns the current default QoS profile for TUM Autonomous Motorsport
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
