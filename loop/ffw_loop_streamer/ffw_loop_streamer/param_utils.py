"""Parameter helpers (same contract as physical_ai_server.utils.parameter_utils).

The robot config YAML is keyed ``<robot_type>.<param>``; these helpers declare and
read that dotted namespace so the streamer can share ROBOTIS' per-robot config
layout without depending on the physical_ai_server package.
"""
from typing import Any, Dict, List

from rclpy.node import Node


def declare_parameters(node: Node, robot_type: str, param_names: List[str],
                       default_value: Any = None) -> None:
    for name in param_names:
        param_path = f'{robot_type}.{name}'
        if not node.has_parameter(param_path):
            node.declare_parameter(param_path, default_value)


def load_parameters(node: Node, robot_type: str, param_names: List[str]) -> Dict[str, Any]:
    return {name: node.get_parameter(f'{robot_type}.{name}').value for name in param_names}


def parse_topic_list_with_names(topic_list: List[str]) -> Dict[str, str]:
    """Parse ``['name:/topic/path', ...]`` into ``{name: '/topic/path'}``; skip malformed entries."""
    parsed: Dict[str, str] = {}
    for entry in topic_list or []:
        try:
            key, value = entry.split(':', 1)
        except ValueError:
            continue
        parsed[key] = value
    return parsed
