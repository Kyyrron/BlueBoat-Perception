"""The simulator's ground truth, for dataset labels only (2026-10-16).

By the owner's decision, the Perception node reads the simulator's per-ping
ground truth in **dataset mode, in a simulator world**, and passes it to the
dataset label source (``core/sim_labels.py``) and nowhere else. The detection
path never sees it, so detections and records are identical with or without
it (test-enforced). This is the only online file that names these topics, and
``perception_node`` calls :func:`subscribe` only from its dataset set-up.
"""

from __future__ import annotations

import json
from typing import List

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..core.sim_labels import GroundTruthBuffer

CONTACTS_TOPIC = "/side_scan_sonar/ground_truth/contacts"
SENSOR_TOPIC = "/side_scan_sonar/ground_truth/sensor"
#: The simulator publishes both with the default profile (RELIABLE, VOLATILE,
#: depth 10). A deeper reader history rides out a busy callback (an image
#: being processed) at 2 x 20 messages per second.
GROUND_TRUTH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=200,
                              durability=DurabilityPolicy.VOLATILE)


def subscribe(node, buffer: GroundTruthBuffer) -> List[object]:
    """Feed ``buffer`` from the simulator's two ground-truth topics."""

    def handler(add):
        def cb(msg: String) -> None:
            try:
                doc = json.loads(msg.data)
            except (TypeError, ValueError):
                buffer.counts["bad_messages"] += 1
                return
            add(doc)
        return cb

    return [node.create_subscription(String, CONTACTS_TOPIC,
                                     handler(buffer.add_contacts), GROUND_TRUTH_QOS),
            node.create_subscription(String, SENSOR_TOPIC,
                                     handler(buffer.add_sensor), GROUND_TRUTH_QOS)]
