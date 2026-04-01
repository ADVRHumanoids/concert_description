#!/usr/bin/env python3

import math
from dataclasses import dataclass
from functools import partial
from typing import List, Sequence

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class UltrasoundScanToRange(Node):
    @dataclass
    class _Channel:
        input_topic: str
        output_topic: str
        frame_id: str

    def __init__(self) -> None:
        super().__init__('ultrasound_scan_to_range')

        # Explicitly type array params so YAML launch overrides with string arrays
        # don't get rejected as BYTE_ARRAY on Jazzy.
        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('frame_ids', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('input_topic', '/ultrasound/scan')
        self.declare_parameter('output_topic', '/ultrasound')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('use_inf_if_no_return', True)
        self.declare_parameter('default_if_no_return', 0.0)

        # Optional list params may be declared but left unset; default them to [].
        input_topics = list(
            self.get_parameter_or('input_topics', Parameter('input_topics', value=[]))
            .get_parameter_value()
            .string_array_value
        )
        output_topics = list(
            self.get_parameter_or('output_topics', Parameter('output_topics', value=[]))
            .get_parameter_value()
            .string_array_value
        )
        frame_ids = list(
            self.get_parameter_or('frame_ids', Parameter('frame_ids', value=[]))
            .get_parameter_value()
            .string_array_value
        )

        if input_topics:
            channels = self._channels_from_lists(input_topics, output_topics, frame_ids)
        else:
            channels = [
                self._Channel(
                    input_topic=self.get_parameter('input_topic').get_parameter_value().string_value,
                    output_topic=self.get_parameter('output_topic').get_parameter_value().string_value,
                    frame_id=self.get_parameter('frame_id').get_parameter_value().string_value,
                )
            ]

        self.use_inf_if_no_return = self.get_parameter('use_inf_if_no_return').get_parameter_value().bool_value
        self.default_if_no_return = (
            self.get_parameter('default_if_no_return').get_parameter_value().double_value
        )

        self.pubs = {}
        self.subs = []
        for channel in channels:
            pub = self.create_publisher(Range, channel.output_topic, 10)
            sub = self.create_subscription(
                LaserScan,
                channel.input_topic,
                partial(self._scan_cb, pub=pub, frame_id=channel.frame_id),
                10,
            )
            self.pubs[channel.input_topic] = pub
            self.subs.append(sub)

            self.get_logger().info(
                f'Bridging {channel.input_topic} (LaserScan) -> {channel.output_topic} (Range)'
            )

    def _scan_cb(self, msg: LaserScan, pub, frame_id: str) -> None:
        out = Range()
        out.header = msg.header
        if frame_id:
            out.header.frame_id = frame_id

        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = float(msg.angle_max - msg.angle_min)
        out.min_range = float(msg.range_min)
        out.max_range = float(msg.range_max)

        valid_ranges = self._valid_ranges(msg.ranges, msg.range_min, msg.range_max)
        if valid_ranges:
            out.range = min(valid_ranges)
        else:
            if self.use_inf_if_no_return:
                out.range = math.inf
            else:
                out.range = float(self.default_if_no_return)

        pub.publish(out)

    def _channels_from_lists(
        self,
        input_topics: List[str],
        output_topics: List[str],
        frame_ids: List[str],
    ) -> List[_Channel]:
        if output_topics and len(output_topics) != len(input_topics):
            raise ValueError('output_topics must have same length as input_topics')
        if frame_ids and len(frame_ids) != len(input_topics):
            raise ValueError('frame_ids must have same length as input_topics')

        channels: List[UltrasoundScanToRange._Channel] = []
        for i, input_topic in enumerate(input_topics):
            output_topic = output_topics[i] if output_topics else self._default_output_topic(input_topic)
            frame_id = frame_ids[i] if frame_ids else self._default_frame_id(input_topic)
            channels.append(self._Channel(input_topic=input_topic, output_topic=output_topic, frame_id=frame_id))
        return channels

    @staticmethod
    def _default_output_topic(input_topic: str) -> str:
        if input_topic.endswith('/scan'):
            return input_topic[:-5]
        return input_topic

    @staticmethod
    def _default_frame_id(input_topic: str) -> str:
        topic = input_topic[:-5] if input_topic.endswith('/scan') else input_topic
        parts = [p for p in topic.split('/') if p]
        return parts[-1] if parts else ''

    @staticmethod
    def _valid_ranges(ranges: Sequence[float], rmin: float, rmax: float) -> List[float]:
        out: List[float] = []
        for r in ranges:
            if math.isfinite(r) and rmin <= r <= rmax:
                out.append(r)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasoundScanToRange()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
