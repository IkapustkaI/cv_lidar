"""Тесты регистратора LiDAR FHL-LD19."""

import argparse
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import lidar_recorder as lidar  # noqa: E402


class PacketParsingTests(unittest.TestCase):

    def test_fake_packet_has_valid_structure(self):
        packet = lidar.build_fake_packet(0)

        self.assertEqual(len(packet), lidar.PACKET_SIZE)

        parsed = lidar.parse_packet(packet)

        self.assertEqual(
            len(parsed["points"]),
            lidar.POINTS_PER_PACKET
        )
        self.assertEqual(parsed["sensor_timestamp_ms"], 0)
        self.assertEqual(parsed["speed_deg_s"], 3600)

    def test_packet_with_wrong_length_is_rejected(self):
        packet = lidar.build_fake_packet(0)

        with self.assertRaises(ValueError):
            lidar.parse_packet(packet[:-1])

    def test_packet_with_wrong_header_is_rejected(self):
        packet = bytearray(lidar.build_fake_packet(0))
        packet[0] = 0x00

        with self.assertRaises(ValueError):
            lidar.parse_packet(bytes(packet))

    def test_packet_with_invalid_crc_is_rejected(self):
        packet = bytearray(lidar.build_fake_packet(0))
        packet[10] ^= 0xFF

        with self.assertRaises(ValueError):
            lidar.parse_packet(bytes(packet))


class RecordingTests(unittest.TestCase):

    def test_fake_recording_creates_valid_files(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir) / "lidar_session"

            args = argparse.Namespace(
                output_dir=str(output_dir),
                seconds=0.1,
                fake_packet_rate=375.0,
            )

            result = lidar.run_fake(args)

            self.assertEqual(result, 0)

            csv_path = output_dir / "packets.csv"
            metadata_path = output_dir / "session_metadata.json"

            self.assertTrue(csv_path.exists())
            self.assertTrue(metadata_path.exists())

            with metadata_path.open(
                encoding="utf-8"
            ) as handle:
                metadata = json.load(handle)

            self.assertEqual(metadata["status"], "stopped")
            self.assertGreater(metadata["packet_count"], 0)
            self.assertEqual(metadata["invalid_packets"], 0)

            with csv_path.open(
                encoding="utf-8",
                newline=""
            ) as handle:
                packets = list(csv.DictReader(handle))

            self.assertEqual(
                len(packets),
                metadata["packet_count"]
            )

            self.assertTrue(
                all(packet["crc_valid"] == "1"
                    for packet in packets)
            )

            for packet in packets:
                points = json.loads(packet["points_json"])
                self.assertEqual(
                    len(points),
                    lidar.POINTS_PER_PACKET
                )

            timestamps = [
                int(packet["host_receive_monotonic_ns"])
                for packet in packets
            ]

            self.assertTrue(
                all(
                    timestamps[i] <= timestamps[i + 1]
                    for i in range(len(timestamps) - 1)
                ),
                "Временные метки не должны идти назад"
            )


if __name__ == "__main__":
    unittest.main()