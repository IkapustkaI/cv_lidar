"""Тесты регистратора LiDAR FHL-LD19."""

import argparse
import csv
import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import lidar_recorder as lidar  # noqa: E402

class FakeSerial:
    def __init__(self, packets):
        self.packets = list(packets)
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(len(packet) for packet in self.packets)

    def read(self, size=1):
        if not self.packets:
            return b""

        packet = self.packets.pop(0)

        if not self.packets:
            lidar._stop_requested = True

        return packet


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
    def test_serial_recording_rotates_csv_files(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir) / "serial_session"

            fake_serial = FakeSerial([
                lidar.build_fake_packet(0),
                lidar.build_fake_packet(1),
            ])

            serial_module = mock.Mock()
            serial_module.Serial.return_value = fake_serial

            args = argparse.Namespace(
                output_dir=str(output_dir),
                seconds=0,
                port="/dev/ttyUSB0",
                baudrate=230400,
                rotation_seconds=1.0,
            )

            with mock.patch.dict(
                sys.modules,
                {"serial": serial_module},
            ), mock.patch.object(
                lidar.signal, "signal"
            ), mock.patch.object(
                lidar.time,
                "monotonic",
                side_effect=[0.0, 0.0, 2.0],
            ):
                result = lidar.run_serial(args)

            self.assertEqual(result, 0)
            self.assertTrue(fake_serial.closed)

            first_csv = output_dir / "packets.csv"
            second_csv = output_dir / "packets_0002.csv"
            metadata_path = output_dir / "session_metadata.json"

            self.assertTrue(first_csv.exists())
            self.assertTrue(second_csv.exists())

            with first_csv.open(
                encoding="utf-8", newline=""
            ) as handle:
                first_packets = list(csv.DictReader(handle))

            with second_csv.open(
                encoding="utf-8", newline=""
            ) as handle:
                second_packets = list(csv.DictReader(handle))

            self.assertEqual(
                [int(packet["packet_id"]) for packet in first_packets],
                [0],
            )
            self.assertEqual(
                [int(packet["packet_id"]) for packet in second_packets],
                [1],
            )

            with metadata_path.open(encoding="utf-8") as handle:
                metadata = json.load(handle)

            self.assertEqual(metadata["packet_count"], 2)
            self.assertEqual(
                metadata["csv_files"],
                ["packets.csv", "packets_0002.csv"],
            )
            self.assertEqual(metadata["csv_rotation_seconds"], 1.0)
            self.assertEqual(metadata["status"], "stopped")

if __name__ == "__main__":
    unittest.main()