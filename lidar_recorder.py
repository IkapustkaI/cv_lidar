#!/usr/bin/env python3
"""Тестовый регистратор пакетов LD19.

На первом этапе поддерживается только fake-источник: он генерирует корректно
структурированные 47-байтовые пакеты LD19 и записывает их в CSV. Это позволяет
проверить формат данных и временные метки без подключённого LiDAR.

Режим serial будет добавлен отдельно после проверки тестового режима.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


PACKET_SIZE = 47
PACKET_HEADER = 0x54
PACKET_VER_LEN = 0x2C
POINTS_PER_PACKET = 12
FULL_TURN_CDEG = 36000

_stop_requested = False


def _make_crc_table() -> tuple[int, ...]:
    """Таблица CRC8 LDROBOT (полином 0x4D, MSB-first)."""
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x4D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc8(data: bytes | bytearray | memoryview) -> int:
    crc = 0
    for value in data:
        crc = _CRC_TABLE[crc ^ value]
    return crc


def parse_packet(packet: bytes) -> dict:
    """Разбирает один LD19-пакет и проверяет его CRC."""
    if len(packet) != PACKET_SIZE:
        raise ValueError(
            f"Ожидался пакет {PACKET_SIZE} байт, получено {len(packet)}"
        )
    if packet[0] != PACKET_HEADER or packet[1] != PACKET_VER_LEN:
        raise ValueError("Неверный заголовок или поле VerLen пакета LD19")

    expected_crc = crc8(packet[:-1])
    if packet[-1] != expected_crc:
        raise ValueError(
            f"Ошибка CRC: в пакете 0x{packet[-1]:02X}, "
            f"рассчитано 0x{expected_crc:02X}"
        )

    speed_dps, start_cdeg = struct.unpack_from("<HH", packet, 2)
    points = []
    for index in range(POINTS_PER_PACKET):
        distance_mm, intensity = struct.unpack_from(
            "<HB", packet, 6 + index * 3
        )
        points.append({
            "distance_mm": distance_mm,
            "intensity": intensity,
        })

    end_cdeg, sensor_timestamp_ms = struct.unpack_from("<HH", packet, 42)

    # Угол каждого измерения интерполируется между начальным и конечным.
    # Если пакет пересекает 0 градусов, разворачиваем конечный угол.
    unwrapped_end = end_cdeg
    if unwrapped_end < start_cdeg:
        unwrapped_end += FULL_TURN_CDEG
    angle_span = unwrapped_end - start_cdeg

    for index, point in enumerate(points):
        angle_cdeg = (
            start_cdeg + angle_span * index / (POINTS_PER_PACKET - 1)
        ) % FULL_TURN_CDEG
        point["angle_deg"] = round(angle_cdeg / 100.0, 4)

    return {
        "speed_deg_s": speed_dps,
        "start_angle_deg": start_cdeg / 100.0,
        "end_angle_deg": end_cdeg / 100.0,
        "sensor_timestamp_ms": sensor_timestamp_ms,
        "points": points,
    }


def build_fake_packet(packet_id: int, packet_rate: float = 375.0) -> bytes:
    """Создаёт синтетический пакет в формате LD19 для стендовой проверки."""
    start_cdeg = round(packet_id * 360000 / packet_rate) % FULL_TURN_CDEG
    end_cdeg = (start_cdeg + 880) % FULL_TURN_CDEG
    sensor_timestamp_ms = round(packet_id * 1000 / packet_rate) % 30001
    speed_dps = 3600

    packet = bytearray(PACKET_SIZE)
    packet[0] = PACKET_HEADER
    packet[1] = PACKET_VER_LEN
    struct.pack_into("<HH", packet, 2, speed_dps, start_cdeg)

    unwrapped_end = end_cdeg
    if unwrapped_end < start_cdeg:
        unwrapped_end += FULL_TURN_CDEG
    angle_span = unwrapped_end - start_cdeg

    for index in range(POINTS_PER_PACKET):
        angle_cdeg = (
            start_cdeg + angle_span * index / (POINTS_PER_PACKET - 1)
        ) % FULL_TURN_CDEG
        angle_rad = math.radians(angle_cdeg / 100.0)

        # Искусственный, но детерминированный профиль расстояний в мм.
        distance_mm = int(
            1800 + 350 * math.sin(angle_rad * 2.0)
            + 120 * math.cos(angle_rad * 5.0)
        )
        intensity = 100 + (index * 7) % 80
        struct.pack_into(
            "<HB", packet, 6 + index * 3, distance_mm, intensity
        )

    struct.pack_into(
        "<HH", packet, 42, end_cdeg, sensor_timestamp_ms
    )
    packet[-1] = crc8(packet[:-1])
    return bytes(packet)


def save_json(path: Path, data: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


def request_stop(signum, frame) -> None:
    global _stop_requested
    _stop_requested = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Тестовая запись пакетов LiDAR LD19"
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="новый каталог lidar внутри каталога сессии",
    )
    parser.add_argument(
        "--mode",
        choices=("fake", "serial"),
        default="fake",
        help="Источник данных: fake или serial",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Последовательный порт LiDAR",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=230400,
        help="Скорость UART",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=10.0,
        help="длительность записи; 0 — до остановки оператором",
    )
    parser.add_argument(
        "--fake-packet-rate",
        type=float,
        default=375.0,
        help="частота синтетических пакетов в секунду",
    )
    return parser


def run_fake(args: argparse.Namespace) -> int:
    global _stop_requested
    _stop_requested = False

    if args.seconds < 0:
        raise ValueError("--seconds должен быть >= 0")
    if args.fake_packet_rate <= 0:
        raise ValueError("--fake-packet-rate должен быть > 0")

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=False)

    metadata_path = output_dir / "session_metadata.json"
    csv_path = output_dir / "packets.csv"

    started_utc = datetime.now(timezone.utc).isoformat()
    metadata = {
        "sensor": "FHL-LD19",
        "source_mode": "fake",
        "status": "starting",
        "started_at_utc": started_utc,
        "finished_at_utc": None,
        "packet_count": 0,
        "invalid_packets": 0,
        "packet_size_bytes": PACKET_SIZE,
        "points_per_packet": POINTS_PER_PACKET,
        "baudrate_expected_for_serial_mode": 230400,
        "timestamp_policy": {
            "host_receive_monotonic_ns": "time.monotonic_ns()",
            "host_receive_utc_ns": "time.time_ns()",
            "sensor_timestamp_ms": "timestamp из пакета LD19; циклический счётчик",
        },
        "error": None,
    }
    save_json(metadata_path, metadata)

    fields = [
        "packet_id",
        "host_receive_monotonic_ns",
        "host_receive_utc_ns",
        "sensor_timestamp_ms",
        "speed_deg_s",
        "start_angle_deg",
        "end_angle_deg",
        "points_json",
        "raw_packet_hex",
        "crc_valid",
    ]

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    interval = 1.0 / args.fake_packet_rate
    next_packet_at = time.monotonic()
    deadline = (
        next_packet_at + args.seconds if args.seconds > 0 else None
    )

    try:
        metadata["status"] = "recording"
        save_json(metadata_path, metadata)

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()

            packet_id = 0
            while not _stop_requested:
                now = time.monotonic()
                if deadline is not None and now >= deadline:
                    break

                packet = build_fake_packet(packet_id, args.fake_packet_rate)
                parsed = parse_packet(packet)

                # Метки ставятся в момент, когда пакет поступил в регистратор.
                host_mono_ns = time.monotonic_ns()
                host_utc_ns = time.time_ns()

                writer.writerow({
                    "packet_id": packet_id,
                    "host_receive_monotonic_ns": host_mono_ns,
                    "host_receive_utc_ns": host_utc_ns,
                    "sensor_timestamp_ms": parsed["sensor_timestamp_ms"],
                    "speed_deg_s": parsed["speed_deg_s"],
                    "start_angle_deg": parsed["start_angle_deg"],
                    "end_angle_deg": parsed["end_angle_deg"],
                    "points_json": json.dumps(
                        parsed["points"], separators=(",", ":")
                    ),
                    "raw_packet_hex": packet.hex(),
                    "crc_valid": 1,
                })

                packet_id += 1
                metadata["packet_count"] = packet_id

                if packet_id % 20 == 0:
                    handle.flush()
                    save_json(metadata_path, metadata)

                next_packet_at += interval
                sleep_for = next_packet_at - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    # Не пытаемся бесконечно догонять расписание после задержки.
                    next_packet_at = time.monotonic()

            handle.flush()

        metadata["status"] = "stopped"
        metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save_json(metadata_path, metadata)
        print(
            f"LiDAR fake: записано пакетов {metadata['packet_count']} "
            f"в {output_dir}",
            flush=True,
        )
        return 0

    except BaseException as exc:
        metadata["status"] = "failed"
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save_json(metadata_path, metadata)
        raise

def run_serial(args: argparse.Namespace) -> int:
    global _stop_requested
    _stop_requested = False

    import serial

    if args.seconds < 0:
        raise ValueError("--seconds должен быть >= 0")
    if args.baudrate <= 0:
        raise ValueError("--baudrate должен быть > 0")

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=False)

    metadata_path = output_dir / "session_metadata.json"
    csv_path = output_dir / "packets.csv"

    metadata = {
        "sensor": "FHL-LD19",
        "source_mode": "serial",
        "port": args.port,
        "baudrate": args.baudrate,
        "status": "starting",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "finished_at_utc": None,
        "packet_count": 0,
        "invalid_packets": 0,
        "packet_size_bytes": PACKET_SIZE,
        "points_per_packet": POINTS_PER_PACKET,
        "error": None,
    }
    save_json(metadata_path, metadata)

    fields = [
        "packet_id",
        "host_receive_monotonic_ns",
        "host_receive_utc_ns",
        "sensor_timestamp_ms",
        "speed_deg_s",
        "start_angle_deg",
        "end_angle_deg",
        "points_json",
        "raw_packet_hex",
        "crc_valid",
    ]

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    deadline = (
        time.monotonic() + args.seconds if args.seconds > 0 else None
    )

    try:
        with serial.Serial(
            args.port,
            args.baudrate,
            timeout=0.2,
        ) as ser:
            metadata["status"] = "recording"
            save_json(metadata_path, metadata)

            buffer = bytearray()

            with csv_path.open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()

                packet_id = 0

                while not _stop_requested:
                    if deadline is not None and time.monotonic() >= deadline:
                        break

                    chunk = ser.read(ser.in_waiting or 1)

                    if not chunk:
                        continue

                    buffer.extend(chunk)

                    while True:
                        header_index = buffer.find(bytes([PACKET_HEADER]))

                        if header_index < 0:
                            buffer.clear()
                            break

                        if header_index > 0:
                            del buffer[:header_index]

                        if len(buffer) < PACKET_SIZE:
                            break

                        packet = bytes(buffer[:PACKET_SIZE])

                        try:
                            parsed = parse_packet(packet)
                        except ValueError:
                            metadata["invalid_packets"] += 1
                            del buffer[0]
                            continue

                        host_mono_ns = time.monotonic_ns()
                        host_utc_ns = time.time_ns()

                        writer.writerow({
                            "packet_id": packet_id,
                            "host_receive_monotonic_ns": host_mono_ns,
                            "host_receive_utc_ns": host_utc_ns,
                            "sensor_timestamp_ms":
                                parsed["sensor_timestamp_ms"],
                            "speed_deg_s": parsed["speed_deg_s"],
                            "start_angle_deg":
                                parsed["start_angle_deg"],
                            "end_angle_deg": parsed["end_angle_deg"],
                            "points_json": json.dumps(
                                parsed["points"],
                                separators=(",", ":"),
                            ),
                            "raw_packet_hex": packet.hex(),
                            "crc_valid": 1,
                        })

                        packet_id += 1
                        metadata["packet_count"] = packet_id

                        del buffer[:PACKET_SIZE]

                        if packet_id % 20 == 0:
                            handle.flush()
                            save_json(metadata_path, metadata)

                handle.flush()

        metadata["status"] = "stopped"
        metadata["finished_at_utc"] = datetime.now(
            timezone.utc
        ).isoformat()
        save_json(metadata_path, metadata)

        print(
            f"LiDAR serial: записано пакетов "
            f"{metadata['packet_count']} в {output_dir}",
            flush=True,
        )

        return 0

    except BaseException as exc:
        metadata["status"] = "failed"
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        metadata["finished_at_utc"] = datetime.now(
            timezone.utc
        ).isoformat()
        save_json(metadata_path, metadata)
        raise

def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    try:
        if args.mode == "fake":
            return run_fake(args)

        if args.mode == "serial":
            return run_serial(args)

        raise ValueError(f"Неизвестный режим: {args.mode}")

    except Exception as exc:
        print(
            f"Ошибка LiDAR-регистратора: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
