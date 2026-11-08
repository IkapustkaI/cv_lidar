"""Тесты интеграции LiDAR-рекордера с общей сессией камер."""

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import record_dual_session as dual


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


class FakeProcess:
    def __init__(self, returncode=None, on_terminate=None):
        self.returncode = returncode
        self.on_terminate = on_terminate
        self.terminate_calls = 0
        self.wait_calls = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminate_calls += 1
        if self.on_terminate:
            self.on_terminate(self)

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        self.wait_calls += 1
        if self.returncode is None:
            self.returncode = 0
        return self.returncode


def make_child(key, process):
    return dual.Child(key, process, Path(f"{key}.log"))


class LidarCommandTests(unittest.TestCase):
    def test_lidar_command_targets_output_directory_and_duration(self):
        command = dual.build_lidar_command(
            "/venv/bin/python",
            "/opt/lidar_recorder.py",
            "/tmp/session/lidar",
            12,
        )
        self.assertEqual(command, [
            "/venv/bin/python",
            "/opt/lidar_recorder.py",
            "--output-dir", "/tmp/session/lidar",
            "--seconds", "12",
        ])

    def test_lidar_command_accepts_zero_duration(self):
        command = dual.build_lidar_command(
            "python", "lidar_recorder.py", "/tmp/session/lidar", 0
        )
        self.assertEqual(command[-1], "0")


class LidarProcessIntegrationTests(unittest.TestCase):
    def test_starts_three_children_and_creates_lidar_log(self):
        commands = [
            {"key": "cam_left", "log_name": "cam-left.log", "argv": ["left"]},
            {"key": "cam_right", "log_name": "cam-right.log", "argv": ["right"]},
            {"key": "lidar", "log_name": "lidar.log", "argv": ["lidar"]},
        ]
        started = []

        def popen(argv, **kwargs):
            process = FakeProcess(returncode=0)
            started.append(argv)
            return process

        with tempfile.TemporaryDirectory() as temp:
            children = dual.start_children(
                commands,
                temp,
                popen=popen,
                stop_grace=1.0,
                clock=FakeClock(),
                sleep=lambda seconds: None,
            )
            self.assertTrue((Path(temp) / "cam-left.log").is_file())
            self.assertTrue((Path(temp) / "cam-right.log").is_file())
            self.assertTrue((Path(temp) / "lidar.log").is_file())

        self.assertEqual(
            [child.key for child in children],
            ["cam_left", "cam_right", "lidar"],
        )
        self.assertEqual(started, [["left"], ["right"], ["lidar"]])

    def test_lidar_failure_stops_both_camera_processes(self):
        left = FakeProcess(
            on_terminate=lambda process: setattr(process, "returncode", 0)
        )
        right = FakeProcess(
            on_terminate=lambda process: setattr(process, "returncode", 0)
        )
        lidar = FakeProcess(returncode=1)

        children = [
            make_child("cam_left", left),
            make_child("cam_right", right),
            make_child("lidar", lidar),
        ]
        clock = FakeClock()
        stop_flag = dual.StopFlag()
        supervisor = dual.DualSupervisor(
            stop_flag,
            clock=clock,
            sleep=clock.sleep,
            stop_grace=1.0,
            poll_interval=0.05,
        )

        result = supervisor.run(children)

        self.assertEqual(result["codes"], {
            "cam_left": 0,
            "cam_right": 0,
            "lidar": 1,
        })
        self.assertEqual(left.terminate_calls, 1)
        self.assertEqual(right.terminate_calls, 1)
        self.assertIn("lidar=1", result["error"])
        self.assertEqual(dual.exit_code_for(result), 1)


if __name__ == "__main__":
    unittest.main()
