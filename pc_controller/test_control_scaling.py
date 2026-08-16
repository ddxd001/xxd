import unittest

from web_controller import RobotBackend


class ControlScalingTests(unittest.TestCase):
    def test_chassis_and_lift_use_independent_scales(self) -> None:
        backend = RobotBackend()
        backend.config["speed_scale"] = 0.20
        backend.config["lift_speed_scale"] = 0.60
        backend.motion = (1.0, -1.0, 0.5, 1.0)

        self.assertEqual(backend._scaled_motion(), (200, -200, 100, 600))

    def test_motion_and_scales_are_clamped(self) -> None:
        backend = RobotBackend()
        backend.config["speed_scale"] = 0.01
        backend.config["lift_speed_scale"] = 2.0
        backend.motion = (2.0, -2.0, 0.0, -2.0)

        self.assertEqual(backend._scaled_motion(), (50, -50, 0, -1000))

    def test_half_duplex_slots_keep_their_identity(self) -> None:
        backend = RobotBackend()
        backend.tx_epoch = 100.0
        backend.tx_cycle = 0
        backend.tx_next_slot = 0

        self.assertEqual(backend._control_slot_due(100.021), 0)
        self.assertEqual(backend._control_slot_due(100.071), 1)
        self.assertIsNone(backend._control_slot_due(100.100))


class FaceEventQueueTests(unittest.IsolatedAsyncioTestCase):
    async def test_latest_unsent_event_replaces_previous(self) -> None:
        backend = RobotBackend()
        backend.port = object()
        backend.sequence_synced = True

        await backend.queue_face_event(1)
        await backend.queue_face_event(10)

        self.assertEqual(backend.pending_face_event, 10)

    async def test_invalid_event_is_not_queued(self) -> None:
        backend = RobotBackend()
        backend.port = object()
        backend.sequence_synced = True

        await backend.queue_face_event(11)

        self.assertIsNone(backend.pending_face_event)


if __name__ == "__main__":
    unittest.main()
