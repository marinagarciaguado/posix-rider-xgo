
import unittest

from bridge.ipc_bridge import (
    CommandSequenceTracker,
    UINT32_MASK,
)


class TestCommandSequenceTracker(unittest.TestCase):

    def setUp(self):
        self.tracker = CommandSequenceTracker()

    def test_first_sequence_is_accepted(self):
        self.assertTrue(self.tracker.accept(10))

    def test_increasing_sequences_are_accepted(self):
        self.assertTrue(self.tracker.accept(10))
        self.assertTrue(self.tracker.accept(11))
        self.assertTrue(self.tracker.accept(12))

    def test_duplicate_sequence_is_rejected(self):
        self.assertTrue(self.tracker.accept(10))
        self.assertFalse(self.tracker.accept(10))

    def test_old_sequence_is_rejected(self):
        self.assertTrue(self.tracker.accept(20))
        self.assertFalse(self.tracker.accept(19))

    def test_sequence_wraparound(self):
        self.assertTrue(self.tracker.accept(UINT32_MASK - 1))
        self.assertTrue(self.tracker.accept(UINT32_MASK))
        self.assertTrue(self.tracker.accept(0))
        self.assertTrue(self.tracker.accept(1))

    def test_old_sequence_after_wraparound(self):
        self.assertTrue(self.tracker.accept(0))
        self.assertFalse(self.tracker.accept(UINT32_MASK))

    def test_invalid_sequence_is_rejected(self):
        self.assertFalse(self.tracker.accept(-1))
        self.assertFalse(self.tracker.accept(UINT32_MASK + 1))

    def test_half_range_difference_is_rejected(self):
        self.assertTrue(self.tracker.accept(0))
        self.assertFalse(self.tracker.accept(0x80000000))


if __name__ == "__main__":
    unittest.main()
