import json
import unittest
from flight_safety.recording_topics import RecordingTopics


class RecordingTopicTests(unittest.TestCase):
    def setUp(self):
        self.policy = RecordingTopics([dict(topics=['/raw'], status_topic='/status',
            field='queue.next_index', equals=0, max_age_s=2, on_unavailable='include')])

    def observe(self, index, now=10, pad='a'):
        self.policy.observe('/status', json.dumps(dict(pad=pad, queue=dict(next_index=index))), now)

    def test_legacy_no_conditions(self):
        self.assertEqual(RecordingTopics().select(['/gt'], 0), (['/gt'], []))

    def test_first_route_and_retry(self):
        self.observe(0)
        self.assertEqual(self.policy.select(['/gt'], 11)[0], ['/gt', '/raw'])
        self.observe(0, now=12)
        self.assertEqual(self.policy.select(['/gt'], 12)[0], ['/gt', '/raw'])

    def test_subsequent_routes_and_pad_switch(self):
        for i in (1, 2, 19):
            self.observe(i)
            self.assertEqual(self.policy.select(['/gt'], 11)[0], ['/gt'])
        self.observe(0, pad='b')
        self.assertEqual(self.policy.select(['/gt'], 11)[0], ['/gt', '/raw'])

    def test_missing_stale_future_malformed_status_records_evidence(self):
        self.assertEqual(self.policy.select([], 11)[0], ['/raw'])
        self.observe(2)
        for now in (9, 13):
            self.assertEqual(self.policy.select([], now)[0], ['/raw'])
        for payload in ('invalid', '[]', '{}', '{"queue": null}'):
            self.policy.observe('/status', payload, 10)
            self.assertEqual(self.policy.select([], 11)[0], ['/raw'])

    def test_boolean_is_not_route_zero(self):
        self.observe(False)
        self.assertEqual(self.policy.select([], 11)[0], [])

    def test_selection_is_a_snapshot(self):
        self.observe(0)
        topics, _ = self.policy.select(['/gt'], 11)
        self.observe(1)
        self.assertEqual(topics, ['/gt', '/raw'])
        self.assertEqual(self.policy.select(['/gt'], 11)[0], ['/gt'])

    def test_duplicate_topics_recorded_once(self):
        self.observe(0)
        self.assertEqual(self.policy.select(['/raw'], 11)[0], ['/raw'])

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            RecordingTopics([dict(topics=[], status_topic='/x', field='y', equals=0)])


if __name__ == '__main__':
    unittest.main()
