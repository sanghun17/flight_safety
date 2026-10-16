"""Generic JSON-status conditions, sampled once when a recording starts."""
import json
import threading


class RecordingTopics:
    def __init__(self, groups=()):
        self.groups = list(groups)
        self.status = {}
        self.lock = threading.Lock()
        for group in self.groups:
            if (not isinstance(group.get('topics'), list) or not group['topics']
                    or not all(isinstance(t, str) and t for t in group['topics'])
                    or not isinstance(group.get('status_topic'), str)
                    or not group['status_topic']
                    or not isinstance(group.get('field'), str) or not group['field']
                    or 'equals' not in group or float(group.get('max_age_s', 2)) <= 0
                    or group.get('on_unavailable', 'include') not in ('include', 'exclude')):
                raise ValueError('Invalid conditional recording topic group')

    @property
    def status_topics(self):
        return sorted(set(g['status_topic'] for g in self.groups))

    def observe(self, topic, payload, now):
        try:
            value = json.loads(payload)
            if not isinstance(value, dict):
                value = None
        except (ValueError, TypeError):
            value = None
        with self.lock:
            self.status[topic] = (now, value)

    def select(self, base, now):
        topics = list(base)
        decisions = []
        with self.lock:
            statuses = dict(self.status)
        for group in self.groups:
            stamp, document = statuses.get(group['status_topic'], (None, None))
            available = (stamp is not None and 0 <= now-stamp <= float(group.get('max_age_s', 2))
                         and document is not None)
            value = document
            if available:
                for key in group['field'].split('.'):
                    if not isinstance(value, dict) or key not in value:
                        available = False
                        break
                    value = value[key]
            include = ((type(value) is type(group['equals']) and value == group['equals'])
                       if available else group.get('on_unavailable', 'include') == 'include')
            if include:
                topics.extend(group['topics'])
            decisions.append(dict(topics=group['topics'], included=include,
                                  reason='matched' if available and include else
                                         'not_matched' if available else 'status_unavailable'))
        return list(dict.fromkeys(topics)), decisions
