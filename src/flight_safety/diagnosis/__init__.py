"""L1 diagnosis reporters. Shared KeyValue schema so /diagnostics is self-describing:
every reporter that has a measurement emits value/units plus the threshold(s) it is judged
against, so a bag of /diagnostics shows WHAT was measured vs WHAT line it crossed (not just
the verdict string). Recorded verbatim by the safety bag (rosbag -a)."""


def add_measurement(stat, value, units, warn=None, error=None):
    stat.add("value", "%.3f" % value)
    stat.add("units", units)
    if warn is not None:
        stat.add("warn", "%.3f" % warn)
    if error is not None:
        stat.add("error", "%.3f" % error)
