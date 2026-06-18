# L2 monitor verdict: the most-severe fault across all watched diagnosis sources.
std_msgs/Header header
uint8    level       # 0 OK · 1 WARN · 2 ERROR (diagnostic_msgs levels)
string[] names       # source(s) at the worst level (can be multiple)
string[] messages    # parallel to names
