class _Log:
    def info(self, *a, **k): print('INFO', a[0])
    warn = info
class _Pub:
    def __init__(self, t): self.t = t; self.n = 0
    def publish(self, m): self.n += 1
class Node:
    def __init__(self, name): self.subs = {}; self.pubs = {}
    def create_publisher(self, typ, topic, q): self.pubs[topic] = _Pub(topic); return self.pubs[topic]
    def create_subscription(self, typ, topic, cb, q): self.subs[topic] = cb; return cb
    def create_timer(self, p, cb): return None
    def get_logger(self): return _Log()
    def get_node_names(self): return ['YB_Node', 'joy_node']
    def get_topic_names_and_types(self): return []
    def count_publishers(self, t): return 1
    def destroy_node(self): pass
