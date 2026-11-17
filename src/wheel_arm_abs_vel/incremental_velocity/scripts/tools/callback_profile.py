"""Per-input wall-time stages. Observation only; never controls admission."""
import time
from contextlib import contextmanager


class CallbackProfile:
    fields = ('snapshot', 'read_filter', 'body_pack', 'incremental_snapshot',
              'incremental_compute', 'commit', 'publish', 'tail', 'total', 'lock_wait')

    def __init__(self):
        self.started = self.previous = time.perf_counter_ns()
        self.values = {name: 0. for name in self.fields}

    def mark(self, name):
        now = time.perf_counter_ns()
        self.values[name] += (now-self.previous)*1.e-6
        self.previous = now

    @contextmanager
    def lock(self, lock):
        start = time.perf_counter_ns()
        lock.acquire()
        self.values['lock_wait'] += (time.perf_counter_ns()-start)*1.e-6
        try:
            yield
        finally:
            lock.release()

    def finish(self):
        self.mark('tail')
        self.values['total'] = (time.perf_counter_ns()-self.started)*1.e-6
        return [self.values[k] for k in self.fields]
