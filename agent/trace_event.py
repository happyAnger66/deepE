import json

from collections import namedtuple
import queue
from threading import Thread

TraceSliceEvent = namedtuple('TraceSliceEvent', 'cat name pid tid ts dur tdur ph')

class TraceEventProc:
    def __init__(self, MAXITEM_SIZE=10000):
        self._events_q = queue.Queue(maxsize=MAXITEM_SIZE)
        self._running = False
        self._meta_proc_maps = {}
        self._meta_thread_maps = {}
        self._proc_thread = Thread(target=self.proc_func)

    def start(self):
        self._running = True
        self._proc_thread.start()

    def stop(self):
        self._running = False
        self._proc_thread.join(timeout=5)

    def proc_func(self):
        header = '{"traceEvents":['
        tail = ']}'
        with open('trace.json', 'w') as f:
            f.write(header)
            f.flush()
            while self._running:
                try:
                    item = self._events_q.get(timeout=0.5)
                except queue.Empty as e:
                    continue

                f.write(json.dumps(item._asdict()))
                f.write(',')
                f.flush()

            for v in self._meta_thread_maps.values():
                f.write(json.dumps(v._asdict()))
                f.write(',')

            for v in self._meta_proc_maps.values():
                f.write(json.dumps(v._asdict()))
                f.write(',')
            f.write(tail)
            f.write(tail)
            f.flush()