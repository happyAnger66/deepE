import json

from collections import namedtuple
import queue
from threading import Thread

TraceSliceEvent = namedtuple('TraceSliceEvent', 'cat name pid tid ts dur tdur ph')

class TraceSliceEventProc:
    def __init__(self, MAXITEM_SIZE=10000, filepath="trace.json"):
        self._events_q = queue.Queue(maxsize=MAXITEM_SIZE)
        self._running = False
        self.meta_proc_maps = {}
        self.meta_thread_maps = {}
        self._proc_thread = Thread(target=self.proc_func)
        self._output_file = filepath

    def push_event(self, te):
        try:
            self._events_q.put_nowait(te)
        except queue.Full as e:
            print('queue full lost:%s' % te._asdict())

    def start(self):
        self._running = True
        self._proc_thread.start()

    def stop(self):
        self._running = False
        self._proc_thread.join(timeout=5)

    def proc_func(self):
        header = '{"traceEvents":['
        tail = ']}'
        with open(self._output_file, 'w') as f:
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

            for v in self.meta_thread_maps.values():
                f.write(json.dumps(v._asdict()))
                f.write(',')

            for v in self.meta_proc_maps.values():
                f.write(json.dumps(v._asdict()))
                f.write(',')
            f.write(tail)
            f.write(tail)
            f.flush()