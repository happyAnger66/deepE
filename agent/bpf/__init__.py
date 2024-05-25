from threading import Thread
import time

from bcc import BPF

from ..event.trace_event import TraceSliceEventProc

class BpfApp:
    def __init__(self, args, output_file):
        self._args = args
        self.is_support_raw_tp = BPF.support_raw_tracepoint()
        self._bpf_text = self.gen_bpf_text(args)
        self._running = False
        self._trace_proc = TraceSliceEventProc(filepath=output_file)

    @property
    def trace_proc(self):
        return self._trace_proc

    @property
    def bpf_text(self):
        return self._bpf_text

    def gen_bpf_text(self, args):
        raise NotImplementedError('BpfApp must implement gen_bpf_text')

    def _loop_thread_start(self):
        self._t = Thread(target=self.loop)
        self._running = True
        self._t.start()

    def start(self):
        self._trace_proc.start()
        self._loop_thread_start()

    def attach(self):
        raise NotImplementedError('BpfApp must implement attach')

    def handle_event(self, cpu, data, size):
        raise NotImplementedError('BpfApp must implement handle_event')

    def _loop_thread_stop(self):
        self._running = False
        self._t.join(timeout=5)

    def stop(self):
        self._trace_proc.stop()
        self._loop_thread_stop()

    def loop(self):
        self.b = self.attach()

        self.start_time = time.time_ns()
        self.b["events"].open_perf_buffer(self.handle_event, page_cnt=64)
        while self._running:
            try:
                self.b.perf_buffer_poll()
            except KeyboardInterrupt:
                self._trace_proc.stop()
                exit()
        self._trace_proc.stop()
