from threading import Thread
import time
import functools
from bcc import BPF

from ..event.trace_event import TraceSliceEventProc

bpf_text = """
#include <uapi/linux/ptrace.h>
#include <linux/sched.h>
#include <linux/nsproxy.h>
#include <linux/pid_namespace.h>
"""
class BpfLoader:
    def __init__(self):
        self._bpf_progs = []
        self._running = False
        self._t = Thread(target=self.loop)

    def add_bpf_prog(self, bpf_prog):
        self._bpf_progs.append(bpf_prog)

    def start(self):
        self._running = True
        self.load()
        self._t.start()

    def stop(self):
        self._running = False
        self._t.join(timeout=5)
        for bpf_prog in self._bpf_progs:
            bpf_prog.stop()

    def loop(self):
        print('bpf looping...')
        while self._running:
            try:
                self.b.perf_buffer_poll()
            except KeyboardInterrupt:
                exit()

    def load(self):
        global bpf_text
        bpf_text_ = bpf_text
        for bpf_prog in self._bpf_progs:
            bpf_text_ += bpf_prog.bpf_text

        start_time = int(BPF.monotonic_time() / 1000)
        max_pid = int(open("/proc/sys/kernel/pid_max").read())
        self.b = BPF(text=bpf_text_, cflags=["-DMAX_PID=%d" % max_pid])

        for bpf_prog in self._bpf_progs:
            bpf_prog.start_time = start_time
            bpf_prog.attach(self.b)
            bpf_prog.start()
            print('bpf_prog:%s open events:%s' % (bpf_prog.__class__.__name__, bpf_prog.events_name()))
            self.b[bpf_prog.events_name()].open_perf_buffer(
                functools.partial(bpf_prog.handle_event, self.b), page_cnt=64)


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

    def start(self):
        self._trace_proc.start()

    def attach(self, bpf):
        raise NotImplementedError('BpfApp must implement attach')

    def handle_event(self, b, cpu, data, size):
        raise NotImplementedError('BpfApp must implement handle_event')

    def events_name(self):
        raise NotImplementedError('BpfApp must implement events_name')

    def stop(self):
        self._trace_proc.stop()