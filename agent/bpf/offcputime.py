#!/usr/bin/env python
#
# offcputime    Summarize off-CPU time by stack trace
#               For Linux, uses BCC, eBPF.
#
# USAGE: offcputime [-h] [-p PID | -t TID | -u | -k] [-U | -K] [-d] [-f] [-s]
#                   [--stack-storage-size STACK_STORAGE_SIZE]
#                   [-m MIN_BLOCK_TIME] [-M MAX_BLOCK_TIME] [--state STATE]
#                   [duration]
#
# Copyright 2016 Netflix, Inc.
# Licensed under the Apache License, Version 2.0 (the "License")
#
# 13-Jan-2016	Brendan Gregg	Created this.
# 27-Mar-2023	Rocky Xing      Added option to show symbol offsets.
# 04-Apr-2023   Rocky Xing      Updated default stack storage size.
import errno
import time
from bcc import BPF
import argparse

from . import BpfApp
from ..event.trace_event import TraceSliceEvent

x86_64_kernel_stack_filters = {
    'do_epoll_wait', 'do_sys_poll', 'do_futex', 'kern_select',
    'core_sys_select', 'do_nanosleep'
}
# arg validation
def positive_int(val):
    try:
        ival = int(val)
    except ValueError:
        raise argparse.ArgumentTypeError("must be an integer")

    if ival < 0:
        raise argparse.ArgumentTypeError("must be positive")
    return ival

def positive_nonzero_int(val):
    ival = positive_int(val)
    if ival == 0:
        raise argparse.ArgumentTypeError("must be nonzero")
    return ival

def stack_id_err(stack_id):
    # -EFAULT in get_stackid normally means the stack-trace is not available,
    # Such as getting kernel stack trace in userspace code
    return (stack_id < 0) and (stack_id != -errno.EFAULT)

# define BPF program
bpf_text = """
#define MINBLOCK_US    MINBLOCK_US_VALUEULL
#define MAXBLOCK_US    MAXBLOCK_US_VALUEULL

BPF_HASH(offt_start, u32);
BPF_STACK_TRACE(offt_stack_traces, STACK_STORAGE_SIZE);

struct offt_event_t {
    u32 pid;
    u32 tgid;
    u64 t_start;
    u64 t_end;
    u64 delta;
    int user_stack_id;
    int kernel_stack_id;
    char name[TASK_COMM_LEN];
};
BPF_PERF_OUTPUT(offt_events);

int oncpu(struct pt_regs *ctx, struct task_struct *prev) {
    u32 pid = prev->pid;
    u32 tgid = prev->tgid;
    u64 ts, *tsp;

    // record previous thread sleep time
    if ((THREAD_FILTER) && (STATE_FILTER)) {
        ts = bpf_ktime_get_ns();
        offt_start.update(&pid, &ts);
    }

    // get the current thread's start time
    pid = bpf_get_current_pid_tgid();
    tgid = bpf_get_current_pid_tgid() >> 32;
    tsp = offt_start.lookup(&pid);
    if (tsp == 0) {
        return 0;        // missed start or filtered
    }

    // calculate current thread's delta time
    u64 t_start = *tsp;
    u64 t_end = bpf_ktime_get_ns();
    offt_start.delete(&pid);
    if (t_start > t_end) {
        return 0;
    }
    u64 delta = t_end - t_start;
    delta = delta / 1000;
    if ((delta < MINBLOCK_US) || (delta > MAXBLOCK_US)) {
        return 0;
    }

    // create map key
    struct offt_event_t oe = {};

    oe.t_start = t_start / 1000;
    oe.t_end = t_end / 1000;
    oe.pid = pid;
    oe.tgid = tgid;
    oe.user_stack_id = USER_STACK_GET;
    oe.kernel_stack_id = KERNEL_STACK_GET;
    oe.delta = delta;
    bpf_get_current_comm(&oe.name, sizeof(oe.name));

    offt_events.perf_submit(ctx, &oe, sizeof(oe));
    return 0;
}
"""

# set thread filter
class OffCpuBpf(BpfApp):
    def __init__(self, args, filepath):
        super().__init__(args, filepath)

    def gen_bpf_text(self, args):
        thread_context = ""
        if args.tgid is not None:
            thread_context = "PID %d" % args.tgid
            thread_filter = 'tgid == %d' % args.tgid
        elif args.pid is not None:
            thread_context = "TID %d" % args.pid
            thread_filter = 'pid == %d' % args.pid
        elif args.user_threads_only:
            thread_context = "user threads"
            thread_filter = '!(prev->flags & PF_KTHREAD)'
        elif args.kernel_threads_only:
            thread_context = "kernel threads"
            thread_filter = 'prev->flags & PF_KTHREAD'
        else:
            thread_context = "all threads"
            thread_filter = '1'
        self.thread_context = thread_context

        if args.state == 0:
            state_filter = 'prev->STATE_FIELD == 0'
        elif args.state:
            # these states are sometimes bitmask checked
            state_filter = 'prev->STATE_FIELD & %d' % args.state
        else:
            state_filter = '1'

        global bpf_text
        bpf_text = bpf_text.replace('THREAD_FILTER', thread_filter)
        bpf_text = bpf_text.replace('STATE_FILTER', state_filter)
        if BPF.kernel_struct_has_field(b'task_struct', b'__state') == 1:
            bpf_text = bpf_text.replace('STATE_FIELD', '__state')
        else:
            bpf_text = bpf_text.replace('STATE_FIELD', 'state')

        # set stack storage size
        bpf_text = bpf_text.replace('STACK_STORAGE_SIZE', str(args.stack_storage_size))
        bpf_text = bpf_text.replace('MINBLOCK_US_VALUE', str(args.min_block_time))
        bpf_text = bpf_text.replace('MAXBLOCK_US_VALUE', str(args.max_block_time))

        # handle stack args
        kernel_stack_get = "offt_stack_traces.get_stackid(ctx, 0)"
        user_stack_get = "offt_stack_traces.get_stackid(ctx, BPF_F_USER_STACK)"
        stack_context = "user + kernel"
        bpf_text = bpf_text.replace('USER_STACK_GET', user_stack_get)
        bpf_text = bpf_text.replace('KERNEL_STACK_GET', kernel_stack_get)
        self.stack_context = stack_context
        return bpf_text

    def attach(self, bpf):
        # initialize BPF
        #b = BPF(text=self.bpf_text)
        bpf.attach_kprobe(event_re="^finish_task_switch$|^finish_task_switch\.isra\.\d$",
                        fn_name="oncpu")
        matched = bpf.num_open_kprobes()
        if matched == 0:
            raise RuntimeError('0 probes attached !!!')
        return bpf

    def events_name(self):
        return "offt_events"

    def handle_event(self, b, cpu, data, size):
        show_offset = True
        missing_stacks = 0

        stack_traces = b.get_table("offt_stack_traces")
        event = b["offt_events"].event(data)
        if stack_id_err(event.kernel_stack_id) or stack_id_err(event.user_stack_id):
            print('missing stacks')
            missing_stacks += 1
        else:
            user_stack = [] if event.user_stack_id < 0 else \
                stack_traces.walk(event.user_stack_id)
            kernel_stack = [] if event.kernel_stack_id < 0 else \
                stack_traces.walk(event.kernel_stack_id)
            user_stack = list(user_stack)
            kernel_stack = list(kernel_stack)

            # print folded stack output
            line = [event.name.decode('utf-8', 'replace')]
            # if we failed to get the stack is, such as due to no space (-ENOMEM) or
            # hash collision (-EEXIST), we still print a placeholder for consistency
            if stack_id_err(event.user_stack_id):
                line.append("[Missed User Stack]")
            else:
                line.extend([b.sym(addr, event.tgid).decode('utf-8', 'replace')
                             for addr in reversed(user_stack)])
            line.extend(["---"])
            if stack_id_err(event.kernel_stack_id):
                line.append("[Missed Kernel Stack]")
            else:
                if event.kernel_stack_id in self.filter_kernel_ids:
                    return

                kernel_stack_names = []
                for addr in reversed(kernel_stack):
                    stack_name = b.ksym(addr).decode('utf-8', 'replace')
                    if self.arch == 'x86_64':
                        if stack_name in x86_64_kernel_stack_filters:
                            self.filter_kernel_ids.add(event.kernel_stack_id)
                            return

                    kernel_stack_names.append(stack_name)

                line.extend(kernel_stack_names)
#            print(event.t_start, self.start_time)
            time_us = int(event.t_start - self.start_time)
            te = TraceSliceEvent("\r\n".join(line), event.name.decode('utf-8', 'replace'),
                                 event.tgid, event.pid, time_us, event.delta, event.delta,"X")
            self.trace_proc.push_event(te)
            #print('%s %dms at %d tid:%d pid:%d ' % (event.name, event.delta/1000, BPF.monotonic_time(), event.pid, event.tgid))
            #print(line)