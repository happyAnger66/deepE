import time

from bcc import BPF

from . import BpfApp
from ..event.trace_event import TraceSliceEvent

# define BPF program
bpf_text = """
BPF_ARRAY(rqs_start, u64, MAX_PID);

struct rqs_data_t {
    u32 pid;
    u32 tid;
    u32 prev_pid;
    char task[TASK_COMM_LEN];
    char prev_task[TASK_COMM_LEN];
    u64 runnable_us;
    u64 delta_us;
};

BPF_PERF_OUTPUT(rqs_events);

// record enqueue timestamp
static int trace_enqueue(u32 tgid, u32 pid)
{
    if (FILTER_PID || FILTER_TGID || pid == 0)
        return 0;
    u64 ts = bpf_ktime_get_ns();
    rqs_start.update(&pid, &ts);
    return 0;
}
"""

bpf_text_kprobe = """
int trace_wake_up_new_task(struct pt_regs *ctx, struct task_struct *p)
{
    return trace_enqueue(p->tgid, p->pid);
}

int trace_ttwu_do_wakeup(struct pt_regs *ctx, struct rq *rq, struct task_struct *p,
    int wake_flags)
{
    return trace_enqueue(p->tgid, p->pid);
}

// calculate latency
int trace_run(struct pt_regs *ctx, struct task_struct *prev)
{
    u32 pid, tgid;

    // ivcsw: treat like an enqueue event and store timestamp
    if (prev->STATE_FIELD == TASK_RUNNING) {
        tgid = prev->tgid;
        pid = prev->pid;
        u64 ts = bpf_ktime_get_ns();
        if (pid != 0) {
            if (!(FILTER_PID) && !(FILTER_TGID)) {
                rqs_start.update(&pid, &ts);
            }
        }
    }

    pid = bpf_get_current_pid_tgid();

    u64 *tsp, delta_us;

    // fetch timestamp and calculate delta
    tsp = rqs_start.lookup(&pid);
    if ((tsp == 0) || (*tsp == 0)) {
        return 0;   // missed enqueue
    }
    delta_us = (bpf_ktime_get_ns() - *tsp) / 1000;

    if (FILTER_US)
        return 0;

    struct rqs_data_t data = {};
    data.pid = pid;
    data.tid = pid;
    data.prev_pid = prev->pid;
    data.runnable_us = *tsp;
    data.delta_us = delta_us;
    bpf_get_current_comm(&data.task, sizeof(data.task));
    bpf_probe_read_kernel_str(&data.prev_task, sizeof(data.prev_task), prev->comm);

    // output
    rqs_events.perf_submit(ctx, &data, sizeof(data));

    //array map has no delete method, set ts to 0 instead
    *tsp = 0;
    return 0;
}
"""

bpf_text_raw_tp = """
RAW_TRACEPOINT_PROBE(sched_wakeup)
{
    // TP_PROTO(struct task_struct *p)
    struct task_struct *p = (struct task_struct *)ctx->args[0];
    return trace_enqueue(p->tgid, p->pid);
}

RAW_TRACEPOINT_PROBE(sched_wakeup_new)
{
    // TP_PROTO(struct task_struct *p)
    struct task_struct *p = (struct task_struct *)ctx->args[0];
    u32 tgid, pid;

    bpf_probe_read_kernel(&tgid, sizeof(tgid), &p->tgid);
    bpf_probe_read_kernel(&pid, sizeof(pid), &p->pid);
    return trace_enqueue(tgid, pid);
}

RAW_TRACEPOINT_PROBE(sched_switch)
{
    // TP_PROTO(bool preempt, struct task_struct *prev, struct task_struct *next)
    struct task_struct *prev = (struct task_struct *)ctx->args[1];
    struct task_struct *next= (struct task_struct *)ctx->args[2];
    u32 tgid, pid;
    long state;

    // ivcsw: treat like an enqueue event and store timestamp
    bpf_probe_read_kernel(&state, sizeof(long), (const void *)&prev->STATE_FIELD);
    bpf_probe_read_kernel(&pid, sizeof(prev->pid), &prev->pid);
    if (state == TASK_RUNNING) {
        bpf_probe_read_kernel(&tgid, sizeof(prev->tgid), &prev->tgid);
        u64 ts = bpf_ktime_get_ns();
        if (pid != 0) {
            if (!(FILTER_PID) && !(FILTER_TGID)) {
                rqs_start.update(&pid, &ts);
            }
        }

    }

    u32 prev_pid;
    u64 *tsp, delta_us;

    prev_pid = pid;
    bpf_probe_read_kernel(&pid, sizeof(next->pid), &next->pid);

    // fetch timestamp and calculate delta
    tsp = rqs_start.lookup(&pid);
    if ((tsp == 0) || (*tsp == 0)) {
        return 0;   // missed enqueue
    }
    delta_us = (bpf_ktime_get_ns() - *tsp) / 1000;

    if (FILTER_US)
        return 0;

    u32 next_tgid;
    bpf_probe_read_kernel(&next_tgid, sizeof(next->tgid), &next->tgid);
    struct rqs_data_t data = {};
    data.pid = next_tgid;
    data.tid = pid;
    data.prev_pid = prev_pid;
    data.runnable_us = *tsp / 1000;
    data.delta_us = delta_us;
    bpf_probe_read_kernel_str(&data.task, sizeof(data.task), next->comm);
    bpf_probe_read_kernel_str(&data.prev_task, sizeof(data.prev_task), prev->comm);

    // output
    rqs_events.perf_submit(ctx, &data, sizeof(data));

    //array map has no delete method, set ts to 0 instead
    *tsp = 0;
    return 0;
}
"""


class RunqSlowerBpf(BpfApp):
    def __init__(self, args, filepath):
        super().__init__(args, filepath)

    def gen_bpf_text(self, args):
        global bpf_text
        bpf_text_ = bpf_text
        if self.is_support_raw_tp:
            bpf_text_ += bpf_text_raw_tp
        else:
            bpf_text_ += bpf_text_kprobe

        min_us = args.min_us
        # code substitutions
        if BPF.kernel_struct_has_field(b'task_struct', b'__state') == 1:
            bpf_text_ = bpf_text_.replace('STATE_FIELD', '__state')
        else:
            bpf_text_ = bpf_text_.replace('STATE_FIELD', 'state')
        if min_us == 0:
            bpf_text_ = bpf_text_.replace('FILTER_US', '0')
        else:
            bpf_text_ = bpf_text_.replace('FILTER_US', 'delta_us <= %s' % str(min_us))

        if args.tid:
            bpf_text_ = bpf_text_.replace('FILTER_PID', 'pid != %s' % args.tid)
        else:
            bpf_text_ = bpf_text_.replace('FILTER_PID', '0')

        if args.pid:
            bpf_text_ = bpf_text_.replace('FILTER_TGID', 'tgid != %s' % args.pid)
        else:
            bpf_text_ = bpf_text_.replace('FILTER_TGID', '0')

        return bpf_text_

    def events_name(self):
        return "rqs_events"

    def handle_event(self, b, cpu, data, size):
        event = b["rqs_events"].event(data)
        time_us = int(event.runnable_us - self.start_time)
        te = TraceSliceEvent(event.prev_task.decode('utf-8', 'replace'), event.task.decode('utf-8', 'replace'),
                        event.pid, event.tid, time_us, event.delta_us, event.delta_us, 'X')
        self.trace_proc.push_event(te)

    def attach(self, bpf):
        #max_pid = int(open("/proc/sys/kernel/pid_max").read())
        #bpf = BPF(text=self.bpf_text, cflags=["-DMAX_PID=%d" % max_pid])
        if not self.is_support_raw_tp:
            bpf.attach_kprobe(event="ttwu_do_wakeup", fn_name="trace_ttwu_do_wakeup")
            bpf.attach_kprobe(event="wake_up_new_task", fn_name="trace_wake_up_new_task")
            bpf.attach_kprobe(event_re="^finish_task_switch$|^finish_task_switch\.isra\.\d$",
                            fn_name="trace_run")
        return bpf


