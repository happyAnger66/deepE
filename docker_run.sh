#!/bin/bash

docker run -tid --name bpf -v $(pwd):/deepE  -v /usr/src:/usr/src:ro  -v /lib/modules/:/lib/modules:ro -v /sys/kernel/debug/:/sys/kernel/debug:rw \
 --net=host --pid=host --privileged harbor.intra.xiaojukeji.com/cargo/alpas:bpftrace_x86_20240328042110 /bin/bash