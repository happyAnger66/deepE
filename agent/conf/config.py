# Copyright (C) @2024 Cargo Team. All rights reserved.
# Author: zhangxiaoan
# Contact: happyAnger66@163.com

BPF_PROGS = ["offcputime", "runqslower"]

OFFCPUTIME_ARGS = {
    "min_block_time": 200000,   # 200ms
    "max_block_time": 5000000,  # 5s
    "stack_storage_size": 16384,
    "user_threads_only": True,
}

RUNQSLOWER_ARGS = {
    "min_us": 10000 # 20ms
}

ARGS = {"offcputime": OFFCPUTIME_ARGS,
        "runqslower": RUNQSLOWER_ARGS}
