# Copyright (C) @2024 Cargo Team. All rights reserved.
# Author: zhangxiaoan
# Contact: zhangxiaoan@didiglobal.com

BPF_PROGS = ["offcputime"]
OFFCPUTIME_ARGS = {
    "min_block_time": 100000,
    "max_block_time": 2000000,
    "stack_storage_size": 16384,
    "user_threads_only": True,
}
RUNQSLOWER_ARGS = {
    "min_us": 10000
}
ARGS = {"offcputime": OFFCPUTIME_ARGS,
        "runqslower": RUNQSLOWER_ARGS}
