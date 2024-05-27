# Copyright (C) @2024 Cargo Team. All rights reserved.
# Author: zhangxiaoan
# Contact: zhangxiaoan@didiglobal.com

class Config(object):
    BPF_PROGS = ["offcputime", "runqslower"]
    OFFCPUTIME_ARGS = {
        "min_block_time": 1000,
        "max_block_time": 10000,
    }
    RUNQSLOWER_ARGS = {
        "min_us": 10000
    }

class ProductionConfig(Config):
    pass

class DevelopmentConfig(Config):
    pass

class TestingConfig(Config):
    pass
