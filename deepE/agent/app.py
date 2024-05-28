# Copyright (C) @2024 Cargo Team. All rights reserved.
# Author: zhangxiaoan
# Contact: zhangxiaoan@didiglobal.com
import os
import time
import types
import errno
import argparse

from .config import Config

args_list = ["tgid", "pid",
             "user_threads_only",
             "kernel_threads_only",
             "state",
             "stack_storage_size"]
class BpfArgs:
    def __init__(self, d):
        self._d = d
        for k, v in d.items():
            setattr(self, k, v)

        for arg in args_list:
            try:
                getattr(self, arg)
            except Exception:
                setattr(self, arg, None)

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {repr(self._d)}>"

class App:
    def __init__(self, args, rootpath=os.path.curdir):
        self.args = args
        self.config = Config(rootpath)
        self.config.from_pyfile(os.path.join(self._get_cur_path(), "conf/config.py"))
        self.bpfs = []

    def _get_cur_path(self):
        return os.path.abspath(os.path.dirname(__file__))

    def _get_log_path(self, bp_name):
        dir_path = self.args.path
        if dir_path is None:
            dir_path = "."
        return os.path.join(dir_path, f"{bp_name}_trace.log")

    def load(self):
        for bp_name in self.config["BPF_PROGS"]:
            args = BpfArgs(self.config["ARGS"][bp_name])
            if bp_name == "offcputime":
                from .bpf.offcputime import OffCpuBpf
                bp_prog = OffCpuBpf(args, self._get_log_path(bp_name))
                bp_prog.start()
                self.bpfs.append(bp_prog)

    def stop(self):
        for bpf_prog in self.bpfs:
            bpf_prog.stop()

    def _load_one_bpf_prog(self, bpf_app):
        pass


def main():
    parser = argparse.ArgumentParser(
        description="deep edge perf tools")
    parser.add_argument("-p", "--path", metavar="PATH", dest="path", type=str,
                        help="output file path.")
    args = parser.parse_args()

    app = App(args)
    app.load()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt as e:
        app.stop()


if __name__ == "__main__":
    main()