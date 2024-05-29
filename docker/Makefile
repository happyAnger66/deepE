NOW = $(shell date -u '+%Y%m%d%I%M%S')

all: build-x86 build-aarch64

build-x86:
	docker build -t harbor.intra.xiaojukeji.com/cargo/alpas:bpftrace_x86_$(NOW) \
	-f Dockerfile.x86 .
#	docker push harbor.intra.xiaojukeji.com/cargo/alpas:bpftrace_x86_$(NOW)
	echo bpftrace_x86_$(NOW)

build-aarch64:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker build -t harbor.intra.xiaojukeji.com/cargo/alpas:bpftrace_aarch64_$(NOW) \
	-f Dockerfile.aarch64 .
#	docker push harbor.intra.xiaojukeji.com/cargo/alpas:bpftrace_aarch64_$(NOW)
	echo bpftrace_aarch64_$(NOW)