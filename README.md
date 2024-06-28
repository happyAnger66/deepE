# 1. What is deepE?

deep Edge perf insight tools

deepE is a performance monitor and analysis tool specially developed for end-side devices. It can be used for edge devices such as autonomous vehicles and robots. deepE can make analyse system and performance problems more easier.

deepE is still experimental and you might experience bugs, but we're working very hard to make it stable as possible.

[Introduction](https://blog.csdn.net/happyAnger6/article/details/139602370?spm=1001.2014.3001.5501)

# 2. Usage examples

## 2.1 offcpu information

![offcpu](https://github.com/happyAnger66/deepE/blob/main/deepE/images/offcputime.png)


## 2.2 schedule latency information

![schedule latency](https://github.com/happyAnger66/deepE/blob/main/deepE/images/runqslower.png)

## 2.3 mutex lock problem.

`test program`

```c++
#include <iostream>
#include <thread>
#include <mutex>

static std::mutex lock;

void func1() {
  int loop = 5;
  std::lock_guard<std::mutex> l(lock);
  while (loop--) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return;
}

void func2() {
  std::lock_guard<std::mutex> l(lock);
  std::cout << "get lock success" << std::endl;
}

int main() {
  std::thread t1(func1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::thread t2(func2);

  t1.join();
  t2.join();
}

```

我们构造两个线程，其中一个线程在加锁状态下sleep 5s, 另外一个线程将会阻塞在锁上, 模拟实际环境中可能因锁造成的阻塞。另外主线程等待子线程结束也会阻塞在锁上.

启动测试程序和deepE
deepE使用说明

启动测试程序
```shell
$ g++ lock_test.cc  -o lock_test -lpthread  
$ ./lock_test
get lock success
```

启动deepE
``` shell
# deepE
bpf_prog:OffCpuBpf open events:offt_events
bpf_prog:RunqSlowerBpf open events:rqs_events
bpf looping...
```

等测试程序运行完后，ctrl+c停止deepE. 将当前目录下的文件offcputime_trace.log拷贝出来.   
然后拖到ui.perfetto.dev 网址页面中. 即可看到阻塞的线程和调用栈.

![lock problem](https://github.com/happyAnger66/deepE/blob/main/deepE/images/lock_test.png)


总结
我们可以利用这个offcputime来分析实际环境中因为各种原因造成的进程阻塞及卡顿问题.

# 3. How to start?

## 3.1 build docker image for x86

```shell
docker build -t <docker-image-name> -f deepE/docker/Dockerfile.x86 .
```

## 3.2 build docker image for aarch64 in x86 host

```shell
# install docker and qemu
$ sudo yum install -y yum-utils
$ sudo yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo 
$ sudo yum install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin \
 qemu-system-arm qemu qemu-user qemu-kvm qemu-kvm-tools libvirt virt-install \
 libvirt-python libguestfs-tools-c

 # restart docker and set up qemu-user-static
$ sudo systemctl restart docker
$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

```

## 3.2 run

```shell
docker run -ti --name deepE -v $(pwd):/deepE  -v /usr/src:/usr/src:ro  -v /lib/modules/:/lib/modules:ro -v /sys/kernel/debug/:/sys/kernel/debug:rw \
 --net=host --pid=host --privileged <docker-image> /bin/bash

~# deepE
bpf_prog:OffCpuBpf open events:offt_events
bpf_prog:RunqSlowerBpf open events:rqs_events
bpf looping...
```


# 4. Q && A

# 5.

[SigNoz](https://github.com/SigNoz)