# 1. What is deepE?

deep Edge perf insight tools

deepE is a performance monitor and analysis tool specially developed for end-side devices. It can be used for edge devices such as autonomous vehicles and robots. deepE can make analyse system and performance problems more easier.

deepE is still experimental and you might experience bugs, but we're working very hard to make it stable as possible.

# 2. Usage examples

## 2.1 offcpu information

![offcpu](./images/offcputime.png)


## 2.2 schedule latency information

![schedule latency](./images/runqslower.png)

# 3. How to start?

## 3.1 build docker image

```shell
docker build -t <docker-image-name> -f deepE/docker/Dockerfile.x86 .
```

## 3.2 run

```shell
docker run -tid --name deepE -v $(pwd):/deepE  -v /usr/src:/usr/src:ro  -v /lib/modules/:/lib/modules:ro -v /sys/kernel/debug/:/sys/kernel/debug:rw \
 --net=host --pid=host --privileged <docker-image> /bin/bash
```


# 4. Q && A