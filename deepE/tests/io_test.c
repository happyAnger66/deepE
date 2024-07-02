#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

int main(int argc, char *argv[]) {
  if (argc < 3) {
    printf("Usage: %s <blocksize> <counts>\r\n", argv[0]);
    return -1;
  }

  int blocksize = atoi(argv[1]);
  int counts = atoi(argv[2]);

  int fd = open("test_file", O_RDWR | O_CREAT | O_DIRECT, 0644);
  if (fd < 0) {
    printf("open file failed: %s", strerror(errno));
    return -1;
  }

  char *buf = (char *)malloc(blocksize);
  if (!buf) {
    printf("buf malloc failed: %s", strerror(errno));
    return -1;
  }

  for (int i = 0; i < counts; i++) {
    int ret = write(fd, buf, blocksize);
    if (ret < 0) {
      printf("write error:%s", strerror(errno));
      break;
    }
  }

  printf("done success.")
  return 0;
}