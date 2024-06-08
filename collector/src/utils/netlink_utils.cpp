#include "utils/netlink_utils.h"
#include <errno.h>

static constexpr int rcvbuf = 1024 * 1024;

void rtnl_close(struct rtnl_handle *rth) {
  if (rth->fd >= 0) {
    close(rth->fd);
    rth->fd = -1;
  }
}

int rtnl_open_byproto(struct rtnl_handle *rth, unsigned int subscriptions,
                      int protocol) {
  socklen_t addr_len;
  int sndbuf = 32768;
  int one = 1;

  memset(rth, 0, sizeof(*rth));

  rth->proto = protocol;
  rth->fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, protocol);
  if (rth->fd < 0) {
    perror("Cannot open netlink socket");
    return -1;
  }

  if (setsockopt(rth->fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
    perror("SO_SNDBUF");
    return -1;
  }

  if (setsockopt(rth->fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
    perror("SO_RCVBUF");
    return -1;
  }

  /* Older kernels may no support extended ACK reporting */
  setsockopt(rth->fd, SOL_NETLINK, NETLINK_EXT_ACK, &one, sizeof(one));

  memset(&rth->local, 0, sizeof(rth->local));
  rth->local.nl_family = AF_NETLINK;
  rth->local.nl_groups = subscriptions;

  if (bind(rth->fd, (struct sockaddr *)&rth->local, sizeof(rth->local)) < 0) {
    perror("Cannot bind netlink socket");
    return -1;
  }
  addr_len = sizeof(rth->local);
  if (getsockname(rth->fd, (struct sockaddr *)&rth->local, &addr_len) < 0) {
    perror("Cannot getsockname");
    return -1;
  }
  if (addr_len != sizeof(rth->local)) {
    fprintf(stderr, "Wrong address length %d\n", addr_len);
    return -1;
  }
  if (rth->local.nl_family != AF_NETLINK) {
    fprintf(stderr, "Wrong address family %d\n", rth->local.nl_family);
    return -1;
  }
  rth->seq = time(NULL);
  return 0;
}

int rtnl_open(struct rtnl_handle *rth, unsigned int subscriptions) {
  return rtnl_open_byproto(rth, subscriptions, NETLINK_ROUTE);
}

int parse_rtattr(struct rtattr *tb[], int max, struct rtattr *rta, int len) {
  return parse_rtattr_flags(tb, max, rta, len, 0);
}

int parse_rtattr_flags(struct rtattr *tb[], int max, struct rtattr *rta,
                       int len, unsigned short flags) {
  unsigned short type;

  memset(tb, 0, sizeof(struct rtattr *) * (max + 1));
  while (RTA_OK(rta, len)) {
    type = rta->rta_type & ~flags;
    if ((type <= max) && (!tb[type])) tb[type] = rta;
    rta = RTA_NEXT(rta, len);
  }
  if (len) fprintf(stderr, "!!!Deficit %d, rta_len=%d\n", len, rta->rta_len);
  return 0;
}

static int rtnl_dump_done(struct nlmsghdr *h) {
  int len = *(int *)NLMSG_DATA(h);

  if (h->nlmsg_len < NLMSG_LENGTH(sizeof(int))) {
    fprintf(stderr, "DONE truncated\n");
    return -1;
  }

  if (len < 0) {
    errno = -len;
    switch (errno) {
      case ENOENT:
      case EOPNOTSUPP:
        return -1;
      case EMSGSIZE:
        fprintf(stderr, "Error: Buffer too small for object.\n");
        break;
      default:
        perror("RTNETLINK answers");
    }
    return len;
  }

  return 0;
}

static void rtnl_dump_error(const struct rtnl_handle *rth, struct nlmsghdr *h) {
  if (h->nlmsg_len < NLMSG_LENGTH(sizeof(struct nlmsgerr))) {
    fprintf(stderr, "ERROR truncated\n");
  } else {
    const struct nlmsgerr *err = (struct nlmsgerr *)NLMSG_DATA(h);

    errno = -err->error;
    if (rth->proto == NETLINK_SOCK_DIAG &&
        (errno == ENOENT || errno == EOPNOTSUPP))
      return;

    if (!(rth->flags & RTNL_HANDLE_F_SUPPRESS_NLERR))
      perror("RTNETLINK answers");
  }
}

int rtnl_dump_filter_l(struct rtnl_handle *rth,
                       const struct rtnl_dump_filter_arg *arg) {
  struct sockaddr_nl nladdr;
  struct iovec iov;
  struct msghdr msg = {
      .msg_name = &nladdr,
      .msg_namelen = sizeof(nladdr),
      .msg_iov = &iov,
      .msg_iovlen = 1,
  };
  char buf[32768];
  int dump_intr = 0;

  iov.iov_base = buf;
  while (1) {
    int status;
    const struct rtnl_dump_filter_arg *a;
    int found_done = 0;
    int msglen = 0;

    iov.iov_len = sizeof(buf);
    status = recvmsg(rth->fd, &msg, 0);

    if (status < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      fprintf(stderr, "netlink receive error %s (%d)\n", strerror(errno),
              errno);
      return -1;
    }

    if (status == 0) {
      fprintf(stderr, "EOF on netlink\n");
      return -1;
    }

    if (rth->dump_fp) fwrite(buf, 1, NLMSG_ALIGN(status), rth->dump_fp);

    for (a = arg; a->filter; a++) {
      struct nlmsghdr *h = (struct nlmsghdr *)buf;

      msglen = status;

      while (NLMSG_OK(h, msglen)) {
        int err = 0;

        h->nlmsg_flags &= ~a->nc_flags;

        if (nladdr.nl_pid != 0 || h->nlmsg_pid != rth->local.nl_pid ||
            h->nlmsg_seq != rth->dump)
          goto skip_it;

        if (h->nlmsg_flags & NLM_F_DUMP_INTR) dump_intr = 1;

        if (h->nlmsg_type == NLMSG_DONE) {
          err = rtnl_dump_done(h);
          if (err < 0) return -1;

          found_done = 1;
          break; /* process next filter */
        }

        if (h->nlmsg_type == NLMSG_ERROR) {
          rtnl_dump_error(rth, h);
          return -1;
        }

        if (!rth->dump_fp) {
          err = a->filter(&nladdr, h, a->arg1);
          if (err < 0) return err;
        }

      skip_it:
        h = NLMSG_NEXT(h, msglen);
      }
    }

    if (found_done) {
      if (dump_intr)
        fprintf(stderr, "Dump was interrupted and may be inconsistent.\n");
      return 0;
    }

    if (msg.msg_flags & MSG_TRUNC) {
      fprintf(stderr, "Message truncated\n");
      continue;
    }
    if (msglen) {
      fprintf(stderr, "!!!Remnant of size %d\n", msglen);
      exit(1);
    }
  }
}

int rtnl_dump_filter_nc(struct rtnl_handle *rth, rtnl_filter_t filter,
                        void *arg1, __u16 nc_flags) {
  const struct rtnl_dump_filter_arg a[2] = {
      {
          .filter = filter,
          .arg1 = arg1,
          .nc_flags = nc_flags,
      },
      {
          .filter = NULL,
          .arg1 = NULL,
          .nc_flags = 0,
      },
  };

  return rtnl_dump_filter_l(rth, a);
}