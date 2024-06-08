#pragma once
#include <arpa/inet.h>
#include <asm/types.h>
#include <linux/if_addr.h>
#include <linux/if_link.h>
#include <linux/neighbour.h>
#include <linux/netconf.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

struct rtnl_handle {
  int fd;
  struct sockaddr_nl local;
  struct sockaddr_nl peer;
  __u32 seq;
  __u32 dump;
  int proto;
  FILE *dump_fp;
#define RTNL_HANDLE_F_LISTEN_ALL_NSID 0x01
#define RTNL_HANDLE_F_SUPPRESS_NLERR 0x02
  int flags;
};

struct nlmsg_list {
  struct nlmsg_list *next;
  struct nlmsghdr h;
};

struct nlmsg_chain {
  struct nlmsg_list *head;
  struct nlmsg_list *tail;
};

int rtnl_open(struct rtnl_handle *rth, unsigned int subscriptions)
    __attribute__((warn_unused_result));

int rtnl_open_byproto(struct rtnl_handle *rth, unsigned int subscriptions,
                      int protocol) __attribute__((warn_unused_result));

void rtnl_close(struct rtnl_handle *rth);

int parse_rtattr(struct rtattr *tb[], int max, struct rtattr *rta, int len);

int parse_rtattr_flags(struct rtattr *tb[], int max, struct rtattr *rta,
                       int len, unsigned short flags);

static inline __u8 rta_getattr_u8(const struct rtattr *rta) {
  return *(__u8 *)RTA_DATA(rta);
}
static inline __u16 rta_getattr_u16(const struct rtattr *rta) {
  return *(__u16 *)RTA_DATA(rta);
}
static inline __be16 rta_getattr_be16(const struct rtattr *rta) {
  return ntohs(rta_getattr_u16(rta));
}
static inline __u32 rta_getattr_u32(const struct rtattr *rta) {
  return *(__u32 *)RTA_DATA(rta);
}
static inline __be32 rta_getattr_be32(const struct rtattr *rta) {
  return ntohl(rta_getattr_u32(rta));
}
static inline __u64 rta_getattr_u64(const struct rtattr *rta) {
  __u64 tmp;

  memcpy(&tmp, RTA_DATA(rta), sizeof(__u64));
  return tmp;
}
static inline const char *rta_getattr_str(const struct rtattr *rta) {
  return (const char *)RTA_DATA(rta);
}

typedef int (*rtnl_filter_t)(const struct sockaddr_nl *, struct nlmsghdr *n,
                             void *);

struct rtnl_dump_filter_arg {
  rtnl_filter_t filter;
  void *arg1;
  __u16 nc_flags;
};

int rtnl_dump_filter_l(struct rtnl_handle *rth,
                       const struct rtnl_dump_filter_arg *arg);
int rtnl_dump_filter_nc(struct rtnl_handle *rth, rtnl_filter_t filter,
                        void *arg, __u16 nc_flags);

#define rtnl_dump_filter(rth, filter, arg) \
  rtnl_dump_filter_nc(rth, filter, arg, 0)
