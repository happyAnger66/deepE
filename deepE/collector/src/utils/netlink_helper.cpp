#include "utils/netlink_helper.h"

#include <unistd.h>
#include <iostream>

namespace cargo {
namespace system_monitor {

#define TCPI_HAS_OPT(info, opt) !!(info->tcpi_options & (opt))

pthread_once_t NetLinkTcp::once = PTHREAD_ONCE_INIT;
std::shared_ptr<NetLinkTcp> NetLinkTcp::tcp_netlink;
std::vector<tcpstat> NetLinkTcp::all_tcp_socks;
std::vector<udp_diag_info> NetLinkTcp::all_udp_socks;
filter NetLinkTcp::current_filter = {
  states : SS_CONN,
  families : (1 << AF_INET) | (1 << AF_INET6),
};

void NetLinkTcp::parse_skmeminfo(struct rtattr *tb[], int attrtype,
                                 struct tcpmem *minfo) {
  const int32_t *skmeminfo;

  if (!tb[attrtype]) {
    if (attrtype == INET_DIAG_SKMEMINFO) {
      if (!tb[INET_DIAG_MEMINFO]) return;
    }
    return;
  }

  skmeminfo = reinterpret_cast<const int32_t *>(RTA_DATA(tb[attrtype]));

  minfo->rcv_mem = skmeminfo[SK_MEMINFO_RMEM_ALLOC];
  minfo->rcv_buf = skmeminfo[SK_MEMINFO_RCVBUF];
  minfo->snd_mem = skmeminfo[SK_MEMINFO_WMEM_ALLOC];
  minfo->snd_buf = skmeminfo[SK_MEMINFO_SNDBUF];
  minfo->cache = skmeminfo[SK_MEMINFO_FWD_ALLOC];
  minfo->snd_mem_queue = skmeminfo[SK_MEMINFO_WMEM_QUEUED];
  minfo->opt_mem = skmeminfo[SK_MEMINFO_OPTMEM];

  if (RTA_PAYLOAD(tb[attrtype]) >= (SK_MEMINFO_BACKLOG + 1) * sizeof(__u32))
    minfo->back_log = skmeminfo[SK_MEMINFO_BACKLOG];

  if (RTA_PAYLOAD(tb[attrtype]) >= (SK_MEMINFO_DROPS + 1) * sizeof(__u32))
    minfo->drops = skmeminfo[SK_MEMINFO_DROPS];
}

void NetLinkTcp::tcp_parse_info(const struct nlmsghdr *nlh,
                                struct inet_diag_msg *r, struct rtattr *tb[],
                                tcpstat *tcp_stat) {
  double rtt = 0;

  tcp_stat->ss.state = r->idiag_state;

  parse_skmeminfo(tb, INET_DIAG_SKMEMINFO, &(tcp_stat->meminfo));

  if (tb[INET_DIAG_INFO]) {
    struct tcp_info_ext *info;
    int len = RTA_PAYLOAD(tb[INET_DIAG_INFO]);

    /* workaround for older kernels with less fields */
    if (len < sizeof(*info)) {
      info = reinterpret_cast<tcp_info_ext *>(alloca(sizeof(*info)));
      memset(reinterpret_cast<char *>(info), 0, sizeof(*info));
      memcpy(info, RTA_DATA(tb[INET_DIAG_INFO]), len);
    } else {
      info = reinterpret_cast<tcp_info_ext *>(RTA_DATA(tb[INET_DIAG_INFO]));
    }

    tcp_stat->has_ts_opt = TCPI_HAS_OPT(info, TCPI_OPT_TIMESTAMPS);
    tcp_stat->has_sack_opt = TCPI_HAS_OPT(info, TCPI_OPT_SACK);
    tcp_stat->has_ecn_opt = TCPI_HAS_OPT(info, TCPI_OPT_ECN);
    tcp_stat->has_ecnseen_opt = TCPI_HAS_OPT(info, TCPI_OPT_ECN_SEEN);
    tcp_stat->has_fastopen_opt = TCPI_HAS_OPT(info, TCPI_OPT_SYN_DATA);

    if (tb[INET_DIAG_CONG])
      strncpy(tcp_stat->cong_alg, rta_getattr_str(tb[INET_DIAG_CONG]),
              sizeof(tcp_stat->cong_alg) - 1);

    if (TCPI_HAS_OPT(info, TCPI_OPT_WSCALE)) {
      tcp_stat->has_wscale_opt = true;
      tcp_stat->snd_wscale = info->tcpi_snd_wscale;
      tcp_stat->rcv_wscale = info->tcpi_rcv_wscale;
    }

    if (info->tcpi_rto && info->tcpi_rto != 3000000)
      tcp_stat->rto = (double)info->tcpi_rto / 1000;

    tcp_stat->backoff = info->tcpi_backoff;
    tcp_stat->rtt = (double)info->tcpi_rtt / 1000;
    tcp_stat->rttvar = (double)info->tcpi_rttvar / 1000;
    tcp_stat->ato = (double)info->tcpi_ato / 1000;
    tcp_stat->mss = info->tcpi_snd_mss;
    tcp_stat->rcv_mss = info->tcpi_rcv_mss;
    tcp_stat->advmss = info->tcpi_advmss;
    tcp_stat->rcv_space = info->tcpi_rcv_space;
    tcp_stat->rcv_rtt = (double)info->tcpi_rcv_rtt / 1000;
    tcp_stat->lastsnd = info->tcpi_last_data_sent;
    tcp_stat->lastrcv = info->tcpi_last_data_recv;
    tcp_stat->lastack = info->tcpi_last_ack_recv;
    tcp_stat->unacked = info->tcpi_unacked;
    tcp_stat->retrans =
        info->tcpi_retrans;  // tcp retrans segs nums. kernel:tp->retrans_out +=
                             // tcp_skb_pcount(skb);
    tcp_stat->retrans_total = info->tcpi_total_retrans;
    tcp_stat->retrans_bytes = info->tcpi_bytes_retrans;
    tcp_stat->lost = info->tcpi_lost;
    tcp_stat->reordering = info->tcpi_reordering;
    tcp_stat->cwnd = info->tcpi_snd_cwnd;

    if (info->tcpi_snd_ssthresh < 0xFFFF)
      tcp_stat->ssthresh = info->tcpi_snd_ssthresh;

    rtt = (double)info->tcpi_rtt;

    if (rtt > 0 && info->tcpi_snd_mss && info->tcpi_snd_cwnd) {
      tcp_stat->send_bps = (double)info->tcpi_snd_cwnd *
                           (double)info->tcpi_snd_mss * 8000000. / rtt;
    }

    if (info->tcpi_pacing_rate && info->tcpi_pacing_rate != ~0ULL) {
      tcp_stat->pacing_rate = info->tcpi_pacing_rate * 8.;

      if (info->tcpi_max_pacing_rate && info->tcpi_max_pacing_rate != ~0ULL)
        tcp_stat->pacing_rate_max = info->tcpi_max_pacing_rate * 8.;
    }
    tcp_stat->bytes_acked = info->tcpi_bytes_acked;
    tcp_stat->bytes_received = info->tcpi_bytes_received;
    tcp_stat->segs_out = info->tcpi_segs_out;
    tcp_stat->segs_in = info->tcpi_segs_in;
    tcp_stat->data_segs_out = info->tcpi_data_segs_out;
    tcp_stat->data_segs_in = info->tcpi_data_segs_in;
    tcp_stat->not_sent = info->tcpi_notsent_bytes;
    if (info->tcpi_min_rtt && info->tcpi_min_rtt != ~0U)
      tcp_stat->min_rtt = (double)info->tcpi_min_rtt / 1000;
    tcp_stat->delivery_rate = info->tcpi_delivery_rate * 8.;
    tcp_stat->app_limited = info->tcpi_delivery_rate_app_limited;
    tcp_stat->busy_time = info->tcpi_busy_time;
    tcp_stat->rwnd_limited = info->tcpi_rwnd_limited;
    tcp_stat->sndbuf_limited = info->tcpi_sndbuf_limited;
  }
}

int NetLinkTcp::inet_parse_sock(struct nlmsghdr *nlh, tcpstat *ts) {
  struct rtattr *tb[INET_DIAG_MAX + 1];
  struct inet_diag_msg *r =
      reinterpret_cast<struct inet_diag_msg *>(NLMSG_DATA(nlh));
  struct sockstat *s = &(ts->ss);

  parse_rtattr(tb, INET_DIAG_MAX, (struct rtattr *)(r + 1),
               nlh->nlmsg_len - NLMSG_LENGTH(sizeof(*r)));

  if (tb[INET_DIAG_PROTOCOL]) s->type = rta_getattr_u8(tb[INET_DIAG_PROTOCOL]);

  if (s->type == IPPROTO_TCP) tcp_parse_info(nlh, r, tb, ts);

  if (s->type == IPPROTO_UDP) {
    udp_diag_info udi{};
    tcpmem mem_info{};
    parse_skmeminfo(tb, INET_DIAG_SKMEMINFO, &mem_info);

    udi.lport = s->lport;
    udi.rcv_buf = mem_info.rcv_buf;
    udi.rcv_mem = mem_info.rcv_mem;
    udi.drops = mem_info.drops;

    all_udp_socks.push_back(udi);
  }

  return 0;
}

void NetLinkTcp::parse_sockstat(struct nlmsghdr *nlh, sockstat *s) {
  struct rtattr *tb[INET_DIAG_MAX + 1];
  struct inet_diag_msg *r =
      reinterpret_cast<struct inet_diag_msg *>(NLMSG_DATA(nlh));

  parse_rtattr(tb, INET_DIAG_MAX, (struct rtattr *)(r + 1),
               nlh->nlmsg_len - NLMSG_LENGTH(sizeof(*r)));

  s->state = r->idiag_state;
  s->local.family = s->remote.family = r->idiag_family;
  s->lport = ntohs(r->id.idiag_sport);
  s->rport = ntohs(r->id.idiag_dport);
  s->wq = r->idiag_wqueue;
  s->rq = r->idiag_rqueue;
  s->ino = r->idiag_inode;
  s->uid = r->idiag_uid;
  s->iface = r->id.idiag_if;
  s->sk = cookie_sk_get(&r->id.idiag_cookie[0]);

  s->mark = 0;
  if (tb[INET_DIAG_MARK]) s->mark = rta_getattr_u32(tb[INET_DIAG_MARK]);
  if (tb[INET_DIAG_PROTOCOL])
    s->raw_prot = rta_getattr_u8(tb[INET_DIAG_PROTOCOL]);
  else
    s->raw_prot = 0;

  if (s->local.family == AF_INET)
    s->local.bytelen = s->remote.bytelen = 4;
  else
    s->local.bytelen = s->remote.bytelen = 16;

  memcpy(s->local.data, r->id.idiag_src, s->local.bytelen);
  memcpy(s->remote.data, r->id.idiag_dst, s->local.bytelen);
}

int NetLinkTcp::parse_one_inet_sock(const struct sockaddr_nl *addr,
                                    struct nlmsghdr *h, void *arg) {
  int err;
  struct inet_diag_arg *diag_arg =
      reinterpret_cast<struct inet_diag_arg *>(arg);
  struct inet_diag_msg *r =
      reinterpret_cast<struct inet_diag_msg *>(NLMSG_DATA(h));
  tcpstat ts{};
  struct sockstat *s = &(ts.ss);

  if (!(diag_arg->f->families & (1 << r->idiag_family))) return 0;

  parse_sockstat(h, s);
  s->type = diag_arg->protocol;

  err = inet_parse_sock(h, &ts);
  if (err < 0) return err;

  all_tcp_socks.push_back(ts);
  return 0;
}

constexpr int kMagicSeq = 123456;

int NetLinkTcp::tcpdiag_send(int fd, int protocol, filter *f) {
  struct sockaddr_nl nladdr = {.nl_family = AF_NETLINK};
  struct {
    struct nlmsghdr nlh;
    struct inet_diag_req r;
  } req;

  req.nlh.nlmsg_len = sizeof(req);
  req.nlh.nlmsg_flags = NLM_F_ROOT | NLM_F_MATCH | NLM_F_REQUEST;
  req.nlh.nlmsg_seq = kMagicSeq;
  req.r.idiag_family = AF_INET;
  req.r.idiag_states = f->states;

  struct msghdr msg;
  struct rtattr rta;
  struct iovec iov[3];
  int iovlen = 1;

  if (protocol == IPPROTO_UDP) return -1;

  if (protocol == IPPROTO_TCP)
    req.nlh.nlmsg_type = TCPDIAG_GETSOCK;
  else
    req.nlh.nlmsg_type = DCCPDIAG_GETSOCK;

  req.r.idiag_ext |= (1 << (INET_DIAG_MEMINFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_SKMEMINFO - 1));

  req.r.idiag_ext |= (1 << (INET_DIAG_INFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_VEGASINFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_CONG - 1));

  iov[0] = (struct iovec){.iov_base = &req, .iov_len = sizeof(req)};

  msg.msg_name = (void *)&nladdr;
  msg.msg_namelen = sizeof(nladdr);
  msg.msg_iov = iov;
  msg.msg_iovlen = static_cast<size_t>(iovlen);

  if (sendmsg(fd, &msg, 0) < 0) {
    close(fd);
    return -1;
  }

  return 0;
}

int NetLinkTcp::sockdiag_send(int family, int fd, int protocol, filter *f) {
  struct sockaddr_nl nladdr = {.nl_family = AF_NETLINK};
  struct {
    struct nlmsghdr nlh;
    struct inet_diag_req_v2 r;
  } req;
  req.nlh.nlmsg_type = SOCK_DIAG_BY_FAMILY;
  req.nlh.nlmsg_flags = NLM_F_ROOT | NLM_F_MATCH | NLM_F_REQUEST;
  req.nlh.nlmsg_seq = kMagicSeq;
  req.nlh.nlmsg_len = sizeof(req);
  struct msghdr msg;
  struct rtattr rta;
  struct iovec iov[3];
  int iovlen = 1;

  if (family == PF_UNSPEC) return tcpdiag_send(fd, protocol, f);

  memset(&req.r, 0, sizeof(req.r));
  req.r.sdiag_family = family;
  req.r.sdiag_protocol = protocol;
  req.r.idiag_states = f->states;
  req.r.idiag_ext |= (1 << (INET_DIAG_MEMINFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_SKMEMINFO - 1));

  req.r.idiag_ext |= (1 << (INET_DIAG_INFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_VEGASINFO - 1));
  req.r.idiag_ext |= (1 << (INET_DIAG_CONG - 1));

  iov[0] = (struct iovec){.iov_base = &req, .iov_len = sizeof(req)};

  msg = (struct msghdr){
      .msg_name = (void *)&nladdr,
      .msg_namelen = sizeof(nladdr),
      .msg_iov = iov,
      .msg_iovlen = static_cast<size_t>(iovlen),
  };

  if (sendmsg(fd, &msg, 0) < 0) {
    close(fd);
    return -1;
  }

  return 0;
}

int NetLinkTcp::inet_get_netlink(filter *f, int protocol) {
  int err = 0;
  struct rtnl_handle rth, rth2;
  int family = PF_INET;
  struct inet_diag_arg arg = {.f = f, .protocol = protocol};

  if (rtnl_open_byproto(&rth, 0, NETLINK_SOCK_DIAG)) return -1;

  rth.dump = kMagicSeq;
  rth.dump_fp = NULL;

again:
  if ((err = sockdiag_send(family, rth.fd, protocol, f))) goto Exit;

  if ((err = rtnl_dump_filter(&rth, parse_one_inet_sock, &arg))) {
    if (family != PF_UNSPEC) {
      family = PF_UNSPEC;
      goto again;
    }
    goto Exit;
  }
  if (family == PF_INET) {
    family = PF_INET6;
    goto again;
  }

Exit:
  rtnl_close(&rth);
  return err;
}

int32_t NetLinkTcp::GetAllSockets() {
  all_tcp_socks.clear();
  return inet_get_netlink(const_cast<filter *>(&current_filter), IPPROTO_TCP);
}

int32_t NetLinkTcp::GetAllUdpSockets() {
  all_udp_socks.clear();
  return inet_get_netlink(const_cast<filter *>(&current_filter), IPPROTO_UDP);
}

const char *format_host(int af, int len, const void *addr) {
  static char buf[256];

  switch (af) {
    case AF_INET:
    case AF_INET6:
      return inet_ntop(af, addr, buf, sizeof(buf));
    default:
      return "???";
  }
}

bool NetLinkTcp::GetOneUdpSocket(int32_t local_port, udp_diag_info *udi) {
  for (auto &udp_sock : all_udp_socks) {
    if (udp_sock.lport == local_port) {
      *udi = udp_sock;
      return true;
    }
  }

  return false;
}

bool NetLinkTcp::GetOneSocket(int32_t local_port, const std::string &peer_host,
                              int32_t peer_port, tcpstat &ts) {
  for (auto &tcp_sock : all_tcp_socks) {
    sockstat *ss = &(tcp_sock.ss);
    const char *dst_addr = format_host(ss->remote.family, 4, ss->remote.data);
    if (ss->lport == local_port && ss->rport == peer_port &&
        peer_host == std::string(dst_addr)) {
      ts = tcp_sock;
      return true;
    } else if (0 == local_port && ss->rport == peer_port &&
               peer_host == std::string(dst_addr)) {
      ts = tcp_sock;
      return true;
    }
  }

  return false;
}

}  // namespace system_monitor
}  // namespace cargo
