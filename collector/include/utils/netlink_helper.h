#pragma once

#include <pthread.h>
#include <memory>
#include <vector>

#include <asm/types.h>
#include <linux/inet_diag.h> /* for IPv4 and IPv6 sockets */
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/sock_diag.h>
#include <linux/tcp.h>
#include <sys/socket.h>

#include "utils/netlink_utils.h"

namespace cargo {
namespace system_monitor {

enum {
  TCP_DB,
  DCCP_DB,
  UDP_DB,
  RAW_DB,
  UNIX_DG_DB,
  UNIX_ST_DB,
  UNIX_SQ_DB,
  PACKET_DG_DB,
  PACKET_R_DB,
  NETLINK_DB,
  SCTP_DB,
  MAX_DB
};

enum {
  SS_UNKNOWN,
  SS_ESTABLISHED,
  SS_SYN_SENT,
  SS_SYN_RECV,
  SS_FIN_WAIT1,
  SS_FIN_WAIT2,
  SS_TIME_WAIT,
  SS_CLOSE,
  SS_CLOSE_WAIT,
  SS_LAST_ACK,
  SS_LISTEN,
  SS_CLOSING,
  SS_MAX
};

enum {
  SCTP_STATE_CLOSED = 0,
  SCTP_STATE_COOKIE_WAIT = 1,
  SCTP_STATE_COOKIE_ECHOED = 2,
  SCTP_STATE_ESTABLISHED = 3,
  SCTP_STATE_SHUTDOWN_PENDING = 4,
  SCTP_STATE_SHUTDOWN_SENT = 5,
  SCTP_STATE_SHUTDOWN_RECEIVED = 6,
  SCTP_STATE_SHUTDOWN_ACK_SENT = 7,
};

#define SS_ALL ((1 << SS_MAX) - 1)
#define SS_CONN                                                          \
  (SS_ALL & ~((1 << SS_LISTEN) | (1 << SS_CLOSE) | (1 << SS_TIME_WAIT) | \
              (1 << SS_SYN_RECV)))

typedef struct {
  uint16_t flags;
  uint16_t bytelen;
  int16_t bitlen;
  /* These next two fields match rtvia */
  uint16_t family;
  uint32_t data[64];
} inet_prefix;

struct filter {
  int states;
  int families;
};

struct sock_diag_msg {
  uint8_t sdiag_family;
};

struct sockstat {
  unsigned int type;
  uint16_t prot;
  uint16_t raw_prot;
  inet_prefix local;
  inet_prefix remote;
  int lport;
  int rport;
  int state;
  int rq, wq;
  unsigned int ino;
  unsigned int uid;
  int refcnt;
  unsigned int iface;
  unsigned long long sk;
  char *name;
  char *peer_name;
  uint32_t mark;
};

struct udp_diag_info {
  int lport;
  int rcv_mem;
  int rcv_buf;
  int snd_mem;
  int snd_buf;
  int cache;  // cache call be used as rcv_mem/snd_mem
  int snd_mem_queue;
  int opt_mem;
  int back_log;
  int drops;
};

struct tcpmem {
  int rcv_mem;
  int rcv_buf;
  int snd_mem;
  int snd_buf;
  int cache;  // cache call be used as rcv_mem/snd_mem
  int snd_mem_queue;
  int opt_mem;
  int back_log;
  int drops;
};

struct tcp_info_ext {
  __u8 tcpi_state;
  __u8 tcpi_ca_state;
  __u8 tcpi_retransmits;
  __u8 tcpi_probes;
  __u8 tcpi_backoff;
  __u8 tcpi_options;
  __u8 tcpi_snd_wscale : 4, tcpi_rcv_wscale : 4;
  __u8 tcpi_delivery_rate_app_limited : 1;

  __u32 tcpi_rto;
  __u32 tcpi_ato;
  __u32 tcpi_snd_mss;
  __u32 tcpi_rcv_mss;

  __u32 tcpi_unacked;
  __u32 tcpi_sacked;
  __u32 tcpi_lost;
  __u32 tcpi_retrans;
  __u32 tcpi_fackets;

  /* Times. */
  __u32 tcpi_last_data_sent;
  __u32 tcpi_last_ack_sent; /* Not remembered, sorry. */
  __u32 tcpi_last_data_recv;
  __u32 tcpi_last_ack_recv;

  /* Metrics. */
  __u32 tcpi_pmtu;
  __u32 tcpi_rcv_ssthresh;
  __u32 tcpi_rtt;
  __u32 tcpi_rttvar;
  __u32 tcpi_snd_ssthresh;
  __u32 tcpi_snd_cwnd;
  __u32 tcpi_advmss;
  __u32 tcpi_reordering;

  __u32 tcpi_rcv_rtt;
  __u32 tcpi_rcv_space;

  __u32 tcpi_total_retrans;

  __u64 tcpi_pacing_rate;
  __u64 tcpi_max_pacing_rate;
  __u64 tcpi_bytes_acked;    /* RFC4898 tcpEStatsAppHCThruOctetsAcked */
  __u64 tcpi_bytes_received; /* RFC4898 tcpEStatsAppHCThruOctetsReceived */
  __u32 tcpi_segs_out;       /* RFC4898 tcpEStatsPerfSegsOut */
  __u32 tcpi_segs_in;        /* RFC4898 tcpEStatsPerfSegsIn */

  __u32 tcpi_notsent_bytes;
  __u32 tcpi_min_rtt;
  __u32 tcpi_data_segs_in;  /* RFC4898 tcpEStatsDataSegsIn */
  __u32 tcpi_data_segs_out; /* RFC4898 tcpEStatsDataSegsOut */

  __u64 tcpi_delivery_rate;

  __u64 tcpi_busy_time;      /* Time (usec) busy sending data */
  __u64 tcpi_rwnd_limited;   /* Time (usec) limited by receive window */
  __u64 tcpi_sndbuf_limited; /* Time (usec) limited by send buffer */
  __u32 tcpi_delivered;
  __u32 tcpi_delivered_ce;

  __u64 tcpi_bytes_sent;    /* RFC4898 tcpEStatsPerfHCDataOctetsOut */
  __u64 tcpi_bytes_retrans; /* RFC4898 tcpEStatsPerfOctetsRetrans */
  __u32 tcpi_dsack_dups;    /* RFC4898 tcpEStatsStackDSACKDups */
  __u32 tcpi_reord_seen;    /* reordering events seen */

  __u32 tcpi_rcv_ooopack; /* Out-of-order packets received */
  __u32 tcpi_snd_wnd;     /* peer's advertised receive window after
                           * scaling (bytes)
                           */
};

struct tcpstat {
  struct sockstat ss;
  unsigned int timer;
  unsigned int timeout;
  int probes;
  char cong_alg[16];
  double rto, ato, rtt, rttvar;
  int qack, ssthresh, backoff;
  double send_bps;
  int snd_wscale;
  int rcv_wscale;
  int mss;
  int rcv_mss;
  int advmss;
  unsigned int cwnd;
  unsigned int lastsnd;
  unsigned int lastrcv;
  unsigned int lastack;
  double pacing_rate;
  double pacing_rate_max;
  double delivery_rate;
  unsigned long long bytes_acked;
  unsigned long long bytes_received;
  unsigned int segs_out;
  unsigned int segs_in;
  unsigned int data_segs_out;
  unsigned int data_segs_in;
  unsigned int unacked;
  unsigned int retrans;        // 	tp->retrans_out += tcp_skb_pcount(skb);
  unsigned int retrans_total;  //	tp->total_retrans += segs;
  unsigned int lost;
  unsigned int sacked;
  unsigned int fackets;
  unsigned int reordering;
  unsigned int not_sent;
  double rcv_rtt;
  double min_rtt;
  int rcv_space;
  unsigned long long busy_time;
  unsigned long long rwnd_limited;
  unsigned long long sndbuf_limited;
  bool has_ts_opt;
  bool has_sack_opt;
  bool has_ecn_opt;
  bool has_ecnseen_opt;
  bool has_fastopen_opt;
  bool has_wscale_opt;
  bool app_limited;
  struct tcpmem meminfo;
  uint64_t retrans_bytes;  // tp->bytes_retrans += skb->len;
};

struct inet_diag_arg {
  struct filter *f;
  int protocol;
  struct rtnl_handle *rth;
};

class NetLinkTcp {
 public:
  NetLinkTcp() {}

  static int32_t GetAllSockets();
  static int32_t GetAllUdpSockets();
  static bool GetOneSocket(int32_t local_port, const std::string &peer_host,
                           int32_t peer_port, tcpstat &ts);
  static bool GetOneUdpSocket(int32_t local_port, udp_diag_info *udi);
  static std::shared_ptr<NetLinkTcp> Instance() {
    pthread_once(&once, &NetLinkTcp::init);
    return tcp_netlink;
  }

 private:
  static void init() { tcp_netlink = std::make_shared<NetLinkTcp>(); }
  static pthread_once_t once;
  static std::shared_ptr<NetLinkTcp> tcp_netlink;
  static std::vector<tcpstat> all_tcp_socks;
  static std::vector<udp_diag_info> all_udp_socks;
  static filter current_filter;

  static inline unsigned long long cookie_sk_get(const uint32_t *cookie) {
    return (((unsigned long long)cookie[1] << 31) << 1) | cookie[0];
  }
  static void parse_skmeminfo(struct rtattr *tb[], int attrtype, tcpmem *minfo);
  static void tcp_parse_info(const struct nlmsghdr *nlh,
                             struct inet_diag_msg *r, struct rtattr *tb[],
                             tcpstat *tcp_stat);
  static int inet_get_netlink(filter *f, int protocol);
  static int inet_parse_sock(struct nlmsghdr *nlh, tcpstat *ts);
  static void parse_sockstat(struct nlmsghdr *nlh, sockstat *s);
  static int parse_one_inet_sock(const struct sockaddr_nl *addr,
                                 struct nlmsghdr *h, void *arg);
  static int tcpdiag_send(int fd, int protocol, filter *f);
  static int sockdiag_send(int family, int fd, int protocol, filter *f);
};
}  // namespace system_monitor
}  // namespace cargo
