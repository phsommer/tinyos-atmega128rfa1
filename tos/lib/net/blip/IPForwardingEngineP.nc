/**
 * Forwarding abstractions for blip IPv6 stack.
 *
 * Routing protocols can manipulate the forwarding state using the
 * ForwardingTable interface and receive notifications of forwarding
 * events using ForwardingEvents.  In particular, the forwarding
 * events are useful for datapath validation and updating link
 * estimates.
 *
 * @author Stephen Dawson-Haggerty <stevedh@eecs.berkeley.edu>
 */

#include <PrintfUART.h>
#include <iprouting.h>
#include <lib6lowpan/ip.h>
module IPForwardingEngineP {
  provides {
    interface ForwardingTable;
    interface ForwardingEvents[uint8_t ifindex];
    interface IP;
    interface IP as IPRaw;
    interface Init;
  }
  uses {
    interface IPForward[uint8_t ifindex];
    interface IPAddress;
    interface IPPacket;

    interface Timer<TMilli> as PrintTimer;
    interface Leds;
  }
} implementation {
#define min(X,Y) (((X) < (Y)) ? (X) : (Y))

  /* simple routing table for now */
  /* we can optimize memory consumption later since most of these
     address will have known prefixes -- either LL or the shared
     global prefix. */
  /* the routing table is sorted by prefix length, so that the entries
     with the longest prefix are at the top. */
  /* if a route to the given prefix already exists, this updates it. */
  struct route_entry routing_table[ROUTE_TABLE_SZ];

  route_key_t last_key = 1;

  command error_t Init.init() {
    memset(routing_table, 0, sizeof(routing_table));
  }

  struct route_entry *alloc_entry(int pfxlen) {
    int i;
    /* full table */
    if (routing_table[ROUTE_TABLE_SZ-1].valid) return NULL;

    for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      /* if there's an invalid entry there are spare entries and we
         don't have to insert in the middle of the table. */
      if (!routing_table[i].valid) goto init_entry;
      /* we keep the table sorted by prefix length so we skip all the
         entries with longer prefixes. */
      else if (routing_table[i].prefixlen >= pfxlen) continue;

      /* we're pointing at a valid entry that is our new slot; we know
         there's at least one free entry in the table, too. */
      /* shift the table down and return the current entry; */
      memmove((void *)&routing_table[i+1], (void *)&routing_table[i],
              sizeof(struct route_entry) * (ROUTE_TABLE_SZ - i - 1));
      goto init_entry;
    }
    return NULL;
  init_entry:
    routing_table[i].valid = 1;
    routing_table[i].key = ++last_key;
    return &routing_table[i];
  }

  command route_key_t ForwardingTable.addRoute(const uint8_t *prefix, 
                                               int prefix_len_bits,
                                               struct in6_addr *next_hop,
                                               uint8_t ifindex) {
    struct route_entry *entry;
    /* no reason to support non-byte length prefixes for now... */
    if (prefix_len_bits % 8 != 0 || prefix_len_bits > 128) return ROUTE_INVAL_KEY;
    entry = call ForwardingTable.lookupRoute(prefix, prefix_len_bits);
    if (entry == NULL || entry->prefixlen != prefix_len_bits) {
      /* if there's no entry, or there's another entry but it has a
         different prefix length, we allocate a new slot in the
         table. */
      entry = alloc_entry(prefix_len_bits);
    }
    if (entry == NULL) 
      return ROUTE_INVAL_KEY;

    entry->prefixlen = prefix_len_bits;
    entry->ifindex = ifindex;
    memcpy(&entry->prefix, prefix, prefix_len_bits / 8);
    if (next_hop)
      memcpy(&entry->next_hop, next_hop, sizeof(struct in6_addr));
    return entry->key;
  }

  command error_t ForwardingTable.delRoute(route_key_t key) {
    int i;
    for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (routing_table[i].key == key) {
        memmove((void *)&routing_table[i], (void *)&routing_table[i+1],
                sizeof(struct route_entry) * (ROUTE_TABLE_SZ - i - 1));
        return SUCCESS;
      }
    }
    return FAIL;
  }

  /**
   * Look up the route to a prefix.
   *
   * If next_hop is not NULL, the next hop will be written in there. 
   * @return the route key associated with this route.
   */
  command struct route_entry *ForwardingTable.lookupRoute(const uint8_t *prefix, 
                                                          int prefix_len_bits) {
    int i;
    for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (routing_table[i].valid &&
	  ((routing_table[i].prefixlen == 0) || 
	   (memcmp(prefix, routing_table[i].prefix.s6_addr, 
		   min(prefix_len_bits, routing_table[i].prefixlen) / 8) == 0 && prefix_len_bits))) {
        /* match! */
        return &routing_table[i];
      }
    }
    return NULL;
  }
  command struct route_entry *ForwardingTable.lookupRouteKey(route_key_t key) {
    int i;
    for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (routing_table[i].valid && 
          routing_table[i].key == key)
        return &routing_table[i];
    }
    return NULL;
  }

  command struct route_entry *ForwardingTable.getTable(int *n) {
    *n = ROUTE_TABLE_SZ;
    return routing_table;
  }

  command error_t IP.send(struct ip6_packet *pkt) {

    struct route_entry *next_hop_entry = 
      call ForwardingTable.lookupRoute(pkt->ip6_hdr.ip6_dst.s6_addr, 128);

    if (!call PrintTimer.isRunning())
      call PrintTimer.startPeriodic(10000);

    if (call IPAddress.isLocalAddress(&pkt->ip6_hdr.ip6_dst) && 
        pkt->ip6_hdr.ip6_dst.s6_addr[0] != 0xff) {
      printfUART("Forwarding -- send with local unicast address!\n");
      return FAIL;
    } else if (call IPAddress.isLLAddress(&pkt->ip6_hdr.ip6_dst) &&
               (!next_hop_entry || next_hop_entry->prefixlen < 128)) {
      /* in this case, we need to figure out which interface the
         source address is attached to, and send the packet out on
         that interface. */
      /* with traditional ND we would check the cache for each
         interface and then start discover on all of them; however,
         since we're assuming that link-local addresses are on-link
         for the 15.4 side, we just send all LL addresses that way. */
      /* this is probably the worst part about not doing ND -- LL
         addressed don't work on other links...  we should probably do
         ND in this case, or at least keep a cache so we can reply to
         messages on the right interface. */
      printfUART("Forwarding -- send to LL address\n");
      pkt->ip6_hdr.ip6_hlim = 1;
      return call IPForward.send[ROUTE_IFACE_154](&pkt->ip6_hdr.ip6_dst, pkt, 
                                                  (void *)ROUTE_INVAL_KEY);
    } else if (next_hop_entry) {
      printfUART("Forwarding -- got from routing table\n");

      /* control messages do not need routing headers */
      if(pkt->ip6_hdr.ip6_nxt != IANA_ICMP)
	if (!(signal ForwardingEvents.initiate[next_hop_entry->ifindex](pkt,
                                             &next_hop_entry->next_hop)))
	  return FAIL;

      return call IPForward.send[next_hop_entry->ifindex](&next_hop_entry->next_hop, pkt, 
                                                          (void *)next_hop_entry->key);
    } 
    return FAIL;
  }

  command error_t IPRaw.send(struct ip6_packet *pkt) {
    return FAIL;
  }

  event void IPForward.recv[uint8_t ifindex](struct ip6_hdr *iph, void *payload, 
                                             struct ip6_metadata *meta) {
    struct ip6_packet pkt;
    struct in6_addr *next_hop;
    size_t len = ntohs(iph->ip6_plen);
    struct ip_iovec v[1];
    route_key_t next_hop_key = ROUTE_INVAL_KEY;
    uint8_t next_hop_ifindex;

    /* signaled before *any* processing  */
    signal IPRaw.recv(iph, payload, len, meta);

    if (call IPAddress.isLocalAddress(&iph->ip6_dst)) {
      /* local delivery */
      printfUART("Local delivery\n");
#ifdef RPL_ROUTING
      if(iph->ip6_nxt != IANA_ICMP)
	signal ForwardingEvents.deleteHeader[RPL_IFACE](iph, payload);
      //len = len - sizeof(rpl_data_hdr_t);
      len = ntohs(iph->ip6_plen);
      //payload = (uint8_t*) payload + sizeof(rpl_data_hdr_t);
#endif
      signal IP.recv(iph, payload, len, meta);
    } else {
      /* forwarding */
      int header_off = call IPPacket.findHeader(payload, len,
                                                iph->ip6_nxt, IPV6_ROUTING);
      if (!(--iph->ip6_hlim)) {
        /* TODO : ICMP error */
        return;
      }

      if (header_off >= 0) {
        //  we found a routing header in the packet
        //  look up the next hop in the header if we understand it (type 4)
        // TODO
        //  next_hop_ifindex = ifindex;
        return;
      } else {
        /* look up the next hop in the routing table */
        struct route_entry *next_hop_entry = 
          call ForwardingTable.lookupRoute(iph->ip6_dst.s6_addr,
                                           128);
        if (next_hop_entry == NULL) {
          /* oops, no route. */
          return; 
        }
        next_hop = &next_hop_entry->next_hop;
        next_hop_key = next_hop_entry->key;
        next_hop_ifindex = next_hop_entry->ifindex;
      }

      memcpy(&pkt.ip6_hdr, iph, sizeof(struct ip6_hdr));
      pkt.ip6_data = &v[0];
      v[0].iov_next = NULL;
      v[0].iov_base = payload;
      v[0].iov_len  = len;

      /* give the routing protocol a chance to do data-path validation
         on this packet. */
      /* RPL uses this to update the flow label fields */
      if (!(signal ForwardingEvents.approve[next_hop_ifindex](iph, (struct ip6_route*) payload, next_hop)))
        return;

      call IPForward.send[next_hop_ifindex](next_hop, &pkt, (void *)next_hop_key);
    }
  }
  
  event void IPForward.sendDone[uint8_t ifindex](struct send_info *status) {
    struct route_entry *entry;
    int key = (int)status->upper_data;
    printfUART("sendDone: iface: %i key: %i\n", ifindex, key);
    if (key != ROUTE_INVAL_KEY) {
      entry = call ForwardingTable.lookupRouteKey(key);
      if (entry) {
        printfUART("got entry... signal\n");
        signal ForwardingEvents.linkResult[ifindex](&entry->next_hop, status);
      }
    }
  }

  event void PrintTimer.fired() {
    int i;
    printfUART("\ndestination                 gateway            interface\n");
    for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (routing_table[i].valid) {
        printfUART_in6addr(&routing_table[i].prefix);
        printfUART("/%i\t\t", routing_table[i].prefixlen);
        printfUART_in6addr(&routing_table[i].next_hop);
        printfUART("\t\t%i\n", routing_table[i].ifindex);
      }
    }
    printfUART("\n");
  }

 default event error_t ForwardingEvents.deleteHeader[uint8_t idx](struct ip6_hdr *iph, 
								  void* payload){
   return SUCCESS;
 }

  default event bool ForwardingEvents.approve[uint8_t idx](struct ip6_hdr *iph,
                                                          struct ip6_route *rhdr,
                                                          struct in6_addr *next_hop) {
    return TRUE;
  }
  default event bool ForwardingEvents.initiate[uint8_t idx](struct ip6_packet *pkt,
                                                            struct in6_addr *next_hop) {
    return TRUE;
  }
  default event void ForwardingEvents.linkResult[uint8_t idx](struct in6_addr *host,
                                                              struct send_info * info) {}
  
  default command error_t IPForward.send[uint8_t ifindex](struct in6_addr *next_hop,
                                                          struct ip6_packet *pkt,
                                                          void *data) {
//     if (ifindex == ROUTE_IFACE_ALL) {
//       call IPForward.send[ROUTE_IFACE_PPP](next_hop, pkt, data);
//       call IPForward.send[ROUTE_IFACE_154](next_hop, pkt, data);
//     }
    return SUCCESS;
  }

  default event void IPRaw.recv(struct ip6_hdr *iph, void *payload,
                                size_t len, struct ip6_metadata *meta) {}

  event void IPAddress.changed(bool global_valid) {}
}
