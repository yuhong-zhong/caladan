/*
 * commands.c - dataplane commands to/from runtimes
 */

#include <rte_mbuf.h>

#include <base/log.h>
#include <base/lrpc.h>
#include <iokernel/queue.h>

#include "defs.h"

static int commands_drain_queue(struct thread *t, unsigned long *bufs, int n)
{
	int i, n_bufs = 0;

	for (i = 0; i < n; i++) {
		uint64_t cmd;
		unsigned long payload;

		if (!lrpc_recv(&t->txcmdq, &cmd, &payload))
			break;

		switch (cmd) {
		case TXCMD_NET_COMPLETE:
			bufs[n_bufs++] = payload;
			/* TODO: validate pointer @buf */
			break;

		default:
			/* kill the runtime? */
			BUG();
		}
	}

	return n_bufs;
}

static int commands_drain_queue_from_seciok(struct msg_chan_in *chan, unsigned long *bufs, int n)
{
	int i, n_bufs = 0;

	for (i = 0; i < n; i++) {
		uint64_t cmd, raw_cmd;
		// uint32_t ip_addr;
		unsigned long payload;
		bool success;

		log_info_duration(success = msg_recv(chan, &cmd, &payload));
		if (!success)
			break;

		raw_cmd = IOK2IOK_GET_RAWCMD(cmd);
		// ip_addr = IOK2IOK_GET_IP(cmd);

		RT_BUG_ON(raw_cmd != TXCMD_NET_COMPLETE);
		bufs[n_bufs++] = payload;
	}

	return n_bufs;
}

static void commands_send_to_pmyiok(struct msg_chan_out *chan, unsigned long *bufs, int n)
{
	int i;
	bool success;

	for (i = 0; i < n; i++) {
		log_info_duration(success = msg_send(chan, IOK2IOK_MAKE_CMD(TXCMD_NET_COMPLETE, 0), bufs[i]));
		if (!success) {
			log_err_ratelimited("commands_send_to_pmyiok: failed to send to primary iokernel");
		}
	}
}

/*
 * Process a batch of commands from runtimes.
 */
bool commands_rx(void)
{
	unsigned long bufs[IOKERNEL_CMD_BURST_SIZE];
	int i, n_bufs = 0;
	static unsigned int pos = 0;

	/*
	 * Poll each thread in each runtime until all have been polled or we
	 * have processed CMD_BURST_SIZE commands.
	 */
	for (i = 0; i < nrts; i++) {
		unsigned int idx = (pos + i) % nrts;

		if (n_bufs >= IOKERNEL_CMD_BURST_SIZE)
			break;
		n_bufs += commands_drain_queue(ts[idx], &bufs[n_bufs],
				IOKERNEL_CMD_BURST_SIZE - n_bufs);
	}
	if (!cfg.is_secondary) {
		for (i = 0; i < MAX_NR_IOK2IOK; ++i) {
			if (n_bufs >= IOKERNEL_CMD_BURST_SIZE)
				break;
			n_bufs += commands_drain_queue_from_seciok(&iok_as_primary_txcmdq[i],
					&bufs[n_bufs], IOKERNEL_CMD_BURST_SIZE - n_bufs);
		}
	}

	STAT_INC(COMMANDS_PULLED, n_bufs);

	pos++;
	if (cfg.is_secondary) {
		commands_send_to_pmyiok(&iok_as_secondary_txcmdq[cfg.seciok_index], bufs, n_bufs);
	} else {
		for (i = 0; i < n_bufs; i++)
			rte_pktmbuf_free(shmptr_to_ptr(&dp.ingress_mbuf_region, bufs[i], sizeof(struct rte_mbuf)));
	}
	return n_bufs > 0;
}
