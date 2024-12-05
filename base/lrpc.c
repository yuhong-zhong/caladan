/*
 * lrpc.c - shared memory communication channels
 */

#include <string.h>

#include <base/lrpc.h>

bool msg_send(struct msg_chan_out *chan, uint64_t cmd,
	      unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(!(cmd & LRPC_DONE_PARITY));

	if (unlikely(chan->send_head - chan->send_tail >= chan->size)) {
		chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
		if (chan->send_head - chan->send_tail == chan->size) {
#ifdef NO_CACHE_COHERENCE
			clflushopt(chan->recv_head_wb);
#endif
			return false;
		}
	}

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	cmd |= (chan->send_head & chan->size) ? 0 : LRPC_DONE_PARITY;
	dst->payload = payload;
	store_release(&dst->cmd, cmd);
	// only allow clwb after the message is written
	store_release(&chan->send_head, chan->send_head + 1);


#ifdef NO_CACHE_COHERENCE
	if (chan->send_head % (CACHE_LINE_SIZE / sizeof(*chan->tbl)) == 0)
		clwb(dst);
#endif

	return true;
}

bool msg_recv(struct msg_chan_in *chan, uint64_t *cmd_out,
	      unsigned long *payload_out)
{
	struct lrpc_msg *m = &chan->tbl[chan->recv_head & (chan->size - 1)];
	uint64_t parity = (chan->recv_head & chan->size) ?
			  0 : LRPC_DONE_PARITY;
	uint64_t cmd;

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
#ifdef NO_CACHE_COHERENCE
		clflushopt(m);
#endif
		return false;
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head++;

	store_release(chan->recv_head_wb, chan->recv_head);

#ifdef NO_CACHE_COHERENCE
	if ((chan->recv_head % (chan->size / 8)) == 0)
		clwb(chan->recv_head_wb);
#endif

	return true;
}

/* internal use only */
bool __lrpc_send(struct lrpc_chan_out *chan, uint64_t cmd,
		 unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(chan->send_head - chan->send_tail == chan->size);

	chan->send_tail = load_acquire(chan->recv_head_wb);
        if (chan->send_head - chan->send_tail == chan->size)
                return false;

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	dst->payload = payload;

	cmd |= (chan->send_head++ & chan->size) ? 0 : LRPC_DONE_PARITY;
	store_release(&dst->cmd, cmd);
	return true;
}

/**
 * lrpc_init_out - initializes an egress shared memory channel
 * @chan: the channel struct to initialize
 * @tbl: a buffer to store channel messages
 * @size: the number of message elements in the buffer
 * @recv_head_wb: a pointer to the head position of the receiver
 *
 * returns 0 if successful, or -EINVAL if @size is not a power of two.
 */
int lrpc_init_out(struct lrpc_chan_out *chan, struct lrpc_msg *tbl,
		  unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}

/**
 * lrpc_init_in - initializes an ingress shared memory channel
 * @chan: the channel struct to initialize
 * @tbl: a buffer to store channel messages
 * @size: the number of message elements in the buffer
 * @recv_head_wb: a pointer to the head position of the receiver
 *
 * returns 0 if successful, or -EINVAL if @size is not a power of two.
 */
int lrpc_init_in(struct lrpc_chan_in *chan, struct lrpc_msg *tbl,
		 unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}
