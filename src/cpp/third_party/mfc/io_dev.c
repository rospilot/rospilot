/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * I/O device implementation.
 *
 * Copyright 2012 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <linux/videodev2.h>
#include <errno.h>

#include<third_party/mfc/io_dev.h>
#include<third_party/mfc/mfc.h>
#include<third_party/mfc/func_dev.h>

/* return immediately if there is at least one device ready,
   waits until at least one device is ready, returns number of ready devices */
int wait_for_ready_devs(struct io_dev *chain[], int ndev)
{
	struct pollfd fds[ndev];
	int nfds;
	int i, j;
	int ret;

	nfds = 0;

	for (i = 0; i < ndev; ++i) {
		if (chain[i]->io[DIR_IN].state == FS_READY)
			return 1;

		if (chain[i]->io[DIR_OUT].state == FS_READY)
			return 1;

		if (chain[i]->fd < 0)
			continue;

		fds[nfds].events = 0;

		if (chain[i]->io[DIR_IN].state == FS_BUSY)
			fds[nfds].events |= POLLOUT;

		if (chain[i]->io[DIR_OUT].state == FS_BUSY)
			fds[nfds].events |= POLLIN | POLLPRI;

		if (fds[nfds].events != 0) {
			fds[nfds].fd = chain[i]->fd;
			dbg("Will poll fd=%d events=%d", fds[nfds].fd, fds[nfds].events);
			++nfds;
		}
	}

	if (nfds == 0)
		return 0;

	ret = poll(fds, nfds, -1);
	if (ret <= 0)
		return ret;

	for (i = 0, j = 0; j < nfds; ++j) {
		while (fds[j].fd != chain[i]->fd)
			++i;

		if (fds[j].revents & POLLOUT)
			chain[i]->io[DIR_IN].state = FS_READY;

		if (fds[j].revents & POLLIN)
			chain[i]->io[DIR_OUT].state = FS_READY;

		if (fds[j].revents & POLLPRI)
			chain[i]->event = 1;
	}

	return ret;
}

void print_chain(struct io_dev *chain[], int ndev)
{
	int i;
	struct io_port *in, *out;
	static char *ch_state[] = {"Off", "Bus", "Rdy", "Evt", "End"};

	fprintf(stderr, "State [enq cnt/max]: ");

	for (i = 0; i < ndev; ++i) {
		in = &chain[i]->io[DIR_IN];
		out = &chain[i]->io[DIR_OUT];
		fprintf(stderr, "[%s%s %d %d/%d|%s %d %d/%d] ",
				ch_state[in->state], chain[i]->event ? "+ev" : "", in->nbufs, in->counter,
				in->limit, ch_state[out->state], out->nbufs,
				out->counter, out->limit);
	}
	fprintf(stderr, "\n");
}

int process_pair(struct io_dev *in, struct io_dev *out)
{
	int idx;

	idx = 0;
	if (in->io[DIR_OUT].state == FS_READY) {
		idx = in->ops->deq_buf(in, DIR_OUT);
		if (out->io[DIR_IN].state != FS_END) {
			if (in->io[DIR_OUT].state == FS_END &&
							!out->io[DIR_IN].limit)
				out->io[DIR_IN].limit = out->io[DIR_IN].counter
					+ out->io[DIR_IN].nbufs
					+ (idx >= 0 ? 1 : 0);
			if (idx >= 0)
				idx = out->ops->enq_buf(out, DIR_IN, idx);
		}
	}

	if (idx < 0)
		return idx;

	if (out->io[DIR_IN].state == FS_READY) {
		idx = out->ops->deq_buf(out, DIR_IN);
		if (in->io[DIR_OUT].state != FS_END && idx >= 0)
			idx = in->ops->enq_buf(in, DIR_OUT, idx);
	}

	if (idx < 0)
		return idx;

	if (in->event)
		idx = in->ops->deq_event(in);

	return idx >= 0 ? 0 : -1;
}

int process_chain(struct io_dev *chain[], int ndev)
{
	int ret;
	int i;

	if (ndev < 2)
		return 1;

	do {
		print_chain(chain, ndev);
		ret = wait_for_ready_devs(chain, ndev);
		print_chain(chain, ndev);
		if (ret <= 0)
			break;
		for (i = 1; i < ndev; ++i) {
			ret = process_pair(chain[i - 1], chain[i]);
			if (ret != 0) {
				dbg("pair %d:%d ret=%d", i-1, i, ret);
				return -1;
			}
		}
	} while (1);

	return 0;
}
