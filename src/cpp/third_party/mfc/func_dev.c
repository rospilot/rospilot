/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * func I/O device implementation.
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
#include <unistd.h>

#include<third_party/mfc/io_dev.h>
#include<third_party/mfc/func_dev.h>
#include<third_party/mfc/mfc.h>

int func_req_bufs(struct io_dev *dev, enum io_dir dir, int nelem)
{
	struct ring_buffer *rb;

	rb = malloc(sizeof(*rb) + (nelem + 1) * sizeof(rb->data[0]));
	rb->begin = 0;
	rb->end = 0;
	rb->size = nelem + 1;

	dev->io[dir].queue = rb;

	dbg("Succesfully requested %d buffers for device %d:%d", nelem,
								dev->fd, dir);

	return nelem;
}

int func_deq_buf(struct io_dev *dev, enum io_dir dir)
{
	int i;
	struct dev_buffers *bufs;
	int ret;
	int lens[MFC_MAX_PLANES];
	struct ring_buffer *q;
	int idx;
	int (*func)(struct io_dev *dev, int nbufs, char **bufs, int *lens);

	q = dev->io[dir].queue;

	if (q->begin == q->end)
		return -1;

	idx = q->data[q->begin++];
	q->begin %= q->size;

	bufs = dev->io[dir].bufs;

	for (i = 0; i < bufs->nplanes; ++i)
		lens[i] = (dir == DIR_IN) ?
				bufs->bytesused[bufs->nplanes * idx + i] :
				bufs->lengths[i];

	func = (dir == DIR_OUT) ? dev->ops->read : dev->ops->write;

	ret = func(dev, bufs->nplanes, &bufs->addr[bufs->nplanes * idx], lens);

	for (i = 0; i < bufs->nplanes; ++i)
		bufs->bytesused[bufs->nplanes * idx + i] = lens[i];

	dbg("Dequeued buffer %d/%d from %d:%d ret=%d", idx, bufs->count, dev->fd, dir, ret);

	--dev->io[dir].nbufs;

	++dev->io[dir].counter;

	if (ret < 0 || (dev->io[dir].limit &&
				dev->io[dir].limit <= dev->io[dir].counter)) {
		dev->io[dir].state = FS_END;
		dbg("End on %d:%d", dev->fd, dir);
	} else if (q->begin == q->end) {
		dev->io[dir].state = FS_OFF;
	} else if (dev->fd >= 0) {
		dev->io[dir].state = FS_BUSY;
	}

	return idx;
}

int func_enq_buf(struct io_dev *dev, enum io_dir dir, int idx)
{
	struct ring_buffer *q;

	q = dev->io[dir].queue;
	q->data[q->end++] = idx;
	q->end %= q->size;

	++dev->io[dir].nbufs;

	if (dev->io[dir].state == FS_OFF)
		dev->io[dir].state = dev->fd >= 0 ? FS_BUSY : FS_READY;
	else if (dev->io[dir].state == FS_BUSY && dev->fd < 0)
		dev->io[dir].state = FS_READY;

	dbg("Enqueued buffer %d/%d to %d:%d", idx, q->size - 1, dev->fd, dir);

	return 0;
}

int func_destroy(struct io_dev *dev)
{
	if (dev->io[DIR_IN].type == IO_FUNC)
		free(dev->io[DIR_IN].queue);

	if (dev->io[DIR_OUT].type == IO_FUNC)
		free(dev->io[DIR_OUT].queue);

	if (dev->fd >= 0)
		close(dev->fd);

	free(dev);

	return 0;
}
