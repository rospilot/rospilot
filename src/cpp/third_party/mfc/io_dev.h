/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * I/O device header file.
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

#ifndef IO_DEV_H
#define IO_DEV_H

#include <linux/videodev2.h>

#include "common.h"

struct dev_buffers {
	int count; /* number of buffers */
	int nplanes; /* number of planes per buffer */
	int *lengths; /* array of plane lengths */
	 /* array of plane addresses, address of plane p in buffer b
	    is at addr[nplanes * b + p] */
	char **addr;
	 /* array of bytes used by plane, bytesused of plane p in buffer b
	    is at bytesused[nplanes * b + p] */
	int *bytesused;
};

struct ring_buffer {
	int begin;
	int end;
	int size;
	int data[0];
};

enum io_type { IO_NONE, IO_FUNC, IO_MMAP, IO_USERPTR };
enum io_dir { DIR_IN = 0, DIR_OUT = 1};
enum func_state { FS_OFF, FS_BUSY, FS_READY, FS_EVENT, FS_END };

struct io_dev_ops;
struct io_dev;

struct io_port {
	enum io_type type;
	enum func_state state;
	int counter; /* total number of dequeued buffers */
	int nbufs; /* number of buffers in queue */
	int limit; /* after dequeuing limit buffers state is changed to END */
	struct dev_buffers *bufs;
	struct ring_buffer *queue; /* used by non V4L devices */
};

struct io_dev {
	int fd;
	int event;
	/* in and out parts of device */
	struct io_port io[2];
	struct io_dev_ops *ops;
	void *priv;
};

struct io_dev_ops {
	int (*read)(struct io_dev *dev, int nbufs, char **bufs, int *lens);
	int (*write)(struct io_dev *dev, int nbufs, char **bufs, int *lens);
	int (*req_bufs)(struct io_dev *dev, enum io_dir dir, int nelem);
	int (*deq_buf)(struct io_dev *dev, enum io_dir dir);
	int (*enq_buf)(struct io_dev *dev, enum io_dir dir, int idx);
	int (*deq_event)(struct io_dev *dev);
	int (*destroy)(struct io_dev *dev);
};

enum v4l2_buf_type io_dir_to_type(enum io_dir dir);
int dev_copy_fmt(int src_fd, enum io_dir src_dir, int dst_fd,
							enum io_dir dst_dir);

int process_chain(struct io_dev *chain[], int nelem);

int process_pair(struct io_dev *in, struct io_dev *out);

int wait_for_ready_devs(struct io_dev *chain[], int ndev);

#endif
