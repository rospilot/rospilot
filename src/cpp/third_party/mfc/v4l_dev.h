/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * V4L I/O device header.
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

#ifndef V4L_DEV_H
#define V4L_DEV_H

#include "io_dev.h"

int v4l_stream_set(struct io_dev *dev, int op);
int v4l_req_bufs(struct io_dev *dev, enum io_dir dir, int nelem);
int v4l_deq_buf(struct io_dev *dev, enum io_dir dir);
int v4l_deq_buf_ext(struct io_dev *dev, enum io_dir dir, int *flags);
int v4l_enq_buf(struct io_dev *dev, enum io_dir dir, int idx);
int v4l_deq_event(struct io_dev *dev);
int v4l_copy_fmt(struct io_dev *src, enum io_dir sdir,
		 struct io_dev *dst, enum io_dir ddir);
int v4l_destroy(struct io_dev *dev);

/* create common struct dev_buffers for two joined devices */
int dev_bufs_create(struct io_dev *in, struct io_dev *out, int nelem);
int dev_bufs_destroy(struct dev_buffers *bufs);

extern struct io_dev_ops v4l_dev_ops;

#endif
