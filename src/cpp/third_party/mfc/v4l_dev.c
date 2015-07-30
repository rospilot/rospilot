/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * V4L I/O device implementation.
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

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>

#include<third_party/mfc/io_dev.h>
#include<third_party/mfc/v4l_dev.h>
#include<third_party/mfc/mfc.h>

enum v4l2_memory io_type_to_memory(enum io_type type)
{
	switch (type) {
	case IO_USERPTR: return V4L2_MEMORY_USERPTR;
	case IO_MMAP: return V4L2_MEMORY_MMAP;
	default: return 0;
	}
}

int is_buf_type(enum io_type type)
{
	return (type == IO_USERPTR) || (type == IO_MMAP);
}

enum v4l2_buf_type io_dir_to_type(enum io_dir dir)
{
	switch (dir) {
	case DIR_IN: return V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	case DIR_OUT: return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	default: return 0;
	}
}

/* switch on/off streaming on v4l device */
int v4l_stream_set(struct io_dev *dev, int op)
{
	int ret;
	int buf_type;

	if (dev->io[DIR_IN].type != IO_NONE) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		ret = ioctl(dev->fd, op ? VIDIOC_STREAMON : VIDIOC_STREAMOFF,
								&buf_type);
		if (ret != 0) {
			err("Cannot %s stream on fd=%d:0",
					op ? "start" : "stop", dev->fd);
			return -1;
		}

		dev->io[DIR_IN].state = op ? FS_BUSY : FS_END;

		dbg("Stream %s on fd=%d:0", op ? "started" : "stopped",
								dev->fd);
	}

	if (dev->io[DIR_OUT].type != IO_NONE) {
		buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		ret = ioctl(dev->fd, op ? VIDIOC_STREAMON : VIDIOC_STREAMOFF,
								&buf_type);
		if (ret != 0) {
			err("Cannot %s stream on fd=%d:1",
					op ? "start" : "stop", dev->fd);
			return -1;
		}

		dev->io[DIR_OUT].state = op ? FS_BUSY : FS_END;

		dbg("Stream %s on fd=%d:1", op ? "started" : "stopped",
								dev->fd);
	}

	return 0;
}

int v4l_req_bufs(struct io_dev *dev, enum io_dir dir, int nelem)
{
	int ret;
	struct v4l2_requestbuffers reqbuf;

	memzero(reqbuf);
	reqbuf.count = nelem;
	reqbuf.type = io_dir_to_type(dir);
	reqbuf.memory = io_type_to_memory(dev->io[dir].type);

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &reqbuf);
	if (ret != 0) {
		err("Failed to request %d buffers for device %d:%d)", nelem,
							dev->fd, dir);
		return -1;
	}

	dbg("Succesfully requested %d buffers for device %d:%d", nelem,
								dev->fd, dir);

	return reqbuf.count;
}

/* dequeue v4l buffer, adjust io_port fields, stop stream when needed,
   set end state when counter reaches limit,
   set limit on out port if in port is stopped */
int v4l_deq_buf(struct io_dev *dev, enum io_dir dir) {
    int flags;
    return v4l_deq_buf_ext(dev, dir, &flags);
}

int v4l_deq_buf_ext(struct io_dev *dev, enum io_dir dir, int *flags)
{
	struct v4l2_plane planes[MFC_MAX_PLANES];
	struct v4l2_buffer buf;
	int ret;
	int idx;
	struct dev_buffers *bufs;
	int i;

	bufs = dev->io[dir].bufs;

	memzero(buf);

	buf.type = io_dir_to_type(dir);
	buf.memory = io_type_to_memory(dev->io[dir].type);
	buf.m.planes = planes;
	buf.length = MFC_MAX_PLANES;

	ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf);
	if (ret != 0) {
		dbg("Dequeue buffer error for %d:%d", dev->fd, dir);
		return -1;
	} else {
		dbg("Dequeued %d/%d buffer(flags=%02X,s=%d) from %d:%d",
					buf.index, bufs->count, buf.flags,
					buf.m.planes[0].bytesused, dev->fd,
					dir);
        *flags = buf.flags;
    }

	idx = buf.index;

	for (i = 0; i < bufs->nplanes; ++i)
		bufs->bytesused[idx * bufs->nplanes + i] =
						buf.m.planes[i].bytesused;

	--dev->io[dir].nbufs;

	++dev->io[dir].counter;

	if (dev->io[dir].limit && dev->io[dir].limit <= dev->io[dir].counter) {
		dev->io[dir].state = FS_END;
		if (dev->io[DIR_OUT - dir].type == IO_NONE ||
					dev->io[DIR_OUT - dir].state == FS_END)
			v4l_stream_set(dev, 0);
		dbg("End on %d:%d", dev->fd, dir);
	} else {
		dev->io[dir].state = FS_BUSY;
	}

	if (dir == DIR_IN && dev->io[DIR_IN].state == FS_END
						&& !dev->io[DIR_OUT].limit)
		dev->io[DIR_OUT].limit = dev->io[DIR_IN].counter;

	return idx;
}

/* enqueue buffer, start stream when needed */
int v4l_enq_buf(struct io_dev *dev, enum io_dir dir, int idx)
{
	struct v4l2_plane planes[MFC_MAX_PLANES];
	struct v4l2_buffer buf;
	int i;
	int ret;
	struct dev_buffers *bufs;

	bufs = dev->io[dir].bufs;

	memzero(buf);

	buf.type = io_dir_to_type(dir);
	buf.memory = io_type_to_memory(dev->io[dir].type);
	buf.index = idx;
	buf.m.planes = planes;
	buf.length = bufs->nplanes;
	for (i = 0; i < bufs->nplanes; ++i) {
		planes[i].bytesused = bufs->bytesused[idx * bufs->nplanes + i];
		planes[i].length = bufs->lengths[i];
		planes[i].m.userptr = (unsigned long)bufs->addr[
						idx * bufs->nplanes + i];
	}

	ret = ioctl(dev->fd, VIDIOC_QBUF, &buf);
	if (ret != 0) {
		err("Error %d enq buffer %d/%d to %d:%d", errno, idx,
						bufs->count, dev->fd, dir);
		return -1;
	} else {
		dbg("Enqueued buffer %d/%d to %d:%d", idx, bufs->count,
								dev->fd, dir);
	}

	++dev->io[dir].nbufs;

	if (dev->io[dir].state == FS_OFF)
		if (dir == DIR_IN || dev->io[DIR_IN].type == IO_NONE)
			v4l_stream_set(dev, 1);

	if (dir == DIR_IN && dev->io[dir].limit && dev->io[dir].limit
				<= dev->io[dir].counter + dev->io[dir].nbufs) {
		struct v4l2_encoder_cmd cmd;
		memset(&cmd, 0, sizeof cmd);
		cmd.cmd = V4L2_ENC_CMD_STOP;
		ret = ioctl(dev->fd, VIDIOC_ENCODER_CMD, &cmd);
		dbg("EOS sent to %d:%d, ret=%d", dev->fd, dir, ret);
	}

	return 0;
}

int v4l_deq_event(struct io_dev *dev)
{
	struct v4l2_event ev;
	int ret;

	memzero(ev);
	ret = ioctl(dev->fd, VIDIOC_DQEVENT, &ev);
	if (ret != 0)
		return ret;

	switch (ev.type) {
	case V4L2_EVENT_EOS:
		dev->io[DIR_OUT].state = FS_END;
		dbg("EOS event received on %d", dev->fd);
	}

	dev->event = 0;

	return 0;
}

/* set format on dst device according to src device */
int v4l_copy_fmt(struct io_dev *src, enum io_dir sdir,
		 struct io_dev *dst, enum io_dir ddir)
{
	struct v4l2_format sfmt;
	struct v4l2_format dfmt;
	int ret;

	memzero(sfmt);
	sfmt.type = io_dir_to_type(sdir);
	ret = ioctl(src->fd, VIDIOC_G_FMT, &sfmt);
	if (ret != 0) {
		err("Failed to get format");
		return -1;
	}

	dfmt = sfmt;
	dfmt.type = io_dir_to_type(ddir);
	ret = ioctl(dst->fd, VIDIOC_S_FMT, &dfmt);
	if (ret != 0) {
		err("Failed to set format");
		return -1;
	}

	err("sfmt(type=%d,size=%dx%d,fmt=%.4s,npl=%d,sizes=%d,%d,bpls=%d,%d)",
	    sfmt.type, sfmt.fmt.pix_mp.width, sfmt.fmt.pix_mp.height,
	    (char *)&sfmt.fmt.pix_mp.pixelformat, sfmt.fmt.pix_mp.num_planes,
	    sfmt.fmt.pix_mp.plane_fmt[0].sizeimage,
	    sfmt.fmt.pix_mp.plane_fmt[1].sizeimage,
	    sfmt.fmt.pix_mp.plane_fmt[0].bytesperline,
	    sfmt.fmt.pix_mp.plane_fmt[1].bytesperline);
	err("dfmt(type=%d,size=%dx%d,fmt=%.4s,npl=%d,sizes=%d,%d,bpls=%d,%d)",
	    dfmt.type, dfmt.fmt.pix_mp.width, dfmt.fmt.pix_mp.height,
	    (char *)&dfmt.fmt.pix_mp.pixelformat, dfmt.fmt.pix_mp.num_planes,
	    dfmt.fmt.pix_mp.plane_fmt[0].sizeimage,
	    dfmt.fmt.pix_mp.plane_fmt[1].sizeimage,
	    dfmt.fmt.pix_mp.plane_fmt[0].bytesperline,
	    dfmt.fmt.pix_mp.plane_fmt[1].bytesperline);

	return 0;
}

int v4l_destroy(struct io_dev *dev)
{
	if (dev->io[DIR_IN].type == IO_MMAP)
		dev_bufs_destroy(dev->io[DIR_IN].bufs);

	if (dev->io[DIR_OUT].type == IO_MMAP)
		dev_bufs_destroy(dev->io[DIR_OUT].bufs);

	if (dev->fd >= 0)
		close(dev->fd);

	free(dev);

	return 0;
}

/* requests mmap buffers from device with proper port type, mmap them
   and initialize struct dev_buffers
*/
int dev_bufs_create(struct io_dev *in, struct io_dev *out, int nelem)
{
	enum io_dir dir;
	struct io_dev *master, *slave;
	struct v4l2_buffer qbuf;
	int ret;
	int n, i;
	struct dev_buffers *bufs;
	struct v4l2_plane planes[MFC_MAX_PLANES];

	if (in->io[DIR_OUT].type == IO_MMAP) {
		master = in;
		slave = out;
		dir = DIR_OUT;
	} else if (out->io[DIR_IN].type == IO_MMAP) {
		master = out;
		slave = in;
		dir = DIR_IN;
	} else {
		err("At least one device must have MMAP buffers.");
		return -1;
	}

	nelem = master->ops->req_bufs(master, dir, nelem);
	if (nelem < 0)
		return -1;

	nelem = slave->ops->req_bufs(slave, DIR_OUT - dir, nelem);
	if (nelem < 0)
		return -1;

	bufs = malloc(sizeof(struct dev_buffers));
	in->io[DIR_OUT].bufs = bufs;
	out->io[DIR_IN].bufs = bufs;

	bufs->count = nelem;

	memzero(qbuf);
	qbuf.type = io_dir_to_type(dir);
	qbuf.memory = V4L2_MEMORY_MMAP;
	qbuf.m.planes = planes;
	qbuf.length = MFC_MAX_PLANES;

	for (n = 0; n < nelem; ++n) {
		qbuf.index = n;
		ret = ioctl(master->fd, VIDIOC_QUERYBUF, &qbuf);
		if (ret != 0) {
			err("QUERYBUF failed");
			return -1;
		}

		if (n == 0) {
			for (i = 0; i < qbuf.length; ++i)
				if (qbuf.m.planes[i].length == 0)
					break;
			bufs->nplanes = i;
			bufs->lengths = malloc(bufs->nplanes
						* sizeof(*bufs->lengths));
			for (i = 0; i < bufs->nplanes; ++i)
				bufs->lengths[i] = qbuf.m.planes[i].length;

			bufs->bytesused = malloc(nelem * bufs->nplanes
						* sizeof(*bufs->bytesused));
			bufs->addr = malloc(nelem * bufs->nplanes
						* sizeof(*bufs->addr));
		}

		for (i = 0; i < bufs->nplanes; ++i) {
			bufs->addr[n * bufs->nplanes + i] = mmap(NULL,
						qbuf.m.planes[i].length,
						PROT_READ | PROT_WRITE,
						MAP_SHARED, master->fd,
						qbuf.m.planes[i].m.mem_offset);
			if (bufs->addr[n * bufs->nplanes + i] == MAP_FAILED) {
				err("Failed mmap buffer %d for %d:%d", n,
							master->fd, dir);
				return -1;
			}
		}
		ret = in->ops->enq_buf(in, DIR_OUT, n);
		if (ret < 0)
			return -1;
	}

	return 0;
}

int dev_bufs_destroy(struct dev_buffers *bufs)
{
	free(bufs->addr);
	free(bufs->bytesused);
	free(bufs->lengths);
	free(bufs);

	return 0;
}

struct io_dev_ops v4l_dev_ops = { .req_bufs = v4l_req_bufs,
				  .enq_buf = v4l_enq_buf,
				  .deq_buf = v4l_deq_buf,
				  .deq_event = v4l_deq_event,
				  .destroy = v4l_destroy
				};
