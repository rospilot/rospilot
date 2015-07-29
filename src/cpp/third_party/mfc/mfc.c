/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * MFC device.
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

#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>

#include<third_party/mfc/common.h>
#include<third_party/mfc/mfc.h>
#include<third_party/mfc/v4l_dev.h>

#define MAX_STREAM_SIZE (2*1024*1024)

int align(int x, int a)
{
	return ((x + a - 1) / a) * a;
}

/* runs v4l_deq_buf and correct ending conditions
   according to MFC driver */
int mfc_deq_buf(struct io_dev *dev, enum io_dir dir)
{
	int idx;
    
    struct mfc_priv *priv = dev->priv;

	idx = v4l_deq_buf_ext(dev, dir, &priv->last_frame_flags);

	if (dir == DIR_IN && dev->io[DIR_IN].state == FS_END
						&& dev->io[DIR_OUT].limit)
		dev->io[DIR_OUT].limit = 0;

	return idx;
}

struct io_dev_ops mfc_dev_ops = { .req_bufs = v4l_req_bufs,
				  .enq_buf = v4l_enq_buf,
				  .deq_buf = mfc_deq_buf,
				  .deq_event = v4l_deq_event,
				  .destroy = v4l_destroy
				};

struct io_dev *mfc_create(char const *name)
{
	struct io_dev *dev;
	int ret;

	dev = malloc(sizeof(*dev));
	memzero(*dev);

    dev->priv = malloc(sizeof(struct mfc_priv));

	dev->ops = &mfc_dev_ops;

	dev->io[DIR_IN].type = IO_MMAP;
	dev->io[DIR_OUT].type = IO_MMAP;

	dev->fd = open(name, O_RDWR, 0);
	if (dev->fd < 0) {
		err("Cannot open MFC device %s", name);
		free(dev);
		return NULL;
	}

	struct v4l2_event_subscription ev_sub;
	memzero(ev_sub);
	ev_sub.type = V4L2_EVENT_EOS;
	ret = ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &ev_sub);
	if (ret != 0)
		err("Cannot subscribe EOS event for MFC");

	dbg("MFC device %s opened with fd=%d", name, dev->fd);

	return dev;
}

int mfc_set_codec(struct io_dev *dev, enum io_dir dir, int codec)
{
	struct v4l2_format fmt;
	int ret;

	memzero(fmt);
	fmt.type = io_dir_to_type(dir);
	fmt.fmt.pix_mp.pixelformat = codec;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = MAX_STREAM_SIZE;

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);

	return ret;
}

/* set format with proper alignement */
int mfc_set_fmt(struct io_dev *dev, enum io_dir dir, int width, int height)
{
	struct v4l2_format fmt;
	int ret;

	memzero(fmt);
	fmt.type = io_dir_to_type(dir);
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
	fmt.fmt.pix_mp.width = width;
	fmt.fmt.pix_mp.height = height;

	fmt.fmt.pix_mp.num_planes = 2;
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = align(width, 128);
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = align(width * height, 2048);
	fmt.fmt.pix_mp.plane_fmt[1].bytesperline = align(width, 128);
	fmt.fmt.pix_mp.plane_fmt[1].sizeimage = align(width * (height / 2),
									2048);

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);
	if (ret != 0)
		err("Cannot set format on %d:%d", dev->fd, dir);

	return ret;
}

int mfc_set_rate(struct io_dev *dev, int rate)
{
	struct v4l2_streamparm fps;
	int ret;

	fps.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fps.parm.output.timeperframe.numerator = 1000;
	fps.parm.output.timeperframe.denominator = rate;

	ret = ioctl(dev->fd, VIDIOC_S_PARM, &fps);
	if (ret != 0)
		err("Cannot set rate on %d", dev->fd);

	return ret;
}

int mfc_set_mpeg_control(struct io_dev *dev, int id, int value)
{
	struct v4l2_ext_control ctrl;
	struct v4l2_ext_controls ctrls;
	int ret;

	ctrl.id = id;
    // These aren't used for integer controls
    ctrl.size = 0;
    memzero(ctrl.reserved2);

	ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
	ctrls.count = 1;
	ctrls.controls = &ctrl;
	
    ret = ioctl(dev->fd, VIDIOC_G_EXT_CTRLS, &ctrls);
	if (ret < 0)
		err("Cannot get control %d on %d", id, dev->fd);

    dbg("Changing control %d from %d to %d", id, ctrl.value, value);

	ctrl.value = value;
	ret = ioctl(dev->fd, VIDIOC_S_EXT_CTRLS, &ctrls);
	if (ret < 0)
		err("Cannot set control %d to %d on %d", id, value, dev->fd);

	return ret;
}

int mfc_set_bitrate(struct io_dev *dev, int bitrate)
{
	struct v4l2_ext_control ctrl;
	struct v4l2_ext_controls ctrls;
	int ret;

	ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
	ctrl.value = bitrate;

	ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
	ctrls.count = 1;
	ctrls.controls = &ctrl;

	ret = ioctl(dev->fd, VIDIOC_S_EXT_CTRLS, &ctrls);
	if (ret < 0)
		err("Cannot set bitrate on %d", dev->fd);

	return ret;
}
