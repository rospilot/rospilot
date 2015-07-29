/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * MFC device header file.
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

#ifndef MFC_H
#define MFC_H

#include <linux/videodev2.h>

#include "io_dev.h"

#define MFC_ENC_IN_NBUF 16
#define MFC_ENC_OUT_NBUF 4
#define MFC_MAX_PLANES 2

struct mfc_priv {
    // V4L2 flags from last buffer that was dequeued. See videodev2.h
    int last_frame_flags;
};

struct io_dev *mfc_create(char const *name);
int mfc_set_codec(struct io_dev *dev, enum io_dir dir, int codec);
int mfc_set_fmt(struct io_dev *dev, enum io_dir dir, int width, int height);
int mfc_set_rate(struct io_dev *dev, int rate);
int mfc_set_bitrate(struct io_dev *dev, int bitrate);
int mfc_set_mpeg_control(struct io_dev *dev, int id, int value);

#endif
