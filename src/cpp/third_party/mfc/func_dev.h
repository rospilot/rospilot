/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * func I/O device header.
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

#ifndef FUNC_DEV_H
#define FUNC_DEV_H

#include "io_dev.h"

int func_req_bufs(struct io_dev *dev, enum io_dir dir, int nelem);
int func_deq_buf(struct io_dev *dev, enum io_dir dir);
int func_enq_buf(struct io_dev *dev, enum io_dir dir, int idx);
int func_destroy(struct io_dev *dev);

#endif
