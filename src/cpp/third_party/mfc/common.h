/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Common structures.
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

#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <string.h>

/* When ADD_DETAILS is defined every debug and error message contains
 * information about the file, function and line of code where it has
 * been called */
#define ADD_DETAILS
/* When DEBUG_MFC is defined debug messages are printed on the screen.
 * Otherwise only error messages are displayed. */
//#define DEBUG_MFC

#ifdef ADD_DETAILS
#define err(msg, ...) \
	fprintf(stderr, "%s:%s:%d: error: " msg "\n", __FILE__, \
		__func__, __LINE__, ##__VA_ARGS__)
#else
#define err(msg, ...) \
	fprintf(stderr, "Error: " msg "\n", __FILE__, ##__VA_ARGS__)
#endif /* ADD_DETAILS */

#ifdef DEBUG_MFC
#ifdef ADD_DETAILS
#include <time.h>
#define dbg(msg, ...) { \
	struct timespec t; \
	clock_gettime(CLOCK_MONOTONIC, &t); \
	fprintf(stderr, "%ld.%ld:%s:%s:%d: " msg "\n", t.tv_sec, t.tv_nsec, __FILE__, \
		__func__, __LINE__, ##__VA_ARGS__); \
}
#else
#define dbg(msg, ...) \
	fprintf(stderr, msg "\n", ##__VA_ARGS__)
#endif /* ADD_DETAILS */
#else /* DEBUG_MFC */
#define dbg(...) {}
#endif /* DEBUG_MFC */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))
#define memzero(d) memset(&(d), 0, sizeof(d))

#endif
