/*
 * {nfc-plugin-nxp}
 *
 * Copyright (c) 2012, 2013 Samsung Electronics Co., Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

/*
 * Name			: pn544.h
 * Part of		: Near Field Communication Driver
 * Description	: Definitions of PN544
 * Version		: 0
 */

#ifndef	_PN544_H_
#define _PN544_H_

#include <sys/ioctl.h>

/* ioctl */
#define PN544_CHAR_BASE			0xE9
#define PN544_IOW(num, dtype)	_IOW(PN544_CHAR_BASE, num, dtype)
#define PN544_SET_PWR			PN544_IOW(1, unsigned int)

#endif // _PN544_H_
