/*
 * Copyright 2008 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Dave Airlie
 */

#ifndef ATOM_TYPES_H
#define ATOM_TYPES_H

/* sync atom types to kernel types */

typedef uchar uint8_t;
typedef uint8_t UCHAR;

typedef ushort uint16_t;
typedef uint16_t USHORT;

typedef u32int uint32_t;
typedef uint32_t ULONG;

typedef vlong uint64_t;

/* Define only if big endian. */
//#define ATOM_BIG_ENDIAN
#define ATOM_LITTLE_ENDIAN

/* Linux: Convert little-endian to host. */
#define le32_to_cpu(x) (x)
#define cpu_to_le32(x) (x)

#endif
