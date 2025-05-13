/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "parse_args.h"

#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <string.h>
#include <zephyr/sys/printk.h>

// Parse the command line to find its arguments.
// Original string is modified to terminate the arguments substrings.
// Returns the number of found arguments or zero.
size_t parse_argv(char *str, char *argv[], size_t size)
{
	size_t argc = 0;
	if (!strlen(str)) {
		return 0;
	}
	while (*str && *str == ' ') {
		str++;
	}
	if (!*str) {
		return 0;
	}
	argv[argc++] = str;
	while ((str = strchr(str, ' '))) {
		*str++ = '\0';
		while (*str && *str == ' ')
			str++;
		if (!*str) break;
		argv[argc++] = str;
		if (argc == size) {
			printk("Too many parameters (max %u)\n", size);
			return 0;
		}
	}
	// keep it POSIX style where argv[argc] is required to be NULL
	argv[argc] = NULL;
	return argc;
}

// Similar to implementation of strtoul from picolibc
uint32_t parse_uint(const char *str, uint8_t base)
{
	const char *s = str;
	unsigned long acc;
	int c;
	uint32_t cutoff;
	int neg = 0, any, cutlim;
	do {
		c = *s++;
	} while (isspace((unsigned char)c) != 0);
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+') {
		c = *s++;
	}
	if (((base == 0) || (base == 16)) &&
	    (c == '0') && ((*s == 'x') || (*s == 'X'))) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0) {
		base = (c == '0') ? 8 : 10;
	}
	cutoff = (uint32_t)UINT32_MAX / (uint32_t)base;
	cutlim = (uint32_t)UINT32_MAX % (uint32_t)base;
	for (acc = 0, any = 0;; c = *s++) {
		if (isdigit((unsigned char)c) != 0) {
			c -= '0';
		} else if (isalpha((unsigned char)c) != 0) {
			c -= isupper((unsigned char)c) != 0 ? 'A' - 10 : 'a' - 10;
		} else {
			break;
		}
		if (c >= base) {
			break;
		}
		if ((any < 0) || (acc > cutoff) || ((acc == cutoff) && (c > cutlim))) {
			any = -1;
		} else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}
	if (any < 0) {
		acc = UINT32_MAX;
		errno = ERANGE;
	} else if (neg != 0) {
		acc = -acc;
	}
	return acc;
}