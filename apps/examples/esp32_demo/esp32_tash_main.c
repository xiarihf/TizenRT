/******************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************/
/****************************************************************************
 * examples/opus_thread/hello_tash_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <apps/shell/tash.h>

#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/kmalloc.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#if defined(CONFIG_ADC)
#include <tinyara/analog/adc.h>
#define ADC_MAX_SAMPLES	4
#define TEST_TIME_SEC   30
#endif

#if defined(CONFIG_SPI)
#include "w25q128.h"
#endif

#if defined(CONFIG_I2S)
#include "i2schar_demo.h"
#endif

#include "esp32_i2c_gpio_test.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define ESP32_TASH_PRI      100
#define ESP32_TASH_STAKSIZE (4096)

/****************************************************************************
 * Private Data & Functions
 ****************************************************************************/
/* example */

/*  Call-back function registered in TASH.
 *   This creates pthread to run an example with ASYNC TASH excution type.
 *   Only three points can be modified
 *   1. priority
 *   2. stacksize
 *   3. register entry function of pthread (example)
 */

extern pthread_addr_t esp32_demo_entry(pthread_addr_t arg);

static int esp32_wifi_os_cb(int argc, char **args)
{
	pthread_t opus_thread;

	pthread_attr_t attr;
	struct sched_param sparam;
	int status;

	/* Initialize the attribute variable */
	status = pthread_attr_init(&attr);
	if (status != 0) {
		printf("opus_thread : pthread_attr_init failed, status=%d\n", status);
	}

	/* 1. set a priority */
	sparam.sched_priority = ESP32_TASH_PRI;
	status = pthread_attr_setschedparam(&attr, &sparam);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setschedparam failed, status=%d\n", status);
	}

	/* 2. set a stacksize */
	status = pthread_attr_setstacksize(&attr, ESP32_TASH_STAKSIZE);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setstacksize failed, status=%d\n", status);
	}
	//pthread_attr_set
	//schedpolicy(&attr, SCHED_RR);
	/* 3. create pthread with entry function */

	status = pthread_create(&opus_thread, &attr, esp32_demo_entry, NULL);
	if (status != 0) {
		printf("opus_thread: pthread_create failed, status=%d\n", status);
	}

	/* Wait for the threads to stop */
	pthread_join(opus_thread, NULL);
	printf("esp32_demo_thread is finished\n");

	return 0;

}




long loop_count,l_c, th_count;
struct timeval t;
double f_avg, i_avg;

// Thread Structure for FLOPS
struct fth
{
	int lc, th_counter;
	float fa, fb, fc, fd;
	pthread_t threads;
};

// FlOPs Benchmark
void *FAdd(void *data)
{
	struct fth *th_d;
	th_d = (struct fth *) data;

	for(th_d->lc = 1; th_d->lc <= l_c; th_d->lc++)
	{
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
		th_d->fa + th_d->fd;
		th_d->fa + th_d->fb;
		th_d->fb + th_d->fc;
		th_d->fa - th_d->fb;
	}
}

void FLOPSBenchmark()
{
	long l, c, m, id;
	double fs_t, fe_t, ft_t;
	struct fth ft[th_count];

	for(l =0; l < th_count; l++)
	{
		ft[l].lc = 1;
		ft[l].th_counter = 1;
		ft[l].fa = 0.02;
		ft[l].fb = 0.2;
		ft[l].fc = 0;
		ft[l].fd = 0;
	}

	gettimeofday(&t, NULL);
	fs_t = t.tv_sec+(t.tv_usec/1000000.0);
	//ets_printf("F0 t.tv_sec is %d, t.tv_usec is %d\n", t.tv_sec, t.tv_usec);

	for(c = 0; c < th_count; c++)
	{
		pthread_create(&ft[c].threads, NULL, &FAdd, (void *)&ft[c]);
	}
	for(m =0; m < th_count; m++)
	{
		pthread_join(ft[m].threads, NULL);
	}

	gettimeofday(&t, NULL);
	//ets_printf("F1 t.tv_sec is %d, t.tv_usec is %d\n", t.tv_sec, t.tv_usec);
	fe_t = t.tv_sec+(t.tv_usec/1000000.0);
	ft_t = fe_t - fs_t;
	//printf("F3 fe_t is %f, fs_t is %f, ft_t is %f , loop_count is %d\n", fe_t, fs_t, ft_t, loop_count);

	
	f_avg += (loop_count * 60) / (ft_t * 100000000);

	//printf("f_avg is %f\n", f_avg);

}

// Thread Structure for IOPs
struct ith
{
	int lc, th_counter;
	int ia, ib, ic, id;
	pthread_t threads;
};

// IOPs Benchmark
void *IAdd(void *data)
{
	struct ith *th_d;
	th_d = (struct ith *) data;

	for(th_d->lc = 1; th_d->lc <= l_c; th_d->lc++)
	{
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;
		th_d->ia + th_d->id;
		th_d->ia + th_d->ib;
		th_d->ib + th_d->ic;
		th_d->ia - th_d->ib;	
	}
}

void IOPSBenchmark()
{
	int k, v, n;
	double is_t, ie_t, it_t;
	struct ith it[th_count];

	for(k =0; k < th_count; k++)
	{
		it[k].lc = 1;
		it[k].th_counter = 1;
		it[k].ia = 1;
		it[k].ib = 2;
		it[k].ic = 0;
		it[k].id = 0;
	}

	gettimeofday(&t, NULL);
	is_t = t.tv_sec+(t.tv_usec/1000000.0);

	for(v = 0; v < th_count; v++)
	{
		pthread_create(&it[v].threads,NULL, &IAdd, (void *)&it[v]);
	}
	for(n =0; n < th_count; n++)
	{
		pthread_join(it[n].threads, NULL);
	}
	
	gettimeofday(&t, NULL);
	ie_t = t.tv_sec+(t.tv_usec/1000000.0);
	it_t = ie_t - is_t;
	i_avg += (loop_count * 60) / (it_t * 100000000);

}

int cpu_test()
{
	loop_count = 10000000; // loop count
	th_count = 1; // thread count
	l_c = loop_count/th_count; // Sharing the operations across the threads
	f_avg = 0;
	i_avg = 0;
	int h;

	printf("\nStarting CPU Benchmark...\n");
	printf("\nThreads Implemented: %ld \n", th_count);
	
	// FLOPs Benchmark
	printf("\nComputing CPU-FLOPs Performance...\n");
	for(h = 1; h <= 5; h++)
	{
		FLOPSBenchmark();
	}
	
	printf("Number of FLOPs: %lf G-FLOPs\n", f_avg/5);
	sleep(2);

	// IOPs Benchmark
	printf("\nComputing CPU-IOPs Performance...\n");
	for(h = 1; h <= 5; h++)
	{
		IOPSBenchmark();
	}
	printf("Number of IOPs: %lf G-FLOPs\n", i_avg/5);
	printf("\nEnding...\n");

	return 0;
}



enum xtal_freq_e {
	CPU_80M = 1,
	CPU_160M = 2,
	CPU_240M = 3,
};

typedef enum {
    RTC_CPU_FREQ_XTAL = 0,      //!< Main XTAL frequency
    RTC_CPU_FREQ_80M = 1,       //!< 80 MHz
    RTC_CPU_FREQ_160M = 2,      //!< 160 MHz
    RTC_CPU_FREQ_240M = 3,      //!< 240 MHz
    RTC_CPU_FREQ_2M = 4,        //!< 2 MHz
} rtc_cpu_freq_t;

extern rtc_cpu_freq_t rtc_clk_cpu_freq_get();
extern void rtc_clk_cpu_freq_set(rtc_cpu_freq_t cpu_freq);

void get_cpu_fre()
{
	rtc_cpu_freq_t currunt_cpu_fre = rtc_clk_cpu_freq_get();
	switch (currunt_cpu_fre)
	{
		case 1:
		{
			ets_printf("Current CPU fre is 80MHz \n");
			break;
		}
		case 2:
		{
			ets_printf("Current CPU fre is 160MHz \n");
			break;
		}
		case 3:
		{
			ets_printf("Current CPU fre is 240MHz \n");
			break;
		}
	}

}

#define MHZ (1000000)
#define XTHAL_GET_CCOUNT()	({ int __ccount; \
				__asm__ __volatile__("rsr.ccount %0" : "=a"(__ccount)); \
				__ccount; })
#define XTHAL_SET_CCOUNT(v)	do { int __ccount = (int)(v); \
			__asm__ __volatile__("wsr.ccount %0" :: "a"(__ccount):"memory"); \
				} while(0)

pthread_addr_t esp32_cpu_demo_entry(pthread_addr_t arg)
{

	enum xtal_freq_e freq;
	uint32_t freq_mhz;
	uint32_t freq_before, freq_after;

	uint32_t fequ_buff[3][2] = {{CPU_80M, 80},{CPU_160M, 160},{CPU_240M, 240}};

	get_cpu_fre();
	int i = *(int *)arg;

	//for (int i = 0; i<3; i++) {
	if (i >= 0 && i<3) {
		ets_printf("Set CPU fre to %dMHz \n", fequ_buff[i][1]);
		freq = fequ_buff[i][0];
		freq_after = fequ_buff[i][1];
		sleep(2);
		freq_before = rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get()) / MHZ;
		rtc_clk_cpu_freq_set((rtc_cpu_freq_t)freq);
		// Re calculate the ccount to make time calculation correct.
		XTHAL_SET_CCOUNT(XTHAL_GET_CCOUNT() * freq_after / freq_before);
		ets_update_cpu_frequency(freq_after);
		xtensa_chang_board_clock(fequ_buff[i][1] * 1000000);
		xtensa_timer_uninitialize();
		xtensa_timer_initialize();
		sleep(2);
		get_cpu_fre();
		//cpu_test();
	}
}


static void print_help()
{
	printf("Please input: \n");
	printf("   esp32_cpu_fre [0,1,2] \n");
	printf("   0: 80Mhz, 1: 160MHz, 2:240MHz\n");
}

static int esp32_cpu_tune_cb(int argc, char **args)
{
	pthread_t opus_thread;

	pthread_attr_t attr;
	struct sched_param sparam;
	int status;
	int cpu_fre = 0;

	if (argc > 2 || argc <= 1) {
		print_help();
		return 0;
	}

	cpu_fre = atoi(args[1]);
	printf(" Set CPU fre to %d\n", cpu_fre);

	/* Initialize the attribute variable */
	status = pthread_attr_init(&attr);
	if (status != 0) {
		printf("opus_thread : pthread_attr_init failed, status=%d\n", status);
	}

	/* 1. set a priority */
	sparam.sched_priority = ESP32_TASH_PRI;
	status = pthread_attr_setschedparam(&attr, &sparam);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setschedparam failed, status=%d\n", status);
	}

	/* 2. set a stacksize */
	status = pthread_attr_setstacksize(&attr, ESP32_TASH_STAKSIZE);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setstacksize failed, status=%d\n", status);
	}
	//pthread_attr_set
	//schedpolicy(&attr, SCHED_RR);
	/* 3. create pthread with entry function */

	status = pthread_create(&opus_thread, &attr, esp32_cpu_demo_entry, &cpu_fre);
	if (status != 0) {
		printf("opus_thread: pthread_create failed, status=%d\n", status);
	}

	/* Wait for the threads to stop */
	pthread_join(opus_thread, NULL);
	printf("esp32_demo_thread is finished\n");

	return 0;

}




#if defined(CONFIG_ADC)
#define CONVERT_PERIODICALLY		1
static int esp32_adc_os_cb(int argc, char **args)
{
	int ret;
	struct adc_msg_s samples[ADC_MAX_SAMPLES];
	ssize_t nbytes;

	int fd_adc = open("/dev/adc0", O_RDONLY);
	if (fd_adc < 0) {
		printf("%s: open failed: %d\n", __func__, errno);
		return -errno;
	}

	int i = 0;
#if defined(CONVERT_PERIODICALLY) && (0 < CONVERT_PERIODICALLY)
	ret = ioctl(fd_adc, ANIOC_TRIGGER, 0);
	if (ret < 0) {
		printf("%s: ioctl failed: %d\n", __func__, errno);
		close(fd_adc);
		return ret;
	}
	while (i++ < TEST_TIME_SEC) {
		nbytes = read(fd_adc, samples, sizeof(samples));
		if (nbytes < 0) {
			if (errno != EINTR) {
				printf("%s: read failed: %d\n", __func__, errno);
				close(fd_adc);
				return -errno;
			}
		} else if (nbytes == 0) {
			printf("%s: No data read, Ignoring\n", __func__);
		} else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);
			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				printf("%s: read size=%ld is not a multiple of sample size=%d, Ignoring\n", __func__, (long)nbytes, sizeof(struct adc_msg_s));
			} else {
				printf("Sample:\n");
				int i;
				for (i = 0; i < nsamples; i++) {
					printf("%d: channel: %d, value: %d\n", i + 1, samples[i].am_channel, samples[i].am_data);
				}
				printf("\n\n");
			}
		}

		sleep(1);
	}
#else
	while (i++ < TEST_TIME_SEC) {
		ret = ioctl(fd_adc, ANIOC_TRIGGER, 0);
		if (ret < 0) {
			printf("%s: ioctl failed: %d\n", __func__, errno);
			close(fd_adc);
			return ret;
		}

		nbytes = read(fd_adc, samples, sizeof(struct adc_msg_s));	//Read one
		if (nbytes < 0) {
			if (errno != EINTR) {
				printf("%s: read failed: %d\n", __func__, errno);
				close(fd_adc);
				return -errno;
			}
		} else if (nbytes == 0) {
			printf("%s: No data read, Ignoring\n", __func__);
		} else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);
			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				printf("%s: read size=%ld is not a multiple of sample size=%d, Ignoring\n", __func__, (long)nbytes, sizeof(struct adc_msg_s));
			} else {
				printf("Sample:\n");
				int i;
				for (i = 0; i < nsamples; i++) {
					printf("%d: channel: %d, value: %d\n", i + 1, samples[i].am_channel, samples[i].am_data);
				}
				printf("\n\n");
			}
		}

		//usleep(3000000);
		sleep(1);
	}
#endif
	close(fd_adc);
	printf("ADC test ends!\n\n\n");
	return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * opus_thread_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int esp32_tash_main(int argc, char **args)
{
	tash_cmd_install("esp32_demo", esp32_wifi_os_cb, TASH_EXECMD_ASYNC);
	
	tash_cmd_install("esp32_cpu_fre", esp32_cpu_tune_cb, TASH_EXECMD_ASYNC);

	
#if defined(CONFIG_ADC)
	tash_cmd_install("esp32_adc_demo", esp32_adc_os_cb, TASH_EXECMD_ASYNC);
#endif
#if defined(CONFIG_SPI)
	ESP32_SPI_command_install();
#endif
#if defined(CONFIG_I2S)
	esp32_i2schar_install();
#endif
	esp32_i2c_gpio_cmd_install();
	return 0;
}
#endif
