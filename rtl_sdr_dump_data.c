/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fftw3.h>
#include <math.h>
#include <pthread.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

#define FFT_LEN 			(4096)
#define SDR_BUF_LEN 		(FFT_LEN*2)
#define SDR_BUF_TOT_NO 	 	(4)


/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)


static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

struct sdr_data_buffer_st
{
	char buf[SDR_BUF_TOT_NO][SDR_BUF_LEN];
    
	char read_pointer;
	char write_pointer;
	char buf_used_cont;
};

struct dongle_cap_data_state
{
	pthread_t thread;
    char c8buff[SDR_BUF_LEN*2];
	char read_pointer;
	char write_pointer;
	char buf_used_cont;
    int  buf_len;
    rtlsdr_dev_t *dev;
    struct iq_fft_state *iq_fft_target;
};

struct iq_fft_state
{
	pthread_t thread;
	float fft_amp_result[FFT_LEN];
    char c8buff[SDR_BUF_LEN*2];
    int data_len;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	
};

//struct iq_fft_output_state
//{
//	pthread_t thread;
//	float fft_amp_result[FFT_LEN];
//    char c8buff[SDR_BUF_LEN*2];
//    int data_len;
//	pthread_rwlock_t rw;
//	pthread_cond_t ready;
//	pthread_mutex_t ready_m;
//	
//}



struct dongle_cap_data_state dongle_cap_data;
struct iq_fft_state iq_fft;




void dongle_cap_data_init(struct dongle_cap_data_state *s,struct iq_fft_state* st)
{
    s->read_pointer = 0;
    s->write_pointer = 0;
    s->buf_used_cont = 0;
    s->iq_fft_target = st;  
    s->buf_len = SDR_BUF_LEN*2;
}

void dongle_iq_fft_init(struct iq_fft_state * s)
{
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL); 
}


static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    int i;
    struct dongle_cap_data_state *s = ctx;
    struct iq_fft_state *f = s->iq_fft_target;
	if (ctx) {
		if (do_exit)
			return;
        for(i = 0;i < (int)len;i++){
            s->c8buff[i] = (char)buf[i] - 127;
        }
        pthread_rwlock_wrlock(&f->rw);
        memcpy(s->c8buff, f->c8buff, len);
        f->data_len = len;
        pthread_rwlock_unlock(&f->rw);
        safe_cond_signal(&f->ready, &f->ready_m);
	}
}

static void *dongle_thread_fn(void *arg)
{
    struct dongle_cap_data_state *s = arg;
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

static void *iq_fft_thread_fn(void *arg)
{
    struct iq_fft_state *f = arg;
    char data_buff[FFT_LEN*2];
    fftw_complex in[FFT_LEN], out[FFT_LEN];
    fftw_plan p;
    int i,j;
    double temp;
    float fft_amp[FFT_LEN];
    double data_max = 0;
    while (!do_exit) {
        safe_cond_wait(&f->ready, &f->ready_m);
		pthread_rwlock_wrlock(&f->rw);
        memcpy(f->c8buff, data_buff, FFT_LEN*2);
		pthread_rwlock_unlock(&f->rw);    
        
        for(i = 0;i < FFT_LEN;i++){
            in[i][0] = data_buff[i*2];
            in[i][1] = data_buff[i*2 + 1];
        }
        p = fftw_plan_dft_1d(FFT_LEN, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        
        fftw_execute(p); /* repeat as needed */

        for(j = 0;j < FFT_LEN;j ++)
        {
            temp = out[j][0]*out[j][0] + out[j][1]*out[j][1];
            temp = sqrt(temp);
            fft_amp[j] = 20*log10(temp);
        }
    }  
}

//static void *output_thread_fn(void *arg)


int main(int argc, char **argv)
{

	struct sigaction sigact;
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int sync_mode = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t frequency = 100000000;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    
    dongle_cap_data_init(&dongle_cap_data,&iq_fft);
    dongle_iq_fft_init(&iq_fft); 
    
    dev_index = verbose_device_search("0");
	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	verbose_ppm_set(dev, ppm_error);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);
    usleep(100000);
    pthread_create(&iq_fft.thread, NULL, iq_fft_thread_fn, (void *)(&iq_fft));
    pthread_create(&dongle_cap_data.thread, NULL, dongle_thread_fn, (void *)(&dongle_cap_data));
    
    
    pthread_join(dongle_cap_data.thread, NULL);
    safe_cond_signal(&iq_fft.ready, &iq_fft.ready_m);
    pthread_join(iq_fft.thread, NULL);
    
	//r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,0, out_block_size);


	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	rtlsdr_close(dev);
out:
	return r >= 0 ? r : -r;
}
