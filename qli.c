// qli.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define QLI_DEBUG 0
#define QLI_PIXEL_FORMAT QLI_PF_RGB444
#define QLI_ENDIAN QLI_BIG_ENDIAN
#define QLI_POSTFIX _q4
#define QLI_STRIDE 0
#define QLI_IMPLEMENTATION
#include "qli.h"

#define QLI_STRIDE 0
#define QLI_PIXEL_FORMAT QLI_PF_RGB565
#define QLI_ENDIAN QLI_LITTLE_ENDIAN
#define QLI_POSTFIX _q5
#define QLI_IMPLEMENTATION
#include "qli.h"

#define QLI_STRIDE 0
#define QLI_PIXEL_FORMAT QLI_PF_RGB888
#define QLI_ENDIAN QLI_BIG_ENDIAN
#define QLI_POSTFIX _q8
#define QLI_IMPLEMENTATION
#include "qli.h"

#define RPPM_IMPLEMENTATION
#include "rppm.h"

#define BE_GET_RED(rgb565)   (uint8_t)(((rgb565)>>8L)&~7L)
#define BE_GET_GREEN(rgb565) (uint8_t)(((rgb565)>>3L)&~3L)
#define BE_GET_BLUE(rgb565)  (uint8_t)(((rgb565)<<3L)&~7L)
#define LE_GET_RED(rgb565)   (uint8_t)BE_GET_RED(((rgb565>>8L)|(rgb565<<8L)))
#define LE_GET_GREEN(rgb565) (uint8_t)BE_GET_GREEN(((rgb565>>8L)|(rgb565<<8L)))
#define LE_GET_BLUE(rgb565)  (uint8_t)BE_GET_BLUE(((rgb565>>8L)|(rgb565<<8L)))

int BUFSIZE = 255;

int main(int argc, char **argv)
{
	qli_image_t_q4 q4;
	qli_image_t_q5 q5;
	qli_image_t_q8 q8;
	struct rppm img;
	int st;
	int new_chunk = 0;

	if (argc < 5) {
		fprintf(stderr, "Usage: %s [458] [ed] file.ppm out.qli\n",
			argv[0]);
		exit(0);
	}
	int mode = atoi(argv[1]);
	if (mode != 4 && mode != 5 && mode != 8) {
		fprintf(stderr,
			"Invalid mode, supported modes:\n 4\tRGB444\n 5\tRGB565\n 8\tRGB888\n");
		exit(1);
	}
	++argv;
	--argc;
	if (argv[1][0] == 'e') {
		if (0 != rppm_load(&img, argv[2])) {
			fprintf(stderr, "Cannot open '%s'\n", argv[2]);
			exit(1);
		}
		if (mode == 4)
			qli_save_q4(img.pixels, img.width, img.height, argv[3]);
		if (mode == 5)
			qli_save_q5(img.pixels, img.width, img.height, argv[3]);
		if (mode == 8)
			qli_save_q8(img.pixels, img.width, img.height, argv[3]);
		rppm_free(&img);
	} else if (argv[1][0] == 'd') {
		int chunk = 128;
		BUFSIZE = atoi(&argv[1][1]);
		if (BUFSIZE == 0)
			BUFSIZE = 255;
		if (argc > 4) {
			chunk = atoi(argv[2]);
			++argv;
			--argc;
		}
		uint8_t *data, *d;
		FILE *f;
		uint8_t *buf = malloc(BUFSIZE + 10);
		f = fopen(argv[2], "rb");
		if (NULL == f)
			exit(1);
		fread(buf, 1, QLI_HEADER_LEN, f);
		fseek(f, 0, SEEK_END);
		int size = ftell(f) - QLI_HEADER_LEN;
		fseek(f, QLI_HEADER_LEN, SEEK_SET);
		if (mode == 4)
			qli_init_header_q4(&q4, buf, 0);
		if (mode == 5)
			qli_init_header_q5(&q5, buf, 0);
		if (mode == 8)
			qli_init_header_q8(&q8, buf, 0);
#if QLI_DEBUG > 0
		if (mode == 4)
			fprintf(stderr, "DEC %dx%d [%d] bufsize=%d chunk=%d\n",
				q4.width, q4.height, size, BUFSIZE, chunk);
		if (mode == 5)
			fprintf(stderr, "DEC %dx%d [%d] bufsize=%d chunk=%d\n",
				q5.width, q5.height, size, BUFSIZE, chunk);
		if (mode == 8)
			fprintf(stderr, "DEC %dx%d [%d] bufsize=%d chunk=%d\n",
				q8.width, q8.height, size, BUFSIZE, chunk);
#endif
		d = data = malloc(size);
		fread(data, 1, size, f);
		if (mode == 4)
			qli_init_q4(&q4, q4.width, q4.height, data, chunk, 0);
		if (mode == 5)
			qli_init_q5(&q5, q5.width, q5.height, data, chunk, 0);
		if (mode == 8)
			qli_init_q8(&q8, q8.width, q8.height, data, chunk, 0);
		FILE *fo = fopen(argv[3], "wb");
		if (NULL == fo)
			exit(1);
		if (mode == 4)
			fprintf(fo, "P6\n%d %d\n255\n", q4.width, q4.height);
		if (mode == 5)
			fprintf(fo, "P6\n%d %d\n255\n", q5.width, q5.height);
		if (mode == 8)
			fprintf(fo, "P6\n%d %d\n255\n", q8.width, q8.height);
		st = 1;
		int total = 0;
		int bytes_written = 0;
		if (mode == 4) {
			while (1) {
				st = qli_decode_q4(&q4, buf, BUFSIZE,
						   &new_chunk, &bytes_written);
				total += st;
				int bytes = bytes_written;	//(st*3)/2;
				if (bytes >= 2) {
					for (int i = 0; i < bytes; i += 3) {
						uint8_t r, g, b;
						r = (buf[i + 0] & 0xf0);
						g = (buf[i + 0] << 4) & 0xf0;
						b = (buf[i + 1] & 0xf0);
						fputc(r, fo);
						fputc(g, fo);
						fputc(b, fo);
						if ((i + 2) >= bytes)
							break;
						r = (buf[i + 1] << 4) & 0xf0;
						g = (buf[i + 2] & 0xf0);
						b = (buf[i + 2] << 4) & 0xf0;
						fputc(r, fo);
						fputc(g, fo);
						fputc(b, fo);
					}
				}
				if ((new_chunk & QLI_RF_END_OF_STREAM) != 0) {
					break;
				}
				if ((new_chunk & QLI_RF_MORE_DATA) != 0) {
					if (size > 0) {
						d += chunk;
						size -= MIN(chunk, size);
						qli_new_chunk_q4(&q4, d,
								 MIN(chunk,
								     size));
					}
					new_chunk = 0;
				}
			}
		}
		if (mode == 5) {
			while (1) {
				st = qli_decode_q5(&q5, buf, BUFSIZE,
						   &new_chunk, &bytes_written);
				if (st < 0)
					break;
				for (int i = 0; i < bytes_written; i += 2) {
					uint16_t col = buf[i] << 8 | buf[i + 1];
					fputc(LE_GET_RED(col), fo);
					fputc(LE_GET_GREEN(col), fo);
					fputc(LE_GET_BLUE(col), fo);
				}
				if ((new_chunk & QLI_RF_END_OF_STREAM) != 0) {
					break;
				}
				if ((new_chunk & QLI_RF_MORE_DATA) != 0) {
					if (size > 0) {
						int currchunk =
						    MIN(chunk, size);
						d += chunk;
						size -= currchunk;
						qli_new_chunk_q5(&q5, d,
								 currchunk);
					}
					new_chunk = 0;
				}
			}
		}
		if (mode == 8) {
			while (1) {
				st = qli_decode_q8(&q8, buf, BUFSIZE,
						   &new_chunk, &bytes_written);
				if (st < 0)
					break;
				if (bytes_written >= 3) {
					for (int i = 0; i < bytes_written;
					     i += 3) {
						fputc(buf[i + 0], fo);
						fputc(buf[i + 1], fo);
						fputc(buf[i + 2], fo);
					}
				}
				if ((new_chunk & QLI_RF_END_OF_STREAM) != 0) {
					break;
				}
				if ((new_chunk & QLI_RF_MORE_DATA) != 0) {
					if (size > 0) {
						int currchunk =
						    MIN(chunk, size);
						d += chunk;
						size -= currchunk;
						qli_new_chunk_q8(&q8, d,
								 currchunk);
					}
					new_chunk = 0;
				}
			}
		}
		free(data);
		free(buf);
	} else {
		fprintf(stderr, "Unknown command '%c'\n", argv[1][0]);
		exit(1);
	}

	return (0);
}
