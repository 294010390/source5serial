/********************************************************************
	created:	2012/02/07
	filename: 	myfb.c
	author:		
	
	purpose:	
*********************************************************************/
#ifndef WIN32
//-------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/fb.h>
#include <linux/kd.h>

struct FB {
    unsigned short *bits;
    unsigned size;
    int fd;
    struct fb_fix_screeninfo fi;
    struct fb_var_screeninfo vi;
};

int fb_bpp(struct FB *fb)
{
	if (fb)
	{
		return fb->vi.bits_per_pixel;
	}
	return 0;
}

int fb_width(struct FB *fb)
{
	if (fb)
	{
		return fb->vi.xres;
	}
	return 0;
}

int fb_height(struct FB *fb)
{
	if (fb)
	{
		return fb->vi.yres;
	}
	return 0;
}

int fb_size(struct FB *fb)
{
	if (fb)
	{
		unsigned bytespp = fb->vi.bits_per_pixel / 8;
		return (fb->vi.xres * fb->vi.yres * bytespp);
	}
	return 0;
}

int fb_virtual_size(struct FB *fb)
{
	if (fb)
	{
		unsigned bytespp = fb->vi.bits_per_pixel / 8;
		return (fb->vi.xres_virtual * fb->vi.yres_virtual * bytespp);
	}
	return 0;
}

void *fb_bits(struct FB *fb)
{
	unsigned short *bits = NULL;
	if (fb)
	{
		int offset, bytespp;
		bytespp = fb->vi.bits_per_pixel / 8;

		/* HACK: for several of our 3d cores a specific alignment
		* is required so the start of the fb may not be an integer number of lines
		* from the base.  As a result we are storing the additional offset in
		* xoffset. This is not the correct usage for xoffset, it should be added
		* to each line, not just once at the beginning */
		offset = fb->vi.xoffset * bytespp;
		offset += fb->vi.xres * fb->vi.yoffset * bytespp;
		bits = fb->bits + offset / sizeof(*fb->bits);
	}
	return bits;
}

void fb_update(struct FB *fb, int n)
{
	if (fb)
	{
		if ( n > 1 ) return;
		fb->vi.yres_virtual = fb->vi.yres * 2;
		fb->vi.yoffset = n * fb->vi.yres;
		if ( ioctl(fb->fd, FBIOPUT_VSCREENINFO, &fb->vi) < 0 ) 
		{
			fprintf(stderr, "active fb swap failed\n");
		}
	}
}

static void fb_close(struct FB *fb)
{
	if (fb)
	{
		if ( fb->bits != MAP_FAILED )
		{
			munmap(fb->bits, fb_virtual_size(fb));
			fb->bits = MAP_FAILED;
		}
		if ( fb->fd >= 0 )
		{
			close(fb->fd);
			fb->fd = -1;
		}
	}
}

static int fb_open(struct FB *fb, int readonly)
{
	if ( NULL == fb )
	{
		return -1;
	}

	fb->bits = MAP_FAILED;
    	fb->fd = open("/dev/fb0", readonly ? O_RDONLY : O_RDWR);
    	if ( fb->fd < 0 )
	{
		printf("open(\"/dev/fb0\") failed\n");
        	return -1;
	}

    	if ( ioctl(fb->fd, FBIOGET_FSCREENINFO, &fb->fi) < 0 )
	{
		printf("FBIOGET_FSCREENINFO failed\n");
        	goto fail;
	}
    	if ( ioctl(fb->fd, FBIOGET_VSCREENINFO, &fb->vi) < 0 )
	{
		printf("FBIOGET_VSCREENINFO failed\n");
        	goto fail;
	}
	// fb_update(fb, buffer_no);
    	fb->bits = mmap(0, fb_virtual_size(fb), PROT_READ | (readonly ? 0 : PROT_WRITE), 
			MAP_SHARED, fb->fd, 0);
	if ( fb->bits == MAP_FAILED )
	{
		printf("mmap() failed\n");
        	goto fail;
	}

    	return 0;

fail:
    	fb_close(fb);
    	return -1;
}

static struct FB g_fb;
struct FB *fb_create(int readonly)
{
	memset(&g_fb, 0, sizeof(struct FB));
	if ( fb_open(&g_fb, readonly) < 0 )
	{
		return NULL;
	}
	return &g_fb;
}

void fb_destory(struct FB *fb)
{
	fb_close(fb);
}

//-------------------------------------------------------------------
#endif//#ifndef WIN32
