/********************************************************************
	created:	2012/02/07
	filename: 	myfb.h
	author:		
	
	purpose:	
*********************************************************************/

#ifndef _myfb_h__
#define _myfb_h__
//-------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

struct FB;

int fb_bpp(struct FB *fb);
int fb_width(struct FB *fb);
int fb_height(struct FB *fb);
int fb_size(struct FB *fb);
int fb_virtual_size(struct FB *fb);
void *fb_bits(struct FB *fb);
void fb_update(struct FB *fb, int n);

struct FB* fb_create(int readonly);
void fb_destory(struct FB *fb);

#ifdef __cplusplus
};
#endif

//-------------------------------------------------------------------
#endif // #ifndef _myfb_h__
