#ifndef QLI_H
#define QLI_H

#include <stdint.h>
#include <string.h>

#define QLI_PF_RGB565 0

// configuration value
#ifndef QLI_DEBUG
#define QLI_DEBUG 0
#endif

// configuration value
#ifndef QLI_NOSTDIO
#define QLI_NOSTDIO 0
#endif

// configuration value
#ifndef QLI_PIXEL_FORMAT
#define QLI_PIXEL_FORMAT QLI_PF_RGB565
#endif

// configuration value
#ifndef QLI_USERDATA
#define QLI_USERDATA
#endif

// configuration value
#ifndef QLI_POSTFIX
#define QLI_POSTFIX
#endif

#if QLI_PIXEL_FORMAT == QLI_PF_RGB565
 #define QLI_BPP (2)
#else
 #error "Unsupported pixel format"
#endif

#define QLI_LITTLE_ENDIAN (QLI_BPP-1)
#define QLI_BIG_ENDIAN    (0)

// configuration value
#ifndef QLI_ENDIAN
#define QLI_ENDIAN QLI_LITTLE_ENDIAN
#endif

#ifndef QLI_ENCODE
#ifndef QLI_DECODE
#define QLI_ENCODE
#define QLI_DECODE
#endif
#endif

#define QLI_INDEX_SIZE 4

#define QLI_MAGIC0 'q'
#define QLI_MAGIC1 'l'
#define QLI_MAGIC2 'i'
#define QLI_MAGIC3 '1'

#if QLI_NOSTDIO == 0
#include <stdio.h>
#endif


struct qli_file
{
  char magic[4];
  uint16_t width;
  uint16_t height;
  uint8_t pixel_format;
  uint8_t flags; 	// bit 0-1:index-size (0: 3, 1: 4, 2: 5, 3: 6)
  uint8_t data[];
};

struct qli_image
{
  uint16_t width;
  uint16_t height;
  uint32_t pos;
  uint32_t size;
  uint8_t *data;
  uint8_t index[QLI_BPP][1L<<QLI_INDEX_SIZE];
  uint8_t px[QLI_BPP];
  uint8_t run;
  QLI_USERDATA
};


#define QLI_CONCAT(a, b) QLI_CONCAT_INNER(a, b)
#define QLI_CONCAT_INNER(a, b) a##b
#define QLI_FUNC_NAME(name, post) QLI_CONCAT(name, post)


#ifdef QLI_DECODE
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (struct qli_image *qli, uint16_t width, uint16_t height, uint8_t *data, uint32_t data_size);
int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (struct qli_image *qli, uint8_t *header, uint32_t size);
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (struct qli_image *qli);
int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (struct qli_image *qli, uint8_t *dest, uint32_t pixel_cnt);
#endif

#ifdef QLI_ENCODE
int QLI_FUNC_NAME(qli_encode,QLI_POSTFIX) (uint32_t *rgb, int width, int height, uint8_t *buf, size_t bufsize);
#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save,QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file);
#endif
#endif

#define QLI_HEADER_LEN (10)

#endif

/***************************************************************************************/

#ifdef QLI_IMPLEMENTATION

static const int qli_index_code[]={0,0,0,0,1,2,3,0};

// opcodes from QOI
#define QLI_OP_RGB   (0xff)
#define QLI_OP_INDEX (0x00)
#define QLI_OP_DIFF  (0x40)
#define QLI_OP_LUMA  (0x80)
#define QLI_OP_RUN   (0xc0)

#ifndef ABS
#define ABS(N) ((N) < 0 ? -(N) : (N))
#endif


#define QLI_CMD_MASK (0xc0)


#if QLI_PIXEL_FORMAT == QLI_PF_RGB565
 typedef uint16_t qli_pixel_t;
 /*         --------
  * 8421842184218421
  * rrrrrggggggbbbbb
  *        /\
  */
 #define R_FACTOR (8)
 #define G_FACTOR (4)
 #define B_FACTOR (8)
 #define QLI_CLAMP255(x) (((x) > 255) ? 255 : (x))
 #define RGB_PACK(r, g, b) ( \
    ((QLI_CLAMP255((r) + 4) >> 3) << 11) | \
    ((QLI_CLAMP255((g) + 2) >> 2) << 5)  | \
    ((QLI_CLAMP255((b) + 4) >> 3)) )
 #define PACK_GET_RED(rgb565)   (uint8_t)(((rgb565)>>8L)&~7L)
 #define PACK_GET_GREEN(rgb565) (uint8_t)(((rgb565)>>3L)&~3L)
 #define PACK_GET_BLUE(rgb565)  (uint8_t)(((rgb565)<<3L)&~7L)
 #define PX_GET_RED(px_)   PACK_GET_RED   ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))
 #define PX_GET_GREEN(px_) PACK_GET_GREEN ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))
 #define PX_GET_BLUE(px_)  PACK_GET_BLUE  ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))
#else
 #error "Unsupported pixel format"
#endif

// hash function is from QOI (alpha removed)
#define QLI_COLOR_HASH(r,g,b) ((r)*3 + (g)*5 + (b)*7)
#define QLI_GET_INDEX(p) QLI_COLOR_HASH(PX_GET_RED(p),PX_GET_GREEN(p),PX_GET_BLUE(p)) % (1L<<QLI_INDEX_SIZE)
#define QLI_UPDATE_INDEX(q,p) \
  do { \
    int idx = QLI_GET_INDEX(p); \
    for(int i=0;i<QLI_BPP;i++) (q)->index[i][idx] = p[i]; \
  } while(0)

/* rewind position pointer
 */
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (struct qli_image *qli)
{
  if(NULL!=qli)
  {
    qli->pos=0;
    memset(qli->index,0,sizeof(qli->index));
    qli->run=0;
    memset(qli->px,0,sizeof(qli->px));
  }
}

/* init user allocated qli struct
 */
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (struct qli_image *qli, uint16_t width, uint16_t height, uint8_t *data, uint32_t data_size)
{
  if(NULL==qli) return(-1);
  qli->width = width;
  qli->height = height;
  qli->data = data;
  qli->size = data_size;
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  return(0);
}

int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (struct qli_image *qli, uint8_t *header, uint32_t size)
{
  if(NULL==qli||NULL==header) return(-1);
  if(   header[0]!=QLI_MAGIC0
     || header[1]!=QLI_MAGIC1
     || header[2]!=QLI_MAGIC2
     || header[3]!=QLI_MAGIC3 ) return(-1);
  header+=4;
  qli->width=header[0]<<8|header[1];
  qli->height=header[2]<<8|header[3];
  if(QLI_PIXEL_FORMAT!=header[4]) return(-1);
  int flags=header[5];
  if(qli_index_code[QLI_INDEX_SIZE]!=((flags)&3)) return(-1);
  qli->size = size;
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  return(0);
}


#ifdef QLI_DECODE

/* decoding pixel_cnt pixels to the supplied destination area
 *
 * RETURN: number of pixels extracted
 */
int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (struct qli_image *qli, uint8_t *dest, uint32_t pixel_cnt)
{
  int ret=0;
  uint8_t d1;
  int i;
  
  if(pixel_cnt == 0) return(0);
  if(!qli || !dest) return(-1);
  
  while(pixel_cnt>0 && qli->pos<=qli->size)
  {
    if(qli->run>0)
    {
      for(i=0;i<QLI_BPP;i++) *dest++ = qli->px[ABS(QLI_ENDIAN-i)];
      ret++;
      --pixel_cnt;
      --qli->run;
      continue;
    }

    if(qli->pos==qli->size) break;

    d1 = qli->data[qli->pos++];
    uint8_t cm = d1&QLI_CMD_MASK;

    if (QLI_OP_RGB == d1)
    {
      for(i=0;i<QLI_BPP;i++) qli->px[i] = qli->data[qli->pos++];
      QLI_UPDATE_INDEX(qli, qli->px);
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_RGB 0x%04x [%02x %02x %02x]\n",qli->px[0],qli->px[1],qli->px[0]<<8|qli->px[1],PX_GET_RED(qli->px), PX_GET_GREEN(qli->px), PX_GET_BLUE(qli->px));
      #endif
      qli->run=1;
    }
    else if(QLI_OP_INDEX == cm)
    {
      d1&=(1L<<QLI_INDEX_SIZE)-1;
      for(int i=0;i<QLI_BPP;i++) qli->px[i] = qli->index[i][d1];
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_INDEX %d %02x %02x %02x\n",qli->px[0],qli->px[1],d1,PX_GET_RED(qli->px), PX_GET_GREEN(qli->px), PX_GET_BLUE(qli->px));
      #endif
      qli->run=1;
    }
    else if(QLI_OP_DIFF == cm)
    {
      int r = PX_GET_RED(qli->px);
      int g = PX_GET_GREEN(qli->px);
      int b = PX_GET_BLUE(qli->px);
      r += R_FACTOR * (((d1 >> 4) & 0x03) - 2);
      g += G_FACTOR * (((d1 >> 2) & 0x03) - 2);
      b += B_FACTOR * (( d1       & 0x03) - 2);
      qli_pixel_t rgb = RGB_PACK(r,g,b);
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_DIFF %d %d %d : %02x %02x %02x => %02x %02x %02x\n",rgb>>8,rgb&0xff,(((d1 >> 4) & 0x03) - 2), (((d1 >> 2) & 0x03) - 2), (( d1       & 0x03) - 2),
                 PX_GET_RED(qli->px), PX_GET_GREEN(qli->px), PX_GET_BLUE(qli->px),
                 PACK_GET_RED(rgb), PACK_GET_GREEN(rgb), PACK_GET_BLUE(rgb)
                 );
      #endif
      for(int i=0;i<QLI_BPP;i++) qli->px[i] = (rgb>>(8*(QLI_BPP-i-1)))&0xff;
      QLI_UPDATE_INDEX(qli, qli->px);
      qli->run=1;
    }
    else if(QLI_OP_LUMA == cm)
    {
      int r = PX_GET_RED(qli->px);
      int g = PX_GET_GREEN(qli->px);
      int b = PX_GET_BLUE(qli->px);
      int d2 = qli->data[qli->pos++];
      int vg = (d1 & 0x3f) - 32;
      r += R_FACTOR * (vg - 8 + ((d2 >> 4) & 0x0f));
      g += G_FACTOR * (vg);
      b += B_FACTOR * (vg - 8 +  (d2       & 0x0f));
      qli_pixel_t rgb = RGB_PACK(r,g,b);
      for(int i=0;i<QLI_BPP;i++) qli->px[i] = (rgb>>(8*(QLI_BPP-i-1)))&0xff;
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_LUMA %d\n",qli->px[0],qli->px[1],vg);
      #endif
      QLI_UPDATE_INDEX(qli, qli->px);
      qli->run=1;
    }
    else if(QLI_OP_RUN == cm)
    {
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_RUN %d %02x %02x %02x\n",qli->px[0],qli->px[1],1+(d1&0x3f), PX_GET_RED(qli->px), PX_GET_GREEN(qli->px), PX_GET_BLUE(qli->px) );
      #endif
      qli->run=1+(d1&0x3f);
    }
  }
  
  return(ret);
}

#endif

#ifdef QLI_ENCODE

#include <stdlib.h>

#define RGB32_RED(p32)   (uint8_t)(((uint32_t)p32)>>24)
#define RGB32_GREEN(p32) (uint8_t)((((uint32_t)p32)>>16)&0xff)
#define RGB32_BLUE(p32)  (uint8_t)((((uint32_t)p32)>>8)&0xff)


/* encoding RGB32 buffer with RGBx format and given dmensions to buf
 *
 * if buf is NULL it will return the size for future allocation
 *
 * RETURN: the number of bytes written to buf (or would be written to if supplied)
 *         or negative on error
 */
int QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (uint32_t *rgb, int width, int height, uint8_t *buf, size_t bufsize)
{
  int i,res;
  struct qli_image img;
  int pos,opos;
  int out_cnt=0;
  uint32_t pix=0;
  uint8_t px[QLI_BPP]={0};
  qli_pixel_t ppx,ppx_prev=0;
  int run=0,end;

  if(rgb==NULL||width<=0||height<=0) return(-1);
  if(0!=qli_init(&img, width, height, buf, bufsize)) return(-1);
  end=width*height;
  if(NULL==buf) bufsize=width*height*(QLI_BPP+1);
  for(pos=opos=0;pos<end;pos++,ppx_prev=ppx)
  {
    pix=rgb[pos];
    ppx=RGB_PACK(RGB32_RED(pix), RGB32_GREEN(pix), RGB32_BLUE(pix));
    for(int i=0;i<QLI_BPP;i++) px[i] = (ppx>>(8*(QLI_BPP-i-1)))&0xff;
    if(ppx==ppx_prev)
    {
      run++;
      if(run==63||pos==end-1)
      {
        if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_RUN|(run-1);
        out_cnt++;
        #if QLI_DEBUG != 0
        fprintf(stderr,"%02x%02x QLI_OP_RUN %d %02x %02x %02x\n",px[0],px[1],run, PACK_GET_RED(ppx), PACK_GET_GREEN(ppx), PACK_GET_BLUE(ppx));
        #endif
        run=0;
      }
      continue;
    }
    if(run>0)
    {
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_RUN|(run-1);
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_RUN %d %02x %02x %02x\n",ppx_prev>>8,ppx_prev&0xff, run, PACK_GET_RED(ppx_prev), PACK_GET_GREEN(ppx_prev), PACK_GET_BLUE(ppx_prev));
      #endif
      run=0;
      out_cnt++;
    }
    int idx=QLI_GET_INDEX(px);
    for(i=res=0;i<QLI_BPP;i++) res|=(px[i]^img.index[i][idx]);
    if(res==0)
    {
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_INDEX|(idx);
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_INDEX %d %02x %02x %02x\n",px[0],px[1],idx, PX_GET_RED(px), PX_GET_GREEN(px), PX_GET_BLUE(px));
      #endif
      out_cnt++;
      continue;
    }
    QLI_UPDATE_INDEX(&img,px);
    signed char vr = ( PACK_GET_RED(ppx)   - PACK_GET_RED(ppx_prev)   ) / R_FACTOR;
    signed char vg = ( PACK_GET_GREEN(ppx) - PACK_GET_GREEN(ppx_prev) ) / G_FACTOR;
    signed char vb = ( PACK_GET_BLUE(ppx)  - PACK_GET_BLUE(ppx_prev)  ) / B_FACTOR;
    if( vr>-3 && vr<2 && vg>-3 && vg<2 && vb>-3 && vb<2 )
    {
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_DIFF|( (vr + 2) << 4 | (vg + 2) << 2 | (vb + 2) );
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_DIFF %d %d %d : %02x %02x %02x => %02x %02x %02x\n",px[0],px[1],vr,vg,vb, 
                         PACK_GET_RED(ppx_prev), PACK_GET_GREEN(ppx_prev), PACK_GET_BLUE(ppx_prev),
                         PACK_GET_RED(ppx), PACK_GET_GREEN(ppx), PACK_GET_BLUE(ppx) );
      #endif
      out_cnt++;
      continue;
    }
    signed char vg_r = vr-vg;
    signed char vg_b = vb-vg;
    if( vg_r>-9 && vg_r<8 && vg>-33 && vg<32 && vg_b>-9 && vg_b<8 )
    {
      if(NULL!=buf&&opos<bufsize-1)
      {
        buf[opos++]=QLI_OP_LUMA | (vg+32);
        buf[opos++]=(vg_r+8)<<4 | (vg_b+8);
        #if QLI_DEBUG != 0
        fprintf(stderr,"%02x%02x QLI_OP_LUMA %d\n",px[0],px[1],vg);
        #endif
      }
      out_cnt+=2;
      continue;
    }
    if(NULL!=buf&&opos<bufsize-QLI_BPP-1)
    {
      buf[opos++]=QLI_OP_RGB;
      for(i=0;i<QLI_BPP;i++) buf[opos++] = px[i];
      #if QLI_DEBUG != 0
      fprintf(stderr,"%02x%02x QLI_OP_RGB 0x%04x [%02x %02x %02x]\n",px[0],px[1],ppx,PX_GET_RED(px), PX_GET_GREEN(px), PX_GET_BLUE(px));
      #endif
    }
    out_cnt+=1+QLI_BPP;
  }
  return(out_cnt);
}

#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save, QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file)
{
  uint8_t *buf;
  int ret=0;
  
  if(rgb==NULL||width<=0||height<=0||NULL==file) return(-1);
  buf=calloc(1,width*height*QLI_BPP);
  ret= QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (rgb, width, height, buf, width*height*QLI_BPP);
  if(ret>0)
  {
    FILE *f=fopen(file,"wb");
    if(NULL!=f)
    {
      fputc(QLI_MAGIC0,f);
      fputc(QLI_MAGIC1,f);
      fputc(QLI_MAGIC2,f);
      fputc(QLI_MAGIC3,f);
      fputc(width>>8,f);
      fputc(width&0xff,f);
      fputc(height>>8,f);
      fputc(height&0xff,f);
      fputc(QLI_PIXEL_FORMAT,f);
      uint8_t flags=0;
      flags|=qli_index_code[QLI_INDEX_SIZE];
      fputc(flags,f);
      fwrite(buf,1,ret,f);
    }
    fclose(f);
  }
  free(buf);
  return(ret);
}
#endif

#endif

#endif
