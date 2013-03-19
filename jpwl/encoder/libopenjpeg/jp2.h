/*
 * Copyright (c) 2003, Yannick Verschueren
 * Copyright (c) 2003,  Communications and remote sensing Laboratory, Universite catholique de Louvain, Belgium
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __JP2_H
#define __JP2_H

#include "j2k.h"

typedef struct {
  int depth;		  
  int sgnd;		   
  int bpcc;//Bits Per Component 框
} jp2_comps_t;

typedef struct {
  unsigned int w;
  unsigned int h;
  unsigned int numcomps;//图像分量总数
  unsigned int bpc;//bits per component框 ,必需,表示位深度
  unsigned int C;//image header 框中的CT表示压缩类型,惟一合法值是7,
  unsigned int UnkC;
  unsigned int IPR;//可选,知识产权框
  unsigned int meth;//header>image header框中的COLOR specification框中的M参数,M=1:表示彩色空间通过 ECS(枚举彩色空间)发送,M=2:表示彩色空间通过ICP发送 (ECS与ICP只有其一)
  unsigned int approx;
  unsigned int enumcs;//ECS,枚举彩色空间
  unsigned int precedence;//优先级
  unsigned int brand;//file type框中的BR框,定义所采用的具体文件格式
  unsigned int minversion;//file type框中的MV框,定义商标最小版本号
  unsigned int numcl;//FILE TYPE 框 里指定文件所符合的标准兼容列表,此处是兼容的数目
  unsigned int *cl;//具体兼容的参数
  jp2_comps_t *comps;
  j2k_image_t *image;
} jp2_struct_t;/*  */

/*框结构*/
typedef struct {
  int length;
  int type;
  int init_pos;
} jp2_box_t;

/* int jp2_init_stdjp2(jp2_struct_t * jp2_struct, j2k_image_t * img); 
 *
 * Create a standard jp2_structure
 * jp2_struct: the structure you are creating
 * img: a j2k_image_t wich will help you to create the jp2_structure
 */
int jp2_init_stdjp2(jp2_struct_t * jp2_struct, j2k_image_t * img);

/* int jp2_write_jp2c(j2k_image_t * img, j2k_cp_t * cp, char *jp2_buffer,
 *		   char *index);
 *
 * Write the jp2c codestream box 
 * img: the j2k_image that will be compressed
 * jp2_buffer: the buffer that will recieve the compressed data
 * index: the name of the index file 
 */
int jp2_write_jp2c(j2k_image_t * img, j2k_cp_t * cp, char *jp2_buffer,
		   char *index);


/* int jp2_write_jp2h(jp2_struct_t * jp2_struct);
 *
 * Write the jp2h header box 
 * jp2_struct: the jp2 structure you are working with
 */
void jp2_write_jp2h(jp2_struct_t * jp2_struct);

/* int jp2_read_jp2h(jp2_struct_t * jp2_struct);
 *
 * Read the jp2h header box 
 * jp2_struct: the jp2 structure you are working with
 */
int jp2_read_jp2h(jp2_struct_t * jp2_struct);

/* int jp2_encode(jp2_struct_t * jp2_struct, j2k_cp_t * cp, char *output,
 *       char *index);
 *
 * Encode a JP2 file
 * jp2_buffer: the buffer containing the pointer to the image to encode
 * cp: coding parameters of the image
 * outbuf: pointer to memory where compressed data will be written
 * index: the name of the index file 
 */
int jp2_encode(jp2_struct_t * jp2_struct, j2k_cp_t * cp, char *output,
	       char *index);

/* int jp2_decode(unsigned char *src, int len, jp2_struct_t * jp2_struct,
 *	       j2k_cp_t * cp);
 *
 * Decode a JP2 file
 * src: pointer to memory where compressed data is stored
 * len: length of src buffer
 * jp2_struct: the jp2 structure that will be created 
 * cp: coding parameters of the image
 */
int jp2_decode(unsigned char *src, int len, jp2_struct_t * jp2_struct,
	       j2k_cp_t * cp);

#endif
