/*
* Copyright (c) 2001-2002, David Janssens
* Copyright (c) 2002-2004, Yannick Verschueren
* Copyright (c) 2002-2004, Communications and remote sensing Laboratory, Universite catholique de Louvain, Belgium
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

#include "tcd.h"
#include "int.h"
#include "t1.h"
#include "t2.h"
#include "dwt.h"
#include "mct.h"
#include <setjmp.h>
#include <float.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static tcd_image_t tcd_image;//

static j2k_image_t *tcd_img;/* 当前分量图像,在tcd_malloc_encode中定义 */
static j2k_cp_t *tcd_cp;/*  当前tile分量 */

static tcd_tile_t *tcd_tile;
static j2k_tcp_t *tcd_tcp;/* 当前切片分量*/
static int tcd_tileno;/* 当前切片号*/

static tcd_tile_t *tile;
static tcd_tilecomp_t *tilec;/* tile 的信息分量*/
static tcd_resolution_t *res;/* tile 的分辨率 */
static tcd_band_t *band;/* tile 的子带*/
static tcd_precinct_t *prc;/* tile 的分区 */
static tcd_cblk_t *cblk;/* tile 的code block 信息 */

extern jmp_buf j2k_error;

void tcd_dump(tcd_image_t * img, int curtileno)
{
	int tileno, compno, resno, bandno, precno, cblkno;
	fprintf(stderr, "image {\n"); 
	fprintf(stderr, "  tw=%d, th=%d x0=%d x1=%d y0=%d y1=%d\n", img->tw, img->th,
		tcd_img->x0, tcd_img->x1, tcd_img->y0, tcd_img->y1);
	for (tileno = 0; tileno < img->th*img->tw; tileno++) {
		tcd_tile_t *tile = &tcd_image.tiles[tileno];
		fprintf(stderr, "  tile {\n"); 
		fprintf(stderr, "    x0=%d, y0=%d, x1=%d, y1=%d, numcomps=%d\n", tile->x0, tile->y0, tile->x1, tile->y1, tile->numcomps); 
		for (compno = 0; compno < tile->numcomps; compno++) {
			tcd_tilecomp_t *tilec = &tile->comps[compno];
			fprintf(stderr, "    tilec {\n"); 
			fprintf(stderr, "      x0=%d, y0=%d, x1=%d, y1=%d, numresolutions=%d\n", tilec->x0, tilec->y0, tilec->x1, tilec->y1, tilec->numresolutions); 
			for (resno = 0; resno < tilec->numresolutions; resno++) {
				tcd_resolution_t *res = &tilec->resolutions[resno];
				fprintf(stderr, "\n   res {\n"); 
				fprintf(stderr, "          x0=%d, y0=%d, x1=%d, y1=%d, pw=%d, ph=%d, numbands=%d\n", res->x0, res->y0, res->x1, res->y1, res->pw, res->ph, res->numbands); 
				for (bandno = 0; bandno < res->numbands; bandno++) {
					tcd_band_t *band = &res->bands[bandno];
					fprintf(stderr, "        band {\n"); 
					fprintf(stderr, "          x0=%d, y0=%d, x1=%d, y1=%d, stepsize=%d, numbps=%d\n", band->x0, band->y0, band->x1, band->y1, band->stepsize, band->numbps); 
					for (precno = 0; precno < res->pw * res->ph; precno++) {
						tcd_precinct_t *prec = &band->precincts[precno];
						fprintf(stderr, "          prec {\n"); 
						fprintf(stderr, "            x0=%d, y0=%d, x1=%d, y1=%d, cw=%d, ch=%d\n", prec->x0, prec->y0, prec->x1, prec->y1, prec->cw, prec->ch); 
						for (cblkno = 0; cblkno < prec->cw * prec->ch; cblkno++) {
							tcd_cblk_t *cblk=&prec->cblks[cblkno]; 
							fprintf(stderr, "            cblk {\n"); 
							fprintf(stderr, "              x0=%d, y0=%d, x1=%d, y1=%d\n", cblk->x0, cblk->y0, cblk->x1, cblk->y1); 
							fprintf(stderr, "            }\n"); 
						}
						fprintf(stderr, "          }\n"); 
					}
					fprintf(stderr, "        }\n"); 
				}
				fprintf(stderr, "      }\n"); 
			}
			fprintf(stderr, "    }\n"); 
		}
		fprintf(stderr, "  }\n"); 
	}
	fprintf(stderr, "}\n"); 
}

/* 为第一个tile分配空间 */
/* 参数:(图像,分量,当前tile编号) */
void tcd_malloc_encode(j2k_image_t * img, j2k_cp_t * cp, int curtileno)
{
	int tileno, compno, resno, bandno, precno, cblkno;
	tcd_img = img;
	tcd_cp = cp;
	tcd_image.tw = cp->tw;
	tcd_image.th = cp->th;
	tcd_image.tiles = (tcd_tile_t *) malloc(sizeof(tcd_tile_t));//tile信息

	for (tileno = 0; tileno < 1; tileno++) {
		//其实只遍历一次
		j2k_tcp_t *tcp = &cp->tcps[curtileno];//获取当前tile分量

		int j;
		/* cfr p59 ISO/IEC FDIS15444-1 : 2000 (18 august 2000) */
		int p = curtileno % cp->tw;	/* si numerotation matricielle .. */
		int q = curtileno / cp->tw;	/* .. coordonnees de la tile (q,p) q pour ligne et p pour colonne */
		//获取curtileno所在图像域的(q,p)=(行,列)

		/* tcd_tile_t *tile=&tcd_image.tiles[tileno]; */
		tile = tcd_image.tiles;//获取tile空间指针

		/* 4 borders of the tile rescale on the image if necessary */
		//如果必须的话把tile的边框定在图像里(5.23),计算每一个tile的原点(x0,y0)与结束点(x1,y1)
		//计算图像在分量中的坐标(公式:5.23)
		tile->x0 = int_max(cp->tx0 + p * cp->tdx, img->x0);//如果图像的X起码点>当前tile的X坐标,则tile的X定为图像的起始点
		tile->y0 = int_max(cp->ty0 + q * cp->tdy, img->y0);//
		tile->x1 = int_min(cp->tx0 + (p + 1) * cp->tdx, img->x1);//如果网格的结束X<图像域的X,则返回网格的X
		tile->y1 = int_min(cp->ty0 + (q + 1) * cp->tdy, img->y1);

		tile->numcomps = img->numcomps;

		//设定RATE,对每个质量层进行赋值
		for (j = 0; j < tcp->numlayers; j++) {
			tcp->rates[j] =int_ceildiv(
				tile->numcomps * (tile->x1 - tile->x0) *(tile->y1 - tile->y0) * img->comps[0].prec, //分量总数*(tile宽)*(tile高)*图像0号分量精准度
				tcp->rates[j] * 8 * img->comps[0].dx *img->comps[0].dy);//tile切片分量参数第J号质量*8(BIT)*图像0号分量水平采样周期*图像0号分量垂直采样周期 

			if (j && tcp->rates[j] < tcp->rates[j - 1] + 10) {//非第一个层且层质量小于(上一个层质量+10)
				tcp->rates[j] = tcp->rates[j - 1] + 20;//当前质量层的质量为前一层+20
			} else {
				if (!j && tcp->rates[j] < 30)//如果是第一层且第一层质量<30
					tcp->rates[j] = 30;//第一层为30
			}
		}
		tile->comps =(tcd_tilecomp_t *) malloc(img->numcomps * sizeof(tcd_tilecomp_t));//针对不同分量上的同一位置的tile分量信息

		//遍历不同分量上同一级别的tile,通过分量采样率映射
		for (compno = 0; compno < tile->numcomps; compno++) {
			j2k_tccp_t *tccp = &tcp->tccps[compno];//取得一个分量上的tile对象

			tilec = &tile->comps[compno];// 取得当前分量上的tile信息空间
			/* border of each tile component (global) */

			//计算左上采样坐标与右下采样坐标(映射到具体分量上的坐标,每个分量上根据不同的采样率不同分量同一位置坐标可能不一致)(5.24)
			tilec->x0 = int_ceildiv(tile->x0, img->comps[compno].dx);//[tile左上X/水平方向采样点]
			tilec->y0 = int_ceildiv(tile->y0, img->comps[compno].dy);
			tilec->x1 = int_ceildiv(tile->x1, img->comps[compno].dx);
			tilec->y1 = int_ceildiv(tile->y1, img->comps[compno].dy);

			tilec->data =(int *) malloc((tilec->x1 - tilec->x0) * (tilec->y1 - tilec->y0) * sizeof(int));//对应分量tile在其对应采样率下的大小*sizeof(int)

			tilec->numresolutions = tccp->numresolutions;//分辨率层数

			tilec->resolutions =(tcd_resolution_t *) malloc(tilec->numresolutions * sizeof(tcd_resolution_t));


		
			for (resno = 0; resno < tilec->numresolutions; resno++) {
				//遍历划分分辨率
				int pdx, pdy;//分区宽高
				int levelno = tilec->numresolutions - 1 - resno;//当前分辨率层次,numresolutions,numresolutions-1,......,0

				int tlprcxstart, tlprcystart, brprcxend, brprcyend;
				int tlcbgxstart, tlcbgystart, brcbgxend, brcbgyend;
				int cbgwidthexpn, cbgheightexpn;/* PPx,PPy */
				int cblkwidthexpn, cblkheightexpn;/* xcb',ycb' */
				/* tcd_resolution_t *res=&tilec->resolutions[resno]; */

				res = &tilec->resolutions[resno];//当前分辨率信息

				/* border for each resolution level (global) */
				//分辨率坐标计算（公式：5.25）
				res->x0 = int_ceildivpow2(tilec->x0, levelno);
				res->y0 = int_ceildivpow2(tilec->y0, levelno);
				res->x1 = int_ceildivpow2(tilec->x1, levelno);
				res->y1 = int_ceildivpow2(tilec->y1, levelno);

				res->numbands = resno == 0 ? 1 : 3;//如果是第0个分辨率,则子带数为1,否则为3个子带数

				/* p. 35, table A-23, ISO/IEC FDIS154444-1 : 2000 (18 august 2000) */
				if (tccp->csty & J2K_CCP_CSTY_PRT) {
					pdx = tccp->prcw[resno];//当前分辨率层次下的分区宽度
					pdy = tccp->prch[resno];//当前分辨率层次下的分区高度
				} else {
					pdx = 15;//否则默认分区大小为15*15
					pdy = 15;//0x1111
				}

				/* p. 64, B.6, ISO/IEC FDIS15444-1 : 2000 (18 august 2000)  */
				//在当前分辨率下计算分区的坐标位置
				tlprcxstart = int_floordivpow2(res->x0, pdx) << pdx;//top left precinct cx start
				tlprcystart = int_floordivpow2(res->y0, pdy) << pdy;//top left precinct cy start
				brprcxend = int_ceildivpow2(res->x1, pdx) << pdx;//bottom right precinct cx start
				brprcyend = int_ceildivpow2(res->y1, pdy) << pdy;//bottom right precinct cy start

				res->pw = (brprcxend - tlprcxstart) >> pdx;
				res->ph = (brprcyend - tlprcystart) >> pdy;

				//////////////////////////////////////////////////////////////////////////
				//计算子带code block 大小(公式:5.28,5.29)
				if (resno == 0) {
					//对第0层分辨率(最低频率成分)进行处理
					tlcbgxstart = tlprcxstart;//top left code block gx start
					tlcbgystart = tlprcystart;//top left code block gy start
					brcbgxend = brprcxend;//bottom right code block gx end
					brcbgyend = brprcyend;//bottom right code block gy end
					cbgwidthexpn = pdx;//code block g width expn=当前分辨率层次下的分区宽度
					cbgheightexpn = pdy;//code block g height expn=当前分辨率层次下的分区高度
				} else {
					tlcbgxstart = int_ceildivpow2(tlprcxstart, 1);
					tlcbgystart = int_ceildivpow2(tlprcystart, 1);
					brcbgxend = int_ceildivpow2(brprcxend, 1);
					brcbgyend = int_ceildivpow2(brprcyend, 1);
					cbgwidthexpn = pdx - 1;//code block g width expn=当前分辨率层次下的分区宽度-1
					cbgheightexpn = pdy - 1;
				}

				cblkwidthexpn = int_min(tccp->cblkw, cbgwidthexpn);
				cblkheightexpn = int_min(tccp->cblkh, cbgheightexpn);
				//////////////////////////////////////////////////////////////////////////

				for (bandno = 0; bandno < res->numbands; bandno++) {
					//遍历划分子带
					int x0b, y0b, i;
					int gain;/* 基于变换编码的压缩特性提高效果尺度 */
					int numbps;
					j2k_stepsize_t *ss;
					band = &res->bands[bandno];// 从分辨率下获取到一个子带
					band->bandno = resno == 0 ? 0 : bandno + 1;//第0层分辨率子数

					x0b = (band->bandno == 1)|| (band->bandno == 3) ? 1 : 0;//对第一个及第三个子带给1
					y0b = (band->bandno == 2)|| (band->bandno == 3) ? 1 : 0;

					if (band->bandno == 0) {
						//LL子带
						/* band border (global) */
						band->x0 = int_ceildivpow2(tilec->x0, levelno);
						band->y0 = int_ceildivpow2(tilec->y0, levelno);
						band->x1 = int_ceildivpow2(tilec->x1, levelno);
						band->y1 = int_ceildivpow2(tilec->y1, levelno);
					} else {
						//HL,LH,HH
						/* band border (global) */
						band->x0 =int_ceildivpow2(tilec->x0 - (1 << levelno) * x0b, levelno + 1);
						band->y0 =int_ceildivpow2(tilec->y0 - (1 << levelno) * y0b, levelno + 1);
						band->x1 =int_ceildivpow2(tilec->x1 - (1 << levelno) * x0b, levelno + 1);
						band->y1 =int_ceildivpow2(tilec->y1 - (1 << levelno) * y0b, levelno + 1);
					}

					ss = &tccp->stepsizes[resno ==0 ? 0 : 3 * (resno - 1) + bandno + 1];

					gain =tccp->qmfbid ==0 ? dwt_getgain_real(band->bandno) : dwt_getgain(band->bandno);

					numbps = img->comps[compno].prec + gain;//BPS=当前分量精准度+压缩特性提高效果尺度
					band->stepsize =(int) floor((1.0 + ss->mant / 2048.0) * pow(2.0, numbps - ss->expn) * 8192.0);
					band->numbps = ss->expn + tccp->numgbits - 1;	/* WHY -1 ? */

					band->precincts =(tcd_precinct_t *) malloc(3 * res->pw * res->ph * sizeof(tcd_precinct_t));

					for (i = 0; i < res->pw * res->ph * 3; i++) {
						band->precincts[i].imsbtree = NULL;
						band->precincts[i].incltree = NULL;
					}
					for (precno = 0; precno < res->pw * res->ph; precno++) {
						//遍历分区 
						int tlcblkxstart, tlcblkystart, brcblkxend, brcblkyend;
						int cbgxstart = tlcbgxstart + (precno % res->pw) * (1 << cbgwidthexpn);
						int cbgystart = tlcbgystart + (precno / res->pw) * (1 << cbgheightexpn);
						int cbgxend = cbgxstart + (1 << cbgwidthexpn);
						int cbgyend = cbgystart + (1 << cbgheightexpn);
						/* tcd_precinct_t *prc=&band->precincts[precno]; */

						prc = &band->precincts[precno];//从当前子带中获取分区

						/* precinct size (global) */
						prc->x0 = int_max(cbgxstart, band->x0);
						prc->y0 = int_max(cbgystart, band->y0);
						prc->x1 = int_min(cbgxend, band->x1);
						prc->y1 = int_min(cbgyend, band->y1);

						tlcblkxstart = int_floordivpow2(prc->x0, cblkwidthexpn) << cblkwidthexpn;//downward
						tlcblkystart = int_floordivpow2(prc->y0, cblkheightexpn) << cblkheightexpn;
						brcblkxend = int_ceildivpow2(prc->x1, cblkwidthexpn) << cblkwidthexpn;//upward
						brcblkyend = int_ceildivpow2(prc->y1, cblkheightexpn) << cblkheightexpn;

						prc->cw = (brcblkxend - tlcblkxstart) >> cblkwidthexpn;
						prc->ch = (brcblkyend - tlcblkystart) >> cblkheightexpn;

						prc->cblks =(tcd_cblk_t *) malloc((prc->cw * prc->ch) * sizeof(tcd_cblk_t));
						prc->incltree = tgt_create(prc->cw, prc->ch);
						prc->imsbtree = tgt_create(prc->cw, prc->ch);
						for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
							//遍历codeblock
							int cblkxstart =
								tlcblkxstart + (cblkno % prc->cw) * (1 << cblkwidthexpn);
							int cblkystart =
								tlcblkystart + (cblkno / prc->cw) * (1 << cblkheightexpn);
							int cblkxend = cblkxstart + (1 << cblkwidthexpn);
							int cblkyend = cblkystart + (1 << cblkheightexpn);

							cblk = &prc->cblks[cblkno];
							/* code-block size (global) */
							cblk->x0 = int_max(cblkxstart, prc->x0);
							cblk->y0 = int_max(cblkystart, prc->y0);
							cblk->x1 = int_min(cblkxend, prc->x1);
							cblk->y1 = int_min(cblkyend, prc->y1);
						}//codeblock for ends
					}//precno for ends
				}//bands for ends
			}//resolutions for ends
		}//compno for ends
	}//tileno for ends
	/* tcd_dump(&tcd_image,curtileno); */
}

void tcd_init_encode(j2k_image_t * img, j2k_cp_t * cp, int curtileno)
{
	int tileno, compno, resno, bandno, precno, cblkno;

	for (tileno = 0; tileno < 1; tileno++) {
		j2k_tcp_t *tcp = &cp->tcps[curtileno];//获取当前tile
		int j;
		//              int previous_x0, previous_x1, previous_y0, previous_y1;
		/* cfr p59 ISO/IEC FDIS15444-1 : 2000 (18 august 2000) */
		int p = curtileno % cp->tw;
		int q = curtileno / cp->tw;//(q,p)=(行,列)
		tile = tcd_image.tiles;//tile分量图像信息

		/* 4 borders of the tile rescale on the image if necessary */
		tile->x0 = int_max(cp->tx0 + p * cp->tdx, img->x0);//将tile的边框限制在图像起点或者图像域的起点(取大)
		tile->y0 = int_max(cp->ty0 + q * cp->tdy, img->y0);
		tile->x1 = int_min(cp->tx0 + (p + 1) * cp->tdx, img->x1);//将tile 的边框限制在图像的末点或者图像域的未点(取小)
		tile->y1 = int_min(cp->ty0 + (q + 1) * cp->tdy, img->y1);

		tile->numcomps = img->numcomps;//每个tile包含的分量数
		/* tile->PPT=img->PPT; */

		/* Modification of the RATE >> */
		for (j = 0; j < tcp->numlayers; j++) {
			//遍历质量层
			tcp->rates[j] =int_ceildiv(
				tile->numcomps * (tile->x1 - tile->x0) *(tile->y1 -tile->y0) * img->comps[0].prec,
				(tcp->rates[j] * 8 * img->comps[0].dx * img->comps[0].dy)
				);

			if (j && tcp->rates[j] < tcp->rates[j - 1] + 10) {
				tcp->rates[j] = tcp->rates[j - 1] + 20;
			} else {
				if (!j && tcp->rates[j] < 30)
					tcp->rates[j] = 30;
			}
		}//rate over
		/* << Modification of the RATE */

		/* tile->comps=(tcd_tilecomp_t*)realloc(tile->comps,img->numcomps*sizeof(tcd_tilecomp_t)); */
		for (compno = 0; compno < tile->numcomps; compno++) {
			//遍历tile中的分量
			j2k_tccp_t *tccp = &tcp->tccps[compno];
			/* int realloc_op; */

			tilec = &tile->comps[compno];//tile中的分量信息
			/* border of each tile component (global) */
			tilec->x0 = int_ceildiv(tile->x0, img->comps[compno].dx);
			tilec->y0 = int_ceildiv(tile->y0, img->comps[compno].dy);
			tilec->x1 = int_ceildiv(tile->x1, img->comps[compno].dx);
			tilec->y1 = int_ceildiv(tile->y1, img->comps[compno].dy);

			tilec->data =(int *) malloc((tilec->x1 - tilec->x0) * (tilec->y1 - tilec->y0) * sizeof(int));//为分量申请数据空间

			tilec->numresolutions = tccp->numresolutions;//设置分辨率层数
			/* tilec->resolutions=(tcd_resolution_t*)realloc(tilec->resolutions,tilec->numresolutions*sizeof(tcd_resolution_t)); */
			for (resno = 0; resno < tilec->numresolutions; resno++) {
				//遍历分辨率 
				int pdx, pdy;
				int levelno = tilec->numresolutions - 1 - resno;
				int tlprcxstart, tlprcystart, brprcxend, brprcyend;
				int tlcbgxstart, tlcbgystart, brcbgxend, brcbgyend;
				int cbgwidthexpn, cbgheightexpn;
				int cblkwidthexpn, cblkheightexpn;

				res = &tilec->resolutions[resno];
				/* border for each resolution level (global) */
				res->x0 = int_ceildivpow2(tilec->x0, levelno);
				res->y0 = int_ceildivpow2(tilec->y0, levelno);
				res->x1 = int_ceildivpow2(tilec->x1, levelno);
				res->y1 = int_ceildivpow2(tilec->y1, levelno);

				res->numbands = resno == 0 ? 1 : 3;
				/* p. 35, table A-23, ISO/IEC FDIS154444-1 : 2000 (18 august 2000) */
				//??????????????????????????????????????
				if (tccp->csty & J2K_CCP_CSTY_PRT) {
					pdx = tccp->prcw[resno];
					pdy = tccp->prch[resno];
				} else {
					pdx = 15;
					pdy = 15;
				}
				/* p. 64, B.6, ISO/IEC FDIS15444-1 : 2000 (18 august 2000)  */
				tlprcxstart = int_floordivpow2(res->x0, pdx) << pdx;
				tlprcystart = int_floordivpow2(res->y0, pdy) << pdy;
				brprcxend = int_ceildivpow2(res->x1, pdx) << pdx;
				brprcyend = int_ceildivpow2(res->y1, pdy) << pdy;

				res->pw = (brprcxend - tlprcxstart) >> pdx;
				res->ph = (brprcyend - tlprcystart) >> pdy;

				if (resno == 0) {
					tlcbgxstart = tlprcxstart;
					tlcbgystart = tlprcystart;
					brcbgxend = brprcxend;
					brcbgyend = brprcyend;
					cbgwidthexpn = pdx;
					cbgheightexpn = pdy;
				} else {
					tlcbgxstart = int_ceildivpow2(tlprcxstart, 1);
					tlcbgystart = int_ceildivpow2(tlprcystart, 1);
					brcbgxend = int_ceildivpow2(brprcxend, 1);
					brcbgyend = int_ceildivpow2(brprcyend, 1);
					cbgwidthexpn = pdx - 1;
					cbgheightexpn = pdy - 1;
				}

				cblkwidthexpn = int_min(tccp->cblkw, cbgwidthexpn);
				cblkheightexpn = int_min(tccp->cblkh, cbgheightexpn);

				for (bandno = 0; bandno < res->numbands; bandno++) {
					//遍历子带
					int x0b, y0b;
					int gain, numbps;
					j2k_stepsize_t *ss;
					band = &res->bands[bandno];
					band->bandno = resno == 0 ? 0 : bandno + 1;
					x0b = (band->bandno == 1) || (band->bandno == 3) ? 1 : 0;
					y0b = (band->bandno == 2) || (band->bandno == 3) ? 1 : 0;

					if (band->bandno == 0) {
						/* band border */
						band->x0 = int_ceildivpow2(tilec->x0, levelno);
						band->y0 = int_ceildivpow2(tilec->y0, levelno);
						band->x1 = int_ceildivpow2(tilec->x1, levelno);
						band->y1 = int_ceildivpow2(tilec->y1, levelno);
					} else {
						band->x0 =
							int_ceildivpow2(tilec->x0 -
							(1 << levelno) * x0b, levelno + 1);
						band->y0 =
							int_ceildivpow2(tilec->y0 -
							(1 << levelno) * y0b, levelno + 1);
						band->x1 =
							int_ceildivpow2(tilec->x1 -
							(1 << levelno) * x0b, levelno + 1);
						band->y1 =
							int_ceildivpow2(tilec->y1 -
							(1 << levelno) * y0b, levelno + 1);
					}

					ss = &tccp->stepsizes[resno ==0 ? 0 : 3 * (resno - 1) + bandno + 1];
					gain =tccp->qmfbid ==0 ? dwt_getgain_real(band->bandno) : dwt_getgain(band->bandno);
					numbps = img->comps[compno].prec + gain;
					band->stepsize = (int) floor((1.0 + ss->mant / 2048.0) *pow(2.0, numbps - ss->expn) * 8192.0);
					band->numbps = ss->expn + tccp->numgbits - 1;	/* WHY -1 ? */

					for (precno = 0; precno < res->pw * res->ph; precno++) {
						//遍历分区
						int tlcblkxstart, tlcblkystart, brcblkxend, brcblkyend;
						int cbgxstart =tlcbgxstart + (precno % res->pw) * (1 << cbgwidthexpn);
						int cbgystart = tlcbgystart + (precno / res->pw) * (1 << cbgheightexpn);
						int cbgxend = cbgxstart + (1 << cbgwidthexpn);
						int cbgyend = cbgystart + (1 << cbgheightexpn);

						prc = &band->precincts[precno];//从子带中获取分区
						/* precinct size (global) */
						prc->x0 = int_max(cbgxstart, band->x0);
						prc->y0 = int_max(cbgystart, band->y0);
						prc->x1 = int_min(cbgxend, band->x1);
						prc->y1 = int_min(cbgyend, band->y1);

						tlcblkxstart =int_floordivpow2(prc->x0, cblkwidthexpn) << cblkwidthexpn;
						tlcblkystart = int_floordivpow2(prc->y0, cblkheightexpn) << cblkheightexpn;
						brcblkxend = int_ceildivpow2(prc->x1, cblkwidthexpn) << cblkwidthexpn;
						brcblkyend =int_ceildivpow2(prc->y1, cblkheightexpn) << cblkheightexpn;
						prc->cw = (brcblkxend - tlcblkxstart) >> cblkwidthexpn;
						prc->ch = (brcblkyend - tlcblkystart) >> cblkheightexpn;

						free(prc->cblks);
						prc->cblks =(tcd_cblk_t *) malloc(prc->cw * prc->ch * sizeof(tcd_cblk_t));//分区的code block信息

						if (prc->incltree != NULL)
							tgt_destroy(prc->incltree);
						if (prc->imsbtree != NULL)
							tgt_destroy(prc->imsbtree);

						prc->incltree = tgt_create(prc->cw, prc->ch);//创建tag-tree
						prc->imsbtree = tgt_create(prc->cw, prc->ch);

						for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
							int cblkxstart =tlcblkxstart + (cblkno % prc->cw) * (1 << cblkwidthexpn);
							int cblkystart =tlcblkystart + (cblkno / prc->cw) * (1 << cblkheightexpn);
							int cblkxend = cblkxstart + (1 << cblkwidthexpn);
							int cblkyend = cblkystart + (1 << cblkheightexpn);
							cblk = &prc->cblks[cblkno];

							/* code-block size (global) */
							cblk->x0 = int_max(cblkxstart, prc->x0);
							cblk->y0 = int_max(cblkystart, prc->y0);
							cblk->x1 = int_min(cblkxend, prc->x1);
							cblk->y1 = int_min(cblkyend, prc->y1);

						}
					}
				}
			}
		}
	}
	/* tcd_dump(&tcd_image,0); */
}

void tcd_free_encode(j2k_image_t * img, j2k_cp_t * cp, int curtileno)
{
	int tileno, compno, resno, bandno, precno;
	tcd_img = img;
	tcd_cp = cp;
	tcd_image.tw = cp->tw;
	tcd_image.th = cp->th;
	for (tileno = 0; tileno < 1; tileno++) {
		/* j2k_tcp_t *tcp=&cp->tcps[curtileno]; */
		tile = tcd_image.tiles;
		for (compno = 0; compno < tile->numcomps; compno++) {
			tilec = &tile->comps[compno];
			for (resno = 0; resno < tilec->numresolutions; resno++) {
				res = &tilec->resolutions[resno];
				for (bandno = 0; bandno < res->numbands; bandno++) {
					band = &res->bands[bandno];
					for (precno = 0; precno < res->pw * res->ph; precno++) {
						prc = &band->precincts[precno];

						if (prc->incltree != NULL)
							tgt_destroy(prc->incltree);
						if (prc->imsbtree != NULL)
							tgt_destroy(prc->imsbtree);
						free(prc->cblks);
					}			/* for (precno */
					free(band->precincts);
				}			/* for (bandno */
			}				/* for (resno */
			free(tilec->resolutions);
		}				/* for (compno */
		free(tile->comps);
	}				/* for (tileno */
	free(tcd_image.tiles);
}

void tcd_init(j2k_image_t * img, j2k_cp_t * cp)
{
	int tileno, compno, resno, bandno, precno, cblkno, i, j;
	unsigned int x0 = 0, y0 = 0, x1 = 0, y1 = 0, w, h, p, q;
	tcd_img = img;
	tcd_cp = cp;
	tcd_image.tw = cp->tw;
	tcd_image.th = cp->th;
	tcd_image.tiles =
		(tcd_tile_t *) malloc(cp->tw * cp->th * sizeof(tcd_tile_t));

	/*for (tileno = 0; tileno < cp->tw * cp->th; tileno++) {
	j2k_tcp_t *tcp = &cp->tcps[tileno];
	tcd_tile_t *tile = &tcd_image.tiles[tileno]; */

	for (i = 0; i < cp->tileno_size; i++) {
		j2k_tcp_t *tcp = &cp->tcps[cp->tileno[i]];
		tcd_tile_t *tile = &tcd_image.tiles[cp->tileno[i]];
		tileno = cp->tileno[i];


		//              int previous_x0, previous_x1, previous_y0, previous_y1;
		/* cfr p59 ISO/IEC FDIS15444-1 : 2000 (18 august 2000) */
		p = tileno % cp->tw;	/* si numerotation matricielle .. */
		q = tileno / cp->tw;	/* .. coordonnees de la tile (q,p) q pour ligne et p pour colonne */

		/* 4 borders of the tile rescale on the image if necessary */
		tile->x0 = int_max(cp->tx0 + p * cp->tdx, img->x0);
		tile->y0 = int_max(cp->ty0 + q * cp->tdy, img->y0);
		tile->x1 = int_min(cp->tx0 + (p + 1) * cp->tdx, img->x1);
		tile->y1 = int_min(cp->ty0 + (q + 1) * cp->tdy, img->y1);

		tile->numcomps = img->numcomps;
		tile->comps =
			(tcd_tilecomp_t *) malloc(img->numcomps * sizeof(tcd_tilecomp_t));
		for (compno = 0; compno < tile->numcomps; compno++) {
			j2k_tccp_t *tccp = &tcp->tccps[compno];
			tcd_tilecomp_t *tilec = &tile->comps[compno];
			/* border of each tile component (global) */
			tilec->x0 = int_ceildiv(tile->x0, img->comps[compno].dx);
			tilec->y0 = int_ceildiv(tile->y0, img->comps[compno].dy);
			tilec->x1 = int_ceildiv(tile->x1, img->comps[compno].dx);
			tilec->y1 = int_ceildiv(tile->y1, img->comps[compno].dy);

			tilec->data =
				(int *) malloc((tilec->x1 - tilec->x0) *
				(tilec->y1 - tilec->y0) * sizeof(int));
			tilec->numresolutions = tccp->numresolutions;
			tilec->resolutions =
				(tcd_resolution_t *) malloc(tilec->numresolutions *
				sizeof(tcd_resolution_t));
			for (resno = 0; resno < tilec->numresolutions; resno++) {
				int pdx, pdy;
				int levelno = tilec->numresolutions - 1 - resno;
				int tlprcxstart, tlprcystart, brprcxend, brprcyend;
				int tlcbgxstart, tlcbgystart, brcbgxend, brcbgyend;
				int cbgwidthexpn, cbgheightexpn;
				int cblkwidthexpn, cblkheightexpn;
				tcd_resolution_t *res = &tilec->resolutions[resno];

				/* border for each resolution level (global) */
				res->x0 = int_ceildivpow2(tilec->x0, levelno);
				res->y0 = int_ceildivpow2(tilec->y0, levelno);
				res->x1 = int_ceildivpow2(tilec->x1, levelno);
				res->y1 = int_ceildivpow2(tilec->y1, levelno);

				res->numbands = resno == 0 ? 1 : 3;
				/* p. 35, table A-23, ISO/IEC FDIS154444-1 : 2000 (18 august 2000) */
				if (tccp->csty & J2K_CCP_CSTY_PRT) {
					pdx = tccp->prcw[resno];
					pdy = tccp->prch[resno];
				} else {
					pdx = 15;
					pdy = 15;
				}
				/* p. 64, B.6, ISO/IEC FDIS15444-1 : 2000 (18 august 2000)  */
				tlprcxstart = int_floordivpow2(res->x0, pdx) << pdx;
				tlprcystart = int_floordivpow2(res->y0, pdy) << pdy;
				brprcxend = int_ceildivpow2(res->x1, pdx) << pdx;
				brprcyend = int_ceildivpow2(res->y1, pdy) << pdy;
				res->pw = (res->x0==res->x1)?0:((brprcxend - tlprcxstart) >> pdx); // Mod Antonin : sizebug1
				res->ph = (res->y0==res->y1)?0:((brprcyend - tlprcystart) >> pdy); // Mod Antonin : sizebug1

				if (resno == 0) {
					tlcbgxstart = tlprcxstart;
					tlcbgystart = tlprcystart;
					brcbgxend = brprcxend;
					brcbgyend = brprcyend;
					cbgwidthexpn = pdx;
					cbgheightexpn = pdy;
				} else {
					tlcbgxstart = int_ceildivpow2(tlprcxstart, 1);
					tlcbgystart = int_ceildivpow2(tlprcystart, 1);
					brcbgxend = int_ceildivpow2(brprcxend, 1);
					brcbgyend = int_ceildivpow2(brprcyend, 1);
					cbgwidthexpn = pdx - 1;
					cbgheightexpn = pdy - 1;
				}

				cblkwidthexpn = int_min(tccp->cblkw, cbgwidthexpn);
				cblkheightexpn = int_min(tccp->cblkh, cbgheightexpn);

				for (bandno = 0; bandno < res->numbands; bandno++) {
					int x0b, y0b;
					int gain, numbps;
					j2k_stepsize_t *ss;
					tcd_band_t *band = &res->bands[bandno];
					band->bandno = resno == 0 ? 0 : bandno + 1;
					x0b = (band->bandno == 1)
						|| (band->bandno == 3) ? 1 : 0;
					y0b = (band->bandno == 2)
						|| (band->bandno == 3) ? 1 : 0;

					if (band->bandno == 0) {
						/* band border (global) */
						band->x0 = int_ceildivpow2(tilec->x0, levelno);
						band->y0 = int_ceildivpow2(tilec->y0, levelno);
						band->x1 = int_ceildivpow2(tilec->x1, levelno);
						band->y1 = int_ceildivpow2(tilec->y1, levelno);
					} else {
						/* band border (global) */
						band->x0 =
							int_ceildivpow2(tilec->x0 -
							(1 << levelno) * x0b, levelno + 1);
						band->y0 =
							int_ceildivpow2(tilec->y0 -
							(1 << levelno) * y0b, levelno + 1);
						band->x1 =
							int_ceildivpow2(tilec->x1 -
							(1 << levelno) * x0b, levelno + 1);
						band->y1 =
							int_ceildivpow2(tilec->y1 -
							(1 << levelno) * y0b, levelno + 1);
					}

					ss = &tccp->stepsizes[resno ==
						0 ? 0 : 3 * (resno - 1) + bandno + 1];
					gain =
						tccp->qmfbid ==
						0 ? dwt_getgain_real(band->bandno) : dwt_getgain(band->bandno);
					numbps = img->comps[compno].prec + gain;
					band->stepsize =
						(int) floor((1.0 + ss->mant / 2048.0) *
						pow(2.0, numbps - ss->expn) * 8192.0);
					band->numbps = ss->expn + tccp->numgbits - 1;	/* WHY -1 ? */

					band->precincts =
						(tcd_precinct_t *) malloc(res->pw * res->ph *
						sizeof(tcd_precinct_t));

					for (precno = 0; precno < res->pw * res->ph; precno++) {
						int tlcblkxstart, tlcblkystart, brcblkxend, brcblkyend;
						int cbgxstart =
							tlcbgxstart + (precno % res->pw) * (1 << cbgwidthexpn);
						int cbgystart =
							tlcbgystart + (precno / res->pw) * (1 << cbgheightexpn);
						int cbgxend = cbgxstart + (1 << cbgwidthexpn);
						int cbgyend = cbgystart + (1 << cbgheightexpn);
						tcd_precinct_t *prc = &band->precincts[precno];
						/* precinct size (global) */
						prc->x0 = int_max(cbgxstart, band->x0);
						prc->y0 = int_max(cbgystart, band->y0);
						prc->x1 = int_min(cbgxend, band->x1);
						prc->y1 = int_min(cbgyend, band->y1);

						tlcblkxstart =
							int_floordivpow2(prc->x0, cblkwidthexpn) << cblkwidthexpn;
						tlcblkystart =
							int_floordivpow2(prc->y0, cblkheightexpn) << cblkheightexpn;
						brcblkxend =
							int_ceildivpow2(prc->x1, cblkwidthexpn) << cblkwidthexpn;
						brcblkyend =
							int_ceildivpow2(prc->y1, cblkheightexpn) << cblkheightexpn;
						prc->cw = (brcblkxend - tlcblkxstart) >> cblkwidthexpn;
						prc->ch = (brcblkyend - tlcblkystart) >> cblkheightexpn;

						prc->cblks =
							(tcd_cblk_t *) malloc(prc->cw * prc->ch *
							sizeof(tcd_cblk_t));

						prc->incltree = tgt_create(prc->cw, prc->ch);
						prc->imsbtree = tgt_create(prc->cw, prc->ch);

						for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
							int cblkxstart =
								tlcblkxstart + (cblkno % prc->cw) * (1 << cblkwidthexpn);
							int cblkystart =
								tlcblkystart + (cblkno / prc->cw) * (1 << cblkheightexpn);
							int cblkxend = cblkxstart + (1 << cblkwidthexpn);
							int cblkyend = cblkystart + (1 << cblkheightexpn);
							tcd_cblk_t *cblk = &prc->cblks[cblkno];
							/* code-block size (global) */
							cblk->x0 = int_max(cblkxstart, prc->x0);
							cblk->y0 = int_max(cblkystart, prc->y0);
							cblk->x1 = int_min(cblkxend, prc->x1);
							cblk->y1 = int_min(cblkyend, prc->y1);



							cblk->lastbp = 0; // Add Antonin : quantizbug1
						}
					}
				}
			}
		}
	}
	//tcd_dump(&tcd_image,0);


	/* Allocate place to store the data decoded = final image */
	/* Place limited by the tile really present in the codestream */


	for (i = 0; i < img->numcomps; i++) {
		for (j = 0; j < cp->tileno_size; j++) {
			tileno = cp->tileno[j];
			x0 = j == 0 ? tcd_image.tiles[tileno].comps[i].x0 : int_min(x0,
				tcd_image.
				tiles[tileno].comps[i].x0);
			y0 = j == 0 ? tcd_image.tiles[tileno].comps[i].y0 : int_min(y0,
				tcd_image.
				tiles[tileno].comps[i].y0);
			x1 = j == 0 ? tcd_image.tiles[tileno].comps[i].x1 : int_max(x1,
				tcd_image.
				tiles[tileno].comps[i].x1);
			y1 = j == 0 ? tcd_image.tiles[tileno].comps[i].y1 : int_max(y1,
				tcd_image.
				tiles[tileno].comps[i].y1);
		}
		//w = int_ceildiv(x1 - x0, img->comps[i].dx);
		//h = int_ceildiv(y1 - y0, img->comps[i].dy);

		w = x1 - x0;

		h = y1 - y0;
		img->comps[i].data = (int *) calloc(w * h, sizeof(int));
		img->comps[i].w = w;
		img->comps[i].h = h;
		img->comps[i].x0 = x0;
		img->comps[i].y0 = y0;
	}
}

void tcd_makelayer_fixed(int layno, int final)
{
	int compno, resno, bandno, precno, cblkno;
	int value;			//, matrice[tcd_tcp->numlayers][tcd_tile->comps[0].numresolutions][3];
	int matrice[10][10][3];
	int i, j, k;

	/*matrice=(int*)malloc(tcd_tcp->numlayers*tcd_tile->comps[0].numresolutions*3*sizeof(int)); */

	for (compno = 0; compno < tcd_tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tcd_tile->comps[compno];
		for (i = 0; i < tcd_tcp->numlayers; i++) {
			for (j = 0; j < tilec->numresolutions; j++) {
				for (k = 0; k < 3; k++) {
					matrice[i][j][k] =
						(int) (tcd_cp->
						matrice[i * tilec->numresolutions * 3 +
						j * 3 +
						k] *
						(float) (tcd_img->comps[compno].prec / 16.0));
				}}}

		for (resno = 0; resno < tilec->numresolutions; resno++) {
			tcd_resolution_t *res = &tilec->resolutions[resno];
			for (bandno = 0; bandno < res->numbands; bandno++) {
				tcd_band_t *band = &res->bands[bandno];
				for (precno = 0; precno < res->pw * res->ph; precno++) {
					tcd_precinct_t *prc = &band->precincts[precno];
					for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
						tcd_cblk_t *cblk = &prc->cblks[cblkno];
						tcd_layer_t *layer = &cblk->layers[layno];
						int n;
						int imsb = tcd_img->comps[compno].prec - cblk->numbps;	/* number of bit-plan equal to zero */
						/* Correction of the matrix of coefficient to include the IMSB information */

						if (layno == 0) {
							value = matrice[layno][resno][bandno];
							if (imsb >= value)
								value = 0;
							else
								value -= imsb;
						} else {
							value =
								matrice[layno][resno][bandno] -
								matrice[layno - 1][resno][bandno];
							if (imsb >= matrice[layno - 1][resno][bandno]) {
								value -= (imsb - matrice[layno - 1][resno][bandno]);
								if (value < 0)
									value = 0;
							}
						}

						if (layno == 0)
							cblk->numpassesinlayers = 0;

						n = cblk->numpassesinlayers;
						if (cblk->numpassesinlayers == 0) {
							if (value != 0)
								n = 3 * value - 2 + cblk->numpassesinlayers;
							else
								n = cblk->numpassesinlayers;
						} else
							n = 3 * value + cblk->numpassesinlayers;

						layer->numpasses = n - cblk->numpassesinlayers;

						if (!layer->numpasses)
							continue;

						if (cblk->numpassesinlayers == 0) {
							layer->len = cblk->passes[n - 1].rate;
							layer->data = cblk->data;
						} else {
							layer->len =
								cblk->passes[n - 1].rate -
								cblk->passes[cblk->numpassesinlayers - 1].rate;
							layer->data =
								cblk->data +
								cblk->passes[cblk->numpassesinlayers - 1].rate;
						}
						if (final)
							cblk->numpassesinlayers = n;
					}
				}
			}
		}
	}
}

void tcd_rateallocate_fixed()
{
	int layno;

	for (layno = 0; layno < tcd_tcp->numlayers; layno++) {
		tcd_makelayer_fixed(layno, 1);
	}
}

void tcd_makelayer(int layno, double thresh, int final)
{
	int compno, resno, bandno, precno, cblkno, passno;

	tcd_tile->distolayer[layno] = 0;	//add fixed_quality

	for (compno = 0; compno < tcd_tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tcd_tile->comps[compno];
		for (resno = 0; resno < tilec->numresolutions; resno++) {
			tcd_resolution_t *res = &tilec->resolutions[resno];
			for (bandno = 0; bandno < res->numbands; bandno++) {
				tcd_band_t *band = &res->bands[bandno];
				for (precno = 0; precno < res->pw * res->ph; precno++) {
					tcd_precinct_t *prc = &band->precincts[precno];
					for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
						tcd_cblk_t *cblk = &prc->cblks[cblkno];
						tcd_layer_t *layer = &cblk->layers[layno];
						int n;

						if (layno == 0) {
							cblk->numpassesinlayers = 0;
						}
						n = cblk->numpassesinlayers;
						for (passno = cblk->numpassesinlayers;
							passno < cblk->totalpasses; passno++) {
								int dr;
								double dd;
								tcd_pass_t *pass = &cblk->passes[passno];
								if (n == 0) {
									dr = pass->rate;
									dd = pass->distortiondec;
								} else {
									dr = pass->rate - cblk->passes[n - 1].rate;
									dd = pass->distortiondec - cblk->passes[n -
										1].distortiondec;
								}
								if (dr == 0) {
									if (dd != 0)
										n = passno + 1;
									continue;
								}
								if (dd / dr > thresh)
									n = passno + 1;
						}
						layer->numpasses = n - cblk->numpassesinlayers;

						if (!layer->numpasses) {
							layer->disto = 0;
							continue;
						}

						if (cblk->numpassesinlayers == 0) {
							layer->len = cblk->passes[n - 1].rate;
							layer->data = cblk->data;
							layer->disto = cblk->passes[n - 1].distortiondec;
						} else {
							layer->len = cblk->passes[n - 1].rate -
								cblk->passes[cblk->numpassesinlayers - 1].rate;
							layer->data =
								cblk->data +
								cblk->passes[cblk->numpassesinlayers - 1].rate;
							layer->disto =
								cblk->passes[n - 1].distortiondec -
								cblk->passes[cblk->numpassesinlayers - 1].distortiondec;
						}

						tcd_tile->distolayer[layno] += layer->disto;	//add fixed_quality

						if (final)
							cblk->numpassesinlayers = n;
					}
				}
			}
		}
	}
}

void tcd_rateallocate(unsigned char *dest, int len, info_image * info_IM)
{
	int compno, resno, bandno, precno, cblkno, passno, layno;
	double min, max;
	double cumdisto[100];		//add fixed_quality
	const double K = 1;		// 1.1; //add fixed_quality

	double maxSE = 0;
	min = DBL_MAX;
	max = 0;

	tcd_tile->nbpix = 0;		//add fixed_quality

	for (compno = 0; compno < tcd_tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tcd_tile->comps[compno];

		tilec->nbpix = 0;
		for (resno = 0; resno < tilec->numresolutions; resno++) {
			tcd_resolution_t *res = &tilec->resolutions[resno];
			for (bandno = 0; bandno < res->numbands; bandno++) {
				tcd_band_t *band = &res->bands[bandno];
				for (precno = 0; precno < res->pw * res->ph; precno++) {
					tcd_precinct_t *prc = &band->precincts[precno];
					for (cblkno = 0; cblkno < prc->cw * prc->ch; cblkno++) {
						tcd_cblk_t *cblk = &prc->cblks[cblkno];
						for (passno = 0; passno < cblk->totalpasses; passno++) {
							tcd_pass_t *pass = &cblk->passes[passno];
							int dr;
							double dd, rdslope;
							if (passno == 0) {
								dr = pass->rate;
								dd = pass->distortiondec;
							} else {
								dr = pass->rate - cblk->passes[passno - 1].rate;
								dd = pass->distortiondec -
									cblk->passes[passno - 1].distortiondec;
							}
							if (dr == 0) {
								continue;
							}
							rdslope = dd / dr;
							if (rdslope < min) {
								min = rdslope;
							}
							if (rdslope > max) {
								max = rdslope;
							}
						}			/* passno */

						tcd_tile->nbpix += ((cblk->x1 - cblk->x0) * (cblk->y1 - cblk->y0));	//add fixed_quality

						tilec->nbpix += ((cblk->x1 - cblk->x0) * (cblk->y1 - cblk->y0));	//add fixed_quality

					}			/* cbklno */
				}			/* precno */
			}				/* bandno */
		}				/* resno */

		maxSE+=(double)(((1<<tcd_img->comps[compno].prec)-1)*((1<<tcd_img->comps[compno].prec)-1))*(tilec->nbpix);
	}				/* compno */

	/* add antonin index */
	if (info_IM->index_on) {
		info_tile *info_TL = &info_IM->tile[tcd_tileno];
		info_TL->nbpix = tcd_tile->nbpix;
		info_TL->distotile = tcd_tile->distotile;
		info_TL->thresh =
			(double *) malloc(tcd_tcp->numlayers * sizeof(double));
	}
	/* dda */

	for (layno = 0; layno < tcd_tcp->numlayers; layno++) {
		volatile double lo = min;
		volatile double hi = max;
		volatile int success = 0;
		volatile int maxlen = int_min(tcd_tcp->rates[layno], len);
		volatile double goodthresh;
		volatile int goodlen;
		volatile int i;
		double distotarget;		//add fixed_quality

		distotarget = tcd_tile->distotile - ((K * maxSE) / pow(10, tcd_tcp->distoratio[layno] / 10));	// add fixed_quality

		for (i = 0; i < 32; i++) {
			volatile double thresh = (lo + hi) / 2;
			int l=0;
			double distoachieved = 0;	// add fixed_quality

			tcd_makelayer(layno, thresh, 0);

			if (tcd_cp->fixed_quality) {	// add fixed_quality
				distoachieved =
					layno ==
					0 ? tcd_tile->distolayer[0] : cumdisto[layno - 1] +
					tcd_tile->distolayer[layno];
				if (distoachieved < distotarget) {
					hi = thresh;
					continue;
				}
				lo = thresh;
			} else {
				l =
					t2_encode_packets(tcd_img, tcd_cp, tcd_tileno, tcd_tile,
					layno + 1, dest, maxlen, info_IM);
				/* fprintf(stderr, "rate alloc: len=%d, max=%d\n", l, maxlen); */
				if (l == -999) {
					lo = thresh;
					continue;
				}
				hi = thresh;
			}

			success = 1;
			goodthresh = thresh;
			goodlen = l;

		}

		if (!success) {
			longjmp(j2k_error, 1);
		}

		if (info_IM->index_on) {	/* Threshold for Marcela Index */
			info_IM->tile[tcd_tileno].thresh[layno] = goodthresh;
		}
		tcd_makelayer(layno, goodthresh, 1);

		cumdisto[layno] =

			layno ==

			0 ? tcd_tile->distolayer[0] : cumdisto[layno - 1] +

			tcd_tile->distolayer[layno]; // add fixed_quality
	}
}

/* pxm核心编码算法 */
int tcd_encode_tile_pxm(int tileno, unsigned char *dest, int len,info_image * info_IM)
{
	int compno;
	int l,i;
	clock_t time7;
	tcd_tile_t *tile;/* 图像的tiles信息 */

	//tcd_cp在tcd_malloc_encode那里分配,为当前的cp分量
	j2k_tcp_t *tcp = &tcd_cp->tcps[0];//取得当前分量的第一个tile的引用
	j2k_tccp_t *tccp = &tcp->tccps[0];//取得当前tile在第一个分量上的引用 

	tcd_tileno = tileno;
	tcd_tile = tcd_image.tiles;//tile 信息,在tcd_malloc_encode那里分配
	tcd_tcp = &tcd_cp->tcps[tileno];//取得当前分量的第一个tile的引用
	tile = tcd_tile;

	/* INDEX >> "Precinct_nb_X et Precinct_nb_Y" */
	if (info_IM->index_on) {
		tcd_tilecomp_t *tilec_idx = &tile->comps[0];  //Based on Component 0

		for (i=0;i<tilec_idx->numresolutions;i++) {

			tcd_resolution_t *res_idx = &tilec_idx->resolutions[i];

			info_IM->tile[tileno].pw[i] = res_idx->pw;
			info_IM->tile[tileno].ph[i] = res_idx->ph;

			info_IM->tile[tileno].pdx[i] = tccp->prcw[i];
			info_IM->tile[tileno].pdy[i] = tccp->prch[i];

		}
	}
	/* << INDEX */
	/*---------------TILE-------------------*/
	time7 = clock();

	for (compno = 0; compno < tile->numcomps; compno++) {
		//遍历分量
		FILE *src;
		char tmp[256];
		int k;
		unsigned char elmt;
		int i, j;
		int tw, w;

		tcd_tilecomp_t *tilec = &tile->comps[compno];//获取 tile 分量
		int adjust = tcd_img->comps[compno].sgnd ? 0 : 1 << (tcd_img->comps[compno]. prec - 1);//???????
		int offset_x, offset_y;//图像域在对应分量上的原点坐标偏移
		//计算得图像域原点在对应分量上的原点坐标
		offset_x = int_ceildiv(tcd_img->x0, tcd_img->comps[compno].dx);//(图像域原点/当前分量的X采样率)取正
		offset_y = int_ceildiv(tcd_img->y0, tcd_img->comps[compno].dy);//计算分量采样,(公式 5.17)

		tw = tilec->x1 - tilec->x0;//tile分量的宽度(采样前)
		w = int_ceildiv(tcd_img->x1 - tcd_img->x0, tcd_img->comps[compno].dx);//(真实图像的宽度/横向采样点)=在对应分量上的对应宽度(采样后)

		sprintf(tmp, "Compo%d", compno);	/* component file,那三个临时文件 */
		src = fopen(tmp, "rb");//缓冲区
		if (!src) {
			fprintf(stderr, "failed to open %s for reading\n", tmp);
			return 1;
		}

		/* read the Compo file to extract data of the tile */
		k = 0;
		fseek(src, (tilec->x0 - offset_x) + (tilec->y0 - offset_y) * w,SEEK_SET);//在缓冲区内跳过
		k = (tilec->x0 - offset_x) + (tilec->y0 - offset_y) * w;

		//从上到下从左到右
		for (j = tilec->y0; j < tilec->y1; j++) {
			//逐行扫描分量信息
			for (i = tilec->x0; i < tilec->x1; i++) {
				//逐列扫描
				if (tcd_tcp->tccps[compno].qmfbid == 1) {
					elmt = fgetc(src);//从缓冲获取一个字节
					tilec->data[i - tilec->x0 + (j - tilec->y0) * tw] =elmt - adjust;//tile图像分量信息数据
					k++;
				} else if (tcd_tcp->tccps[compno].qmfbid == 0) {
					elmt = fgetc(src);
					tilec->data[i - tilec->x0 + (j - tilec->y0) * tw] =(elmt - adjust) << 13;
					k++;
				}
			}
			fseek(src, (tilec->x0 - offset_x) + (j + 1 - offset_y) * w - k,
				SEEK_CUR);
			k = tilec->x0 - offset_x + (j + 1 - offset_y) * w;
		}
		fclose(src);
	}

	/*----------------MCT-------------------*/

	if (tcd_tcp->mct) {
		if (tcd_tcp->tccps[0].qmfbid == 0) {
			mct_encode_real(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		} else {
			mct_encode(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		}
	}
	/*----------------DWT---------------------*/

	/* time3=clock(); */
	for (compno = 0; compno < tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tile->comps[compno];
		if (tcd_tcp->tccps[compno].qmfbid == 1) {
			dwt_encode(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec, tilec->numresolutions - 1);
		} else if (tcd_tcp->tccps[compno].qmfbid == 0) {
			dwt_encode_real(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec,
				tilec->numresolutions - 1);
		}
	}
	/*------------------TIER1-----------------*/

	t1_init_luts();
	t1_encode_cblks(tile, tcd_tcp);

	/*-----------RATE-ALLOCATE------------------*/
	info_IM->index_write = 0;	/* INDEX     */

	if (tcd_cp->disto_alloc || tcd_cp->fixed_quality)	// mod fixed_quality
		/* Normal Rate/distortion allocation */
		tcd_rateallocate(dest, len, info_IM);
	else
		/* Fixed layer allocation */
		tcd_rateallocate_fixed();

	/*--------------TIER2------------------*/
	info_IM->index_write = 1;	/* INDEX     */
	l = t2_encode_packets(tcd_img, tcd_cp, tileno, tile,
		tcd_tcp->numlayers, dest, len, info_IM);
	/*---------------CLEAN-------------------*/

	time7 = clock() - time7;
	printf("total:     %ld.%.3ld s\n", time7 / CLOCKS_PER_SEC,
		(time7 % CLOCKS_PER_SEC) * 1000 / CLOCKS_PER_SEC);

	/* cleaning memory */
	for (compno = 0; compno < tile->numcomps; compno++) {
		tilec = &tile->comps[compno];
		free(tilec->data);
	}

	return l;
}

int tcd_encode_tile_pgx(int tileno, unsigned char *dest, int len,
	info_image * info_IM)
{
	int compno;
	int l,i;
	clock_t time;
	tcd_tile_t *tile;
	j2k_tcp_t *tcp = &tcd_cp->tcps[0];
	j2k_tccp_t *tccp = &tcp->tccps[0];

	tcd_tileno = tileno;
	tcd_tile = tcd_image.tiles;
	tcd_tcp = &tcd_cp->tcps[tileno];
	tile = tcd_tile;
	/* INDEX >> "Precinct_nb_X et Precinct_nb_Y" */

	if (info_IM->index_on) {

		tcd_tilecomp_t *tilec_idx = &tile->comps[0];  //Based on Component 0

		for (i=0;i<tilec_idx->numresolutions;i++) {

			tcd_resolution_t *res_idx = &tilec_idx->resolutions[i];



			info_IM->tile[tileno].pw[i] = res_idx->pw;

			info_IM->tile[tileno].ph[i] = res_idx->ph;



			info_IM->tile[tileno].pdx[i] = tccp->prcw[i];

			info_IM->tile[tileno].pdy[i] = tccp->prch[i];

		}

	}

	/* << INDEX */
	/*---------------TILE-------------------*/
	time = clock();

	for (compno = 0; compno < tile->numcomps; compno++) {
		FILE *src;
		char tmp[256];
		int k;
		int elmt;
		int i, j;
		int tw, w;
		tcd_tilecomp_t *tilec = &tile->comps[compno];
		int adjust =
			tcd_img->comps[compno].sgnd ? 0 : 1 << (tcd_img->comps[compno].
			prec - 1);
		int offset_x, offset_y;

		offset_x = int_ceildiv(tcd_img->x0, tcd_img->comps[compno].dx);
		offset_y = int_ceildiv(tcd_img->y0, tcd_img->comps[compno].dy);
		tw = tilec->x1 - tilec->x0;
		w = int_ceildiv(tcd_img->x1 - tcd_img->x0, tcd_img->comps[compno].dx);
		sprintf(tmp, "bandtile%d", tileno / tcd_cp->tw + 1);	/* bandtile file opening */
		src = fopen(tmp, "rb");
		if (!src) {
			fprintf(stderr, "failed to open %s for reading\n", tmp);
			return 1;
		}
		/* Extract data from bandtile file limited to the current tile */
		k = 0;
		while (k < tilec->x0 - offset_x) {
			k++;
			fscanf(src, "%d", &elmt);
		}

		for (j = 0; j < tilec->y1 - tilec->y0; j++) {
			for (i = tilec->x0; i < tilec->x1; i++) {
				if (tcd_tcp->tccps[compno].qmfbid == 1) {
					fscanf(src, "%d", &elmt);
					tilec->data[i - tilec->x0 + (j) * tw] = elmt - adjust;
					k++;
				} else if (tcd_tcp->tccps[compno].qmfbid == 0) {
					fscanf(src, "%d", &elmt);
					tilec->data[i - tilec->x0 + (j) * tw] = (elmt - adjust) << 13;
					k++;
				}
			}
			while (k < tilec->x0 - offset_x + (j + 1) * w) {
				k++;
				fscanf(src, "%d", &elmt);
			}
		}
		fclose(src);
	}

	/*----------------MCT-------------------*/

	if (tcd_tcp->mct) {
		if (tcd_tcp->tccps[0].qmfbid == 0) {
			mct_encode_real(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		} else {
			mct_encode(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		}
	}

	/*----------------DWT---------------------*/

	for (compno = 0; compno < tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tile->comps[compno];
		if (tcd_tcp->tccps[compno].qmfbid == 1) {
			dwt_encode(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec, tilec->numresolutions - 1);
		} else if (tcd_tcp->tccps[compno].qmfbid == 0) {
			dwt_encode_real(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec,
				tilec->numresolutions - 1);
		}
	}

	/*------------------TIER1-----------------*/

	t1_init_luts();
	t1_encode_cblks(tile, tcd_tcp);

	/*-----------RATE-ALLOCATE------------------*/

	info_IM->index_write = 0;	/* INDEX */

	if (tcd_cp->disto_alloc || tcd_cp->fixed_quality)	// mod fixed_quality

		/* Normal Rate/distortion allocation */

		tcd_rateallocate(dest, len, info_IM);

	else

		/* Fixed layer allocation */

		tcd_rateallocate_fixed();

	/*--------------TIER2------------------*/
	info_IM->index_write = 1;	/* INDEX */

	l = t2_encode_packets(tcd_img, tcd_cp, tileno, tile,
		tcd_tcp->numlayers, dest, len, info_IM);

	/*---------------CLEAN-------------------*/
	time = clock() - time;
	printf("total:     %ld.%.3ld s\n", time / CLOCKS_PER_SEC,
		(time % CLOCKS_PER_SEC) * 1000 / CLOCKS_PER_SEC);

	for (compno = 0; compno < tile->numcomps; compno++) {
		tilec = &tile->comps[compno];
		free(tilec->data);
	}

	return l;
}


int tcd_decode_tile(unsigned char *src, int len, int tileno)
{
	int l;
	int compno;
	int eof = 0;
	clock_t time;
	tcd_tile_t *tile;

	tcd_tileno = tileno;
	tcd_tile = &tcd_image.tiles[tileno];
	tcd_tcp = &tcd_cp->tcps[tileno];
	tile = tcd_tile;

	time = clock();

	fprintf(stderr, "tile decoding time %d/%d: ", tileno + 1,
		tcd_cp->tw * tcd_cp->th);

	/*--------------TIER2------------------*/

	l = t2_decode_packets(src, len, tcd_img, tcd_cp, tileno, tile);

	if (l == -999) {
		eof = 1;
		fprintf(stderr, "tcd_decode: incomplete bistream\n");
	}

	/*------------------TIER1-----------------*/
	t1_init_luts();
	t1_decode_cblks(tile, tcd_tcp);

	/*----------------DWT---------------------*/

	for (compno = 0; compno < tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tile->comps[compno];
		if (tcd_cp->reduce_on == 1) {
			tcd_img->comps[compno].resno_decoded =
				tile->comps[compno].numresolutions - tcd_cp->reduce_value - 1;
		}


		if (tcd_tcp->tccps[compno].qmfbid == 1) {
			dwt_decode(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec,
				tilec->numresolutions - 1,
				tilec->numresolutions - 1 -
				tcd_img->comps[compno].resno_decoded);
		} else {
			dwt_decode_real(tilec->data, tilec->x1 - tilec->x0,
				tilec->y1 - tilec->y0, tilec,
				tilec->numresolutions - 1,
				tilec->numresolutions - 1 -
				tcd_img->comps[compno].resno_decoded);
		}

		if (tile->comps[compno].numresolutions > 0)
			tcd_img->comps[compno].factor =
			tile->comps[compno].numresolutions -
			(tcd_img->comps[compno].resno_decoded + 1);
	}

	/*----------------MCT-------------------*/

	if (tcd_tcp->mct) {
		if (tcd_tcp->tccps[0].qmfbid == 1) {
			mct_decode(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		} else {
			mct_decode_real(tile->comps[0].data, tile->comps[1].data,
				tile->comps[2].data,
				(tile->comps[0].x1 -
				tile->comps[0].x0) * (tile->comps[0].y1 -
				tile->comps[0].y0));
		}
	}

	/*---------------TILE-------------------*/

	for (compno = 0; compno < tile->numcomps; compno++) {
		tcd_tilecomp_t *tilec = &tile->comps[compno];
		tcd_resolution_t *res =
			&tilec->resolutions[tcd_img->comps[compno].resno_decoded];
		int adjust =
			tcd_img->comps[compno].sgnd ? 0 : 1 << (tcd_img->comps[compno].
			prec - 1);
		int min =
			tcd_img->comps[compno].
			sgnd ? -(1 << (tcd_img->comps[compno].prec - 1)) : 0;
		int max =
			tcd_img->comps[compno].
			sgnd ? (1 << (tcd_img->comps[compno].prec - 1)) -
			1 : (1 << tcd_img->comps[compno].prec) - 1;

		int tw = tilec->x1 - tilec->x0;
		int w = tcd_img->comps[compno].w;

		int i, j;
		int offset_x = int_ceildivpow2(tcd_img->comps[compno].x0,
			tcd_img->comps[compno].factor);
		int offset_y = int_ceildivpow2(tcd_img->comps[compno].y0,
			tcd_img->comps[compno].factor);

		for (j = res->y0; j < res->y1; j++) {
			for (i = res->x0; i < res->x1; i++) {

				int v;

				double tmp= (double) tilec->data[i - res->x0 + (j - res->y0) * tw];
				if (tcd_tcp->tccps[compno].qmfbid == 1) {
					v = (int) tmp;
				} else {

					//v = (int) tmp >> 13;

					//Mod antonin : multbug1
					v = (int) ((fabs(tmp/8192.0)>=floor(fabs(tmp/8192.0))+0.5)?fabs(tmp/8192.0)+1.0:fabs(tmp/8192.0));

					v = (tmp<0)?-v:v;

					//doM
				}
				v += adjust;

				tcd_img->comps[compno].data[(i - offset_x) +
					(j - offset_y) * w] =
					int_clamp(v, min, max);
			}
		}
	}

	time = clock() - time;
	fprintf(stderr, "total:     %ld.%.3ld s\n", time / CLOCKS_PER_SEC,
		(time % CLOCKS_PER_SEC) * 1000 / CLOCKS_PER_SEC);

	for (compno = 0; compno < tile->numcomps; compno++) {
		free(tcd_image.tiles[tileno].comps[compno].data);
	}

	if (eof) {
		longjmp(j2k_error, 1);
	}

	return l;
}
