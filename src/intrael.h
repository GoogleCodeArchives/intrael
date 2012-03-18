/* 
This file is part of the Intrael Project which provides computer vision for the web.
Copyright (C) 2012 Yannis Gravezas. You can contact the author at wizgrav@gmail.com.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <libfreenect.h>
#include <libfreenect-registration.h>
#define XMD_H
#include <jpeglib.h>
#undef XMD_H
#include <math.h>
#include "queue.h"
#include "md5.h"

#if defined USE_SSE
#include <xmmintrin.h>
#elif defined USE_SSE2
#include <emmintrin.h>
#elif defined USE_NEON
#include <arm_neon.h>
#endif

#define VMAJOR 1
#define VMINOR 9

int die = 0;

#define ERR(s, c) if(opterr){\
		    fputs(s, stderr);\
		    fputc(c,stderr);\
		    fputc('\n',stderr);\
		  }

#if defined WIN32
 
 #include <winsock2.h>
 #include <ws2tcpip.h>
 
 #define NULL	0
 #define EOF	(-1)
 #define IOVEC struct WSABUF
 #define NONBLOCKING(s) ioctlsocket(s, FIONBIO, &NonBlock)
 #define INITSOCKET(s) 	if ((s = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED)) == INVALID_SOCKET) exit(1)
 #define SIGEXIT() 	die=1;\
			WSACleanup();\
			return( TRUE )
 #define VSEND(client,hbuf,buf,hlen,len)\
		sdata[0].buf = (hbuf) + client->b;\
		sdata[0].len = (hlen) - client->b;\
		sdata[1].buf = buf;\
		sdata[1].len = (len);\
		if(WSASend(client->s,sdata,&itmp,NULL,NULL,NULL)) itmp=-1
 typedef int in_addr_t;
 
#else
 
 #include <sys/signal.h>
 #include <sys/time.h>
 #include <unistd.h>
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <netinet/tcp.h>
 #include <arpa/inet.h>
 #include <sys/uio.h>
 #include <fcntl.h>
 
 #define INVALID_SOCKET -1
 #define SOCKET_ERROR   -1
 #define IOVEC struct iovec
 #define closesocket(s) close(s)
 #define NONBLOCKING(s) fcntl(s, F_SETFL, fcntl(s,F_GETFL,0) | O_NONBLOCK)
 #define INITSOCKET(s) 	if((s = socket(AF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR) exit(1);\
			if(setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == SOCKET_ERROR) exit(1)
 #define VSEND(hbuf,buf,hlen,len)\
		sdata[0].iov_base = (hbuf)+(client->b);\
		sdata[0].iov_len = (hlen)-(client->b);\
		sdata[1].iov_base = (buf);\
		sdata[1].iov_len = (len);\
		itmp = writev(client->s, sdata, 2)

 typedef int SOCKET;
 
#endif

#define MAX_BYTES 65400
#define FRAME_PIXELS 307200
#define MAX(x1,x2) ((x1) > (x2) ? (x1):(x2))
#define MIN(x1,x2) ((x1) < (x2) ? (x1):(x2))
#define HASH(d)\
	MD5_Init(md5);\
	MD5_Update(md5,mbuf,sprintf((char *)mbuf,"%s%d",secret,d));\
	MD5_Final(mtemp, md5);\
	for(itmp = 0; itmp < 16; itmp++) sprintf(hash+2*itmp,"%02x",mtemp[itmp])
#define IOPT(arg,var,min,max)\
	itmp = atoi(arg);\
	itmp = MIN(itmp,max);\
	var = MAX(itmp,min);\
	break
#define COPT(ch,min,max,optarg)\
	itmp=atoi(optarg);\
	itmp = MIN(itmp,max);\
	itmp = MAX(itmp,min);\
	cbuf[(int)ch] = itmp;\
	break
#define ASSIGN()\
	r0  = *(raw);\
	r1  = *(raw+1);\
	r2  = *(raw+2);\
	r3  = *(raw+3);\
	r4  = *(raw+4);\
	r5  = *(raw+5);\
	r6  = *(raw+6);\
	r7  = *(raw+7);\
	r8  = *(raw+8);\
	r9  = *(raw+9);\
	r10 = *(raw+10)
#define PCK()\
	frame[0] =  (r0<<3)  | (r1>>5);\
	frame[1] = ((r1<<6)  | (r2>>2) )           & 2047;\
	frame[2] = ((r2<<9)  | (r3<<1) | (r4>>7) ) & 2047;\
	frame[3] = ((r4<<4)  | (r5>>4) )           & 2047;\
	frame[4] = ((r5<<7)  | (r6>>1) )           & 2047;\
	frame[5] = ((r6<<10) | (r7<<2) | (r8>>6) ) & 2047;\
	frame[6] = ((r8<<5)  | (r9>>3) )           & 2047;\
	frame[7] = ((r9<<8)  | (r10)   )           & 2047
#if defined USE_SSE
#define UNPACK()\
	__m64 md = _mm_setr_pi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ));\
	__m64 mmd = _mm_setr_pi16(((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si64(md, mmask);\
	mmd = _mm_and_si64(mmd, mmask);\
	__m64 mz = _mm_cmpgt_pi16(md, *((__m64 *) drz));\
	__m64 mmz = _mm_cmpgt_pi16(md, *((__m64 *) drz+4));\
	__m64 mZ = _mm_cmpgt_pi16( *((__m64 *)drZ),md);\
	__m64 mmZ = _mm_cmpgt_pi16( *((__m64 *)drZ+4),md);\
	mz = _mm_and_si64(mz, mZ);\
	mmz = _mm_and_si64(mmz, mmZ);\
	md = _mm_and_si64(mz, md);\
	mmd = _mm_and_si64(mmz, mmd);\
	*((__m64 *) frame)=md;\
	*((__m64 *) frame+4)=mmd;\
	mZ = _mm_or_si64(mz, mmz);\
	if(_mm_movemask_pi8(mZ)){
#define ZUNPACK()\
	__m64 md = _mm_setr_pi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ));\
	__m64 mmd = _mm_setr_pi16(((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si64(md, mmask);\
	mmd = _mm_and_si64(mmd, mmask);\
	__m64 mz = _mm_cmpgt_pi16(md, rmz);\
	__m64 mmz = _mm_cmpgt_pi16(mmd, rmz);\
	__m64 mZ = _mm_cmpgt_pi16(rmZ,md);\
	__m64 mmZ = _mm_cmpgt_pi16(rmZ,mmd);\
	mz = _mm_and_si64(mz, mZ);\
	mmz = _mm_and_si64(mmz, mmZ);\
	md = _mm_and_si64(mz, md);\
	mmd = _mm_and_si64(mmz, mmd);\
	*((__m64 *) frame)=md;\
	*((__m64 *) frame+4)=mmd;\
	mZ = _mm_or_si64(mz, mmz);\
	if(_mm_movemask_pi8(mZ)){
#define IF_THR(idx)\
	da=frame[idx];\
	if(da)
#define IF_EXT(idx)\
	da=frame[idx];\
	if(da)
#elif defined USE_SSE2
#define UNPACK()\
	__m128i mz = _mm_load_si128((__m128i *) drz);\
	__m128i mZ = _mm_load_si128((__m128i *) drZ);\
	__m128i md = _mm_setr_epi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ), ((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si128(md, mmask);\
	mz = _mm_cmpgt_epi16(md, mz);\
	mZ = _mm_cmpgt_epi16(mZ, md);\
	mz = _mm_and_si128(mz, mZ);\
	md = _mm_and_si128(mz, md);\
	_mm_store_si128((__m128i *) frame,md);\
	if(_mm_movemask_epi8(mz)){
#define ZUNPACK()\
	__m128i md = _mm_setr_epi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ), ((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si128(md, mmask);\
	__m128i mz = _mm_cmpgt_epi16(md, rmz);\
	__m128i mZ = _mm_cmpgt_epi16(rmZ, md);\
	mz = _mm_and_si128(mz, mZ);\
	md = _mm_and_si128(mz, md);\
	_mm_store_si128((__m128i *) frame,md);\
	if(_mm_movemask_epi8(mz)){
#define IF_THR(idx)\
	da=frame[idx];\
	if(da)
#define IF_EXT(idx) IF_THR(idx)
#elif defined USE_NEON
#define UNPACK()\
	uint32x4_t md32;\
	uint16x8_t mz = vld1q_u16(drz);\
	uint16x8_t mZ = vld1q_u16(drz);\
	md32 = vsetq_lane_u32(((r1<<6)  | (r2>>2) ) << 16 | ((r0<<3)  | (r1>>5)),md32,0);\
	md32 = vsetq_lane_u32(((r4<<4)  | (r5>>4) ) << 16 | ((r2<<9)  | (r3<<1) | (r4>>7) ),md32,1);\
	md32 = vsetq_lane_u32(((r6<<10) | (r7<<2) | (r8>>6) ) << 16 | ((r5<<7)  | (r6>>1) ),md32,2);\
	md32 = vsetq_lane_u32(((r9<<8)  | (r10)   )<<16 | ((r8<<5)  | (r9>>3) ),md32,3);\
	uint16x8_t md = vreinterpretq_u16_u32(md32);\
	md = vandq_u16(md, mmask);\
	mz = vcgtq_u16(md,mz);\
	mZ = vcgtq_u16(mZ,md);\
	mz = vandq_u16(mz,mZ);\
	uint8x8_t m8=vqmovn_u16(mz);\
	md = vandq_u16(mz,md);\
	vst1q_u16(frame,md);\
	uint32x2_t m32 = vreinterpret_u32_u8(m8);\
	uint32x2_t mmm = vpadd_u32(m32,m32);\
	if(vget_lane_u32(mmm,0)){
#define ZUNPACK()\
	uint32x4_t md32;\
	md32 = vsetq_lane_u32(((r1<<6)  | (r2>>2) ) << 16 | ((r0<<3)  | (r1>>5)),md32,0);\
	md32 = vsetq_lane_u32(((r4<<4)  | (r5>>4) ) << 16 | ((r2<<9)  | (r3<<1) | (r4>>7) ),md32,1);\
	md32 = vsetq_lane_u32(((r6<<10) | (r7<<2) | (r8>>6) ) << 16 | ((r5<<7)  | (r6>>1) ),md32,2);\
	md32 = vsetq_lane_u32(((r9<<8)  | (r10)   )<<16 | ((r8<<5)  | (r9>>3) ),md32,3);\
	uint16x8_t md = vreinterpretq_u16_u32(md32);\
	md = vandq_u16(md, mmask);\
	uint16x8_t mz = vcgtq_u16(md,rmz);\
	uint16x8_t mZ = vcgtq_u16(rmZ,md);\
	mz = vandq_u16(mz,mZ);\
	uint8x8_t m8=vqmovn_u16(mz);\
	md = vandq_u16(mz,md);\
	vst1q_u16(frame,md);\
	uint32x2_t m32 = vreinterpret_u32_u8(m8);\
	uint32x2_t mmm = vpadd_u32(m32,m32);\
	if(vget_lane_u32(mmm,0)){
#define IF_THR(idx)\
	da=frame[idx];\
	if(da)
#define IF_EXT(idx)
#define IF_EXT(idx) IF_THR(idx)
#else
#define UNPACK() PCK()
#define ZUNPACK() PCK()
#define IF_THR(idx)\
	da=frame[idx];\
	if(da > rawz && da < rawZ)
#define IF_EXT(idx)\
	da=frame[idx];\
	if(da > drz[idx] && da < drZ[idx])
#endif
#define EPROC(id)\
	X=i+id;\
	run_y[n] = y;\
	run_z[n] = dz;\
	run_Z[n] = dZ;\
	run_zv[n] = dzv;\
	run_Zv[n] = dZv;\
	run_sum[n]=running;\
	run_e[n] = X;\
	run_ev[n] = depth[X-1];\
	if(!rt)	rt=n;\
	if(rs){\
		 for(running = rs;running != re;running++){\
				if(run_e[running] < x) rs=running;\
				else if(run_s[running] < X){\
						da=run_label[running];\
						if(!label){\
								label = da;\
								sr=running;\
						}else if(da < label) label=da;\
						er=running;\
				}else break;\
		}\
		if(label){\
			label=r_label[label];\
			run_label[n] = label;\
			do{\
				r_label[run_label[sr]]  = label;\
			}while(sr++ != er);\
			label=0;\
		}else{\
			run_label[n] = l;\
			r_label[l] = l;\
			l++;\
		}\
	}else{\
		run_label[n] = l;\
		r_label[l] = l;\
		l++;\
	}\
	running=0;\
	n++
#define PROC(id)\
	if(running){\
		running += da;\
		if(da < dzv){\
			 dz = i+id;\
			 dzv = da;\
		 }else if(da > dZv){\
		  dZ= i+id;\
		  dZv=da;\
		}\
	}else{\
		x=i+id;\
		dZv = da;\
		dzv = da;\
		running = da;\
		dz=x;\
		dZ=x;\
		run_s[n] = x;\
		run_sv[n] = da;\
	}\
	}else if(running){\
		EPROC(id)
#if defined USE_SSE2 || USE_NEON || USE_SSE
#define IF_NEXT()\
	}else if(running){ EPROC(0); }\
	i+=8;\
	if(i != t_X)
#else
#define IF_NEXT()\
	i+=8;\
	if(i != t_X)
#endif
#define JUMP()\
	if(running){ EPROC(0); }\
	rs = rt;\
	if(rt){\
		if(dmap){\
			for(re=rt,i=0;re!=n;re++){\
				for(ni=run_s[re];i!=ni;i++) dref[i]=0;\
				for(ni=run_e[re];i!=ni;i++) dref[i]=depth_to_mm[depth[i]];\
			}\
			for(ni=t_X;i!=640;i++) dref[i]=0;\
			dref += 640;\
		}\
		if(encode){\
			for(i=t_x;rt!=n;rt++){\
				memset(gray+i,0,run_s[rt]-i);\
				for(i=run_s[rt],ni=run_e[rt];i!=ni;i++) gray[i]=depth_to_gray[depth[i]];\
			}\
			memset(gray+i,0,t_X-i);\
			jpeg_write_scanlines(&ginfo,&jgray, TRUE);\
		}\
		rt=0;\
	}else{\
		if(dmap){\
			memset(dref,0,1280);\
			dref += 640;\
		}\
		if(encode) jpeg_write_scanlines(&ginfo,&jblack, TRUE);\
	}\
	re = n;\
	y++;\
	if(y == t_Y) break;\
	i=t_x;\
	raw = draw + (((int)((640*y+i)/8)) * 11);\
	frame=depth+i	
#define FOP(file)\
	if(!file) break;\
	if(!(fp=fopen(file,"rb"))){\
	   ERR("[ERR8] Could not open file for reading",' ')\
	   file=NULL;\
	   break;\
	}\
	fseek(fp,0,SEEK_END);\
	itmp=ftell(fp);\
	fseek(fp,0,SEEK_SET);\
	if(!itmp){\
		fclose(fp);\
		break;\
	}\
	fbuf=(char *)malloc(itmp+1);\
	itmp=fread(fbuf,1,itmp,fp);\
	fclose(fp);\
	fbuf[itmp]='\0'
#define CSWITCH(ch,optarg)\
	switch (ch) {\
		case 'p': IOPT(optarg,lport,1,65534);\
		case 'l': listeneraddr.sin_addr.s_addr = inet_addr(optarg);\
				break;\
		case 'k': if(strlen(optarg) > 1){\
					serial=strdup(optarg);\
					break;\
				  }\
				  IOPT(optarg,camera,0,9);\
		case 'h':\
		case 'v': printf("\nIntrael v%d.%d (C) 2011, 2012 Yannis Gravezas @ http://www.intrael.com\nThis software is being licenced to you under the terms of the AGPLv3\nKinect driver is provided by libfreenect @ http://www.openkinect.org\n\n",VMAJOR,VMINOR);\
			  return 0;\
		case 'n': video = 0;\
				  break;\
		case 'j': IOPT(optarg,quality,25,75);\
		case 's': if(strlen(optarg)==1 && optarg[0]=='0'){sec=0; break;}\
				  secret = strdup(optarg);\
				  mbuf = (unsigned char *) malloc(strlen(secret)+16);\
				  break;\
		case 'u': IOPT(optarg,umax,0,1000);\
	    case 'f': IOPT(optarg,frmax,3,1000);\
	    case 'd': sfile = strdup(optarg);\
				  break;\
		case 'b': lfile = strdup(optarg);\
				  break;\
		case 'o': ofile = strdup(optarg);\
				  break;\
		case 'O': admin_org = strdup(optarg);\
				  admin_org_len = strlen(admin_org);\
				  break;\
		case 'i': ifile = strdup(optarg);\
				  break;\
		case 'I': if(inet_addr(targ)!= INADDR_NONE) admin_addr=inet_addr(targ);\
				  break;\
		case 'm': IOPT(optarg,cmax,1,1000);\
	}
#define RSWITCH(ch,optarg)\
	switch (ch) {\
		case 'g': COPT(ch,0,1,optarg);\
	    case 'x': COPT(ch,0,631,optarg);\
	    case 'X': COPT(ch,0,632,optarg);\
	    case 'y': COPT(ch,0,479,optarg);\
	    case 'Y': COPT(ch,0,480,optarg);\
	    case 'z': COPT(ch,1,9999,optarg);\
	    case 'Z': COPT(ch,1,9999,optarg);\
	    case 'c': COPT(ch,1,FRAME_PIXELS,optarg);\
	    case 'C': COPT(ch,0,FRAME_PIXELS,optarg);\
	    case 'e': COPT(ch,-999,999,optarg);\
	    case 'a': COPT(ch,-31,31,optarg);\
	    case 'r': COPT(ch,1,999,optarg);\
		case 'b': if(lfile){ cbuf['b'] = 1;}\
				  break;\
		case 'd': if(sfile){ cbuf['d'] = 1;}\
				  break;\
		case 'o': if(ofile){ cbuf['o'] = 1;}\
				  break;\
		case 'i': if(ifile){ cbuf['i'] = 1;}\
				  break;\
	}
#define NSWITCH(ch)\
	switch(ch){\
		case 'o': FOP(ofile);\
				  LIST_FOREACH(origin, &origins, entries){\
						LIST_REMOVE(origin,entries);\
						free(origin->n);\
						free(origin);\
					}\
					targ=strtok(fbuf,",; \r\n\t[]\"'");\
					while(targ){\
						origin = (origin_t *)calloc(1, sizeof(*origin));\
						itmp=strlen(targ);\
						origin->n = strdup(targ);\
						origin->l=itmp;\
						LIST_INSERT_HEAD(&origins, origin, entries);\
						targ=strtok(NULL,",; \r\n\t[]\"'");\
					}\
					free(fbuf);\
					break;\
		case 'i':   FOP(ifile);\
					LIST_FOREACH(host, &hosts, entries){\
						LIST_REMOVE(host,entries);\
						free(host);\
					}\
					targ=strtok(fbuf,",; \r\n\t[]\"'");\
					while(targ){\
						if(inet_addr(targ)!= INADDR_NONE){\
							host = ((host_t *)calloc(1, sizeof(*host)));\
							host->a = inet_addr(targ);\
							LIST_INSERT_HEAD(&hosts, host, entries);\
						}\
						targ=strtok(NULL,",; \r\n\t[]\"'");\
					}\
					free(fbuf);\
					break;\
	}
#define CFRAME(frame,head,size)\
	frame = (frame_t *)malloc(sizeof(frame_t));\
	if(size) frame->buf =(char *)malloc(size);\
	frame->h = head;\
	head->c++;\
	frame->c=0;\
	frame->t=0
#define DFRAME(frame)\
	free(frame->buf);\
	frame->h->c--;\
	frame->h=NULL;\
	free(frame);\
	frame=NULL
#define FSWAP(ok,frame,nframe,miss)\
	pthread_mutex_lock(&net_mutex);\
	if(ok){\
		ok=0;\
		nframe = frame;\
		frame = NULL;\
	}else{\
		if(++miss < 2){\
			if(nframe){\
				tframe=nframe;\
				nframe=frame;\
				frame=tframe;\
			}else{\
				nframe = frame;\
				frame=NULL;\
			}\
		}else{\
			if(nframe){\
				free(nframe->buf);\
				nframe->h->c--;\
				nframe->h=NULL;\
				free(nframe);\
				nframe=NULL;\
			}\
		}\
	}\
	pthread_mutex_unlock(&net_mutex)
#define SEL(count,ok,miss,entry,frame)\
	if(!LIST_EMPTY(&count)){\
		if(frame){\
			LIST_FOREACH(client, &count, entry){\
				 if(client->t == frame->t) continue;\
				 if(client->m && ++(client->c) < client->m)	continue;\
				 LIST_REMOVE(client,entry);\
				 LIST_INSERT_HEAD(&clients, client, entries);\
				 FD_SET(client->s,&streaming);\
				 FD_SET(client->s,&master);\
				 client->f=frame;\
				 frame->c++;\
				 client->c=0;\
				 client->t=frame->t;\
				 ok=1;\
				 fdmax=MAX(client->s,fdmax);\
			}\
		}\
		miss=0;\
	}
#define STREAM(cls,entr,mod,off,nframe)\
	if(client->m){\
		if(client->b < client->f->h->ml){\
			VSEND(client->f->h->mbuf,client->f->buf + mod,client->f->h->ml,client->f->l - mod);\
		}else{\
			itmp = send(client->s,(const char *)(client->f->buf + (client->b - client->f->h->ml + mod) ),client->f->l + client->f->h->ml - client->b - mod,0);\
		}\
	}else{\
		if(client->b < client->f->h->sl){\
			VSEND(client->f->h->sbuf,client->f->buf,client->f->h->sl,client->f->l);\
		}else{\
			itmp = send(client->s,(const char *)(client->f->buf + (client->b - client->f->h->sl) ),client->f->l + client->f->h->sl - client->b,0);\
		}\
	}\
	if(itmp<1){\
		kick=1;\
	}else{\
		client->b += itmp;\
		if(client->b == (client->m ? client->f->l + client->f->h->ml - mod : client->f->l + client->f->h->sl)){\
			FD_CLR(client->s,&streaming);\
			if(client->m){\
				FD_CLR(client->s,&master);\
				LIST_REMOVE(client,entries);\
				LIST_INSERT_HEAD(&cls, client, entr);\
				client->b = (off);\
			}else{\
				 client->b=0;\
				 fdmax=MAX(fdmax,client->s);\
			}\
			client->c=0;\
			pthread_mutex_lock(&net_mutex);\
			if(!(--client->f->c)){\
				 if(nframe == client->f) nframe=NULL;\
				 pthread_mutex_unlock(&net_mutex);\
				 DFRAME(client->f); }else{\
					 pthread_mutex_unlock(&net_mutex);\
				 	 client->f = NULL;\
				 }\
		}else{\
			fdmax=MAX(fdmax,client->s);\
		}\
	}\
	break
#define RSEL(frame)\
	FD_SET(client->s,&streaming);\
	client->c=0;\
	client->b=0;\
	frame->c++;\
	client->f=frame;\
	fdmax = MAX(client->s,fdmax);\
	c2=0;\
	break							  
#define HSET(head,strs,strm,type)\
	head = (header_t *)malloc(sizeof(header_t));\
	head->sbuf = (char *)malloc(256);\
	head->sl = sprintf(head->sbuf,strs,VMAJOR,VMINOR);\
	head->mbuf = (char *)malloc(256);\
	head->ml = sprintf(head->mbuf,strm,VMAJOR,VMINOR);\
	head->t = type;\
	head->c =0
#define JSET(inf,space,comp)\
	memset(&inf, 0, sizeof(inf));\
	inf.err = jpeg_std_error(&jerr);\
	jpeg_create_compress(&inf);\
	jpeg_memory_dest(&inf);\
	inf.image_width=640;\
	inf.image_height=480;\
	inf.dct_method=JDCT_FASTEST;\
	inf.input_components = comp;\
	inf.in_color_space   = space;\
    jpeg_set_defaults(&inf);\
	jpeg_set_quality(&inf,quality,TRUE)
#define MSTATE()\
	freenect_update_tilt_state(f_dev);\
	state = freenect_get_tilt_state(f_dev);\
	freenect_get_mks_accel(state,&ax,&ay,&az)	
#define FAIL(error) {\
	ERR(error,' ');\
	return 1;\
    }

#define STR_JSON_M "HTTP/1.1 200 OK\r\nConnection: Close\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: text/event-stream\r\n\r\ndata:"
#define STR_JSON_S "HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\n"
#define STR_JSON "Content-Type: application/json\r\nContent-Length: %7d\r\n\r\n"
#define STR_JPEG_M "HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nConnection: Close\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: multipart/x-mixed-replace; boundary=INTRAEL\r\n\r\n--INTRAEL\r\n"
#define STR_JPEG_S "HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nConnection: close\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\n"
#define STR_JPEG "Content-Type: image/jpeg\r\nContent-Length: %7d\r\n\r\n"
#define STR_DUMP "HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: application/octet-stream\r\nContent-Length: 614400\r\n\r\n"
#define STR_REG "HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: application/octet-stream\r\nContent-Length: 2497600\r\n\r\n"

typedef struct header_t{
    uint16_t sl,ml,t,c;
    char *sbuf,*mbuf;
} header_t;

typedef struct frame_t{
    uint32_t l,c,t;
    header_t *h;
    char *buf;
} frame_t;


typedef struct origin_t{
    char *n;
	uint16_t l;
	LIST_ENTRY(origin_t) entries;
} origin_t;

typedef struct host_t{
    in_addr_t a;
    uint16_t c,s;
    LIST_ENTRY(host_t) all;
	LIST_ENTRY(host_t) entries;
} host_t;

typedef struct client_t{
	in_addr_t a;
	host_t *h;
    uint32_t b,t;
    uint8_t m,c;
	SOCKET s;
	frame_t *f;
	LIST_ENTRY(client_t) all;
	LIST_ENTRY(client_t) entries;
	LIST_ENTRY(client_t) dentries;
	LIST_ENTRY(client_t) ventries;
	LIST_ENTRY(client_t) gentries;
	LIST_ENTRY(client_t) rentries;
} client_t;

typedef struct {
struct jpeg_destination_mgr pub;
JOCTET* buffer;
int bufsize;
size_t datasize; 
uint16_t* outsize;
uint16_t* offset; 
} memory_destination_mgr;
typedef memory_destination_mgr* mem_dest_ptr;

#ifdef _MSC_VER
__declspec(align(16)) uint16_t depth[640];
__declspec(align(16)) uint16_t depth_ref_Z[FRAME_PIXELS];
__declspec(align(16)) uint16_t depth_ref_z[FRAME_PIXELS];
__declspec(align(16)) uint8_t depth_raw[3][640*480*11/8];
__declspec(align(16)) uint8_t video_raw[3][640*480];
__declspec(align(16)) uint8_t rgb[640*3];
__declspec(align(16)) uint8_t black[640];
__declspec(align(16)) uint8_t gray[640];
#else
uint16_t depth[640] __attribute__ ((aligned (16)));
uint16_t depth_ref_z[FRAME_PIXELS] __attribute__ ((aligned (16)));
uint16_t depth_ref_Z[FRAME_PIXELS] __attribute__ ((aligned (16)));
uint8_t depth_raw[3][640*480*11/8] __attribute__ ((aligned (16)));
uint8_t video_raw[3][640*480] __attribute__ ((aligned (16)));
uint8_t rgb[640*3] __attribute__ ((aligned (16)));
uint8_t black[640] __attribute__ ((aligned (16)));
uint8_t gray[640] __attribute__ ((aligned (16)));
#endif

pthread_mutex_t depth_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  depth_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t video_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  video_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t net_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t conf_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_attr_t attr;
pthread_t dthread;
pthread_t vthread;
	 
char *lfile,*sfile,*ofile,*ifile;
int32_t regbuf[FRAME_PIXELS*2+10000];
frame_t *ndframe,*ngframe,*nvframe,*nrframe;
uint8_t dwait,vwait,dok,dmiss,gok,gmiss,vok,vmiss,rok,rmiss,depth_to_gray[2048],quality;
uint16_t depth_ref[FRAME_PIXELS],depth_to_raw[10000],depth_to_mm[2048],run_y[FRAME_PIXELS/2+1],run_zv[FRAME_PIXELS/2+1],run_Zv[FRAME_PIXELS/2+1],run_sv[FRAME_PIXELS/2+1],run_ev[FRAME_PIXELS/2+1],run_s[FRAME_PIXELS/2+1],run_e[FRAME_PIXELS/2+1],run_z[FRAME_PIXELS/2+1],run_Z[FRAME_PIXELS/2+1];
uint32_t r_label[FRAME_PIXELS/2+1],run_sum[FRAME_PIXELS/2+1],run_label[FRAME_PIXELS/2+1],l_pos_x[FRAME_PIXELS/2+1],l_pos_X[FRAME_PIXELS/2+1],l_pos_y[FRAME_PIXELS/2+1],l_pos_Y[FRAME_PIXELS/2+1],l_pos_z[FRAME_PIXELS/2+1],l_pos_Z[FRAME_PIXELS/2+1],l_cx[FRAME_PIXELS/2+1],l_cy[FRAME_PIXELS/2+1],l_sum[FRAME_PIXELS/2+1],l_count[FRAME_PIXELS/2+1],l_runs[FRAME_PIXELS/2+1],l_vrun[FRAME_PIXELS/2+1],l_checked[FRAME_PIXELS/2+1],dstamp,vstamp,dcb,vcb,led;
int32_t gbuf[128],umax,frmax;
freenect_context *f_ctx;
freenect_device *f_dev;

	
