/* 
This file is part of the Intrael Project which provides computer vision for the web.
Copyright (C) 2011 Yannis Gravezas. You can contact the author at wizgrav@gmail.com.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the flogLicense, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License v3 for more details.

You should have received a copy of the GNU General Public License v3
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libfreenect.h>
#include <libfreenect-registration.h>
#define XMD_H
#include <jpeglib.h>
#undef XMD_H
#include <math.h>
#include "queue.h"
#include "md5.h"

#if defined USE_SSE2
#include <emmintrin.h>
#elif defined USE_NEON || USE_MORE_NEON
#include <arm_neon.h>
#endif

#define VMAJOR 1
#define VMINOR 0

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
 #define NONBLOCKING(s) ioctlsocket(s, FIONBIO, &NonBlock)
 #define INITSOCKET(s) 	if ((s = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED)) == INVALID_SOCKET) exit(1)
 #define SIGEXIT() 	die=1;\
			WSACleanup();\
			return( TRUE )
 
 typedef int in_addr_t;
 int	optind = 1;
 int 	opterr = 1;
 int	optopt;
 char	*optarg;
 ULONG NonBlock = 1;
 HANDLE winMutex; 

 int getopt(int argc, char **argv, char *opts){
	 static int sp = 1;
	 register int c;
	 register char *cp;
	 if(sp == 1)
		 if(optind >= argc ||  argv[optind][0] != '-' || argv[optind][1] == '\0') return(EOF);
		 else if(strcmp(argv[optind], "--") == NULL) {
			 optind++;
			 return(EOF);
		 }
	 optopt = c = argv[optind][sp];
	 if(c == ':' || (cp=strchr(opts, c)) == NULL) {
		 ERR("illegal option -- ", c);
		 if(argv[optind][++sp] == '\0') {
			 optind++;
			 sp = 1;
		 }
		 return('?');
	 }
	 if(*++cp == ':') {
		 if(argv[optind][sp+1] != '\0')  optarg = &argv[optind++][sp+1];
		 else if(++optind >= argc) {
			 ERR("option requires an argument -- ", c);
			 sp = 1;
			 return('?');
		 } else	 optarg = argv[optind++];
		 sp = 1;
	 } else {
		 if(argv[optind][++sp] == '\0') {
			 sp = 1;
			 optind++;
		 }
		 optarg = NULL;
	 }
	 return(c);
 }
 
 typedef int socklen_t;
 BOOL CtrlHandler( DWORD fdwCtrlType ){
     switch( fdwCtrlType ){
     case CTRL_C_EVENT: SIGEXIT();
     case CTRL_CLOSE_EVENT: SIGEXIT();
     case CTRL_BREAK_EVENT: return FALSE;
     case CTRL_LOGOFF_EVENT: return FALSE;
     case CTRL_SHUTDOWN_EVENT: SIGEXIT();
     default: return FALSE;
     }
 }
 
 static void sig_register(void){
     if( !SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE ) )
	 ERR("unable to register control handler",' ');
 }

#else
 
 #include <sys/signal.h>
 #include <sys/time.h>
 #include <unistd.h>
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <netinet/tcp.h>
 #include <arpa/inet.h>
 #include <fcntl.h>
 
 #define INVALID_SOCKET -1
 #define SOCKET_ERROR   -1
 #define closesocket(s) close(s)
 #define NONBLOCKING(s) fcntl(s, F_SETFL, fcntl(s,F_GETFL,0) | O_NONBLOCK)
 #define INITSOCKET(s) 	if((s = socket(AF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR) exit(1);\
			if(setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == SOCKET_ERROR) exit(1)
 
 typedef int SOCKET;
 
 static void sigexit(int signo){
	  die=1;
 }
 
 static void sig_register(void){
     struct sigaction sigact;
     sigact.sa_handler = sigexit;
     sigemptyset(&sigact.sa_mask);
     sigact.sa_flags = 0;
     sigaction(SIGINT, &sigact, NULL);
     sigaction(SIGTERM, &sigact, NULL);
 }
#endif

#define FRAME_PIXELS 307200
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define MAX(x1,x2) ((x1) > (x2) ? (x1):(x2))
#define MIN(x1,x2) ((x1) < (x2) ? (x1):(x2))
#define HASH(d)\
	MD5_Init(md5);\
	MD5_Update(md5,mbuf,sprintf((char *)mbuf,"%s%u",secret,d));\
	MD5_Final(mtemp, md5);\
	for(y = 0; y < 16; y++) sprintf(hash+2*y,"%02x",mtemp[y])
#define IOPT(arg,var,min,max)\
	itmp = atoi(arg);\
	itmp = MIN(itmp,max);\
	var = MAX(itmp,min);\
	break
#define OPTFIX()\
	t_X=((int) (t_X/8))*8;\
	t_x=((int) (t_x/8))*8;\
	if(t_x >= t_X){\
		t_x=0;\
		t_X=632;\
	}\
	if(t_y >= t_Y){\
		t_y=0;\
		t_Y=479;\
	}\
	if(t_z >= t_Z){\
		t_z=0;\
		t_Z=1000;\
	}\
	if(!mode){\
		if(refcount > 0) memset(depth,0,FRAME_PIXELS*2);\
		refcount=0;\
	}else if(refcount > 1){\
		memcpy(depth_ref,depth,FRAME_PIXELS*2);\
		if(f_dev) freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);\
	}\
	if(cangle != angle){\
		cangle=angle;\
		freenect_set_tilt_degs(f_dev,cangle);\
	}
#define SEL(count,fdm,entry,type,off)\
	LIST_FOREACH(client, &count, entry){\
		 LIST_REMOVE(client,entry);\
		 LIST_INSERT_HEAD(&clients, client, entries);\
		 FD_SET(client->s,&streaming);\
		 FD_SET(client->s,&master);\
		 client->t=type;\
		 client->k=0;\
		 client->f=0;\
		 fdmax=MAX(client->s,fdmax);\
	}
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
	frame[1] = ((r1<<6)  | (r2>>2) )           & baseMask;\
	frame[2] = ((r2<<9)  | (r3<<1) | (r4>>7) ) & baseMask;\
	frame[3] = ((r4<<4)  | (r5>>4) )           & baseMask;\
	frame[4] = ((r5<<7)  | (r6>>1) )           & baseMask;\
	frame[5] = ((r6<<10) | (r7<<2) | (r8>>6) ) & baseMask;\
	frame[6] = ((r8<<5)  | (r9>>3) )           & baseMask;\
	frame[7] = ((r9<<8)  | (r10)   )           & baseMask
#if defined USE_SSE2
#define UNPACK()\
	__m128i mz = _mm_load_si128((__m128i *) &depth_ref_z[i]);\
	__m128i mZ = _mm_load_si128((__m128i *) &depth_ref_Z[i]);\
	__m128i md = _mm_setr_epi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ), ((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si128(md, mmask);\
	mz = _mm_cmpgt_epi16(md, mz);\
	mZ = _mm_cmpgt_epi16(mZ, md);\
	mz = _mm_and_si128(mz, mZ);\
	md = _mm_and_si128(mz, md);\
	_mm_store_si128((__m128i *) frame,md)
#define ZUNPACK()\
	__m128i md = _mm_setr_epi16((r0<<3)  | (r1>>5), ((r1<<6)  | (r2>>2) ), ((r2<<9)  | (r3<<1) | (r4>>7) ), ((r4<<4)  | (r5>>4) ), ((r5<<7)  | (r6>>1) ),((r6<<10) | (r7<<2) | (r8>>6) ), ((r8<<5)  | (r9>>3) ), ((r9<<8)  | (r10)   ));\
	md = _mm_and_si128(md, mmask);\
	__m128i mz = _mm_cmpgt_epi16(md, rmz);\
	__m128i mZ = _mm_cmpgt_epi16(rmZ, md);\
	mz = _mm_and_si128(mz, mZ);\
	md = _mm_and_si128(mz, md);\
	_mm_store_si128((__m128i *) frame,md)
#define CPROC(d,idx)\
	d=frame[idx];\
	if(d){
#define ZPROC(d,idx)\
	d=frame[idx];\
	if(d){
#define FPROC(id) }
#elif defined USE_NEON || USE_MORE_NEON
#define UNPACK()\
	uint32x4_t md32;\
	uint16x8_t mz = vld1q_u16(&depth_ref_z[i]);\
	uint16x8_t mZ = vld1q_u16(&depth_ref_z[i]);\
	vsetq_lane_u32(((r1<<6)  | (r2>>2) ) << 16 | ((r0<<3)  | (r1>>5)),md32,0);\
	vsetq_lane_u32(((r4<<4)  | (r5>>4) ) << 16 | ((r2<<9)  | (r3<<1) | (r4>>7) ),md32,1);\
	vsetq_lane_u32(((r6<<10) | (r7<<2) | (r8>>6) ) << 16 | ((r5<<7)  | (r6>>1) ),md32,2);\
	vsetq_lane_u32(((r9<<8)  | (r10)   )<<16 | ((r8<<5)  | (r9>>3) ),md32,3);\
	uint16x8_t md = vreinterpretq_u16_u32(md32);\
	md = vandq_u16(md, mmask);\
	mz = vcgtq_u16(md,mz);\
	mZ = vcgtq_u16(mZ,md);\
	mz = vandq_u16(mz,mZ);\
	md = vandq_u16(mz,md);\
	vst1q_u16(frame,md)
#define ZUNPACK()\
	uint32x4_t md32;\
	vsetq_lane_u32(((r1<<6)  | (r2>>2) ) << 16 | ((r0<<3)  | (r1>>5)),md32,0);\
	vsetq_lane_u32(((r4<<4)  | (r5>>4) ) << 16 | ((r2<<9)  | (r3<<1) | (r4>>7) ),md32,1);\
	vsetq_lane_u32(((r6<<10) | (r7<<2) | (r8>>6) ) << 16 | ((r5<<7)  | (r6>>1) ),md32,2);\
	vsetq_lane_u32(((r9<<8)  | (r10)   )<<16 | ((r8<<5)  | (r9>>3) ),md32,3);\
	uint16x8_t md = vreinterpretq_u16_u32(md32);\
	md = vandq_u16(md, mmask);\
	uint16x8_t mz = vcgtq_u16(md,rmz);\
	uint16x8_t mZ = vcgtq_u16(rmZ,md);\
	mz = vandq_u16(mz,mZ);\
	md = vandq_u16(mz,md);\
	vst1q_u16(frame,md)
#define CPROC(d,idx)\
	d=frame[idx];\
	if(d){
#define ZPROC(d,idx)\
	d=frame[idx];\
	if(d){
#define FPROC(id) }
#else
#define UNPACK() PCK()
#define ZUNPACK() UNPACK()
#define CPROC(d,idx)\
	d=frame[idx];\
	if(d > depth_ref_z[i] && d < depth_ref_Z[i]){
#define ZPROC(d,idx)\
	d=frame[idx];\
	if(d > rawz && d < rawZ){
#define FPROC(id) frame[id]=0; }else frame[id]=0;
#endif
#define EPROC(dp,id)\
	x=i+id;\
	run_z[n] = dz;\
	run_Z[n] = dZ;\
	run_sum[n]=running;\
	run_s[n] = y;\
	run_e[n] = x;\
	if(!rt) rt=n;\
	if(rs){\
		 y -= 642;\
		 x -= 638;\
		 for(dp = rs; dp!=re;dp++){\
			if(run_e[dp] < y) rs=dp;\
			else if(run_s[dp] < x){\
				dz=run_label[dp];\
				if(!label){\
					label = dz;\
					dzv=dp;\
				}else if(dz < label) label=dz;\
				dZv=dp;\
			}else break;\
		 }\
		if(label){\
			label=r_label[label];\
			do{\
				r_label[run_label[dzv]]  = label;\
			} while(dzv++ != dZv);\
			run_label[n] = label;\
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
	n++;
#define PROC(dp,id)\
	if(running){\
		running += dp;\
		if(dp < dzv){\
			 dz = i+id;\
			 dzv = dp;\
		 }else if(dp > dZv){\
		  dZ= i+id;\
		  dZv=dp;\
		}\
	}else{\
		dZv = dp;\
		dzv = dp;\
		running = dp;\
		y=i+id;\
		dz=y;\
		dZ=y;\
	}\
	}else if(running){\
		EPROC(dp,id)\
	FPROC(id) 
#define REV()\
	i+=8;\
	if(i != ni){\
		raw += 11;\
		frame += 8;\
	}else{\
		if(running){\
				EPROC(da,0)\
		}\
		re = n;\
		rs = rt;\
		rt = 0;\
		ni += 640;\
		i +=(640-t_X+t_x);\
		if(i==written) break;\
		frame = depth+i;\
		raw = ((uint8_t *)v_depth)+(((int)i/8) * 11);\
	}
#define FOP(file)\
	if(!file) break;\
	if(!(fp=fopen(file,"rb"))){\
	   ERR("[ERR8] Could not open file for reading",' ')\
	   break;\
	}\
	if((itmp=fread(fbuf,1,16383,fp)) < 1){\
	fclose(fp);\
	break;\
	}\
	fclose(fp);\
	fbuf[itmp]='\0'
#define CSWITCH(ch,optarg)\
	switch (ch) {\
		case 'p': IOPT(optarg,lport,1,65535);\
		case 'l': listeneraddr.sin_addr.s_addr = inet_addr(optarg);\
				break;\
		case 'd': if(strlen(optarg) > 1){\
					serial=strdup(optarg);\
					break;\
				  }\
				  IOPT(optarg,camera,0,9);\
		case 'h':\
		case 'v': printf("\nIntrael v%d.%d (C) 2011 Yannis Gravezas. Info @ http://www.intrael.com\nThis software is being licenced to you under the terms of the GPL v3\nKinect driver is provided by libfreenect @ http://www.openkinect.org\nIn memory of my cousin Kostas. We shall meet again on a better world\n\n",VMAJOR,VMINOR);\
			  return 0;\
		case 'm': IOPT(optarg,multipart,0,7);\
		case 'n': novideo=1;\
				  break;\
		case 'b': max_bytes=16384;\
				  break;\
		case 'j': IOPT(optarg,t_j,25,75);\
		case 's': sec = (strlen(optarg)==1 && optarg[0]=='0') ? 0 : 1;\
				  secret = strdup(optarg);\
				  mbuf = (unsigned char *) malloc(strlen(secret)+16);\
				  break;\
		case 'f': lfile = strdup(optarg);\
				  break;\
		case 'F': sfile = strdup(optarg);\
				  break;\
		case 'o': ofile = strdup(optarg);\
				  break;\
		case 'i': ifile = strdup(optarg);\
				  break;\
	}
#define RSWITCH(ch,optarg)\
	switch (ch) {\
	    case 'x': IOPT(optarg,t_x,0,632);\
	    case 'X': IOPT(optarg,t_X,0,632);\
	    case 'y': IOPT(optarg,t_y,0,479);\
	    case 'Y': IOPT(optarg,t_Y,0,479);\
	    case 'z': if(refcount < 1) refcount=1;\
				  IOPT(optarg,t_z,1,9999);\
	    case 'Z': if(refcount < 1) refcount=1;\
				  IOPT(optarg,t_Z,1,9999);\
	    case 'c': IOPT(optarg,t_c,1,FRAME_PIXELS);\
	    case 'C': IOPT(optarg,t_C,0,FRAME_PIXELS);\
	    case 'e': if(refcount < 1) refcount=1;\
				  IOPT(optarg,mode,-1,1054);\
	    case 'r': IOPT(optarg,refcount,1,999);\
	    case 'f': if(!lfile) break;\
				  if((fp = fopen(lfile,"rb"))){\
					fseek(fp,17,SEEK_CUR);\
					if((i=fread(depth_ref, 2,FRAME_PIXELS, fp)) != FRAME_PIXELS ){\
						 memset(depth_ref,0,FRAME_PIXELS*2);\
						 ERR("[ERR5] Invalid background frame",' ');\
					}\
					fclose(fp);\
				  }else{\
					 ERR("[ERR6] Could not open file for reading the background frame",' ');\
					 memset(depth_ref,0,FRAME_PIXELS*2);\
				  }\
				  refcount = 1;\
				  break;\
		case 'F': if(!sfile || refcount > 0) break;\
				  if((fp=fopen(sfile,"wb"))){\
					fprintf(fp, "P5 640 480 65535\n");\
					fwrite(depth_ref, FRAME_PIXELS*2, 1, fp);\
					fclose(fp);\
				  }else ERR("[ERR7] Could not write the background frame to the specified file",' ');\
				  break;\
		case 'a': IOPT(optarg,angle,-31,31);\
	    case 'o': 	FOP(ofile);\
					LIST_FOREACH(origin, &origins, entries){\
						LIST_REMOVE(origin,entries);\
						free(origin->n);\
						free(origin);\
					}\
					targ=strtok(fbuf,", \r\n\t");\
					while(targ){\
						origin = (ori *)calloc(1, sizeof(*origin));\
						itmp=strlen(targ);\
						origin->n = strdup(targ);\
						origin->l=itmp;\
						LIST_INSERT_HEAD(&origins, origin, entries);\
						targ=strtok(NULL,",");\
					}\
					break;\
		case 'i': 	FOP(ifile);\
					LIST_FOREACH(host, &hosts, entries){\
						LIST_REMOVE(host,entries);\
						free(host);\
					}\
					targ=strtok(fbuf,", \r\n\t");\
					while(targ){\
						if((tempaddr = inet_addr(targ))!= INADDR_NONE){\
							host = (ho *)calloc(1, sizeof(*host));\
							host->a = tempaddr;\
							LIST_INSERT_HEAD(&hosts, host, entries);\
						}\
						targ=strtok(NULL,",");\
					}\
					break;\
	}
#define STREAM(buf,cnt,off,fdm,cls,entr,md,extra)\
	i = send(client->s,(const char *)buf+client->b ,cnt-client->b ,0);\
	if(i<1) y=1; else client->b += i;\
	if(!y){\
		if(client->b == cnt){\
			FD_CLR(client->s,&streaming);\
			if((md)){\
				FD_CLR(client->s,&master);\
				LIST_REMOVE(client,entries);\
				LIST_INSERT_HEAD(&cls, client, entr);\
				client->b = off-extra;\
			}else{\
				 client->t=0;\
				 client->b=0;\
				 fdmax=MAX(fdmax,client->s);\
			}\
		}else{\
				client->f=1;\
				fdmax=MAX(fdmax,client->s);\
				fdm=MAX(fdm,client->s);\
		}\
	}\
	break
#define JSET(inf,buf,space,wr,off,comp)\
	memset(&inf, 0, sizeof(inf));\
	inf.err = jpeg_std_error(&jerr);\
	jpeg_create_compress(&inf);\
	jpeg_memory_dest(&inf,(JOCTET*)buf,320*240,&wr,&off);\
	inf.image_width=640;\
	inf.image_height=480;\
	inf.dct_method=JDCT_FASTEST;\
	inf.input_components = comp;\
	inf.in_color_space   = space;\
    jpeg_set_defaults(&inf);\
	jpeg_set_quality(&inf,t_j,TRUE)
#define FAIL(error) {\
	ERR(error,' ');\
	return 1;\
    }

typedef struct cli{
    int b,t,k,f;
	SOCKET s;
	LIST_ENTRY(cli) entries;
	LIST_ENTRY(cli) dentries;
	LIST_ENTRY(cli) ventries;
	LIST_ENTRY(cli) gentries;
} cli;
LIST_HEAD(, cli) clients;
LIST_HEAD(, cli) dclients;
LIST_HEAD(, cli) vclients;
LIST_HEAD(, cli) gclients;

typedef struct ori{
    char *n;
	int l;
	LIST_ENTRY(ori) entries;
} ori;
LIST_HEAD(, ori) origins;

typedef struct ho{
    in_addr_t a;
	LIST_ENTRY(ho) entries;
} ho;
LIST_HEAD(, ho) hosts;

typedef struct {
struct jpeg_destination_mgr pub;
JOCTET* buffer;
int bufsize;
size_t datasize; 
int* outsize;
uint16_t* offset; 
} memory_destination_mgr;
typedef memory_destination_mgr* mem_dest_ptr;

#ifdef _MSC_VER
__declspec(align(16)) uint16_t depth_temp[8];
__declspec(align(16)) uint16_t depth[FRAME_PIXELS];
__declspec(align(16)) uint16_t depth_ref_Z[FRAME_PIXELS];
__declspec(align(16)) uint16_t depth_ref_z[FRAME_PIXELS];
__declspec(align(16)) uint8_t depth_raw[640*480*11/8];
__declspec(align(16)) uint8_t video_raw[640*480];
__declspec(align(16)) uint8_t rgb[640*4];
#else
uint16_t depth_temp[8] __attribute__ ((aligned (16)));
uint16_t depth[FRAME_PIXELS] __attribute__ ((aligned (16)));
uint16_t depth_ref_z[FRAME_PIXELS] __attribute__ ((aligned (16)));
uint16_t depth_ref_Z[FRAME_PIXELS] __attribute__ ((aligned (16)));
uint8_t depth_raw[640*480*11/8] __attribute__ ((aligned (16)));
uint8_t video_raw[640*480] __attribute__ ((aligned (16)));
uint8_t rgb[640*4] __attribute__ ((aligned (16)));
#endif

char message[16384], *lfile,*sfile,*ofile,*ifile,*secret, *serial, hash[33],buf[4096],cbuf[4096],fbuf[16384];
unsigned char *mbuf, mtemp[16],pbuf[320*240],gbuf[320*240];
MD5_CTX *md5;
struct cli *client;
struct ori *origin;
struct ho *host;
struct timeval timeout;
struct jpeg_compress_struct cinfo,ginfo;
struct jpeg_error_mgr jerr;
fd_set master,streaming;
SOCKET fdmax,dfdmax,vfdmax,gfdmax,listener,newfd;
struct sockaddr_in listeneraddr;
in_addr_t tempaddr;
struct sockaddr_storage clientaddr;
socklen_t addrsize;
uint16_t depth_ref[FRAME_PIXELS],depth_ref_x[FRAME_PIXELS],depth_ref_y[FRAME_PIXELS],depth_to_raw[9999],depth_to_mm[2048],joffset,doffset,goffset, novideo,rawz,rawZ;
uint32_t run_s[FRAME_PIXELS/2+1],run_e[FRAME_PIXELS/2+1],run_z[FRAME_PIXELS/2+1],run_Z[FRAME_PIXELS/2+1],run_sum[FRAME_PIXELS/2+1],run_label[FRAME_PIXELS/2+1],r_label[FRAME_PIXELS/2+1],l_pos_x[FRAME_PIXELS/2+1],l_pos_X[FRAME_PIXELS/2+1],l_pos_y[FRAME_PIXELS/2+1],l_pos_Y[FRAME_PIXELS/2+1],l_pos_z[FRAME_PIXELS/2+1],l_pos_Z[FRAME_PIXELS/2+1],l_cx[FRAME_PIXELS/2+1],l_cy[FRAME_PIXELS/2+1],l_sum[FRAME_PIXELS/2+1],l_count[FRAME_PIXELS/2+1],l_runs[FRAME_PIXELS/2+1],l_checked[FRAME_PIXELS/2+1],t_x,t_X,t_y,t_Y,t_z,t_Z,t_c,t_C,lport,gstamp,astamp,gdump,t_j,multipart,max_bytes, i,ni,label,running,da,n,l,rs,re,rt,dz,dZ,dzv,dZv;
int x,y,itmp,camera, refcount,angle,yes,mode,written,jwritten,gwritten,sec,l_vrun[FRAME_PIXELS/2+1];
double ax,az,ay;
freenect_raw_tilt_state* state;
freenect_context *f_ctx;
freenect_device *f_dev;
FILE *fp = NULL;
freenect_registration reg;

METHODDEF(void) init_destination (j_compress_ptr cinfo){
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->pub.next_output_byte = dest->buffer + *dest->offset; 
	dest->pub.free_in_buffer = dest->bufsize - *dest->offset; 
	dest->datasize = *dest->offset; 
}

METHODDEF(boolean) empty_output_buffer (j_compress_ptr cinfo){
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->pub.next_output_byte = dest->buffer + *dest->offset;
	dest->pub.free_in_buffer = dest->bufsize - *dest->offset;
	return TRUE;
}

METHODDEF(void) term_destination (j_compress_ptr cinfo){
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->datasize = dest->bufsize - dest->pub.free_in_buffer;
	*dest->outsize = (int)dest->datasize;
}

GLOBAL(void) jpeg_memory_dest (j_compress_ptr cinfo, JOCTET* buffer, int bufsize, int* outsize,uint16_t* offset){
	mem_dest_ptr dest;
	if (cinfo->dest == 0) cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,	sizeof (memory_destination_mgr));
	dest = (mem_dest_ptr) cinfo->dest;
	dest->offset = offset;
	dest->bufsize = bufsize;
	dest->buffer = buffer;
	dest->outsize = outsize;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_output_buffer;
	dest->pub.term_destination = term_destination;
}

void depth_cb(freenect_device* dev, void *v_depth, uint32_t timestamp){	
	uint8_t r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,*raw;
	uint16_t baseMask = (1 << 11) - 1;
	uint16_t *frame;
	#if defined WIN32
    WaitForSingleObject(winMutex,INFINITE);
	#endif
	if(!gdump && sec > 0){
			 gdump=timestamp;
			 HASH(gdump);
	}
	if(refcount > 0 && mode){
		refcount--;
		if(refcount){
			raw=(uint8_t *) v_depth;
			frame=depth_temp;
			uint16_t *dref=depth_ref;
			for(ni=0;ni!=FRAME_PIXELS;ni+=8,dref+=8,raw+=11){
				ASSIGN();
				PCK();
				for(i=0;i!=8;i++){
						da=frame[i];
						dz=dref[i];
						if(da>1054) da=0;
						if(!dz){
								dref[i]=da;
						}else if(da){
								dref[i]=MIN(da,dz);
						}		
				}
			}
		}else{
			dz=depth_to_raw[t_z];
			dZ=depth_to_raw[t_Z];
			rs=0;
			for(i=1;i!=FRAME_PIXELS;i++){
				if(depth_ref_x[i]>631){
					rs=0;
					continue;
				}
				da=depth_ref[i];
				if(rs){
					if(da){
						dz=(dz+da)/2;
						for(ni=rs;ni!=i;ni++)
							depth_ref[ni]=dz;
						rs=0;	
					}
				}else if(!da){
					rs=i-1;
					if(!(dz=depth_ref[rs])) rs=0; 
				}
			}
			if(mode<0){
				rt=abs(mode);
				for(i=FRAME_PIXELS-1;i--;){
					itmp=depth_to_mm[depth_ref[i]]-t_Z;
					if(itmp > 0){
						dz = depth_to_raw[itmp];
						if(depth_ref[i] > rt){
							depth_ref_z[i] = (dz > depth_ref[i]-rt) ? depth_ref[i]-rt : dz;
						}else{
							depth_ref_z[i] = 2048;
						}
					}else{
						depth_ref_z[i] = 2048;
					}
					itmp=depth_to_mm[depth_ref[i]]-t_z;
					if(itmp > 0){
						dz = depth_to_raw[itmp];
						if(depth_ref[i] > rt){
							depth_ref_Z[i] = (dz > depth_ref[i]-rt) ? depth_ref[i]-rt : dz;
						}else{
							depth_ref_Z[i] = 0;
						}
					}else{
						depth_ref_Z[i] = 0;
					}
				}
			}else{
				rawz=depth_to_raw[t_z];
				rawZ=depth_to_raw[t_Z];
				for(i=FRAME_PIXELS-1;i--;){
					itmp=depth_ref[i]-mode;
					if(itmp > 0){
						depth_ref_z[i] = rawz < itmp ? rawz : itmp;
						depth_ref_Z[i] = rawZ < itmp ? rawZ : itmp;
					}else{
						depth_ref_z[i] = 2048;
						depth_ref_Z[i] = 0;
					}
				}
			}
		}
		#if defined WIN32
    	ReleaseMutex(winMutex);
		#endif
		return;
	}
	//Slightly modified run based component labelling by Suzuki and friends
	if(!(LIST_EMPTY(&dclients)) && !dfdmax){
		#if defined USE_SSE2
		__m128i mmask=_mm_set1_epi16(baseMask);
		#elif defined USE_NEON || USE_MORE_NEON
		uint16x8_t mmask = vdupq_n_u16(baseMask);
		#endif
		n = l = 1 ;
		label = running = rs = rt = re = dz = dZ = dzv = dZv =  0;
		i = t_x+640*t_y;
		frame = depth+i;
		raw = ((uint8_t *)v_depth)+(((int)i/8)*11);
		ni = 640*t_y+t_X;
		written=640*t_Y+t_x;
		if(mode){
			while(1){
				ASSIGN();
				UNPACK();
				#if defined USE_SSE2
				if(_mm_movemask_epi8(mz)){
				#elif defined USE_MORE_NEON
				uint8x8_t m8=vqmovn_u16(mz);
				uint32x2_t m32 = vreinterpret_u32_u8(m8);
				uint32x2_t mmm = vpadd_u32(m32,m32);
				if(vget_lane_u32(mmm,0)){
				#endif
				CPROC(da,0) PROC(da,0)
				CPROC(da,1) PROC(da,1)
				CPROC(da,2) PROC(da,2)
				CPROC(da,3) PROC(da,3)
				CPROC(da,4) PROC(da,4)
				CPROC(da,5) PROC(da,5)
				CPROC(da,6) PROC(da,6)
				CPROC(da,7) PROC(da,7)
				#if defined USE_SSE2 || USE_MORE_NEON
				}else if(running){
						EPROC(da,0)
				}
				#endif
				REV()
			}
		}else{
			rawz=depth_to_raw[t_z];
			rawZ=depth_to_raw[t_Z];
			#if defined USE_SSE2
			__m128i rmz=_mm_set1_epi16(rawz);
			__m128i rmZ=_mm_set1_epi16(rawZ);
			#elif defined USE_NEON || USE_MORE_NEON
			uint16x8_t rmz=vdupq_n_u16(rawz);
			uint16x8_t rmZ=vdupq_n_u16(rawZ);
			#endif
			while(1){
				ASSIGN();
				ZUNPACK();
				#if defined USE_SSE2
				if(_mm_movemask_epi8(mz)){
				#elif defined USE_MORE_NEON
				uint8x8_t m8=vqmovn_u16(mz);
				uint32x2_t m32 = vreinterpret_u32_u8(m8);
				uint32x2_t mmm = vpadd_u32(m32,m32);
				if(vget_lane_u32(mmm,0)){
				#endif
				ZPROC(da,0) PROC(da,0)
				ZPROC(da,1) PROC(da,1)
				ZPROC(da,2) PROC(da,2)
				ZPROC(da,3) PROC(da,3)
				ZPROC(da,4) PROC(da,4)
				ZPROC(da,5) PROC(da,5)
				ZPROC(da,6) PROC(da,6)
				ZPROC(da,7) PROC(da,7)
				#if defined USE_SSE2  || USE_MORE_NEON
				}else if(running){
						EPROC(da,0)
				}
				#endif
				REV()
			}
		}
		da=0;
		ni=n-1;
		if(ni)
			while(ni--){
				label = r_label[r_label[run_label[ni]]];
				rs=run_s[ni];
				re=run_e[ni];
				y=depth_ref_y[rs];
				x=depth_ref_x[rs];
				l=re-rs;
				if(l_count[label]){
					dz=run_z[ni];
					dZ=run_Z[ni];
					if(x < depth_ref_x[l_pos_x[label]]) l_pos_x[label] = rs; else if(depth_ref_x[re] > depth_ref_x[l_pos_X[label]]) l_pos_X[label] = re;
					if(depth[dz]<depth[l_pos_z[label]]) l_pos_z[label] = dz; else if(depth[dZ]>depth[l_pos_Z[label]]) l_pos_Z[label] = dZ;
					l_count[label] += l;
					l_pos_y[label] = ni;
					l_cx[label]+=((l*((x<<1)+l)));
					l_vrun[label] += y;
					l_cy[label]+= l*y;
					l_sum[label] += run_sum[ni];
					l_runs[label]++;
				}else{
					l_pos_z[label] = run_z[ni];
					l_pos_Z[label] = run_Z[ni];
					l_pos_x[label] = rs;
					l_pos_X[label] = re;
					l_pos_y[label] = ni;
					l_pos_Y[label] = ni;
					l_count[label] = l;
					l_cx[label]=((l*((x<<1)+l)));
					l_vrun[label] = y;
					l_cy[label]=l*y;
					l_sum[label] = run_sum[ni];
					l_runs[label] = 1;
					l_checked[da++]=label;
				}
			}
		written=doffset;
		written+=sprintf(message+written,"[%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%d,%d",timestamp,((sec > 0)? gdump : sec),mode,t_x,t_X,t_y,t_Y,t_z,t_Z,t_c,t_C,ax, ay, az,(int)freenect_get_tilt_degs(state),freenect_get_tilt_status(state));
		ni= max_bytes-160;
		if(da){ 
			for(da--;da--;){
				i=l_checked[da];
				l=l_count[i];
				l_count[i]=0;
				if(written>ni) break;
				if((l < t_c)  || (t_C && l > t_C)  ) continue;
				uint32_t posz=l_pos_z[i];
				uint32_t posZ=l_pos_Z[i];
				rt=l_pos_y[i];
				uint32_t posy=(run_s[rt]+run_e[rt])>>1;
				rt=l_pos_Y[i];
				uint32_t posY=(run_s[rt]+run_e[rt])>>1;
				uint32_t posx=l_pos_x[i];
				uint32_t posX=l_pos_X[i];
				uint32_t vruns=l_vrun[i];
				uint32_t numruns=l_runs[i];
				rs=(uint32_t) l_cx[i]/(2*l)+640*(l_cy[i]/l);
				rt=depth_to_mm[l_sum[i]/l];
				int rshift=0;
				y =  reg.registration_table[rs][1];
				x = (reg.registration_table[rs][0] + reg.depth_to_rgb_shift[rt]) >> 8;
				if(x < 640){
						x-=depth_ref_x[rs];
						y-=depth_ref_y[rs];
						rshift=640*y+x;
				}
				posX--;
				written+=sprintf(message+written,",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d",depth_ref_x[rs],depth_ref_y[rs],rt,depth_to_mm[depth_ref[rs]],depth_ref_x[posx],depth_ref_y[posx],depth_to_mm[depth[posx]],depth_to_mm[depth_ref[posx]],depth_ref_x[posX],depth_ref_y[posX],depth_to_mm[depth[posX]],depth_to_mm[depth_ref[posX]],depth_ref_x[posy],depth_ref_y[posy],depth_to_mm[depth[posy]],depth_to_mm[depth_ref[posy]],depth_ref_x[posY],depth_ref_y[posY],depth_to_mm[depth[posY]],depth_to_mm[depth_ref[posY]],depth_ref_x[posz],depth_ref_y[posz],depth_to_mm[depth[posz]],depth_to_mm[depth_ref[posz]],depth_ref_x[posZ],depth_ref_y[posZ],depth_to_mm[depth[posZ]],depth_to_mm[depth_ref[posZ]],l,numruns,(uint32_t)vruns/numruns,rshift);
			}
		}
		gstamp=timestamp;
		message[written++]=']';
		memcpy(message+doffset-10,"      ",6);
		memcpy(message+doffset-10,buf,sprintf(buf,"%d",written-doffset));
		SEL(dclients,dfdmax,dentries,3,doffset)
	}else{
		#if defined WIN32
    	ReleaseMutex(winMutex);
		#endif
		return;
	}
	if(!(LIST_EMPTY(&gclients) || gfdmax)){
		jpeg_start_compress( &ginfo, TRUE );
		unsigned char*data=(unsigned char*)rgb;
		#if defined USE_SSE2
		__m128i *c = (__m128i *) depth;
		for(i=0;i!=480;i++){
			__m128i *rgb128 = (__m128i *) rgb;
			for(da=0;da!=40;da++,c+=2){
					__m128i m1 = _mm_load_si128(c);
					__m128i m2 = _mm_load_si128(c+1);
					__m128i m12 = _mm_packus_epi16(_mm_srai_epi16 (m1, 2),_mm_srai_epi16 (m2, 2));
					_mm_store_si128(rgb128+da,m12);
			}
			jpeg_write_scanlines(&ginfo,&data, TRUE);		
		}
		#elif defined USE_NEON || USE_MORE_NEON
		uint16_t *c = (uint16_t *) depth;
		for(i=0;i!=480;i++){
			for(da=0;da!=640;da+=16,c+=16){
					uint16x8_t m1 = vld1q_u16(c);
					uint16x8_t m2 = vld1q_u16(c+8);
					uint8x16_t m12 = vcombine_u8(vqmovn_u16(vshrq_n_u16 (m1, 2)),vqmovn_u16(vshrq_n_u16 (m2, 2)));
					vst1q_u8(rgb+da,m12);
			}
			jpeg_write_scanlines(&ginfo,&data, TRUE);		
		}
		#else
		uint16_t *c  = depth;
		for(i=0;i!=480;i++,c+=640){
			for(da=0;da!=640;da++) rgb[da] =  c[da] >> 2 ;
			jpeg_write_scanlines(&ginfo,&data, TRUE);		
		}
		#endif
		jpeg_finish_compress(&ginfo);
		memcpy((char *)gbuf+goffset-11,"       ",7);
		memcpy((char *)gbuf+goffset-11,buf,sprintf(buf,"%d",gwritten-goffset));
		SEL(gclients,gfdmax,gentries,1,goffset)
	}
	#if defined WIN32
    ReleaseMutex(winMutex);
	#endif
}

LOCAL(void) video_cb(freenect_device *dev, void *raw_buf, uint32_t timestamp){
	if(LIST_EMPTY(&vclients) || vfdmax)	 return;
	#if defined WIN32
    WaitForSingleObject(winMutex,INFINITE);
	#endif
	//Bayer demosaicing by libfreenect
    uint8_t *prevLine=NULL;      
	uint8_t *curLine;       
	uint8_t *nextLine;
	uint32_t hVals;
	uint32_t vSums;
	unsigned char*data=(unsigned char*)rgb;
	jpeg_start_compress( &cinfo, TRUE );
	curLine  = (uint8_t *)raw_buf;
	nextLine = curLine + 640;
	for (y = 0; y != 480; ++y) {
		uint8_t *dst = rgb;
		if (y && (y < 479))
			prevLine = curLine - 640;
		else if (y == 0)
			prevLine = nextLine;
		else
			nextLine = prevLine;
		hVals  = (*(curLine++) << 8);
		hVals |= (*curLine << 16);
		vSums = ((*(prevLine++) + *(nextLine++)) << 7) & 0xFF00;
		vSums |= ((*prevLine + *nextLine) << 15) & 0xFF0000;
		uint8_t yOdd = y & 1;
		for (x = 0; x != 639; ++x) {
			hVals |= *(curLine++);
			vSums |= (*(prevLine++) + *(nextLine++)) >> 1;
			uint8_t hSum = ((uint8_t)(hVals >> 16) + (uint8_t)(hVals)) >> 1;
			if (yOdd == 0) {
				if ((x & 1) == 0) {
					*(dst++) = hSum;		
					*(dst++) = hVals >> 8;	
					*(dst++) = vSums >> 8;	
				} else {
					*(dst++) = hVals >> 8;
					*(dst++) = (hSum + (uint8_t)(vSums >> 8)) >> 1;
					*(dst++) = ((uint8_t)(vSums >> 16) + (uint8_t)(vSums)) >> 1;
				}
			} else {
				if ((x & 1) == 0) {
					*(dst++) = ((uint8_t)(vSums >> 16) + (uint8_t)(vSums)) >> 1;
					*(dst++) = (hSum + (uint8_t)(vSums >> 8)) >> 1;
					*(dst++) = hVals >> 8;
				} else {
					*(dst++) = vSums >> 8;
					*(dst++) = hVals >> 8;
					*(dst++) = hSum;
				}
			}
			hVals <<= 8;
			vSums <<= 8;
		}
		hVals |= (uint8_t)(hVals >> 16);
		vSums |= (uint8_t)(vSums >> 16);
		uint8_t hSum = (uint8_t)(hVals);
		if (yOdd == 0) {
			if ((x & 1) == 0) {
				*(dst++) = hSum;
				*(dst++) = hVals >> 8;
				*(dst++) = vSums >> 8;
			} else {
				*(dst++) = hVals >> 8;
				*(dst++) = (hSum + (uint8_t)(vSums >> 8)) >> 1;
				*(dst++) = vSums;
			}
		} else {
			if ((x & 1) == 0) {
				*(dst++) = vSums;
				*(dst++) = (hSum + (uint8_t)(vSums >> 8)) >> 1;
				*(dst++) = hVals >> 8;
			} else {
				*(dst++) = vSums >> 8;
				*(dst++) = hVals >> 8;
				*(dst++) = hSum;
			}
		}
		jpeg_write_scanlines(&cinfo,&data, TRUE);		
	}
	jpeg_finish_compress(&cinfo);
	memcpy((char *)pbuf+joffset-11,"       ",7);
	memcpy((char *)pbuf+joffset-11,buf,sprintf(buf,"%d",jwritten-joffset));
	SEL(vclients,vfdmax,ventries,2,joffset)
	#if defined WIN32
    ReleaseMutex(winMutex);
	#endif
}

int main(int argc, char **argv){	
 	 #if defined WIN32
     WSADATA wsa_data;
     WSAStartup(MAKEWORD(2,2), &wsa_data);
	 winMutex = CreateMutex(NULL,FALSE,NULL);
	 #endif
	 int cangle = 32;
	 char *targ;
	 fd_set rd,wr;
	 LIST_INIT(&clients);
	 LIST_INIT(&dclients);
	 LIST_INIT(&vclients);
	 LIST_INIT(&gclients);
	 LIST_INIT(&origins);
	 LIST_INIT(&hosts);
	 t_c = 256;
	 lport = 6661;
	 refcount = angle = 32;
	 yes = 1;
	 depth_to_mm[0] = depth_to_mm[2047] = listener   = mode   = camera  = gdump =    dfdmax = vfdmax = gfdmax =  t_x = t_X = t_y = t_Y = t_z = t_Z  = astamp =  novideo = 0;
	 multipart = 6;
	 t_j = 50;
	 max_bytes=4096;
	 sec=-1;
	 lfile = NULL;
	 sfile = NULL;
	 ofile = NULL;
	 ifile = NULL;
	 secret = NULL;
	 serial = NULL;
	 for(i=0;i!=FRAME_PIXELS;i++){
		depth_ref_x[i]=(uint16_t)(i%640);
		depth_ref_y[i]=(uint16_t)(floor(((float)i/640)));
	 }
	md5=(MD5_CTX *)calloc(1,sizeof(MD5_CTX));
	FD_ZERO(&streaming);
	FD_ZERO(&master);
	sig_register();
	memset(depth_ref,0,640*480*2);
	memset(depth,0,640*480*2);
	memset(l_count,0,sizeof(l_count));
	listeneraddr.sin_addr.s_addr = INADDR_ANY;
	while ((i = getopt(argc, argv, "p:z:Z:x:X:y:Y:s:o:m:i:a:c:C:r:l:j:f:e:d:F:k:bhvn")) != -1){
		 CSWITCH(i,optarg);
		 RSWITCH(i,optarg);		
	}
	if (freenect_init(&f_ctx, NULL) < 0) FAIL("[ERR0] Could not initialize libfreenect")
	freenect_set_log_level(f_ctx, FREENECT_LOG_FATAL);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
	if (freenect_num_devices (f_ctx) < 1) FAIL("[ERR1] Could not find any connected kinects")
	if(serial){
		if(!strncmp(serial,"list",4)){
			struct freenect_device_attributes* attrs;
			if(freenect_list_device_attributes(f_ctx, &attrs) > 0){
				while(attrs){
					printf("%s\n",attrs->camera_serial);
					attrs=attrs->next;
				}
				return 0;
			}else{
				FAIL("[ERR1] Could not find any connected kinects")
			}
		}else{
			if (freenect_open_device_by_camera_serial(f_ctx, &f_dev, serial) < 0) FAIL("[ERR2] Could not open specified kinect")
		}
	}else{
		if (freenect_open_device(f_ctx, &f_dev, camera) < 0) FAIL("[ERR2] Could not open specified kinect")
	}
	OPTFIX()
	memset(&jerr, 0, sizeof(jerr));
	JSET(cinfo,pbuf,JCS_RGB,jwritten,joffset,3);
	JSET(ginfo,gbuf,JCS_GRAYSCALE,gwritten,goffset,1);
	freenect_set_depth_buffer(f_dev,depth_raw);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT_PACKED));
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_buffer(f_dev,video_raw);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_BAYER));
	freenect_set_video_callback(f_dev, video_cb);
	freenect_start_depth(f_dev);
	if(!novideo) freenect_start_video(f_dev);
	reg=freenect_copy_registration(f_dev);
	for(i=0;i!=2048;i++) depth_to_mm[i]=reg.raw_to_mm_shift[i];
	for(ni=0,i=0;i!=10000;i++){
		while(depth_to_mm[ni]<i) ni++;
		depth_to_raw[i]=ni;
	}
	depth_to_mm[0] = depth_to_raw[0] = 0;
	doffset= CHECK_BIT(multipart,0)? sprintf(message,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nConnection: Close\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: multipart/x-mixed-replace; boundary=INTRAEL\r\n\r\n--INTRAEL\r\nContent-Type: application/json\r\nContent-Length:       \r\n\r\n",VMAJOR,VMINOR) : sprintf(message,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: application/json\r\nContent-Length:       \r\n\r\n",VMAJOR,VMINOR);	
	goffset=CHECK_BIT(multipart,1) ? sprintf((char *)gbuf,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nConnection: Close\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: multipart/x-mixed-replace; boundary=INTRAEL\r\n\r\n--INTRAEL\r\nContent-type: image/jpeg\r\nContent-length:        \r\n\r\n",VMAJOR,VMINOR):sprintf((char *)gbuf,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: image/jpeg\r\nContent-length:        \r\n\r\n",VMAJOR,VMINOR);
	joffset=CHECK_BIT(multipart,2) ? sprintf((char *)pbuf,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nConnection: Close\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: multipart/x-mixed-replace; boundary=INTRAEL\r\n\r\n--INTRAEL\r\nContent-type: image/jpeg\r\nContent-length:        \r\n\r\n",VMAJOR,VMINOR):sprintf((char *)pbuf,"HTTP/1.1 200 OK\r\nServer: Intrael %d.%d\r\nCache-Control: no-cache,  no-store\r\nAccess-Control-Allow-Origin: *\r\nContent-type: image/jpeg\r\nContent-length:        \r\n\r\n",VMAJOR,VMINOR);
	INITSOCKET(listener);
	NONBLOCKING(listener);
	addrsize = sizeof clientaddr;
	listeneraddr.sin_family = AF_INET;
	listeneraddr.sin_port = htons(lport);
	memset(&(listeneraddr.sin_zero), '\0', 8);
	if(bind(listener, (struct sockaddr *)&listeneraddr, sizeof(listeneraddr)) == SOCKET_ERROR) FAIL("[ERR3] Bind() failed")
	if(listen(listener, 50) == SOCKET_ERROR) FAIL("[ERR4] Listen() failed")
	FD_SET(listener,&master);
	fdmax=listener;
	freenect_update_tilt_state(f_dev);
	state = freenect_get_tilt_state(f_dev);
	freenect_get_mks_accel(state,&ax,&ay,&az);
	while (!die && freenect_process_events(f_ctx) >= 0){
		if(refcount>0) continue;
		#if defined WIN32
     	WaitForSingleObject(winMutex,INFINITE);
		#endif
     	if(!refcount){
		 	freenect_set_led(f_dev,LED_GREEN);
			refcount=-1;
		}
		if(astamp != gstamp){
				astamp=gstamp;
				freenect_update_tilt_state(f_dev);
				state = freenect_get_tilt_state(f_dev);
				freenect_get_mks_accel(state,&ax,&ay,&az);
		}
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		rd = master;
		wr = streaming;
		ni=((LIST_EMPTY(&dclients) && LIST_EMPTY(&vclients) && LIST_EMPTY(&gclients) && !vfdmax && !dfdmax && !gfdmax)) ? select(fdmax+1, &rd, &wr, NULL,NULL): select(fdmax+1, &rd, &wr, NULL,&timeout);
		if(ni > 0){
			fdmax = listener;
			dfdmax=0;
			vfdmax=0;
			gfdmax=0;
			LIST_FOREACH(client, &clients, entries){
			 y=0;
			 if(FD_ISSET(client->s,&rd)){
				 if((i = recv(client->s,buf,4095,0))>6){
					 l=0;
					 ni=buf[5];
					 buf[i]='\0'; 
					 if(client->t){
						 y=1;
					 }else{
						 if(!LIST_EMPTY(&origins)){
							 y=1;
							 strncpy(cbuf,buf,i);
							 targ=strtok(cbuf," \r\n");
							 while(targ){
									if(!strncmp(targ,"Origin:",7)){
										targ=strtok(NULL," \r\n");
										if(!l) LIST_FOREACH(origin, &origins, entries) if(!strncmp(origin->n,targ,origin->l)){ y=0;break;}
										break;
									}
									targ=strtok(NULL," \r\n");
							 }
						 }
					 	 if(!y && ni == '0' && sec != -1){
							 if(sec){
									strncpy(cbuf,buf,i);
									targ=strtok(cbuf+5," \n\r");
									if(targ){
										strcpy(cbuf,targ);
										targ=strtok(cbuf,"/?&");
										while(targ){
											if(strlen(targ)==34){
												if(targ[0]=='s' && targ[1]=='=' && !strncmp(targ+2,hash,32)){
													gdump=0;
													targ=strtok(buf+6," \n\r");
													strcpy(cbuf,targ);
													targ = strtok(cbuf,"/&?");
													while(targ){
														if(strlen(targ)>2 && targ[1] == '='){
															RSWITCH(targ[0],(targ+2));
														}
														targ=strtok(NULL,"/&?");
													}
													OPTFIX()
													break;
												}
											}
											targ=strtok(NULL,"/?&");
										}
									}
							 }else{
									targ=strtok(buf+6," \n\r");
									strcpy(cbuf,targ);
									targ = strtok(cbuf,"/&?");
									while(targ){
										if(strlen(targ)>2 && targ[1] == '='){
											RSWITCH(targ[0],(targ+2));
										}
										targ=strtok(NULL,"/&?");
									}
									OPTFIX()
							 }
						 }
					 }
					 if(!y){
						  FD_CLR(client->s,&master);
						  LIST_REMOVE(client,entries);
						  if((ni=='1')){
							   LIST_INSERT_HEAD(&gclients, client, gentries);
						  } else if((ni=='2') && !novideo){
							  LIST_INSERT_HEAD(&vclients, client, ventries);
						  }else{
							  LIST_INSERT_HEAD(&dclients, client, dentries);
						  }
					 }
				}else y = 1;
			 }else if(FD_ISSET(client->s,&wr)){
				 switch(client->t){
					 case 1: STREAM(gbuf,gwritten,goffset,gfdmax,gclients,gentries,CHECK_BIT(multipart,1),66);
							 break;
					 case 2: STREAM(pbuf,jwritten,joffset,vfdmax,vclients,ventries,CHECK_BIT(multipart,2),66);
							 break;
					 default: STREAM(message,written,doffset,dfdmax,dclients,dentries,CHECK_BIT(multipart,0),71);
							 break;
				 }
			 }else{
				 fdmax=MAX(client->s,fdmax);
				 switch(client->t){
					 case 1: gfdmax=MAX(client->s,gfdmax);
							 break;
					 case 2: vfdmax=MAX(client->s,vfdmax);
							 break;
					 case 3: dfdmax=MAX(client->s,dfdmax);
							 break;
					 default: break;
				 }
			 }
			 if(y){ 
				LIST_REMOVE(client,entries);
				FD_CLR(client->s,&master);
				if(client->t) FD_CLR(client->s,&streaming);
				closesocket(client->s);
				free(client);
			 }	
		  }
		  if(FD_ISSET(listener,&rd)){
			  if((newfd = accept(listener, (struct sockaddr *)&clientaddr, &addrsize)) != SOCKET_ERROR ){
				  y=0;
				  if(!LIST_EMPTY(&hosts)){
					 y=1;
					 LIST_FOREACH(host, &hosts, entries) if(host->a==((struct sockaddr_in *)&clientaddr)->sin_addr.s_addr){ y=0;break;}
				  }
				  if(!y){
					  NONBLOCKING(newfd);
				      setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, (const char*) &yes, sizeof(int));
					  client = (cli *)calloc(1, sizeof(*client));
					  client->s=newfd;
					  client->b=0;
					  client->t=0;
					  FD_SET(client->s,&master);
					  fdmax=MAX(client->s,fdmax);
					  LIST_INSERT_HEAD(&clients, client, entries);
				  }else closesocket(newfd);
			  }
		  }
		}
		#if defined WIN32
     	ReleaseMutex(winMutex);
		#endif
	}
	freenect_set_led(f_dev,LED_RED);
	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	closesocket(listener);
	#if defined WIN32
	WSACleanup();
	#endif
	return 0;
}
