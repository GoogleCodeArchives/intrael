/* 
This file is part of the Intrael Project. http://www.intrael.com
Copyright (c) 2011 Yannis Gravezas(wizgrav@intrael.com).

This code is licensed to you under the terms of the GNU General Public
License, version 3.0. See the GPL3 file for the text of the license,
or the following URL: <http://www.gnu.org/licenses/gpl-3.0.txt>

If you redistribute this file in source form, modified or unmodified,
you must leave this header intact, distribute it under the same terms,
and accompany it with the GPL3 file and. Binary distributions must 
follow the binary distribution requirements of the GPL version 3. 
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libfreenect.h>
#include <pthread.h>
#include <math.h>
#include "queue.h"
#include "md5.h"

volatile int die = 0;
volatile int idle = 30;

pthread_t freenect_thread;
pthread_cond_t  idle_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t idle_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t main_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t conf_mutex = PTHREAD_MUTEX_INITIALIZER;

#define ERR(s, c) if(opterr){\
		    fputs("Intrael: ", stderr);\
		    fputs(s, stderr);\
		    fputc(c,stderr);\
		    fputc('\n',stderr);\
		  }
#define WAKEUP()  pthread_mutex_lock(&idle_mutex);\
		  idle=30;\
		  pthread_cond_signal(&idle_cond);\
		  pthread_mutex_unlock(&idle_mutex)

#if defined WIN32
 
 #include <winsock2.h>
 #include <ws2tcpip.h>
 
 #define NULL	0
 #define EOF	(-1)
 #define NONBLOCKING(s) ioctlsocket(s, FIONBIO, &NonBlock)
 #define INITSOCKET(s) 	if ((s = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED)) == INVALID_SOCKET) exit(1)
 #define SIGEXIT() 	die=1;\
			WAKEUP();\
			WSACleanup();\
			return( TRUE )
 
 int	optind = 1;
 int 	opterr = 1;
 int	optopt;
 char	*optarg;
 ULONG NonBlock = 1;
 
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
	  WAKEUP();
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

#define DEPTHTOMM(depth) (floor(123.6 * tanf((depth) / 2842.5 + 1.1863)))
#define MMTODEPTH(mm) (floor((atanf((mm)/123.6)-1.1863)*2842.5))
#define MAX(x1,x2) ((x1) > (x2) ? (x1):(x2))
#define MIN(x1,x2) ((x1) < (x2) ? (x1):(x2))
#define DUMP(data,head,id,ext,size)\
			  written=sprintf(message,"%s/%d.%s",dumpdir,id,ext);\
			  message[written]='\0';\
			  if((fp=fopen(message,"wb"))){\
			    fprintf(fp, head);\
			    fwrite(data, size, 1, fp);\
			    fclose(fp);\
			  }else ERR("Could not open dump file",' ')
#define IOPT(arg,var,min,max) itmp = atoi(arg);\
			  itmp = MIN(itmp,max);\
			  var = MAX(itmp,min);\
			  break
#define FOPT(arg,var,min,max) ftmp = atof(arg);\
			  ftmp = MIN(ftmp,max);\
			  var = MAX(ftmp,min);\
			  break
#define ACCEPT(manager,val) if(FD_ISSET(manager,&rd))\
			      if((newfd = accept(manager, NULL, NULL)) != -1){\
				      NONBLOCKING(newfd);\
				      setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, (const char*) &yes, sizeof(int));\
				      client = (cli *)calloc(1, sizeof(*client));\
				      client->hash = client->sid = NULL;\
				      client->t = client->lc = 0;\
				      client->m =(uint32_t)val;\
				      connections++;\
				      if(client->m){\
					      client->sid = (char *)malloc(12);\
					      client->lid = sprintf(client->sid,"%u,",connections);\
				      }\
				      client->s=newfd;\
				      FD_SET(client->s,&master);\
				      fdmax=MAX(fdmax,client->s);\
				      pthread_mutex_lock( &main_mutex );\
				      LIST_INSERT_HEAD(&clients, client, entries);\
				      pthread_mutex_unlock( &main_mutex );\
			      }
#define BIND(NAME,ADDR,VAR) {\
 	INITSOCKET(NAME);\
 	NONBLOCKING(NAME);\
 	ADDR.sin_family = AF_INET;\
 	ADDR.sin_port = htons(VAR);\
 	memset(&(ADDR.sin_zero), '\0', 8);\
 	if(bind(NAME, (struct sockaddr *)&ADDR, sizeof(ADDR)) == SOCKET_ERROR) FAIL("error on bind()")\
 	if(listen(NAME, 10) == SOCKET_ERROR) FAIL("error on listen()")\
 	FD_SET(NAME,&master);\
	}
#define CSWITCH(ch,optarg) switch (ch) {\
			    case 'p': IOPT(optarg,lport,0,65535);\
			    case 'P': IOPT(optarg,mport,0,65535);\
			    case 'l': listeneraddr.sin_addr.s_addr = inet_addr(optarg);\
				      break;\
			    case 'L': manageraddr.sin_addr.s_addr = inet_addr(optarg);\
				      break;\
			    case 'k': IOPT(optarg,camera,0,9);\
			    case 'K': IOPT(optarg,kmode,0,2);\
			    case 'M': IOPT(optarg,max_bytes,3948,64000);\
			    case 'v': printf("\nIntrael v20110704 (C) 2011 Yannis Gravezas. Visit http://www.intrael.com\n\n");\
				      return 0;\
			    case 's': if(strlen(optarg)) strncpy(secret,optarg,32);\
				      break;\
			    case 'S': if(strlen(optarg))\
				      	if((fp=fopen(optarg,"r")))\
					    if(fread(buf,sizeof(char),128,fp))\
						    if((optarg=strtok((char *)buf," \n\r\t")))\
							    if(strlen(optarg)) strncpy(secret,optarg,32);\
				      if(fp) fclose(fp); else ERR("could not load the secret passphrase from the specified file",' ')\
				      break;\
			    case 'b': if(strlen(strncpy(loadfile,optarg,1023))) lf=1;\
				      break;\
			    case 'o': if(strlen(strncpy(dumpdir,optarg,1023))) df=1;\
				      break;\
			    }
#define RSWITCH(ch,optarg) switch (ch) {\
			    case 'x': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_x,0,632);\
			    case 'X': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_X,0,632);\
			    case 'y': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_y,0,478);\
			    case 'Y': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_Y,0,478);\
			    case 'z': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_z,0,9999);\
			    case 'Z': if(refcount<1)refcount=reled=1;\
						  IOPT(optarg,t_Z,0,9999);\
			    case 'c': IOPT(optarg,t_c,1,FRAME_PIXELS);\
			    case 'C': IOPT(optarg,t_C,0,FRAME_PIXELS);\
			    case 'w': IOPT(optarg,t_w,0,2147483647);\
			    case 'W': IOPT(optarg,t_W,0,2147483647);\
			    case 'h': IOPT(optarg,t_h,0,2147483647);\
			    case 'H': IOPT(optarg,t_H,0,2147483647);\
			    case 'f': IOPT(optarg,t_d,0,9999);\
			    case 'F': IOPT(optarg,t_D,0,9999);\
			    case 'd': IOPT(optarg,t_d,0,9999);\
			    case 'D': IOPT(optarg,t_D,0,9999);\
			    case 't': FOPT(optarg,t_t,0,9999);\
			    case 'T': FOPT(optarg,t_T,0,9999);\
			    case 'q': FOPT(optarg,t_q,0,1);\
			    case 'Q': FOPT(optarg,t_Q,0,1);\
			    case 'm': IOPT(optarg,mode,-1,9999);\
			    case 'r': reled=reset=1;\
						  IOPT(optarg,refcount,1,999);\
			    case 'R': IOPT(optarg,reset,1,1);\
			    case 'a': IOPT(optarg,angle,-31,31);\
			    case 'i': IOPT(optarg,instance,-2147483647,2147483647);\
			    case 'B': if(strlen(loadfile)) lf=1;\
						  if(refcount<1) refcount=reled=1;\
				      break;\
			    case 'O': IOPT(optarg,dump,1,2147483646);\
			    }
#define FRAME_PIXELS 307200
#define FAIL(error) {\
	ERR(error,' ');\
	return 1;\
       }

typedef struct cli {
    	uint32_t m,c,t,lid,lc;
	char *sid,*hash;
	SOCKET s;
	LIST_ENTRY(cli) entries;
} cli;
LIST_HEAD(, cli) clients;

/* In the beginning God created the heavens and the earth. */
char message[4096],httpmessage[4096],nmessage[4096],nhttpmessage[4096],tempmessage[256],confmessage[4096], loadfile[1024], savefile[1024],dumpdir[1024],secret[32];
uint8_t *video=NULL;
uint16_t *depth,depth_ref[FRAME_PIXELS],depth_ref_z[FRAME_PIXELS],depth_ref_Z[FRAME_PIXELS],depth_ref_x[FRAME_PIXELS],depth_ref_y[FRAME_PIXELS],depth_to_raw[16384],depth_to_mm[2048],written,headwritten,tempwritten,nwritten,nheadwritten;
uint32_t run_s[FRAME_PIXELS/2+1],run_e[FRAME_PIXELS/2+1],run_z[FRAME_PIXELS/2+1],run_Z[FRAME_PIXELS/2+1],run_sum[FRAME_PIXELS/2+1],run_label[FRAME_PIXELS/2+1],r_label[FRAME_PIXELS/2+1],l_pos_x[FRAME_PIXELS/2+1],l_pos_X[FRAME_PIXELS/2+1],l_pos_y[FRAME_PIXELS/2+1],l_pos_Y[FRAME_PIXELS/2+1],l_pos_z[FRAME_PIXELS/2+1],l_pos_Z[FRAME_PIXELS/2+1],l_cx[FRAME_PIXELS/2+1],l_cy[FRAME_PIXELS/2+1],l_sum[FRAME_PIXELS/2+1],l_count[FRAME_PIXELS/2+1],l_runs[FRAME_PIXELS/2+1],l_reg[FRAME_PIXELS/2+1],l_checked[FRAME_PIXELS/2+1],t_x,t_X,t_y,t_Y,t_z,t_Z,t_f,t_F,t_d,t_D,t_w,t_W,t_h,t_H,t_c,t_C,connections, counter,ncounter, gstamp,dump,lport,mport;
int lf,df,mode,kmode,refcount,angle,reset,reled,yes,itmp,instance,vdump,gdump,max_bytes;
float t_q,t_Q,t_t,t_T,ftmp;
freenect_raw_tilt_state* state;
freenect_context *f_ctx;
freenect_device *f_dev;
FILE *fp = NULL;
 
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{	
 	/* and it was good */ 
	depth = (uint16_t *)v_depth;
	register uint32_t i,j,ni,y,n,label,ls,le,running,dp,l,rs,re,rt,dz,dZ,dzv,dZv;
	
	if(refcount>0){	 	
		refcount--;
		if(reset){
		 memcpy(depth_ref,depth,FRAME_PIXELS*2);
		 reset=0;
		}
		if(refcount){
		 for(i = 0; i < FRAME_PIXELS; i++)
		 	depth_ref[i] = depth_ref[i] ? MIN(depth[i],depth_ref[i]) : depth[i];
		}else{
		 	if(lf){
				if((fp = fopen(loadfile,"rb"))){
					fseek(fp,17,SEEK_SET);
					if(fread(depth, 1,FRAME_PIXELS*2, fp) != FRAME_PIXELS*2 ){ 
						 ERR("reference frame file is invalid or corrupt",' ') 
					}else
					 	memcpy(depth_ref,depth,FRAME_PIXELS*2);
					fclose(fp);
				}else 
				 	ERR("could not open the reference frame file",' ')
				lf = 0;
			}
			for(i = 0;i < FRAME_PIXELS;i++){
			 	dp = depth_to_mm[depth_ref[i]];
			 	if(depth_ref_x[i]>t_X) 
					depth_ref_Z[i]=depth_ref_z[i] = 0;
				else if(mode < 0){
					depth_ref_Z[i] = (dp - t_z > 340) ? depth_to_raw[dp-t_z] : 0;
					depth_ref_z[i] = (dp - t_Z > 340) ? depth_to_raw[dp-t_Z] : 0;
				}else if(mode && dp){
					depth_ref_z[i] = dp > t_z ? depth_to_raw[t_z]:  depth_to_raw[dp-mode];
					depth_ref_Z[i] = dp > t_Z ? depth_to_raw[t_Z]:  depth_to_raw[dp-mode];
				}else{
				 	depth_ref_z[i] = depth_to_raw[t_z];
					depth_ref_Z[i] = depth_to_raw[t_Z];
				}
			}
			if(df){ DUMP(depth_ref,"P5 640 480 65535\n",0,"pgm",(640*480*2)) }
		}
	}else{
		/* And God divided the light from the darkness */
		n = l = 1;
		label = running = ls = le  = rs = rt = re = dz = dZ = dzv = dZv = 0;
		y = t_y;
		i = t_x+640*t_y;
		ni = 640*t_y+t_X;
		while(1){
			dp = depth[i];
		    if(dp > depth_ref_z[i] && dp < depth_ref_Z[i]){ 
		        if(running){
					running += dp;
				 	if(dp > dZv){
						 dZ = i;
						 dZv = dp;
					 }
					else if(dp < dzv){
						 dz = i;
						 dzv = dp;
					 }
				}else{
					running = dp;
					run_s[n] = dz = dZ  = i;
					dzv = dZv = dp;
					if(!rt) rt = n;
				}
			}else if(running) {
			 	run_e[n] = i;
				run_z[n] = dz;
				run_Z[n] = dZ;
				run_sum[n]=running;
				running = label = 0;
				if(rs){
				 	 for(j = rs; j != re; j++){
						if(run_e[j] < (run_s[n] - 641)) rs=j+1;
						else if(run_s[j] < (run_e[n] - 638)){
							if(!label || label > run_label[j]) label = run_label[j];
							if(!ls)	ls=j;
							le=j;
						}
					 }
				}
				if(label){
					for(j=ls;j!=le;j++) r_label[run_label[j]] = label;
					r_label[run_label[le]] = run_label[n] = label;
					ls = 0;
				}else{
				 	run_label[n] = r_label[l] = l;
				 	l_count[l] = 0;
					l++;
				}				
				n++;
			 }
			if(i == ni){
				y++;
				if(y > t_Y) break;
				ni+=640;
				re = n;
				rs = rt ?  rt : 0;
				rt = 0;
				i = 640*y+t_x;
			}else i++;
	    }
		for(dp=0,i = 1; i != n; i++){
			label = r_label[run_label[i]];
			rs=run_s[i];
			re=run_e[i];
			j=re-rs;
			if(l_count[label]){
			 	dz=run_z[i];
				dZ=run_Z[i];
	  			if(depth_ref_x[rs] < depth_ref_x[l_pos_x[label]]) l_pos_x[label] = rs;
	  			if(depth_ref_x[re] > depth_ref_x[l_pos_X[label]]) l_pos_X[label] = re;
	  			if(depth[dz]<depth[l_pos_z[label]]) l_pos_z[label] = dz;
	  			if(depth[dZ]>depth[l_pos_Z[label]]) l_pos_Z[label] = dZ;
	  			l_count[label] += j;
				l_sum[label] += run_sum[i];
				l_pos_Y[label] = rs;
				l_cx[label]+=((uint32_t)(j*(depth_ref_x[re-1]+depth_ref_x[rs]))/2);
				l_cy[label]+=(j*depth_ref_y[rs]);
				l_runs[label]++;
			}else{
				l_pos_z[label] = run_z[i];
				l_pos_Z[label] = run_Z[i];
				l_pos_x[label] = l_pos_y[label] = l_pos_Y[label] = rs;
				l_pos_X[label] = re;
				l_count[label] = j;
				l_sum[label] = run_sum[i];
				l_cx[label]=(uint32_t)(j*(depth_ref_x[re-1]+depth_ref_x[rs]))/2;
				l_cy[label]=j*depth_ref_y[rs];
				l_runs[label] = 1;
				l_reg[label] = 0;
				l_checked[dp]=label;
				dp++;	
	   		}
	   		if(run_label[i] != label) l_reg[label]++;
	   	}
		counter++;
		written=sprintf(message,"%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%d,%d,%f,%f,%f,%f,%f,%d,%d,%u",counter,timestamp,kmode,gdump,instance,mode,t_x,t_X,t_y,t_Y,t_z,t_Z,t_w,t_W,t_h,t_H,t_f,t_F,t_d,t_D,t_t,t_T,t_c,t_C,t_q,t_Q,state->accelerometer_x/819*9.80665, state->accelerometer_y/819*9.80665, state->accelerometer_z/819*9.80665,((int)(((double)state->tilt_angle) / 2.)), state->tilt_status,dp);
		for(j=0;j!=dp;j++){
	 		i=l_checked[j];
			if((l_count[i] < t_c)  || (t_C && l_count[i] > t_C)  ) continue;
			rs = depth_ref_x[l_pos_X[label]]-depth_ref_x[l_pos_x[label]];
			re = depth_ref_y[l_pos_Y[label]]-depth_ref_y[l_pos_Y[label]];
			if(re) if((t_t && t_t < rs/re) || (t_T && t_T > rs/re)  ) continue;
	 		rt = (rs && re) ? l_count[i]/(rs*re):0;
			if((t_q && t_q < rt)  || (t_Q && t_Q > rt )  ) continue;
			dz = depth_to_mm[depth[l_pos_z[i]]];
			dZ = depth_to_mm[depth[l_pos_Z[i]]];
			if((t_f && t_f < dZ-dz) || (t_F && t_F > dZ-dz)) continue;
	 		rt = depth_to_mm[((uint32_t)(l_sum[i]/l_count[i]))];
			if((t_d && t_d < rt) || (t_D && t_D > rt)) continue;
	 		if((t_w && t_w < rs*rt) || (t_W && t_W > rs*rt) || (t_h && t_h < re*rt) || (t_H && t_H > re*rt)  ) continue;
			l=(uint32_t) l_cx[i]/l_count[i]+640*(l_cy[i]/l_count[i]);
			--l_pos_X[i];
			tempwritten = sprintf(tempmessage,",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",depth_ref_x[l_pos_x[i]],depth_ref_y[l_pos_x[i]],depth_to_mm[depth[l_pos_x[i]]],depth_to_mm[depth_ref[l_pos_x[i]]],depth_ref_x[l_pos_X[i]],depth_ref_y[l_pos_X[i]],depth_to_mm[depth[l_pos_X[i]]],depth_to_mm[depth_ref[l_pos_X[i]]],depth_ref_x[l_pos_y[i]],depth_ref_y[l_pos_y[i]],depth_to_mm[depth[l_pos_y[i]]],depth_to_mm[depth_ref[l_pos_y[i]]],depth_ref_x[l_pos_Y[i]],depth_ref_y[l_pos_Y[i]],depth_to_mm[depth[l_pos_Y[i]]],depth_to_mm[depth_ref[l_pos_Y[i]]],depth_ref_x[l_pos_z[i]],depth_ref_y[l_pos_z[i]],dz,depth_to_mm[depth_ref[l_pos_z[i]]],depth_ref_x[l_pos_Z[i]],depth_ref_y[l_pos_Z[i]],dZ,depth_to_mm[depth_ref[l_pos_Z[i]]],depth_ref_x[l],depth_ref_y[l],depth_to_mm[depth[l]],depth_to_mm[depth_ref[l]],rt,l_count[i],l_runs[i],l_reg[i]);
			if(written+tempwritten>max_bytes) break;
			memcpy(message+written,tempmessage,tempwritten);
			written+=tempwritten;
			message[written]='\0';
		}
		headwritten = sprintf(httpmessage,"HTTP/1.1 200 OK\r\nContent-Type: application/x-javascript\r\nCache-Control: no-cache\r\nCache-Control: no-store\r\nContent-Length:%d\r\n\r\nIntrael.p([%s]);\n",written+15,message);
		message[written] = '\n';
		written++;
		
		pthread_mutex_lock( &main_mutex );
		memcpy(nmessage,message,written);
		memcpy(nhttpmessage,httpmessage,headwritten);
		gstamp = timestamp;
		nheadwritten=headwritten;
		nwritten=written;
		ncounter=counter;
		pthread_mutex_unlock( &main_mutex );
	}
	pthread_mutex_lock(&idle_mutex);
    if(idle && refcount == -1) idle--;
	while (!idle) pthread_cond_wait(&idle_cond, &idle_mutex);
	pthread_mutex_unlock(&idle_mutex);
}

void video_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	video=v_depth;
}

void *freenect_threadfunc(void *arg)
{	
 	/* and there was light */
	int cangle = 32;
	char *targ;
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_start_depth(f_dev);
	freenect_set_depth_callback(f_dev, depth_cb);
	if(kmode){
		freenect_set_video_mode(f_dev, freenect_find_video_mode((kmode>1?FREENECT_RESOLUTION_HIGH:FREENECT_RESOLUTION_MEDIUM), FREENECT_VIDEO_RGB));
		freenect_start_video(f_dev);
		freenect_set_video_callback(f_dev, video_cb);
		}
		 
	/* And God saw the light */
	while (!die && freenect_process_events(f_ctx) >= 0) {
		pthread_mutex_lock(&conf_mutex);
		if(confmessage[0] != '\0'){
			targ = strtok(confmessage," \t\r\n");
			if(targ) while((targ=strtok(NULL," \t\r\n"))){
				if(strlen(targ)>1) 
					RSWITCH(targ[0],(targ+1));
				}
			if(mode >= 0 && t_z < 340) t_z=340;
			confmessage[0]='\0';
		}
		pthread_mutex_unlock(&conf_mutex);
		if(reled){
			freenect_set_led(f_dev,LED_YELLOW);
			reled=0;
			} 
		if(dump && df && depth && ((kmode && video) || !kmode) && refcount==-1){
			DUMP(depth,"P5 640 480 65535\n",dump,"pgm",(640*480*2));
			if(kmode){ DUMP(video,(kmode > 1 ? "P6 1280 1024 255\n":"P6 640 480 255\n"),dump,"ppm",(kmode > 1?(1280*1024*3):(640*480*3))); }
			gdump=dump;
			dump=0;
			}
		if(cangle != angle){
		 cangle=angle;
		 freenect_set_tilt_degs(f_dev,cangle);
		}
		freenect_update_tilt_state(f_dev);
		state = freenect_get_tilt_state(f_dev);
		if(!refcount){
		 	freenect_set_led(f_dev,LED_GREEN);
			refcount=-1;
		}
	}
	freenect_set_led(f_dev,LED_RED);
	freenect_stop_depth(f_dev);
	if(kmode) freenect_stop_video(f_dev);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	return NULL;
}

/*In the beggining was the Code and the Code was God */
int main(int argc, char **argv)
{
 #if defined WIN32
     WSADATA wsa_data;
     WSAStartup(MAKEWORD(2,2), &wsa_data);
 #endif
 LIST_INIT(&clients);
 fd_set rd,wr,master,pending;
 SOCKET fdmax,listener,newfd,manager;
 struct sockaddr_in listeneraddr, manageraddr;
 struct cli *client;
 int res,ch,rm,ln,camera;
 char buf[4096];
 unsigned char mbuf[128], mtemp[16];
 t_X = 632;
 t_Y = 478;
 t_z = 340;
 t_c = 50;
 lport = 6661;
 refcount = angle = 32;
 connections = counter = ncounter = reset  = reled = yes = 1;
 depth_to_mm[2047] = manager = listener = t_x = t_y = t_f = t_F = t_d = t_D = t_w = t_W = t_h = t_H = t_C = t_q = t_Q = t_t = t_T = mport = mode  = gstamp = camera = instance =  dump = vdump = gdump =  lf = df =0;
 depth = NULL;
 max_bytes=3948;
 loadfile[0] = savefile[0] = confmessage[0] = '\0';
 for(res=0;res<16384;res++) depth_to_raw[res]=(res < 9779) ? (uint16_t)MMTODEPTH(res) : (uint16_t)MMTODEPTH(9778);
 for(res=0;res<2048;res++) depth_to_mm[res]= (res < 1058) ? (uint16_t)DEPTHTOMM(res) : (uint16_t)DEPTHTOMM(1057);
 for(res=0;res<FRAME_PIXELS;res++){
  	depth_ref_x[res]=(uint16_t)(res%640);
	depth_ref_y[res]=(uint16_t)(floor(((float)res/640)));
 }
 FD_ZERO(&pending);
 FD_ZERO(&master);
 sig_register();
 MD5_CTX *md5=(MD5_CTX *)calloc(1,sizeof(MD5_CTX));
 
 while ((ch = getopt(argc, argv, "p:P:z:Z:x:X:y:Y:w:W:h:H:d:D:t:T:q:Q:p:P:s:S:o:b:m:a:A:i:c:C:B:O:R:l:L:k:K:v")) != -1) {
		 CSWITCH(ch,optarg);
		 RSWITCH(ch,optarg);		
 }
 
 /* And he shal set ye shepe on his right honde and the goates on the lefte.*/ 
 listeneraddr.sin_addr.s_addr = manageraddr.sin_addr.s_addr = INADDR_ANY;
 if(lport) BIND(listener,listeneraddr,lport)
 if(mport) BIND(manager,manageraddr,mport)
 if(!(FD_ISSET(listener,&master) || FD_ISSET(manager,&master))) FAIL("no http or management listener set")
 fdmax=MAX(listener,manager);
 
 /* And God said, let there be light */
 if (freenect_init(&f_ctx, NULL) < 0) FAIL("freenect_init() failed")
 if (freenect_num_devices (f_ctx) < 1) FAIL("kinect was not Found")
 if (freenect_open_device(f_ctx, &f_dev, camera) < 0) FAIL("could not open kinect")
 if ((res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL))) FAIL("pthread_create() failed")
 
 /* Go ye into all the world and preach the Gospel */
 while(!die){
	 rd = master;
	 wr = pending;
	 select(fdmax+1, &rd, &wr, NULL,NULL);
	 fdmax = MAX(listener,manager);
	 LIST_FOREACH(client, &clients, entries){
	  	rm = 0;
		if(FD_ISSET(client->s,&wr) && client->lc != ncounter && gstamp){
			pthread_mutex_lock( &main_mutex );
	 	 	if(!client->t){
				client->t=gstamp;
				client->c=ncounter;
			}
			FD_CLR(client->s,&pending);
			if(client->m){
	 			if(send(client->s,client->sid,client->lid,0) < client->lid) rm=1;
				if(send(client->s,nmessage,nwritten,0) < written) rm=1;
			}else if(send(client->s,nhttpmessage,nheadwritten,0) < nheadwritten) rm=1;
			if(!rm) client->lc=ncounter;
			pthread_mutex_unlock( &main_mutex );
	 	}
		if(FD_ISSET(client->s,&rd) && !rm){
		 	 if((ln = recv(client->s,buf,4095,0))>0){
			  	 buf[ln]='\0';
			  	 if(client->m)
					 	 if(buf[0]=='\n')
							 FD_SET(client->s,&pending);
						 else if(strlen(buf)>32 && client->t && strlen(secret)){
			 			 		if(!client->hash){
									client->hash=(char *)malloc(32);
									MD5_Init(md5);
									MD5_Update(md5,mbuf,sprintf((char *)mbuf,"%s,%s%u,%u",secret,client->sid,client->c,client->t));
									MD5_Final(mtemp, md5);
									for(ch = 0; ch < 16; ch++) sprintf(&client->hash[2*ch],"%02x",mtemp[ch]);
								}
								if(!strncmp(buf,client->hash,32)){
								  	pthread_mutex_lock( &conf_mutex );
								  	strcpy(confmessage,buf);
									pthread_mutex_unlock( &conf_mutex );
	 	 							FD_SET(client->s,&pending);
								}else
									rm = 1;
						}else
						 	rm = 1;
				else
					FD_SET(client->s,&pending);
				WAKEUP();
			 }else
			  	rm = 1;
		 }
		 if(rm){ 
		  	LIST_REMOVE(client,entries);
			FD_CLR(client->s,&master);
			if(FD_ISSET(client->s,&pending)) FD_CLR(client->s,&pending);
			closesocket(client->s);
			if(client->sid) free(client->sid);
			if(client->hash) free(client->hash);
			free(client);
		 }else
		  	fdmax=MAX(fdmax,client->s);
	 }
	 ACCEPT(listener,0);
	 ACCEPT(manager,1);
 } 
 if(listener) closesocket(listener);
 if(manager) closesocket(manager);
 pthread_join(freenect_thread, NULL);
 #if defined WIN32
     WSACleanup();
 #endif
 return 0;
}