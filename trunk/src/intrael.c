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

#include "intrael.h"

#if defined WIN32

int	optind = 1;
int 	opterr = 1;
int	optopt;
char	*optarg;
ULONG NonBlock = 1;
HANDLE winMutex;

int getopt(int argc, char **argv, char *opts)
{
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
BOOL CtrlHandler( DWORD fdwCtrlType )
{
	switch( fdwCtrlType ) {
	case CTRL_C_EVENT:
		SIGEXIT();
	case CTRL_CLOSE_EVENT:
		SIGEXIT();
	case CTRL_BREAK_EVENT:
		return FALSE;
	case CTRL_LOGOFF_EVENT:
		return FALSE;
	case CTRL_SHUTDOWN_EVENT:
		SIGEXIT();
	default:
		return FALSE;
	}
}

static void sig_register(void)
{
	if( !SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE ) )
		ERR("unable to register control handler",' ');
}

#else

static void sigexit(int signo)
{
	die=1;
}

static void sig_register(void)
{
	struct sigaction sigact;
	sigact.sa_handler = sigexit;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
}
#endif


METHODDEF(void) init_destination (j_compress_ptr cinfo)
{
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->pub.next_output_byte = dest->buffer;
	dest->pub.free_in_buffer = dest->bufsize;
	dest->datasize = 0;
}

METHODDEF(boolean) empty_output_buffer (j_compress_ptr cinfo)
{
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->pub.next_output_byte = dest->buffer;
	dest->pub.free_in_buffer = dest->bufsize;
	return TRUE;
}

METHODDEF(void) term_destination (j_compress_ptr cinfo)
{
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->datasize = dest->bufsize - dest->pub.free_in_buffer;
	*dest->outsize = (int)dest->datasize;
}

GLOBAL(void) jpeg_memory_dest (j_compress_ptr cinfo)
{
	mem_dest_ptr dest;
	if (cinfo->dest == 0) cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,	sizeof (memory_destination_mgr));
	dest = (mem_dest_ptr) cinfo->dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_output_buffer;
	dest->pub.term_destination = term_destination;
}

METHODDEF(void) jpeg_set_frame (j_compress_ptr cinfo,char *buf, uint16_t *len)
{
	mem_dest_ptr dest = (mem_dest_ptr)cinfo->dest;
	dest->buffer =  (unsigned char *)buf;
	dest->bufsize = 32700;
	dest->outsize = len;
}

void depth_cb(freenect_device* dev, void *v_depth, uint32_t timestamp)
{
	g_mutex_lock(depth_mutex);
	dstamp=timestamp;
	if (dwait<2) {
		dwait++;
		freenect_set_depth_buffer(f_dev,depth_raw[(dcb+dwait)%3]);
		g_cond_signal(depth_cond);
	}
	g_mutex_unlock(depth_mutex);
}

void *depth_thread()
{
	header_t *dhead,*ghead,*rhead;
	struct jpeg_compress_struct ginfo;
	struct jpeg_error_mgr jerr;
	FILE *fp = NULL;
	char opts[] = "xXzZyYcCe";
	freenect_raw_tilt_state* state=NULL;
	static uint16_t X,i,dz,dZ,dzv,dZv,x,y,rawz,rawZ,*frame,*dref,*drz,*drZ,len,gmin,gmax,skelcount;
	static int32_t ni,da,l,n,rs,re,rt,er,sr,label,running;
	uint8_t framefix,encode,dmap,json;
	static uint8_t *raw,*draw;
	static int refcount,mode,gdump,t_x,t_X,t_y,t_Y,t_z,t_Z,t_c,t_C,t_g,t_s,itmp;
	static SkeltrackJoint *joint;
	static SkeltrackJointList list;
	GValue dval = G_VALUE_INIT;
	g_value_init (&dval, G_TYPE_INT);
	int32_t cbuf[128];
	double ax,az,ay,angle;
	static float ftmp;
	static char buf[64];
	static frame_t *dframe, *gframe, *rframe, *tframe;
	unsigned char*jblack=(unsigned char*)black;
	unsigned char*jgray=(unsigned char*)gray;
	framefix = refcount = t_z = t_Z = mode = 0;
	skeleton = SKELTRACK_SKELETON (skeltrack_skeleton_new ());
	g_value_set_int (&dval,8);
	g_object_set_property (G_OBJECT (skeleton), "dimension-reduction", &dval);
	t_s=0;
	angle=32;
	HSET(dhead,STR_JSON_S,STR_JSON_M,0);
	HSET(ghead,STR_JPEG_S,STR_JPEG_M,1);
	HSET(rhead,STR_DUMP,STR_DUMP,3);
	LIST_HEAD(, frame_t) dframes;
	LIST_INIT(&dframes);
	LIST_HEAD(, frame_t) gframes;
	LIST_INIT(&gframes);
	LIST_HEAD(, frame_t) rframes;
	LIST_INIT(&rframes);
	memset(&jerr, 0, sizeof(jerr));
	JSET(ginfo,JCS_GRAYSCALE,1);
	g_mutex_lock(dev_mutex);
	if(f_dev){
		MSTATE();
	}
	g_mutex_unlock(dev_mutex);
	while(1) {
		g_mutex_lock(depth_mutex);
		while(!dwait) g_cond_wait(depth_cond, depth_mutex);
		draw=depth_raw[dcb%3];
		g_mutex_unlock(depth_mutex);
		g_mutex_lock(conf_mutex);
		if(gbuf[0]) {
			memcpy(cbuf,gbuf,512);
			gbuf[0]=0;
		}
		g_mutex_unlock(conf_mutex);
		while(cbuf[0]) {
			if(cbuf['b']) {
				cbuf['b'] = 0;
				if((fp = fopen(lfile,"rb"))) {
					if((ni=fread(depth_ref, 2,FRAME_PIXELS, fp)) != FRAME_PIXELS ) {
						memset(depth_ref,0,FRAME_PIXELS*2);
						ERR("[ERR5] Invalid background frame",' ');
					}
					fclose(fp);
				} else {
					ERR("[ERR6] Could not open file for reading the background frame",' ');
					memset(depth_ref,0,FRAME_PIXELS*2);
				}
				refcount = -1;
				for(i=0; i!=9; i++) {
					memcpy(buf, ((char *)depth_ref) + 640*((int)opts[i]) - 6,6);
					buf[6]='\0';
					RSWITCH(opts[i],buf);
				}
				continue;
			}
			n=0;
			t_x = cbuf['x'];
			t_X = cbuf['X'];
			t_y = cbuf['y'];
			t_Y = cbuf['Y'];
			if(t_z != cbuf['z'] && cbuf['e']) n=1;
			t_z = cbuf['z'];
			if(t_Z != cbuf['Z'] && cbuf['e']) n=1;
			t_Z = cbuf['Z'];
			t_c = cbuf['c'];
			t_C = cbuf['C'];
			t_s = cbuf['s'];
			g_mutex_lock(dev_mutex);
			if(cbuf['a'] != angle && f_dev) freenect_set_tilt_degs(f_dev,(double)(angle=cbuf['a']));
			g_mutex_unlock(dev_mutex);
			t_g = cbuf['g'];
			if(mode != cbuf['e']) n=1;
			mode = cbuf['e'];
			if(n) refcount= -1;
			if(cbuf['r']) refcount = cbuf['r'];
			if(refcount > 0) memset(depth_ref,0,FRAME_PIXELS*2);
			if(!mode) refcount = 0;
			gdump = cbuf[0];
			cbuf[0]=0;
			t_X=((int) (t_X/8))*8;
			t_x=((int) (t_x/8))*8;
			if(t_x >= t_X) {
				t_x=0;
				t_X=320;
			}
			if(t_y >= t_Y) {
				t_y=0;
				t_Y=240;
			}
			if(t_z >= t_Z) {
				t_z=350;
				t_Z=1500;
			}
			if(mode<0) {
				gmin = 0;
				gmax = 1054;
			} else {
				gmin = depth_to_raw[t_z];
				gmax = depth_to_raw[t_Z];
			}
			for(i=0; i!=9; i++) {
				sprintf(buf,"%6d",cbuf[(int)opts[i]]);
				memcpy(((char *)depth_ref) + 640*((int)opts[i]) - 6,buf,6);
			}
		}
		g_mutex_lock(dev_mutex);
		if(f_dev){
			if(!refcount && led != LED_GREEN) freenect_set_led(f_dev,(led = LED_GREEN));
			else if(refcount > 1 && led != LED_BLINK_RED_YELLOW ) freenect_set_led(f_dev,(led = LED_BLINK_RED_YELLOW));
		}
		g_mutex_unlock(dev_mutex);
		if(refcount) {
			if(refcount>0) {
				framefix=1;
				frame=depth;
				dref=depth_ref;
				for(y=0; y!=240; y++) {
					dref=depth_ref+(320*y);
					raw=draw+1760*y;
					for(i=0; i!=320; i++,raw+=22,dref+=8) {
						PCK();
						for(ni=0; ni!=8; ni++) {
							da=frame[ni];
							dz=dref[ni];
							if(da>1054) da=0;
							if(!dz) {
								dref[ni]=da;
							} else if(da) {
								dref[ni]=MIN(da,dz);
							}
						}
					}
				}
			}
			refcount = refcount == -1 ? 0: refcount-1;
			if(refcount) {
				g_mutex_lock(depth_mutex);
				dcb++;
				dwait--;
				g_mutex_unlock(depth_mutex);
				continue;
			} else {
				if(framefix) {
					for(y=0; y!=240; y++) {
						dref=depth_ref+(320*y);
						for(x=0,i=1; i!=316; i++) {
							da=depth_ref[i];
							if(x) {
								if(da) {
									dz=(dz+da)/2;
									for(ni=x; ni!=i; ni++)
										dref[ni]=dz;
									x=0;
								}
							} else if(!da) {
								x=i-1;
								if(!(dz=dref[x])) x=0;
							}
						}
						if(x) for(ni=x; ni!=i; ni++) dref[ni]=dz;
					}
					for(ni=0; ni!=FRAME_PIXELS; ni++) depth_ref[ni] = depth_to_mm[depth_ref[ni]];
				}
				if(mode<0) {
					dz=depth_to_raw[t_z];
					dZ=depth_to_raw[t_Z];
					x=abs(mode);
					for(y=0; y!=240; y++) {
						dref=depth_ref+(320*y);
						drz=depth_ref_z+(320*y);
						drZ=depth_ref_Z+(320*y);
						for(i=0; i!=316; i++) {
							ni=dref[i]-t_Z;
							if(ni > 0) {
								da = depth_to_raw[ni];
								dzv = depth_to_raw[dref[i]];
								if(dz > x) {
									drz[i] = (da > dzv-x) ? dzv-x : da;
								} else {
									drz[i] = 2048;
								}
							} else {
								drz[i] = 2048;
							}
							ni=dref[i]-t_z;
							if(ni > 0) {
								da = depth_to_raw[ni];
								dzv = dref[i];
								if(dzv > x) {
									drZ[i] = (dz > dzv-x) ? dzv-x : dz;
								} else {
									drZ[i] = 0;
								}
							} else {
								drZ[i] = 0;
							}
						}
					}
				} else if(mode > 0) {
					rawz=depth_to_raw[t_z];
					rawZ=depth_to_raw[t_Z];
					for(y=0; y!=240; y++) {
						dref=depth_ref+(320*y);
						drz=depth_ref_z+(320*y);
						drZ=depth_ref_Z+(320*y);
						for(i=0; i!=316; i++) {
							ni=dref[i]-mode;
							if(ni > 0) {
								itmp=depth_to_raw[ni];
								drz[i] = t_z < ni ? itmp:rawz;
								drZ[i] = t_Z < ni ? itmp:rawZ;
							} else {
								drz[i] = 2048;
								drZ[i] = 0;
							}
						}
					}
				}
			}

		}
		if(cbuf['d']) {
			if((fp=fopen(sfile,"wb"))) {
				fwrite(depth_ref, FRAME_PIXELS*2, 1, fp);
				fclose(fp);
			} else ERR("[ERR7] Could not write the background frame to the specified file",' ');
			cbuf['d']=0;
		}
		//Slightly modified run based component labelling by Dr Suzuki and friends
		do {
			g_mutex_lock(net_mutex);
			IF_NOT_FRAME(dframe,dframes,dhead,dentries,frmax,dmiss) {
				json=0;
			}else{
				json=1;
			}
			IF_NOT_FRAME(gframe,gframes,ghead,gentries,frmax,gmiss) {
				encode=0;
			}
			else {
				encode=1;
			}
			IF_NOT_FRAME(rframe,rframes,rhead,rentries,umax,rmiss) {
				dmap=0;
			}
			else {
				dmap=1;
			}
			if(!(json || dmap || encode)) {
				FDUMP(ndframe,dmiss,dframes,dentries);
				FDUMP(ngframe,gmiss,gframes,gentries);
				FDUMP(nrframe,rmiss,rframes,rentries);
				g_mutex_unlock(net_mutex);
				g_mutex_lock(depth_mutex);
				dcb++;
				dwait--;
				g_mutex_unlock(depth_mutex);
				break;
			}
			g_mutex_unlock(net_mutex);
			
			if(json){
				if(!dframe) {
					CFRAME(dframe,dhead,32768);
				}
			}
			if(encode) {
				if(gmin != gmax) {
					ni = depth_to_mm[gmax];
					ftmp = ni - depth_to_mm[gmin];
					for(; gmin != gmax; gmin++) {
						depth_to_gray[gmin] = ((ni-depth_to_mm[gmin])/ftmp)*255;
					}
#if defined USE_SSE
					_mm_empty();
#endif
				}
				if(!gframe) {
					CFRAME(gframe,ghead,32768);
				}
				jpeg_set_frame(&ginfo,gframe->buf+53, &len);
				jpeg_start_compress( &ginfo, TRUE );
				for(y=0; y!=t_y; y++) jpeg_write_scanlines(&ginfo,&jblack, TRUE);
			}
			if(dmap) {
				if(!rframe) {
					CFRAME(rframe,rhead,153600);
				}
				for(dref=(uint16_t *)rframe->buf,y=0; y!=t_y; y++,dref+=320) memset(dref,0,640);
			}
			
			l = n = 1 ;
			if(json){
				if(t_s){
					raw = draw;
					if(mode){
						for(ni=0,i=40;ni!=1200;ni++,raw+=22){
							da=*(raw) << 3 | *(raw+1) >> 5;
							sbuf[ni] = da > depth_ref_z[ni] && da < depth_ref_Z[ni] ? depth_to_mm[da]:0;	
							if(!(--i)){
								raw += 13200;
								i=40;
							}
						}	
					}else{
						rawz=depth_to_raw[t_z];
						rawZ=depth_to_raw[t_Z];
						for(ni=0,i=40;ni!=1200;ni++,raw+=22){
							da=*(raw) << 3 | *(raw+1) >> 5;
							sbuf[ni] = da > rawz && da < rawZ ? depth_to_mm[da]:0;	
							if(!(--i)){
								raw += 13200;
								i=40;
							}
						}
					}
				}
				if(!(t_s < 0 && !(dmap || encode))){
					#if defined USE_SSE
					__m64 mmask=_mm_set1_pi16(2047);
					#elif defined USE_SSE2
					__m128i mmask=_mm_set1_epi16(2047);
					#elif defined USE_NEON
					uint16x8_t mmask = vdupq_n_u16(2047);
					#endif
					i = t_x;
					y = t_y;
					running = label = dz = dZ = dzv = dZv = ni = rs = rt = re =  0;
					itmp=i+320*y;
					drz=depth_ref_z + itmp;
					drZ=depth_ref_Z + itmp;
					frame=depth + itmp;
					raw = draw + (((int)(640*y+i)/4)*11);
					if(mode) {
						while(1) {
							UNPACK();
							IF_EXT(0) {PROC(0);}
							IF_EXT(1) {PROC(1);}
							IF_EXT(2) {PROC(2);}
							IF_EXT(3) {PROC(3);}
							IF_EXT(4) {PROC(4);}
							IF_EXT(5) {PROC(5);}
							IF_EXT(6) {PROC(6);}
							IF_EXT(7) {PROC(7);}
							IF_NEXT() {
								raw += 22;
								frame += 8;
								drz+=8;
								drZ+=8;
							}
							else {
								JUMP();
								itmp=i+320*y;
								drz=depth_ref_z + itmp;
								drZ=depth_ref_Z + itmp;
							}
						}
					} else {
						rawz=depth_to_raw[t_z];
						rawZ=depth_to_raw[t_Z];
						#if defined USE_SSE
						__m64 rmz=_mm_set1_pi16(rawz);
						__m64 rmZ=_mm_set1_pi16(rawZ);
						#elif defined USE_SSE2
						__m128i rmz=_mm_set1_epi16(rawz);
						__m128i rmZ=_mm_set1_epi16(rawZ);
						#elif defined USE_NEON
						uint16x8_t rmz=vdupq_n_u16(rawz);
						uint16x8_t rmZ=vdupq_n_u16(rawZ);
						#endif
						while(1) {
							ZUNPACK();
							IF_THR(0) {PROC(0);}
							IF_THR(1) {PROC(1);}
							IF_THR(2) {PROC(2);}
							IF_THR(3) {PROC(3);}
							IF_THR(4) {PROC(4);}
							IF_THR(5) {PROC(5);}
							IF_THR(6) {PROC(6);}
							IF_THR(7) {PROC(7);}
							IF_NEXT() {
								raw += 22;
								frame += 8;
							}
							else {
								JUMP();
							}
						}
					}
				}
			}else{
				#if defined USE_SSE
				__m64 mmask=_mm_set1_pi16(2047);
				#elif defined USE_SSE2
				__m128i mmask=_mm_set1_epi16(2047);
				#elif defined USE_NEON
				uint16x8_t mmask = vdupq_n_u16(2047);
				#endif
				i=t_x;
				y=t_y;
				itmp=i+320*y;
				drz=depth_ref_z + itmp;
				drZ=depth_ref_Z + itmp;
				frame=depth + itmp;
				raw = draw + (((int)(640*y+i)/4)*11);
				if(mode){
					while(1){
						UNPACK();
						IF_NEXT(){
							raw += 22;
							frame += 8;
							drz+=8;
							drZ+=8;
						}else{
							if(encode){
								for(i=t_x;i!=t_X;i++) gray[i] = depth_to_gray[depth[i]];
								jpeg_write_scanlines(&ginfo,&jgray, TRUE);\
							}
							if(dmap){
								for(i=0;i!=t_x;i++) dref[i]=0;
								for(i=t_x;i!=t_X;i++) dref[i] = depth_to_mm[depth[i]];
								for(i=t_X;i!=320;i++) dref[i]=0;
								dref+=320;
							}
							if(++y == t_Y) break;
							i=t_x;
							frame=depth+i;
							raw = draw + (((int)(640*y+i)/4)*11);
							itmp=i+320*y;
							drz=depth_ref_z + itmp;
							drZ=depth_ref_Z + itmp;
						}
					}
				}else{
					#if defined USE_SSE
					__m64 rmz=_mm_set1_pi16(rawz);
					__m64 rmZ=_mm_set1_pi16(rawZ);
					#elif defined USE_SSE2
					__m128i rmz=_mm_set1_epi16(rawz);
					__m128i rmZ=_mm_set1_epi16(rawZ);
					#elif defined USE_NEON
					uint16x8_t rmz=vdupq_n_u16(rawz);
					uint16x8_t rmZ=vdupq_n_u16(rawZ);
					#endif
					while(1){
						ZUNPACK();
						IF_NEXT(){
							raw += 22;
							frame += 8;
						}else{
							if(encode){
								for(i=t_x;i!=t_X;i++) gray[i] = depth_to_gray[depth[i]];
								jpeg_write_scanlines(&ginfo,&jgray, TRUE);\
							}
							if(dmap){
								for(i=0;i!=t_x;i++) dref[i]=0;
								for(i=t_x;i!=t_X;i++) dref[i] = depth_to_mm[depth[i]];
								for(i=t_X;i!=320;i++) dref[i]=0;
								dref+=320;
							}
							if(++y == t_Y) break;
							i=t_x;
							frame=depth+i;
							raw = draw + (((int)(640*y+i)/4)*11);
						}
					}
				}	
			}
			g_mutex_lock(depth_mutex);
			dcb++;
			dwait--;
			g_mutex_unlock(depth_mutex);
			if(encode) {
				for(; y!=240; y++) jpeg_write_scanlines(&ginfo,&jblack, TRUE);
				jpeg_finish_compress(&ginfo);
				memcpy(gframe->buf,buf,sprintf(buf,STR_JPEG,len));
				gframe->l=len+53;
				gframe->t=dstamp;
				memset(gray,0,320);
			}
			if(dmap) {
				for(; y!=240; y++,dref += 320) memset(dref,0, 640);
				rframe->l=153600;
				rframe->t=dstamp;
			}
			ni=0;
			while(--n) {
				label = r_label[r_label[run_label[n]]];
				rs=run_s[n];
				re=run_e[n];
				y=run_y[n];
				l=re-rs;
				if(l_count[label]) {
					dzv=run_zv[n];
					dZv=run_Zv[n];
					if(rs < run_s[l_pos_x[label]]) l_pos_x[label] = n;
					if(re > run_e[l_pos_X[label]]) l_pos_X[label] = n;
					if(dzv < run_zv[l_pos_z[label]]) l_pos_z[label] = n;
					if(dZv > run_Zv[l_pos_Z[label]]) l_pos_Z[label] = n;
					l_count[label] += l;
					l_pos_y[label] = n;
					l_cx[label]+=((l*(rs+re)));
					l_vrun[label] += y;
					l_cy[label]+= l*y;
					l_sum[label] += run_sum[n];
					l_runs[label]++;
				} else {
					l_pos_z[label] = n;
					l_pos_Z[label] = n;
					l_pos_x[label] = n;
					l_pos_X[label] = n;
					l_pos_y[label] = n;
					l_pos_Y[label] = n;
					l_count[label] = l;
					l_cx[label]=((l*(rs+re)));
					l_vrun[label] = y;
					l_cy[label]=l*y;
					l_sum[label] = run_sum[n];
					l_runs[label] = 1;
					l_checked[ni++]=label;
				}
			}
			#if defined USE_SSE
			_mm_empty();
			#endif
			if(json){
				if(t_g) {
					g_mutex_lock(dev_mutex);
					if(f_dev){
						MSTATE();
					}
					g_mutex_unlock(dev_mutex);
				}
				len = 59 + sprintf(dframe->buf+59,"[%u,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%d",dstamp,gdump,mode,t_s,t_x,t_X,t_y,t_Y,t_z,t_Z,t_c,t_C,ax, ay, az,freenect_get_tilt_status(state) ? 31+freenect_get_tilt_status(state) : (int)freenect_get_tilt_degs(state));
				if(t_s < 0 && !(dmap || encode)){
					BOUNDSET(0,40,0,30);
					list = skeltrack_skeleton_track_joints_sync(skeleton,sbuf,40,30,NULL,NULL);
					SKEL();
				}else{
					skelcount = ABS(t_s);
					while(ni--) {
						label=l_checked[ni];
						l=l_count[label];
						l_count[label]=0;
						if(len > MAX_BYTES) continue;
						if((l < t_c)  || (t_C && l > t_C)  ) continue;
						int posx = l_pos_x[label];
						int posX = l_pos_X[label];
						int posy = l_pos_y[label];
						int posY = l_pos_Y[label];
						int posz = l_pos_z[label];
						int posZ = l_pos_Z[label];
						x= l_cx[label]/(2*l);
						y= l_cy[label]/l;
						da = (uint32_t) (x+320*y);
						dzv=depth_to_mm[l_sum[label]/l];
						if(skelcount){
							BOUNDSET(run_s[posx] >> 3,run_e[posX] >>3,run_y[posy]>>3,run_y[posY]>>3);
							list = skeltrack_skeleton_track_joints_sync(skeleton,sbuf,40,30,NULL,NULL);
							SKEL();
							skelcount--;
						}
						int rshift=0;
						int dy =  regbuf[FRAME_PIXELS+da];
						int dx =  regbuf[da] + regbuf[FRAME_PIXELS*2+dzv];
						if(dx < 320) {
							rshift=320*dy+dx;
						}
						len += sprintf(dframe->buf+len,",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d",x,y,dzv,depth_ref[da],run_s[posx],run_y[posx],depth_to_mm[run_sv[posx]],depth_ref[320*run_y[posx] + run_s[posx]],run_e[posX],run_y[posX],depth_to_mm[run_ev[posX]],depth_ref[320*run_y[posX] + run_e[posX]],run_s[posy],run_y[posy],depth_to_mm[run_sv[posy]],depth_ref[320*run_y[posy] + run_s[posy]],run_e[posY],run_y[posY],depth_to_mm[run_ev[posY]],depth_ref[320*run_y[posY] + run_e[posY]],run_z[posz],run_y[posz],depth_to_mm[run_zv[posz]],depth_ref[320*run_y[posz] + run_z[posz]],run_Z[posZ],run_y[posZ],depth_to_mm[run_Zv[posZ]],depth_ref[320*run_y[posZ] + run_Z[posZ]],l,l_runs[label],(uint32_t)l_vrun[label]/l_runs[label],rshift);
					}
				}
				(dframe->buf)[len++]=']';
				(dframe->buf)[len++]='\n';
				(dframe->buf)[len++]='\n';
				memcpy(dframe->buf,buf,sprintf(buf,STR_JSON,len-59));
				dframe->l = len;
				dframe->t = dstamp;
				g_mutex_lock(net_mutex);
				FSWAP(dframe,ndframe,dmiss,dframes,dentries);
			}else{
				FDUMP(ndframe,dmiss,dframes,dentries);
			}
			if(encode) {
				FSWAP(gframe,ngframe,gmiss,gframes,gentries);
			} else {
				FDUMP(ngframe,gmiss,gframes,gentries);
			}
			if(dmap) {
				FSWAP(rframe,nrframe,rmiss,rframes,rentries);
			} else {
				FDUMP(nrframe,rmiss,rframes,rentries);
			}
			g_mutex_unlock(net_mutex);
		} while(0);
	}
	return 0;
}

void video_cb(freenect_device *dev, void *raw_buf, uint32_t timestamp)
{
	g_mutex_lock(video_mutex);
	vstamp=timestamp;
	if (vwait<2) {
		vwait++;
		freenect_set_video_buffer(f_dev,video_raw[(vcb+vwait)%3]);
		g_cond_signal(video_cond);
	}
	g_mutex_unlock(video_mutex);
}
void *video_thread()
{
	header_t *vhead,*bhead;
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	static uint16_t len,encode,bmap;
	static int x,y;
	static frame_t *vframe,*bframe,*tframe;
	static char buf[64];
	static uint8_t *vraw;
	HSET(vhead,STR_JPEG_S,STR_JPEG_M,2);
	HSET(bhead,STR_BMP,STR_BMP,4);
	LIST_HEAD(, frame_t) vframes;
	LIST_HEAD(, frame_t) bframes;
	LIST_INIT(&vframes);
	LIST_INIT(&bframes);
	memset(&jerr, 0, sizeof(jerr));
	JSET(cinfo,JCS_RGB,3);
	while(1) {
		g_mutex_lock(video_mutex);
		while(!vwait) g_cond_wait(video_cond, video_mutex);
		vraw=video_raw[vcb%3];
		g_mutex_unlock(video_mutex);
		do {
			g_mutex_lock(net_mutex);
			IF_NOT_FRAME(vframe,vframes,vhead,ventries,frmax,vmiss) {
				encode=0;
			}else{
				encode=1;
			}
			IF_NOT_FRAME(bframe,bframes,bhead,bentries,umax,bmiss) {
				bmap=0;
			}else{
				bmap=1;
			}
			if(!(bmap || encode)){
				FDUMP(nvframe,vmiss,vframes,ventries);
				FDUMP(nbframe,bmiss,bframes,bentries);
				g_mutex_unlock(net_mutex);
				g_mutex_lock(video_mutex);
				vcb++;
				vwait--;
				g_mutex_unlock(video_mutex);
				break;
			}
			g_mutex_unlock(net_mutex);
			if(encode){
				if(!vframe) {
					CFRAME(vframe,vhead,32768);
				}
				jpeg_set_frame(&cinfo, vframe->buf+53,&len);
				jpeg_start_compress( &cinfo, TRUE );
			}
			if(bmap){
				if(!bframe) {
					CFRAME(bframe,bhead,230400);
					bframe->l=230400;
				}
			}
			for(y=0;y!=240;vraw += 640,y++){
					uint8_t *prgb = bmap ? (uint8_t *)(bframe->buf)+y*720:rgb;
					for(x=320;x--;prgb+=3,vraw+=2){
						prgb[0]=vraw[1];
						prgb[1]=vraw[0];
						prgb[2]=vraw[640];
					}
					if(encode){
						unsigned char *jrgb=(unsigned char*)prgb-960;
						jpeg_write_scanlines(&cinfo,&jrgb, TRUE);
						jrgb=(unsigned char*)prgb;
					}
			}
			g_mutex_lock(video_mutex);
			vcb++;
			vwait--;
			g_mutex_unlock(video_mutex);
			if(encode){
				jpeg_finish_compress(&cinfo);
				memcpy(vframe->buf,buf,sprintf(buf,STR_JPEG,len));
				vframe->l=len+53;
				vframe->t=vstamp;
				g_mutex_lock(net_mutex);
				FSWAP(vframe,nvframe,vmiss,vframes,ventries);
				g_mutex_unlock(net_mutex);
			}else {
				FDUMP(nvframe,vmiss,vframes,ventries);
			}
			if(bmap){
				bframe->t=vstamp;
				g_mutex_lock(net_mutex);
				FSWAP(bframe,nbframe,bmiss,bframes,bentries);
				g_mutex_unlock(net_mutex);
			}else {
				FDUMP(nbframe,bmiss,bframes,bentries);
			}
		} while(0);
	}
	return 0;
}


int main(int argc, char **argv)
{
#if defined WIN32
	WSADATA wsa_data;
	WSAStartup(MAKEWORD(2,2), &wsa_data);
#endif
	static freenect_registration reg;
	char *secret,*serial, hash[33],*fbuf;
	unsigned char *mbuf, mtemp[16];
	MD5_CTX *md5;
	f_dev=NULL;
	static IOVEC sdata[2];
	FILE *fp = NULL;
	fd_set rd,wr,master,streaming;
	struct sockaddr_storage clientaddr;
	socklen_t addrsize;
	struct timeval timeout;
	struct sockaddr_in listeneraddr;
	in_addr_t admin_addr;
	static DWORD stmp;
	static int i,ni,kick,admin,itmp,yes,camera,video,sec;
	static SOCKET listener,newfd,fdmax;
	static char *targ,*oarg,*admin_org,buf[4096];
	static client_t *client,tclient;
	static origin_t *origin,torigin;
	static host_t *host,thost;
	static header_t *head403,*head404,*head503,*headreg;
	static frame_t *frame404,*frame403,*frame503,*framereg;
	int32_t c1,c2,cc,ncbuf[128],cbuf[128];
	uint16_t lport,cmax,admin_org_len,coffset;
	uint32_t ncount,count;
	size_t stacksize=65536;
	cc = 1;
	g_type_init();
	g_thread_init(NULL);
	depth_mutex=g_mutex_new();
	video_mutex=g_mutex_new();
	net_mutex=g_mutex_new();
	conf_mutex=g_mutex_new();
	dev_mutex=g_mutex_new();
	depth_cond=g_cond_new();
	video_cond=g_cond_new();
	LIST_HEAD(, client_t) clients;
	LIST_HEAD(, client_t) dclients;
	LIST_HEAD(, client_t) vclients;
	LIST_HEAD(, client_t) gclients;
	LIST_HEAD(, client_t) rclients;
	LIST_HEAD(, client_t) bclients;
	LIST_HEAD(, host_t) hosts;
	LIST_HEAD(, host_t) allhosts;
	LIST_HEAD(, origin_t) origins;
	LIST_INIT(&clients);
	LIST_INIT(&dclients);
	LIST_INIT(&vclients);
	LIST_INIT(&gclients);
	LIST_INIT(&rclients);
	LIST_INIT(&bclients);
	LIST_INIT(&origins);
	LIST_INIT(&hosts);
	LIST_INIT(&allhosts);
	HSET(head404,STR_404,STR_404,10);
	HSET(head403,STR_403,STR_403,10);
	HSET(head503,STR_503,STR_503,10);
	HSET(headreg,STR_REG,STR_REG,5);
	CFRAME(frame404,head404,256);
	CFRAME(frame403,head403,256);
	CFRAME(frame503,head503,256);
	CFRAME(framereg,head503,sizeof(regbuf));
	frame404->l=sprintf(frame404->buf,"%s",STR_404B);
	frame403->l=sprintf(frame403->buf,"%s",STR_403B);
	frame503->l=sprintf(frame503->buf,"%s",STR_503B);
	framereg->l=sizeof(regbuf);
	active=0;
	admin_addr = 0;
	lport = 6661;
	nrframe = ndframe = ngframe = nvframe = nbframe =  NULL;
	yes =   video =  sec = 1;
	depth_to_mm[0] = depth_to_mm[2047] = listener   = dwait = vwait = camera = rmiss = dmiss = vmiss = gmiss  = admin_org_len = coffset = ncount = 0;
	lfile = sfile = ofile = ifile = rfile = secret = serial = admin_org = NULL;
	mbuf=NULL;
	cmax=1024;
	quality = 75;
	frmax = 30;
	umax = 3;
	md5=(MD5_CTX *)calloc(1,sizeof(MD5_CTX));
	FD_ZERO(&streaming);
	FD_ZERO(&master);
	sig_register();
	memset(depth_ref,0,FRAME_PIXELS*2);
	memset(depth,0,320*2);
	memset(l_count,0,(FRAME_PIXELS/2+1)*sizeof(uint32_t));
	listeneraddr.sin_addr.s_addr = INADDR_ANY;
	memset(cbuf,0,512);
	memset(ncbuf,0,512);
	cbuf['X'] = 320;
	cbuf['Y'] = 240;
	cbuf['Z'] = 1350;
	cbuf['c'] = 512;
	cbuf['a'] = 32;
	cbuf['g'] = 1;
	cbuf['s'] = 0;
	cbuf[0] = cc;
	while ((c1 = getopt(argc, argv, "p:z:Z:x:X:y:Y:s:o:O:i:I:a:c:C:r:l:j:e:d:b:k:f:u:g:t:q:nhv")) != -1) {
		CSWITCH(c1,optarg);
		RSWITCH(c1,optarg);
		NSWITCH(c1);
	}
	if(!quality) video=0;
	if(umax && umax < 3) umax=3;
	if(secret) {
		HASH(cc);
	}
	memcpy(ncbuf,cbuf,512);
	memcpy(gbuf,cbuf,512);
	gbuf[0] = cc;
	if (freenect_init(&f_ctx, NULL) < 0) FAIL("Could not initialize libfreenect, aborting")
	freenect_set_log_level(f_ctx, FREENECT_LOG_FATAL);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
	if(serial)
		if(!strncmp(serial,"list",4)) {
			struct freenect_device_attributes* attrs;
			if(freenect_list_device_attributes(f_ctx, &attrs) > 0) {
				while(attrs) {
					printf("%s\n",attrs->camera_serial);
					attrs=attrs->next;
				}
				return 0;
			} else {
				FAIL("Could not find any connected kinects")
			}
		}
	if(!g_thread_create_full((GThreadFunc) &depth_thread,NULL,stacksize,FALSE,TRUE,G_THREAD_PRIORITY_NORMAL,NULL)) FAIL("Could not initialize depth thread, aborting");
	if(video) if(!g_thread_create_full((GThreadFunc) &video_thread,NULL,stacksize,FALSE,TRUE,G_THREAD_PRIORITY_NORMAL,NULL))  FAIL("Could not initialize video thread, aborting");
	INITSOCKET(listener);
	NONBLOCKING(listener);
	addrsize = sizeof clientaddr;
	listeneraddr.sin_family = AF_INET;
	listeneraddr.sin_port = htons(lport);
	memset(&(listeneraddr.sin_zero), '\0', 8);
	if(bind(listener, (struct sockaddr *)&listeneraddr, sizeof(listeneraddr)) == SOCKET_ERROR) FAIL("Bind failed, aborting")
		if(listen(listener, 50) == SOCKET_ERROR) FAIL("Listen failed, aborting")
			FD_SET(listener,&master);
	fdmax=listener;
	while (!die) {
		if(active)
		 if(freenect_process_events(f_ctx) < 0){
			  g_mutex_lock(dev_mutex);
			  active=0;
			  g_mutex_unlock(dev_mutex);
		 }
		if(!active) {
			if(f_dev){
				freenect_close_device(f_dev);
				f_dev=NULL;
			}
			itmp = serial ? freenect_open_device_by_camera_serial(f_ctx, &f_dev, serial):freenect_open_device(f_ctx, &f_dev, camera);
			if(itmp >= 0){
				freenect_set_depth_buffer(f_dev,depth_raw[0]);
				freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT_PACKED));
				freenect_set_depth_callback(f_dev, depth_cb);
				freenect_set_video_buffer(f_dev,video_raw[0]);
				freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_BAYER));
				freenect_set_video_callback(f_dev, video_cb);
				freenect_start_depth(f_dev);
				if(video) freenect_start_video(f_dev);
				reg=freenect_copy_registration(f_dev);
				depth_to_mm[0]=depth_to_raw[0]=0;
				for(i=1; i!=1054; i++) depth_to_mm[i] = reg.raw_to_mm_shift[i];
				for(; i!=2048; i++) depth_to_mm[i] = 0;
				for(ni=0,i=1; i!=10000; i++) {
					while(depth_to_mm[ni]<i) ni++;
					depth_to_raw[i]=ni;
				}
				for(; i!=16384; i++) depth_to_raw[i]=0;
				for(ni=0,i=0,itmp=320; ni!=FRAME_PIXELS; ni++,i+=2) {
					regbuf[ni] = reg.registration_table[i][0] >> 9;
					regbuf[FRAME_PIXELS+ni] = reg.registration_table[i][1] >> 1;
					if(!(--itmp)){
						itmp=320;
						i+=640;
					}
				}
				for(ni=FRAME_PIXELS*2,i=0; ni!=FRAME_PIXELS*2+10000; ni++,i++) {
					regbuf[ni] = reg.depth_to_rgb_shift[i]>>9;
				}
				freenect_destroy_registration(&reg);
				
				g_mutex_lock(conf_mutex);
				memcpy(gbuf,cbuf,512);
				gbuf[0] = cc;
				g_mutex_unlock(conf_mutex);
				
				g_mutex_lock(dev_mutex);
				active=1;
				g_mutex_unlock(dev_mutex);
				led = LED_OFF;
			}
			timeout.tv_sec = 1;
			timeout.tv_usec = 0;
			select(0,NULL,NULL,NULL,&timeout);
		}
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		g_mutex_lock(depth_mutex);
		count=gcount;
		g_mutex_unlock(depth_mutex);
		if(coffset && count >= ncount){
			ncount=0;
			LIST_FOREACH(client, &clients, entries) {
				if(client->f && count >= client->n){
					FD_CLR(client->s,&master);
					DFRAME();
					FD_CLR(client->s,&streaming);
					closesocket(client->s);
					if(!(--(client->h->c))) {
						LIST_REMOVE(client->h,all);
						free(client->h);
					}
					client->h=NULL;
					tclient.entries.le_next = client->entries.le_next; 
					LIST_REMOVE(client,entries);
					free(client);
					client=&tclient;
				}else{
					if(!ncount) ncount = client->n; else ncount=MIN(ncount,client->n);
				}
			}
		}
		g_mutex_lock(net_mutex);
		SEL(dclients,dmiss,dentries,ndframe,frmod);
		SEL(gclients,gmiss,gentries,ngframe,frmod);
		SEL(rclients,rmiss,rentries,nrframe,umod);
		SEL(vclients,vmiss,ventries,nvframe,frmod);
		SEL(bclients,bmiss,bentries,nbframe,umod);
		g_mutex_unlock(net_mutex);
		rd=master;
		wr=streaming;
		itmp=select(fdmax+1, &rd, &wr, NULL,&timeout);
		if(itmp > 0) {
			fdmax=listener;
			LIST_FOREACH(client, &clients, entries) {
				kick=0;
				admin=sec;
				oarg=NULL;
				do {
					if(FD_ISSET(client->s,&rd)) {
						if((itmp = recv(client->s,buf,4094,0))>6) {
							if(!f_dev){
								FRAME40(frame503);
								break;
							}else if(client->f) {
								kick=1;
								break;
							} else {
								buf[itmp]='\0';
								c1 = buf[5];
								c2 = buf[6];
								targ=strtok(buf+5," \n\r");
								if(!LIST_EMPTY(&origins) || admin_org) {
									if(!LIST_EMPTY(&origins)) kick=1;
									oarg=strtok(NULL," \n\r");
									while(oarg) {
										if(!strncmp(targ,"Origin:",7)) {
											oarg=strtok(NULL," \r\n");
											if(oarg) {
												if(admin_org && strlen(oarg) == admin_org_len) {
													if(!strncmp(oarg,admin_org,admin_org_len)) {
														if(admin_addr && (client->a == admin_addr)) admin=1;
														kick=0;
														break;
													}
												}
												if(!admin) LIST_FOREACH(origin, &origins, entries) if(!strncmp(origin->n,oarg,origin->l)) {
														kick=0;
														break;
													}
												break;
											}
										}
										targ=strtok(NULL," \r\n");
									}
								} else if(admin_addr && (client->a == admin_addr)) admin=1;
								if(targ && admin) {
									oarg=strchr(targ,'?');
									if(oarg) {
										oarg=strtok(oarg,"?&");
										memcpy(cbuf,ncbuf,512);
										if(secret) admin=0;
										if(oarg) {
											do {
												if(secret && strlen(oarg)==34) {
													if(oarg[0]=='s' && !strncmp(oarg+2,hash,32)) admin=1;
												} else if(strlen(oarg) > 2 && oarg[1]=='=') {
													RSWITCH(oarg[0],oarg+2);
												}
											} while((oarg=strtok(NULL,"?&")));
											if(admin) {
												do {
													cc++;
												} while(!cc);
												cbuf[0] = cc;
												if(secret) {
													HASH(cc);
												}
												g_mutex_lock(conf_mutex);
												memcpy(gbuf,cbuf,512);
												g_mutex_unlock(conf_mutex);
												if(cbuf['o']) NSWITCH('o');
												if(cbuf['i']) NSWITCH('i');
												memcpy(ncbuf,cbuf,512);
												ncbuf['b']=0;
												ncbuf['d']=0;
												ncbuf['o']=0;
												ncbuf['i']=0;
												ncbuf['r']=0;
											} else {
												kick=1;
											}
										}
									}
								}else if(targ) if(strchr(targ,'?')) kick=1;
								if(kick) {
									FRAME40(frame403);
									kick=0;
									break;
								}
							}
							c2 -= '0';
							if(c2 < 0 || c2 > 9) c2=0;
							switch(c1) {
							case '1':
								if(quality) {FCHK(gclients,gentries);} else {FRAME40(frame404);}
								break;
							case '2':
								if(video) {FCHK(vclients,ventries);} else {FRAME40(frame404);}
								break;
							case '3':
								if(umax) {FCHK(rclients,rentries);} else {FRAME40(frame404);}
								c2=0;
								break;
							case '4':
								if(umax) {FCHK(bclients,bentries);} else {FRAME40(frame404);}
								c2=0;
								break;
							case '5':
								if(umax) {FRAME40(framereg);}else{FRAME40(frame404);}
								c2=0;
								break;
							default:
								FCHK(dclients,dentries);
								break;
							}
							client->m=c2;
						} else {
							kick = 1;
						}
					}else if(FD_ISSET(client->s,&wr)) {
						switch(client->f->h->t) {
						case 1:
							STREAM(gclients,gentries,0,client->f->h->ml - 11);
						case 2:
							STREAM(vclients,ventries,0,client->f->h->ml - 11);
						case 3:
						case 4:
						case 5:
							STREAM(rclients,rentries,0,0);
						default:
							STREAM(dclients,dentries,59,client->f->h->ml - 5);
						}
					} else {
						fdmax=MAX(client->s,fdmax);
					}
				} while(0);
				if(kick) {
					FD_CLR(client->s,&master);
					if(client->f) {
						DFRAME();
						FD_CLR(client->s,&streaming);
					}
					closesocket(client->s);
					if(!(--(client->h->c))) {
						LIST_REMOVE(client->h,all);
						free(client->h);
					}
					client->h=NULL;
					tclient.entries.le_next = client->entries.le_next; 
					LIST_REMOVE(client,entries);
					free(client);
					client=&tclient;
				}
			}
			if(FD_ISSET(listener,&rd)) {
				if((newfd = accept(listener, (struct sockaddr *)&clientaddr, &addrsize)) != SOCKET_ERROR ) {
					kick=0;
					if(!LIST_EMPTY(&hosts)) {
						kick=1;
						LIST_FOREACH(host, &hosts, entries) {
							if(host->a==((struct sockaddr_in *)&clientaddr)->sin_addr.s_addr  ) {
								kick=0;
								break;
							}
						}
					}
					if(!kick) {
						kick=1;
						LIST_FOREACH(host, &allhosts, all) {
							if(host->a==((struct sockaddr_in *)&clientaddr)->sin_addr.s_addr) {
								kick=0;
								break;
							}
						}
						if(kick) {
							host=((host_t *)calloc(1, sizeof(*host)));
							host->a = ((struct sockaddr_in *)&clientaddr)->sin_addr.s_addr;
							LIST_INSERT_HEAD(&allhosts, host, all);
							kick=0;
						}
						if(host->c > cmax) {
							closesocket(newfd);
						} else {
							NONBLOCKING(newfd);
							setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, (const char*) &yes, sizeof(int));
							client = (client_t *)calloc(1, sizeof(client_t));
							client->s=newfd;
							client->h=host;
							client->h->c++;
							client->b=0;
							client->f=NULL;
							client->m=0;
							client->c=0;
							client->t=0;
							fdmax=MAX(newfd,fdmax);
							FD_SET(newfd,&master);
							LIST_INSERT_HEAD(&clients, client, entries);
						}
					} else closesocket(newfd);
				}
			}
		}
	}
	if(f_dev){
		freenect_set_led(f_dev,LED_RED);
		freenect_stop_depth(f_dev);
		freenect_stop_video(f_dev);
		freenect_close_device(f_dev);
	}
	freenect_shutdown(f_ctx);
	closesocket(listener);
#if defined WIN32
	WSACleanup();
#endif
	return 0;
}
