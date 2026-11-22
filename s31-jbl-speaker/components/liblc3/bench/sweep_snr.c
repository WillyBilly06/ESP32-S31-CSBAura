#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "lc3.h"
int main(int argc,char**argv){
  int nbytes = argc>1? atoi(argv[1]) : 100;
  const int fs=48000, N=480, frames=1000; /* 10 s */
  int16_t *x = calloc(N*(frames+2),2), *y = calloc(N*(frames+2),2);
  float ph=0,f=20; float k=powf(1000.f,1.f/(10.f*fs));
  for(int n=0;n<N*frames;n++){ x[n]=(int16_t)(0.5f*sinf(6.2831853f*ph)*32767.f); ph+=f/fs; if(ph>=1)ph-=1; f*=k; }
  void *em=malloc(lc3_encoder_size(10000,fs)), *dm=malloc(lc3_decoder_size(10000,fs));
  lc3_encoder_t e=lc3_setup_encoder(10000,fs,fs,em); lc3_decoder_t d=lc3_setup_decoder(10000,fs,fs,dm);
  uint8_t buf[400]; int D=lc3_delay_samples(10000,fs);
  for(int i=0;i<frames;i++){ lc3_encode(e,LC3_PCM_FORMAT_S16,x+i*N,1,nbytes,buf); lc3_decode(d,buf,nbytes,LC3_PCM_FORMAT_S16,y+i*N,1); }
  /* per-2s SNR like the device log */
  for(int w=0; w<5; w++){ double s=0,er=0, worst=999; int bad=0;
    for(int i=w*200;i<(w+1)*200;i++){ double fs_=0,fe=0;
      for(int n=0;n<N;n++){ int idx=i*N+n; if(idx<D) continue; double xv=x[idx-D], dv=y[idx]-xv; fs_+=xv*xv; fe+=dv*dv; }
      if(fs_>480.0*1e4){ double snr=10*log10(fs_/(fe+1)); if(snr<worst)worst=snr; if(snr<10)bad++; s+=fs_; er+=fe; } }
    printf("%3d B: window %d (%.0f-%.0f Hz): SNR avg %.1f dB, worst frame %.1f dB, <10 dB: %d\n", nbytes, w, 20*pow(1000,w*0.2), 20*pow(1000,(w+1)*0.2), 10*log10(s/(er+1)), worst, bad); }
  return 0; }
