import numpy as np, math, random
TAPS,HALF,PH=32,16,256
fc=20000/44100
kern=np.zeros((PH+1,TAPS))
for p in range(PH+1):
    fr=p/PH
    t=np.arange(TAPS)-(HALF-1)-fr
    x=2*fc*t
    s=np.where(np.abs(x)<1e-9,1.0,np.sin(np.pi*x)/(np.pi*x+1e-30))
    w=0.42+0.5*np.cos(np.pi*t/HALF)+0.08*np.cos(2*np.pi*t/HALF); w[np.abs(t)>=HALF]=0
    k=2*fc*s*w; kern[p]=k/k.sum()
fs_in=44100; N=fs_in*2
n=np.arange(N); sig=0.4*np.sin(2*np.pi*1000*n/fs_in)+0.4*np.sin(2*np.pi*10000*n/fs_in)
hist=list(np.zeros(TAPS)); pos=HALF-1.0; out=[]
step=44100/48000; i0=0
random.seed(1)
while i0<N:
    m=random.randint(100,1200); hist+=list(sig[i0:i0+m]); i0+=m
    while int(pos)+HALF<len(hist):
        i=int(pos); ph=(pos-i)*PH; p0=int(ph); t=ph-p0
        c=kern[p0]+(kern[p0+1]-kern[p0])*t
        out.append(float(np.dot(c,hist[i-(HALF-1):i-(HALF-1)+TAPS]))); pos+=step
    kf=int(pos)-(HALF-1)
    if kf>0: hist=hist[kf:]; pos-=kf
y=np.array(out[2000:2000+65536]); win=np.hanning(len(y)); Y=np.abs(np.fft.rfft(y*win)); f=np.fft.rfftfreq(len(y),1/48000)
db=20*np.log10(Y/Y.max()+1e-12)
mask=np.ones_like(f,bool)
for ft in (1000,10000): mask&=np.abs(f-ft)>150
print("sinc: worst spur vs tone: %.1f dB at %.0f Hz"%(db[mask].max(), f[mask][np.argmax(db[mask])]))
# old linear for comparison
lin=[];pos=0.0
while pos<N-1:
    i=int(pos);fr=pos-i; lin.append(sig[i]+(sig[i+1]-sig[i])*fr); pos+=step
y=np.array(lin[2000:2000+65536]); Y=np.abs(np.fft.rfft(y*win)); db=20*np.log10(Y/Y.max()+1e-12)
print("linear: worst spur vs tone: %.1f dB at %.0f Hz"%(db[mask].max(), f[mask][np.argmax(db[mask])]))
