/*
 * Target-side benchmark for rv32sim (bench only, never in firmware).
 * Reads interleaved stereo s16 PCM (48 kHz) from the simulator input,
 * encodes every 10 ms frame with two mono LC3 encoders (100 bytes/frame,
 * exactly as the firmware does), writes the frames to the simulator output.
 * Only the lc3_encode() calls are inside the profiled region.
 */
#include <stdint.h>
#include <lc3.h>

static long sys3(long n, long a, long b, long c)
{
    register long a0 asm("a0") = a, a1 asm("a1") = b, a2 asm("a2") = c;
    register long a7 asm("a7") = n;
    asm volatile ("ecall" : "+r"(a0) : "r"(a1), "r"(a2), "r"(a7) : "memory");
    return a0;
}

#define NS 480
#define NBYTES 100

static lc3_encoder_mem_48k_t enc_mem[2];
static int16_t pcm[2 * NS];
static uint8_t out[2 * NBYTES];

int main(void)
{
    lc3_encoder_t enc[2];
    for (int ch = 0; ch < 2; ch++)
        enc[ch] = lc3_setup_encoder(10000, 48000, 48000, &enc_mem[ch]);

    while (sys3(63, 0, (long)pcm, sizeof(pcm)) == sizeof(pcm)) {
        sys3(500, 1, 0, 0);
        for (int ch = 0; ch < 2; ch++)
            lc3_encode(enc[ch], LC3_PCM_FORMAT_S16, pcm + ch, 2,
                       NBYTES, out + ch * NBYTES);
        sys3(500, 0, 0, 0);
        sys3(64, 3, (long)out, sizeof(out));
    }
    return 0;
}
