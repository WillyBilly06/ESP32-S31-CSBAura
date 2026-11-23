/*
 * rv32sim -- tiny RV32IMFC (+Zicsr) instruction-set simulator / profiler.
 *
 * Purpose: run liblc3 compiled by the real ESP-IDF RISC-V toolchain (same
 * flags as the firmware) and count dynamic instructions per function, plus a
 * rough in-order pipeline cycle estimate.  Host only, never part of firmware.
 *
 *   rv32sim prog.elf [in.bin [out.bin]]
 *
 * Syscalls (ecall, a7):
 *   63  read(fd, buf, len)    -> reads from `in.bin`
 *   64  write(fd, buf, len)   -> fd 1: stdout, fd 3: `out.bin`
 *   93  exit(code)
 *   500 profile on (a0 = 1) / off (a0 = 0)
 *
 * Cycle model (estimate only): single issue, in-order, scoreboard on
 * register results.  Latencies are guesses for an ESP32-P4/S31 class core:
 *   ALU 1, load 2, mul 2, div 18, fadd/fmul/fma 3, fcvt/fmv/fcmp 2,
 *   fdiv 10, fsqrt 10, taken branch / jump +1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MEM_BASE 0x00100000u
#define MEM_SIZE (16u << 20)

static uint8_t *mem;
static uint32_t x[32];
static union { uint32_t u; float f; } f[32];
static uint32_t pc;

static int prof_on;
static uint64_t *pc_count;      /* per halfword */
static uint64_t *pc_cycles;
static uint64_t n_insn, n_cycles, cyc;
static uint64_t n_fdiv, n_fsqrt, n_idiv;   /* divisions in profiled region */
static uint8_t *pc_div;                      /* per halfword: 1 fdiv 2 fsqrt 3 idiv */
static uint64_t ready_x[32], ready_f[32];

static FILE *fin, *fout;

static void die(const char *m) { fprintf(stderr, "rv32sim: %s (pc=%08x)\n", m, pc); exit(2); }

static inline uint8_t *addr(uint32_t a, int n)
{
    if (a < MEM_BASE || a + n > MEM_BASE + MEM_SIZE) {
        fprintf(stderr, "bad access %08x\n", a); die("segv");
    }
    return mem + (a - MEM_BASE);
}

static inline uint32_t ld32(uint32_t a) { uint32_t v; memcpy(&v, addr(a, 4), 4); return v; }
static inline uint16_t ld16(uint32_t a) { uint16_t v; memcpy(&v, addr(a, 2), 2); return v; }
static inline uint8_t  ld8 (uint32_t a) { return *addr(a, 1); }
static inline void st32(uint32_t a, uint32_t v) { memcpy(addr(a, 4), &v, 4); }
static inline void st16(uint32_t a, uint16_t v) { memcpy(addr(a, 2), &v, 2); }
static inline void st8 (uint32_t a, uint8_t  v) { *addr(a, 1) = v; }

/* ---- ELF loading & symbols ---- */

typedef struct { uint32_t addr, size; char *name; uint64_t cnt, cyc; } sym_t;
static sym_t *syms; static int nsyms;

static int sym_cmp(const void *a, const void *b)
{ const sym_t *p = a, *q = b; return p->addr < q->addr ? -1 : p->addr > q->addr; }

static uint32_t load_elf(const char *path, uint32_t *sp_init)
{
    FILE *fp = fopen(path, "rb"); if (!fp) die("open elf");
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    uint8_t *b = malloc(sz); fread(b, 1, sz, fp); fclose(fp);

    uint32_t entry; memcpy(&entry, b + 24, 4);
    uint32_t phoff, shoff; memcpy(&phoff, b + 28, 4); memcpy(&shoff, b + 32, 4);
    uint16_t phentsize, phnum, shentsize, shnum;
    memcpy(&phentsize, b + 42, 2); memcpy(&phnum, b + 44, 2);
    memcpy(&shentsize, b + 46, 2); memcpy(&shnum, b + 48, 2);

    for (int i = 0; i < phnum; i++) {
        uint8_t *ph = b + phoff + i * phentsize;
        uint32_t type, off, vaddr, filesz, memsz;
        memcpy(&type, ph, 4); memcpy(&off, ph + 4, 4); memcpy(&vaddr, ph + 8, 4);
        memcpy(&filesz, ph + 16, 4); memcpy(&memsz, ph + 20, 4);
        if (type != 1) continue;
        memset(addr(vaddr, memsz), 0, memsz);
        memcpy(addr(vaddr, filesz), b + off, filesz);
    }

    /* symbols */
    for (int i = 0; i < shnum; i++) {
        uint8_t *sh = b + shoff + i * shentsize;
        uint32_t type, off, size, link, entsize;
        memcpy(&type, sh + 4, 4); memcpy(&off, sh + 16, 4); memcpy(&size, sh + 20, 4);
        memcpy(&link, sh + 24, 4); memcpy(&entsize, sh + 36, 4);
        if (type != 2) continue; /* SHT_SYMTAB */
        uint8_t *strsh = b + shoff + link * shentsize;
        uint32_t stroff; memcpy(&stroff, strsh + 16, 4);
        int n = size / entsize;
        syms = calloc(n, sizeof(*syms));
        for (int k = 0; k < n; k++) {
            uint8_t *s = b + off + k * entsize;
            uint32_t name, value, ssize; memcpy(&name, s, 4);
            memcpy(&value, s + 4, 4); memcpy(&ssize, s + 8, 4);
            char *nm = (char *)b + stroff + name;
            if ((s[12] & 0xf) == 2 && ssize) {           /* STT_FUNC */
                syms[nsyms].addr = value; syms[nsyms].size = ssize;
                syms[nsyms].name = strdup(nm); nsyms++;
            }
            if (!strcmp(nm, "__stack_top")) *sp_init = value;
        }
    }
    qsort(syms, nsyms, sizeof(*syms), sym_cmp);
    free(b);
    return entry;
}

/* ---- FP helpers ---- */

static float rnd(float v, int rm)
{
    switch (rm) {
    case 0: case 7: return nearbyintf(v);  /* RNE (host default) */
    case 1: return truncf(v);
    case 2: return floorf(v);
    case 3: return ceilf(v);
    case 4: return roundf(v);
    }
    die("rm"); return 0;
}

static int32_t fcvt_w(float v, int rm)
{
    if (isnan(v)) return INT32_MAX;
    float r = rnd(v, rm);
    if (r >= 2147483648.f) return INT32_MAX;
    if (r < -2147483648.f) return INT32_MIN;
    return (int32_t)r;
}

static uint32_t fcvt_wu(float v, int rm)
{
    if (isnan(v)) return UINT32_MAX;
    float r = rnd(v, rm);
    if (r >= 4294967296.f) return UINT32_MAX;
    if (r <= 0) return 0;
    return (uint32_t)r;
}

static float fminmax(float a, float b, int max)
{
    if (isnan(a)) return b;
    if (isnan(b)) return a;
    if (a == 0 && b == 0) {
        int sa = signbit(a), sb = signbit(b);
        return max ? (sa ? b : a) : (sa ? a : b);
    }
    return max ? (a > b ? a : b) : (a < b ? a : b);
}

static uint32_t fclass(float v)
{
    union { float f; uint32_t u; } u = { v };
    int s = u.u >> 31;
    if (isinf(v)) return s ? 1 : 1 << 7;
    if (isnan(v)) return (u.u & 0x400000) ? 1 << 9 : 1 << 8;
    if (v == 0) return s ? 1 << 3 : 1 << 4;
    if (fpclassify(v) == FP_SUBNORMAL) return s ? 1 << 2 : 1 << 5;
    return s ? 1 << 1 : 1 << 6;
}

/* ---- Syscalls ---- */

static void do_ecall(void)
{
    uint32_t a0 = x[10], a1 = x[11], a2 = x[12], a7 = x[17];
    switch (a7) {
    case 63: x[10] = fin ? (uint32_t)fread(addr(a1, a2), 1, a2, fin) : 0; break;
    case 64:
        if (a0 == 3 && fout) fwrite(addr(a1, a2), 1, a2, fout);
        else fwrite(addr(a1, a2), 1, a2, stdout);
        x[10] = a2; break;
    case 93: {
        fflush(stdout);
        if (fout) fclose(fout);
        /* report */
        uint64_t tot = 0, totc = 0;
        for (uint32_t a = 0; a < MEM_SIZE / 2; a++) { tot += pc_count[a]; totc += pc_cycles[a]; }
        for (int i = 0; i < nsyms; i++) {
            for (uint32_t a = syms[i].addr; a < syms[i].addr + syms[i].size; a += 2) {
                syms[i].cnt += pc_count[(a - MEM_BASE) / 2];
                syms[i].cyc += pc_cycles[(a - MEM_BASE) / 2];
            }
        }
        fprintf(stderr, "\n=== profile: %llu insns, %llu est. cycles (profiled region) ===\n",
            (unsigned long long)tot, (unsigned long long)totc);
        fprintf(stderr, "=== divisions: %llu fdiv.s, %llu fsqrt.s, %llu div/rem ===\n",
            (unsigned long long)n_fdiv, (unsigned long long)n_fsqrt, (unsigned long long)n_idiv);
        for (int i = 0; i < nsyms; i++) {
            uint64_t d[4] = { 0 };
            for (uint32_t a = syms[i].addr; a < syms[i].addr + syms[i].size; a += 2) {
                uint32_t j = (a - MEM_BASE) / 2;
                if (pc_div[j]) d[pc_div[j]] += pc_count[j];
            }
            if (d[1] | d[2] | d[3])
                fprintf(stderr, "    %8llu fdiv %8llu fsqrt %8llu idiv  %s\n",
                    (unsigned long long)d[1], (unsigned long long)d[2],
                    (unsigned long long)d[3], syms[i].name);
        }
        const char *dump = getenv("RVSIM_PCDUMP");
        if (dump) {     /* "pc insn cycles" per executed address, for addr2line */
            FILE *fp = fopen(dump, "w");
            for (uint32_t a = 0; a < MEM_SIZE / 2; a++)
                if (pc_count[a]) fprintf(fp, "%08x %llu %llu\n", MEM_BASE + 2 * a,
                    (unsigned long long)pc_count[a], (unsigned long long)pc_cycles[a]);
            fclose(fp);
        }
        for (;;) {
            int best = -1;
            for (int i = 0; i < nsyms; i++)
                if (syms[i].cnt && (best < 0 || syms[i].cyc > syms[best].cyc)) best = i;
            if (best < 0) break;
            fprintf(stderr, "%12llu insn %12llu cyc %5.1f%%  %s\n",
                (unsigned long long)syms[best].cnt, (unsigned long long)syms[best].cyc,
                100.0 * syms[best].cyc / (totc ? totc : 1), syms[best].name);
            syms[best].cnt = 0;
        }
        exit(a0);
    }
    case 500: prof_on = a0; break;
    default: die("unknown ecall");
    }
}

/* ---- CPU ---- */

#define RD   ((ins >> 7) & 31)
#define RS1  ((ins >> 15) & 31)
#define RS2  ((ins >> 20) & 31)
#define RS3  ((ins >> 27) & 31)
#define F3   ((ins >> 12) & 7)
#define F7   (ins >> 25)

static inline int32_t imm_i(uint32_t ins) { return (int32_t)ins >> 20; }
static inline int32_t imm_s(uint32_t ins) { return ((int32_t)ins >> 25 << 5) | ((ins >> 7) & 31); }
static inline int32_t imm_b(uint32_t ins)
{
    return ((int32_t)ins >> 31 << 12) | (((ins >> 7) & 1) << 11) |
           (((ins >> 25) & 63) << 5) | (((ins >> 8) & 15) << 1);
}
static inline int32_t imm_j(uint32_t ins)
{
    return ((int32_t)ins >> 31 << 20) | (ins & 0xff000) |
           (((ins >> 20) & 1) << 11) | (((ins >> 21) & 0x3ff) << 1);
}

/* Expand a compressed instruction into a 32-bit one (RV32C + F) */
static uint32_t expand_c(uint16_t c)
{
    int op = c & 3, f3 = c >> 13;
    int rd = (c >> 7) & 31, rs2 = (c >> 2) & 31;
    int rdp = 8 + ((c >> 2) & 7), rs1p = 8 + ((c >> 7) & 7);

#define I_TYPE(imm, rs1, f3_, rd_, opc) ((((uint32_t)(imm) & 0xfff) << 20) | ((rs1) << 15) | ((f3_) << 12) | ((rd_) << 7) | (opc))
#define S_TYPE(imm, rs2_, rs1, f3_, opc) (((((uint32_t)(imm) >> 5) & 0x7f) << 25) | ((rs2_) << 20) | ((rs1) << 15) | ((f3_) << 12) | (((imm) & 31) << 7) | (opc))
#define R_TYPE(f7, rs2_, rs1, f3_, rd_, opc) (((f7) << 25) | ((rs2_) << 20) | ((rs1) << 15) | ((f3_) << 12) | ((rd_) << 7) | (opc))

    if (op == 0) {
        uint32_t uimm_w = (((c >> 6) & 1) << 2) | (((c >> 10) & 7) << 3) | (((c >> 5) & 1) << 6);
        switch (f3) {
        case 0: { /* c.addi4spn */
            uint32_t nz = (((c >> 6) & 1) << 2) | (((c >> 5) & 1) << 3) |
                          (((c >> 11) & 3) << 4) | (((c >> 7) & 15) << 6);
            if (!nz) die("illegal c");
            return I_TYPE(nz, 2, 0, rdp, 0x13);
        }
        case 2: return I_TYPE(uimm_w, rs1p, 2, rdp, 0x03);           /* c.lw */
        case 3: return I_TYPE(uimm_w, rs1p, 2, rdp, 0x07);           /* c.flw */
        case 6: return S_TYPE(uimm_w, rdp, rs1p, 2, 0x23);           /* c.sw */
        case 7: return S_TYPE(uimm_w, rdp, rs1p, 2, 0x27);           /* c.fsw */
        }
        die("illegal c0");
    }
    if (op == 1) {
        int32_t imm6 = ((int32_t)(((c >> 12) & 1) << 31) >> 26) | ((c >> 2) & 31);
        switch (f3) {
        case 0: return I_TYPE(imm6, rd, 0, rd, 0x13);                /* c.addi */
        case 1: case 5: {                                            /* c.jal / c.j */
            int32_t o = (((c >> 12) & 1) << 11) | (((c >> 11) & 1) << 4) |
                        (((c >> 9) & 3) << 8) | (((c >> 8) & 1) << 10) |
                        (((c >> 7) & 1) << 6) | (((c >> 6) & 1) << 7) |
                        (((c >> 3) & 7) << 1) | (((c >> 2) & 1) << 5);
            o = (o << 20) >> 20;
            uint32_t u = (uint32_t)o;
            uint32_t j = (((u >> 20) & 1) << 31) | (((u >> 1) & 0x3ff) << 21) |
                         (((u >> 11) & 1) << 20) | (((u >> 12) & 0xff) << 12);
            j |= ((uint32_t)(o < 0) * 0xff) << 12;
            return j | ((f3 == 1 ? 1 : 0) << 7) | 0x6f;
        }
        case 2: return I_TYPE(imm6, 0, 0, rd, 0x13);                 /* c.li */
        case 3:
            if (rd == 2) {                                           /* c.addi16sp */
                int32_t n = (((c >> 12) & 1) << 9) | (((c >> 6) & 1) << 4) |
                            (((c >> 5) & 1) << 6) | (((c >> 3) & 3) << 7) |
                            (((c >> 2) & 1) << 5);
                n = (n << 22) >> 22;
                return I_TYPE(n, 2, 0, 2, 0x13);
            }
            return ((uint32_t)(imm6 << 12) & 0xfffff000) | (rd << 7) | 0x37; /* c.lui */
        case 4: {
            int f2 = (c >> 10) & 3;
            int sh = (((c >> 12) & 1) << 5) | ((c >> 2) & 31);
            if (f2 == 0) return R_TYPE(0x00, sh & 31, rs1p, 5, rs1p, 0x13); /* srli */
            if (f2 == 1) return R_TYPE(0x20, sh & 31, rs1p, 5, rs1p, 0x13); /* srai */
            if (f2 == 2) return I_TYPE(imm6, rs1p, 7, rs1p, 0x13);          /* andi */
            int f = (c >> 5) & 3;
            if ((c >> 12) & 1) die("illegal c1 (rv64)");
            static const int f3s[] = { 0, 4, 6, 7 };
            return R_TYPE(f == 0 ? 0x20 : 0, rdp, rs1p, f3s[f], rs1p, 0x33);
        }
        case 6: case 7: {                                            /* c.beqz/bnez */
            int32_t o = (((c >> 12) & 1) << 8) | (((c >> 10) & 3) << 3) |
                        (((c >> 5) & 3) << 6) | (((c >> 3) & 3) << 1) |
                        (((c >> 2) & 1) << 5);
            o = (o << 23) >> 23;
            uint32_t u = (uint32_t)o;
            uint32_t b = (((u >> 12) & 1) << 31) | (((u >> 5) & 63) << 25) |
                         (((u >> 1) & 15) << 8) | (((u >> 11) & 1) << 7);
            return b | (0 << 20) | (rs1p << 15) | ((f3 == 6 ? 0 : 1) << 12) | 0x63;
        }
        }
    }
    if (op == 2) {
        switch (f3) {
        case 0: return R_TYPE(0, rs2, rd, 1, rd, 0x13);              /* c.slli */
        case 2: case 3: {                                            /* c.lwsp / c.flwsp */
            uint32_t o = (((c >> 12) & 1) << 5) | (((c >> 4) & 7) << 2) | (((c >> 2) & 3) << 6);
            return I_TYPE(o, 2, 2, rd, f3 == 2 ? 0x03 : 0x07);
        }
        case 4:
            if (!((c >> 12) & 1)) {
                if (rs2 == 0) return I_TYPE(0, rd, 0, 0, 0x67);      /* c.jr */
                return R_TYPE(0, rs2, 0, 0, rd, 0x33);               /* c.mv */
            }
            if (rs2 == 0) {
                if (rd == 0) return 0x00100073;                      /* c.ebreak */
                return I_TYPE(0, rd, 0, 1, 0x67);                    /* c.jalr */
            }
            return R_TYPE(0, rs2, rd, 0, rd, 0x33);                  /* c.add */
        case 6: case 7: {                                            /* c.swsp / c.fswsp */
            uint32_t o = (((c >> 9) & 15) << 2) | (((c >> 7) & 3) << 6);
            return S_TYPE(o, rs2, 2, 2, f3 == 6 ? 0x23 : 0x27);
        }
        }
    }
    die("illegal compressed");
    return 0;
}

static inline float F(int r) { return f[r].f; }
static inline void SETF(int r, float v) { f[r].f = v; }

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: rv32sim prog.elf [in] [out]\n"); return 1; }
    mem = calloc(1, MEM_SIZE);
    pc_count = calloc(MEM_SIZE / 2, sizeof(uint64_t));
    pc_cycles = calloc(MEM_SIZE / 2, sizeof(uint64_t));
    pc_div = calloc(MEM_SIZE / 2, 1);
    uint32_t sp = MEM_BASE + MEM_SIZE - 16;
    pc = load_elf(argv[1], &sp);
    x[2] = sp;
    if (argc > 2) { fin = fopen(argv[2], "rb"); if (!fin) die("open in"); }
    if (argc > 3) { fout = fopen(argv[3], "wb"); if (!fout) die("open out"); }

    for (;;) {
        uint32_t ins = ld16(pc), len = 4;
        if ((ins & 3) != 3) { ins = expand_c(ins); len = 2; }
        else ins = ld32(pc);

        uint32_t npc = pc + len;
        int opc = ins & 0x7f;

        /* scoreboard: sources */
        uint64_t t0 = cyc, issue = cyc;
        int lat = 1, dst_x = -1, dst_f = -1, taken = 0, is_div = 0;
        int sx1 = -1, sx2 = -1, sf1 = -1, sf2 = -1, sf3 = -1;

        switch (opc) {
        case 0x37: x[RD] = ins & 0xfffff000; dst_x = RD; break;
        case 0x17: x[RD] = pc + (ins & 0xfffff000); dst_x = RD; break;
        case 0x6f: x[RD] = npc; dst_x = RD; npc = pc + imm_j(ins); taken = 1; break;
        case 0x67: { sx1 = RS1; uint32_t t = (x[RS1] + imm_i(ins)) & ~1u; x[RD] = npc; dst_x = RD; npc = t; taken = 1; break; }
        case 0x63: {
            sx1 = RS1; sx2 = RS2;
            uint32_t a = x[RS1], b = x[RS2]; int c = 0;
            switch (F3) {
            case 0: c = a == b; break; case 1: c = a != b; break;
            case 4: c = (int32_t)a < (int32_t)b; break; case 5: c = (int32_t)a >= (int32_t)b; break;
            case 6: c = a < b; break; case 7: c = a >= b; break; default: die("br");
            }
            if (c) { npc = pc + imm_b(ins); taken = 1; }
            break;
        }
        case 0x03: {
            sx1 = RS1; uint32_t a = x[RS1] + imm_i(ins), v;
            switch (F3) {
            case 0: v = (int8_t)ld8(a); break; case 1: v = (int16_t)ld16(a); break;
            case 2: v = ld32(a); break; case 4: v = ld8(a); break; case 5: v = ld16(a); break;
            default: die("ld"); v = 0;
            }
            x[RD] = v; dst_x = RD; lat = 2; break;
        }
        case 0x23: {
            sx1 = RS1; sx2 = RS2; uint32_t a = x[RS1] + imm_s(ins);
            switch (F3) {
            case 0: st8(a, x[RS2]); break; case 1: st16(a, x[RS2]); break;
            case 2: st32(a, x[RS2]); break; default: die("st");
            }
            break;
        }
        case 0x13: {
            sx1 = RS1; uint32_t a = x[RS1]; int32_t im = imm_i(ins); uint32_t v;
            switch (F3) {
            case 0: v = a + im; break;
            case 1: v = a << (im & 31); break;
            case 2: v = (int32_t)a < im; break;
            case 3: v = a < (uint32_t)im; break;
            case 4: v = a ^ im; break;
            case 5: v = (ins >> 30) & 1 ? (uint32_t)((int32_t)a >> (im & 31)) : a >> (im & 31); break;
            case 6: v = a | im; break;
            default: v = a & im; break;
            }
            x[RD] = v; dst_x = RD; break;
        }
        case 0x33: {
            sx1 = RS1; sx2 = RS2;
            uint32_t a = x[RS1], b = x[RS2], v = 0;
            if (F7 == 1) {
                lat = 2;
                switch (F3) {
                case 0: v = a * b; break;
                case 1: v = (uint32_t)(((int64_t)(int32_t)a * (int32_t)b) >> 32); break;
                case 2: v = (uint32_t)(((int64_t)(int32_t)a * (uint64_t)b) >> 32); break;
                case 3: v = (uint32_t)(((uint64_t)a * b) >> 32); break;
                case 4: lat = 18; is_div = 3; v = b == 0 ? ~0u : ((int32_t)a == INT32_MIN && (int32_t)b == -1) ? a : (uint32_t)((int32_t)a / (int32_t)b); break;
                case 5: lat = 18; is_div = 3; v = b == 0 ? ~0u : a / b; break;
                case 6: lat = 18; is_div = 3; v = b == 0 ? a : ((int32_t)a == INT32_MIN && (int32_t)b == -1) ? 0 : (uint32_t)((int32_t)a % (int32_t)b); break;
                case 7: lat = 18; is_div = 3; v = b == 0 ? a : a % b; break;
                }
            } else {
                switch (F3) {
                case 0: v = F7 ? a - b : a + b; break;
                case 1: v = a << (b & 31); break;
                case 2: v = (int32_t)a < (int32_t)b; break;
                case 3: v = a < b; break;
                case 4: v = a ^ b; break;
                case 5: v = F7 ? (uint32_t)((int32_t)a >> (b & 31)) : a >> (b & 31); break;
                case 6: v = a | b; break;
                case 7: v = a & b; break;
                }
            }
            x[RD] = v; dst_x = RD; break;
        }
        case 0x0f: break; /* fence */
        case 0x73: {
            if (ins == 0x00000073) { do_ecall(); break; }
            if (ins == 0x00100073) die("ebreak");
            /* CSR: fflags/frm/fcsr read as 0, writes ignored */
            if (F3) { sx1 = RS1; x[RD] = 0; dst_x = RD; }
            break;
        }
        case 0x07: sx1 = RS1; f[RD].u = ld32(x[RS1] + imm_i(ins)); dst_f = RD; lat = 2; break;
        case 0x27: sx1 = RS1; sf2 = RS2; st32(x[RS1] + imm_s(ins), f[RS2].u); break;
        case 0x43: case 0x47: case 0x4b: case 0x4f: {
            sf1 = RS1; sf2 = RS2; sf3 = RS3; lat = 3;
            float a = F(RS1), b = F(RS2), c = F(RS3), v;
            if (opc == 0x43) v = fmaf(a, b, c);
            else if (opc == 0x47) v = fmaf(a, b, -c);
            else if (opc == 0x4b) v = fmaf(-a, b, c);
            else v = fmaf(-a, b, -c);
            SETF(RD, v); dst_f = RD; break;
        }
        case 0x53: {
            int f5 = ins >> 27, rm = F3;
            float a = F(RS1), b = F(RS2);
            sf1 = RS1; sf2 = RS2; lat = 3;
            if (((ins >> 25) & 3) != 0) die("non-single fp op");
            switch (f5) {
            case 0x00: SETF(RD, a + b); dst_f = RD; break;
            case 0x01: SETF(RD, a - b); dst_f = RD; break;
            case 0x02: SETF(RD, a * b); dst_f = RD; break;
            case 0x03: SETF(RD, a / b); dst_f = RD; lat = 10; is_div = 1; break;
            case 0x0b: sf2 = -1; SETF(RD, sqrtf(a)); dst_f = RD; lat = 10; is_div = 2; break;
            case 0x04: {
                uint32_t sa = f[RS1].u & 0x7fffffff, sb = f[RS2].u & 0x80000000;
                if (rm == 1) sb ^= 0x80000000;
                else if (rm == 2) sb = (f[RS1].u ^ f[RS2].u) & 0x80000000, sa = f[RS1].u & 0x7fffffff;
                f[RD].u = sa | sb; dst_f = RD; lat = 1; break;
            }
            case 0x05: SETF(RD, fminmax(a, b, rm == 1)); dst_f = RD; lat = 2; break;
            case 0x18: sf2 = -1; lat = 2; dst_x = RD;
                x[RD] = RS2 == 0 ? (uint32_t)fcvt_w(a, rm) : fcvt_wu(a, rm); break;
            case 0x1c: sf2 = -1; lat = 2; dst_x = RD;
                x[RD] = rm == 0 ? f[RS1].u : fclass(a); break;
            case 0x14: lat = 2; dst_x = RD;
                x[RD] = rm == 2 ? a == b : rm == 1 ? a < b : a <= b; break;
            case 0x1a: sf1 = sf2 = -1; sx1 = RS1; lat = 2; dst_f = RD;
                SETF(RD, RS2 == 0 ? (float)(int32_t)x[RS1] : (float)x[RS1]); break;
            case 0x1e: sf1 = sf2 = -1; sx1 = RS1; lat = 1; dst_f = RD;
                f[RD].u = x[RS1]; break;
            default: die("fp op");
            }
            break;
        }
        default: die("illegal opcode");
        }
        x[0] = 0;

        /* scoreboard timing */
        if (sx1 > 0 && ready_x[sx1] > issue) issue = ready_x[sx1];
        if (sx2 > 0 && ready_x[sx2] > issue) issue = ready_x[sx2];
        if (sf1 >= 0 && ready_f[sf1] > issue) issue = ready_f[sf1];
        if (sf2 >= 0 && ready_f[sf2] > issue) issue = ready_f[sf2];
        if (sf3 >= 0 && ready_f[sf3] > issue) issue = ready_f[sf3];
        if (dst_x > 0) ready_x[dst_x] = issue + lat;
        if (dst_f >= 0) ready_f[dst_f] = issue + lat;
        cyc = issue + 1 + taken;

        if (prof_on) {
            uint32_t i = (pc - MEM_BASE) >> 1;
            pc_count[i]++; pc_cycles[i] += cyc - t0;
            n_insn++;
            if (is_div) {
                pc_div[i] = is_div;
                n_fdiv += is_div == 1; n_fsqrt += is_div == 2; n_idiv += is_div == 3;
            }
        }
        pc = npc;
    }
}
