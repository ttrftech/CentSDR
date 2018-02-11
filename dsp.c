#include <arm_math.h>
#include "nanosdr.h"

int16_t buffer[2][AUDIO_BUFFER_LEN];
int16_t buffer2[2][AUDIO_BUFFER_LEN];


const int16_t cos_sin_table[256][4] = {
    { -32767,   10,      0, -804 },
    { -32757,   29,   -804, -804 },
    { -32728,   50,  -1608, -802 },
    { -32678,   69,  -2410, -802 },
    { -32609,   88,  -3212, -799 },
    { -32521,  109,  -4011, -797 },
    { -32412,  127,  -4808, -794 },
    { -32285,  148,  -5602, -791 },
    { -32137,  166,  -6393, -786 },
    { -31971,  186,  -7179, -783 },
    { -31785,  205,  -7962, -777 },
    { -31580,  224,  -8739, -773 },
    { -31356,  243,  -9512, -766 },
    { -31113,  261, -10278, -761 },
    { -30852,  281, -11039, -754 },
    { -30571,  298, -11793, -746 },
    { -30273,  317, -12539, -740 },
    { -29956,  335, -13279, -731 },
    { -29621,  353, -14010, -722 },
    { -29268,  370, -14732, -714 },
    { -28898,  388, -15446, -705 },
    { -28510,  405, -16151, -695 },
    { -28105,  422, -16846, -684 },
    { -27683,  438, -17530, -674 },
    { -27245,  455, -18204, -664 },
    { -26790,  471, -18868, -651 },
    { -26319,  487, -19519, -640 },
    { -25832,  503, -20159, -628 },
    { -25329,  518, -20787, -616 },
    { -24811,  532, -21403, -602 },
    { -24279,  548, -22005, -589 },
    { -23731,  561, -22594, -576 },
    { -23170,  576, -23170, -561 },
    { -22594,  589, -23731, -548 },
    { -22005,  602, -24279, -532 },
    { -21403,  616, -24811, -518 },
    { -20787,  628, -25329, -503 },
    { -20159,  640, -25832, -487 },
    { -19519,  651, -26319, -471 },
    { -18868,  664, -26790, -455 },
    { -18204,  674, -27245, -438 },
    { -17530,  684, -27683, -422 },
    { -16846,  695, -28105, -405 },
    { -16151,  705, -28510, -388 },
    { -15446,  714, -28898, -370 },
    { -14732,  722, -29268, -353 },
    { -14010,  731, -29621, -335 },
    { -13279,  740, -29956, -317 },
    { -12539,  746, -30273, -298 },
    { -11793,  754, -30571, -281 },
    { -11039,  761, -30852, -261 },
    { -10278,  766, -31113, -243 },
    { -9512,  773, -31356, -224 },
    { -8739,  777, -31580, -205 },
    { -7962,  783, -31785, -186 },
    { -7179,  786, -31971, -166 },
    { -6393,  791, -32137, -148 },
    { -5602,  794, -32285, -127 },
    { -4808,  797, -32412, -109 },
    { -4011,  799, -32521,  -88 },
    { -3212,  802, -32609,  -69 },
    { -2410,  802, -32678,  -50 },
    { -1608,  804, -32728,  -29 },
    {  -804,  804, -32757,  -10 },
    {     0,  804, -32767,   10 },
    {   804,  804, -32757,   29 },
    {  1608,  802, -32728,   50 },
    {  2410,  802, -32678,   69 },
    {  3212,  799, -32609,   88 },
    {  4011,  797, -32521,  109 },
    {  4808,  794, -32412,  127 },
    {  5602,  791, -32285,  148 },
    {  6393,  786, -32137,  166 },
    {  7179,  783, -31971,  186 },
    {  7962,  777, -31785,  205 },
    {  8739,  773, -31580,  224 },
    {  9512,  766, -31356,  243 },
    { 10278,  761, -31113,  261 },
    { 11039,  754, -30852,  281 },
    { 11793,  746, -30571,  298 },
    { 12539,  740, -30273,  317 },
    { 13279,  731, -29956,  335 },
    { 14010,  722, -29621,  353 },
    { 14732,  714, -29268,  370 },
    { 15446,  705, -28898,  388 },
    { 16151,  695, -28510,  405 },
    { 16846,  684, -28105,  422 },
    { 17530,  674, -27683,  438 },
    { 18204,  664, -27245,  455 },
    { 18868,  651, -26790,  471 },
    { 19519,  640, -26319,  487 },
    { 20159,  628, -25832,  503 },
    { 20787,  616, -25329,  518 },
    { 21403,  602, -24811,  532 },
    { 22005,  589, -24279,  548 },
    { 22594,  576, -23731,  561 },
    { 23170,  561, -23170,  576 },
    { 23731,  548, -22594,  589 },
    { 24279,  532, -22005,  602 },
    { 24811,  518, -21403,  616 },
    { 25329,  503, -20787,  628 },
    { 25832,  487, -20159,  640 },
    { 26319,  471, -19519,  651 },
    { 26790,  455, -18868,  664 },
    { 27245,  438, -18204,  674 },
    { 27683,  422, -17530,  684 },
    { 28105,  405, -16846,  695 },
    { 28510,  388, -16151,  705 },
    { 28898,  370, -15446,  714 },
    { 29268,  353, -14732,  722 },
    { 29621,  335, -14010,  731 },
    { 29956,  317, -13279,  740 },
    { 30273,  298, -12539,  746 },
    { 30571,  281, -11793,  754 },
    { 30852,  261, -11039,  761 },
    { 31113,  243, -10278,  766 },
    { 31356,  224,  -9512,  773 },
    { 31580,  205,  -8739,  777 },
    { 31785,  186,  -7962,  783 },
    { 31971,  166,  -7179,  786 },
    { 32137,  148,  -6393,  791 },
    { 32285,  127,  -5602,  794 },
    { 32412,  109,  -4808,  797 },
    { 32521,   88,  -4011,  799 },
    { 32609,   69,  -3212,  802 },
    { 32678,   50,  -2410,  802 },
    { 32728,   29,  -1608,  804 },
    { 32757,   10,   -804,  804 },
    { 32767,  -10,      0,  804 },
    { 32757,  -29,    804,  804 },
    { 32728,  -50,   1608,  802 },
    { 32678,  -69,   2410,  802 },
    { 32609,  -88,   3212,  799 },
    { 32521, -109,   4011,  797 },
    { 32412, -127,   4808,  794 },
    { 32285, -148,   5602,  791 },
    { 32137, -166,   6393,  786 },
    { 31971, -186,   7179,  783 },
    { 31785, -205,   7962,  777 },
    { 31580, -224,   8739,  773 },
    { 31356, -243,   9512,  766 },
    { 31113, -261,  10278,  761 },
    { 30852, -281,  11039,  754 },
    { 30571, -298,  11793,  746 },
    { 30273, -317,  12539,  740 },
    { 29956, -335,  13279,  731 },
    { 29621, -353,  14010,  722 },
    { 29268, -370,  14732,  714 },
    { 28898, -388,  15446,  705 },
    { 28510, -405,  16151,  695 },
    { 28105, -422,  16846,  684 },
    { 27683, -438,  17530,  674 },
    { 27245, -455,  18204,  664 },
    { 26790, -471,  18868,  651 },
    { 26319, -487,  19519,  640 },
    { 25832, -503,  20159,  628 },
    { 25329, -518,  20787,  616 },
    { 24811, -532,  21403,  602 },
    { 24279, -548,  22005,  589 },
    { 23731, -561,  22594,  576 },
    { 23170, -576,  23170,  561 },
    { 22594, -589,  23731,  548 },
    { 22005, -602,  24279,  532 },
    { 21403, -616,  24811,  518 },
    { 20787, -628,  25329,  503 },
    { 20159, -640,  25832,  487 },
    { 19519, -651,  26319,  471 },
    { 18868, -664,  26790,  455 },
    { 18204, -674,  27245,  438 },
    { 17530, -684,  27683,  422 },
    { 16846, -695,  28105,  405 },
    { 16151, -705,  28510,  388 },
    { 15446, -714,  28898,  370 },
    { 14732, -722,  29268,  353 },
    { 14010, -731,  29621,  335 },
    { 13279, -740,  29956,  317 },
    { 12539, -746,  30273,  298 },
    { 11793, -754,  30571,  281 },
    { 11039, -761,  30852,  261 },
    { 10278, -766,  31113,  243 },
    {  9512, -773,  31356,  224 },
    {  8739, -777,  31580,  205 },
    {  7962, -783,  31785,  186 },
    {  7179, -786,  31971,  166 },
    {  6393, -791,  32137,  148 },
    {  5602, -794,  32285,  127 },
    {  4808, -797,  32412,  109 },
    {  4011, -799,  32521,   88 },
    {  3212, -802,  32609,   69 },
    {  2410, -802,  32678,   50 },
    {  1608, -804,  32728,   29 },
    {   804, -804,  32757,   10 },
    {     0, -804,  32767,  -10 },
    {  -804, -804,  32757,  -29 },
    { -1608, -802,  32728,  -50 },
    { -2410, -802,  32678,  -69 },
    { -3212, -799,  32609,  -88 },
    { -4011, -797,  32521, -109 },
    { -4808, -794,  32412, -127 },
    { -5602, -791,  32285, -148 },
    { -6393, -786,  32137, -166 },
    { -7179, -783,  31971, -186 },
    { -7962, -777,  31785, -205 },
    { -8739, -773,  31580, -224 },
    { -9512, -766,  31356, -243 },
    { -10278, -761,  31113, -261 },
    { -11039, -754,  30852, -281 },
    { -11793, -746,  30571, -298 },
    { -12539, -740,  30273, -317 },
    { -13279, -731,  29956, -335 },
    { -14010, -722,  29621, -353 },
    { -14732, -714,  29268, -370 },
    { -15446, -705,  28898, -388 },
    { -16151, -695,  28510, -405 },
    { -16846, -684,  28105, -422 },
    { -17530, -674,  27683, -438 },
    { -18204, -664,  27245, -455 },
    { -18868, -651,  26790, -471 },
    { -19519, -640,  26319, -487 },
    { -20159, -628,  25832, -503 },
    { -20787, -616,  25329, -518 },
    { -21403, -602,  24811, -532 },
    { -22005, -589,  24279, -548 },
    { -22594, -576,  23731, -561 },
    { -23170, -561,  23170, -576 },
    { -23731, -548,  22594, -589 },
    { -24279, -532,  22005, -602 },
    { -24811, -518,  21403, -616 },
    { -25329, -503,  20787, -628 },
    { -25832, -487,  20159, -640 },
    { -26319, -471,  19519, -651 },
    { -26790, -455,  18868, -664 },
    { -27245, -438,  18204, -674 },
    { -27683, -422,  17530, -684 },
    { -28105, -405,  16846, -695 },
    { -28510, -388,  16151, -705 },
    { -28898, -370,  15446, -714 },
    { -29268, -353,  14732, -722 },
    { -29621, -335,  14010, -731 },
    { -29956, -317,  13279, -740 },
    { -30273, -298,  12539, -746 },
    { -30571, -281,  11793, -754 },
    { -30852, -261,  11039, -761 },
    { -31113, -243,  10278, -766 },
    { -31356, -224,   9512, -773 },
    { -31580, -205,   8739, -777 },
    { -31785, -186,   7962, -783 },
    { -31971, -166,   7179, -786 },
    { -32137, -148,   6393, -791 },
    { -32285, -127,   5602, -794 },
    { -32412, -109,   4808, -797 },
    { -32521,  -88,   4011, -799 },
    { -32609,  -69,   3212, -802 },
    { -32678,  -50,   2410, -802 },
    { -32728,  -29,   1608, -804 },
    { -32757,  -10,    804, -804 }
};

static uint32_t
cos_sin(uint16_t phase)
{
    uint16_t idx = phase / 256;
    uint16_t mod = phase & 0xff;
    uint32_t r = __PKHBT(0x0100, mod, 16);
    uint32_t *e = (uint32_t *)&cos_sin_table[idx];
    uint32_t cd = e[0];
    uint32_t sd = e[1];
    int32_t c = __SMUAD(r, cd);
    int32_t s = __SMUAD(r, sd);
    c /= 256;
    s /= 256;
    return __PKHBT(s, c, 16);
}


uint16_t nco1_phase = 0;
uint16_t nco2_phase = 0;
#define SSB_NCO_PHASESTEP (65536L*SSB_FREQ_OFFSET/48000)


// Bi-Quad IIR Filter state
q15_t bq_i_state[4 * 3];
q15_t bq_q_state[4 * 3];
#if 0
// 6th order elliptic lowpass filter fc=1300Hz, 40dB
q15_t bq_coeffs[] = {
		  515, 0,   -906,   515, 30977, -14714,
		 5171, 0, -10087,  5171, 31760, -15739,
		16384, 0, -32182, 16384, 32165, -16253,
};
#else
// 6th order elliptic lowpass filter fc=1300Hz, 60dB
q15_t bq_coeffs[] = {
		  157, 0,   -238,   157, 31237, -14936,
		 3643, 0,  -6974,  3643, 31656, -15580,
		 8272, 0, -16096,  8272, 32074, -16158
};
#endif

arm_biquad_casd_df1_inst_q15 bq_i = { 3, bq_i_state, bq_coeffs, 1};
arm_biquad_casd_df1_inst_q15 bq_q = { 3, bq_q_state, bq_coeffs, 1};


// 6th order elliptic lowpass filter fc=6*1300=7800Hz
q15_t bq_coeffs_am[] = {
		 1186, 0,   1108,  1186, 20883,  -8328,
		 6829, 0,  -4129,  6829, 17973, -13228,
		16384, 0, -14411, 16384, 16788, -15733
};

arm_biquad_casd_df1_inst_q15 bq_am_i = { 3, bq_i_state, bq_coeffs_am, 1};
arm_biquad_casd_df1_inst_q15 bq_am_q = { 3, bq_q_state, bq_coeffs_am, 1};

void
ssb_demod(int16_t *src, int16_t *dst, size_t len, int phasestep)
{
	q15_t *bufi = buffer[0];
	q15_t *bufq = buffer[1];
    int32_t *s = __SIMD32(src);
    int32_t *d = __SIMD32(dst);
    uint32_t i;

    disp_fetch_samples(B_CAPTURE, BT_C_INTERLEAVE, src, NULL, len);

    // shift frequency
    for (i = 0; i < len/2; i++) {
		uint32_t cossin = cos_sin(nco1_phase);
		nco1_phase -= phasestep;
		uint32_t iq = *s++;
		*bufi++ = __SMLSDX(iq, cossin, 0) >> (15-0);
		*bufq++ = __SMLAD(iq, cossin, 0) >> (15-0);
	}
    disp_fetch_samples(B_IF1, BT_IQ, buffer[0], buffer[1], len/2);

    // apply low pass filter
	arm_biquad_cascade_df1_q15(&bq_i, buffer[0], buffer2[0], len/2);
	arm_biquad_cascade_df1_q15(&bq_q, buffer[1], buffer2[1], len/2);

    disp_fetch_samples(B_IF2, BT_IQ, buffer2[0], buffer2[1], len/2);

    // shift frequency inverse
	bufi = buffer2[0];
	bufq = buffer2[1];
    for (i = 0; i < len/2; i++) {
		uint32_t cossin = cos_sin(nco2_phase);
		nco2_phase += phasestep;
		uint32_t iq = __PKHBT(*bufi++, *bufq++, 16);
		uint32_t r = __SMLAD(iq, cossin, 0) >> (15-0);
        *d++ = __PKHBT(r, r, 16);
	}

    disp_fetch_samples(B_PLAYBACK, BT_R_INTERLEAVE, dst, NULL, len);
}

__attribute__ ( ( always_inline ) ) __STATIC_INLINE
float _VSQRTF(float op1) {
  float result;
  __ASM volatile ("vsqrt.f32 %0,%1" : "=w"(result) : "w"(op1) );
  return(result);
}

int32_t ave_z = 0;

void
am_demod(int16_t *src, int16_t *dst, size_t len)
#if defined(AM_FREQ_OFFSET) && AM_FREQ_OFFSET
{
#define PHASESTEP 65536L*AM_FREQ_OFFSET/48000

	q15_t *bufi = buffer[0];
	q15_t *bufq = buffer[1];
    int32_t *s = __SIMD32(src);
    int32_t *d = __SIMD32(dst);
    uint32_t i;

    disp_fetch_samples(B_CAPTURE, BT_C_INTERLEAVE, src, NULL, len);

    for (i = 0; i < len/2; i++) {
        uint32_t cossin = cos_sin(nco1_phase);
        nco1_phase -= PHASESTEP;
		uint32_t iq = *s++;
		*bufi++ = __SMLSDX(iq, cossin, 0) >> (15-0);
		*bufq++ = __SMLAD(iq, cossin, 0) >> (15-0);
	}
    disp_fetch_samples(B_IF1, BT_IQ, buffer[0], buffer[1], len/2);

    // apply low pass filter
	arm_biquad_cascade_df1_q15(&bq_am_i, buffer[0], buffer2[0], len/2);
	arm_biquad_cascade_df1_q15(&bq_am_q, buffer[1], buffer2[1], len/2);

    disp_fetch_samples(B_IF2, BT_IQ, buffer2[0], buffer2[1], len/2);
    
    //int32_t acc_z = 0;
	bufi = buffer2[0];
	bufq = buffer2[1];
    for (i = 0; i < len/2; i++) {
      int32_t x = *bufi++;
      int32_t y = *bufq++;
      int32_t z;
      //x = x/2;
      //y = y/2;
      z = (int16_t)_VSQRTF((float)(x*x+y*y));
      //z = (int16_t)sqrtf(x*x+y*y);
      //acc_z += z;
      //z -= ave_z;
      if (z > 32767) z = 32767;
      if (z < -32768) z = -32768;
      *d++ = __PKHBT(z, z, 16);
	}
    //ave_z = ave_z * 0.98 + (0.02 * acc_z / (len/2));

    disp_fetch_samples(B_PLAYBACK, BT_R_INTERLEAVE, dst, NULL, len);
}
#else
{
  uint32_t i;
  for (i = 0; i < len; i += 2) {
    int32_t x = src[i];
    int32_t y = src[i+1];
    int32_t z;
#define DCOFFSET 16383
    x = x/2;
    y = y/2;
    z = (int16_t)_VSQRTF((float)(x*x+y*y)) - DCOFFSET;
    //z = (int16_t)sqrtf(x*x+y*y) - DCOFFSET;
    dst[i] = dst[i+1] = z;
  }
}
#endif

void
lsb_demod(int16_t *src, int16_t *dst, size_t len)
{
  ssb_demod(src, dst, len, -SSB_NCO_PHASESTEP);
}

void
usb_demod(int16_t *src, int16_t *dst, size_t len)
{
  ssb_demod(src, dst, len, SSB_NCO_PHASESTEP);
}


struct {
  uint32_t last;
  uint32_t pre1;
  uint32_t pre2;
} fm_demod_state;

const int16_t arctantbl[256+2] = {
	    0,   128,   256,   384,   512,   640,   768,   896,  1024,
	        1152,  1279,  1407,  1535,  1663,  1790,  1918,  2045,  2173,
	        2300,  2428,  2555,  2682,  2809,  2936,  3063,  3190,  3317,
	        3443,  3570,  3696,  3823,  3949,  4075,  4201,  4327,  4452,
	        4578,  4703,  4829,  4954,  5079,  5204,  5329,  5453,  5578,
	        5702,  5826,  5950,  6073,  6197,  6320,  6444,  6567,  6689,
	        6812,  6935,  7057,  7179,  7301,  7422,  7544,  7665,  7786,
	        7907,  8027,  8148,  8268,  8388,  8508,  8627,  8746,  8865,
	        8984,  9102,  9221,  9339,  9456,  9574,  9691,  9808,  9925,
	       10041, 10158, 10274, 10389, 10505, 10620, 10735, 10849, 10964,
	       11078, 11192, 11305, 11418, 11531, 11644, 11756, 11868, 11980,
	       12092, 12203, 12314, 12424, 12535, 12645, 12754, 12864, 12973,
	       13082, 13190, 13298, 13406, 13514, 13621, 13728, 13835, 13941,
	       14047, 14153, 14258, 14363, 14468, 14573, 14677, 14781, 14884,
	       14987, 15090, 15193, 15295, 15397, 15499, 15600, 15701, 15801,
	       15902, 16002, 16101, 16201, 16300, 16398, 16497, 16595, 16693,
	       16790, 16887, 16984, 17080, 17176, 17272, 17368, 17463, 17557,
	       17652, 17746, 17840, 17933, 18027, 18119, 18212, 18304, 18396,
	       18488, 18579, 18670, 18760, 18851, 18941, 19030, 19120, 19209,
	       19297, 19386, 19474, 19561, 19649, 19736, 19823, 19909, 19995,
	       20081, 20166, 20252, 20336, 20421, 20505, 20589, 20673, 20756,
	       20839, 20922, 21004, 21086, 21168, 21249, 21331, 21411, 21492,
	       21572, 21652, 21732, 21811, 21890, 21969, 22047, 22126, 22203,
	       22281, 22358, 22435, 22512, 22588, 22664, 22740, 22815, 22891,
	       22966, 23040, 23115, 23189, 23262, 23336, 23409, 23482, 23555,
	       23627, 23699, 23771, 23842, 23914, 23985, 24055, 24126, 24196,
	       24266, 24335, 24405, 24474, 24542, 24611, 24679, 24747, 24815,
	       24882, 24950, 25017, 25083, 25150, 25216, 25282, 25347, 25413,
	       25478, 25543, 25607, 25672, 25736, 25736
};

#define Q15_PI_4	25736	// 3.14159/4*32768


static inline int16_t
atan_2iq(uint32_t iq0, uint32_t iq1)
{
  int32_t re = __SMUAD(iq1, iq0);	// I0*I1 + Q0*Q1
  int32_t im = __SMUSDX(iq1, iq0);	// I0*Q1 - I1*Q0
  int32_t ang = 0;
  uint8_t neg = 0;
  if (re < 0) {
    re = -re;
    neg = !neg;
    ang += -Q15_PI_4 * 4;
  }
  if (im < 0) {
    im = -im;
    neg = !neg;
  }
  if (im >= re) {
    int32_t x = im;
    im = re;
    re = x;
    neg = !neg;
    ang = -ang - Q15_PI_4 * 2;
  }
  {
    uint32_t d, f;
    int32_t a, b;
    int idx;
    d = im << 0;
    d /= re >> 16;
    idx = (d >> 8) & 0xff;
    f = d & 0xff;
    a = arctantbl[idx];
    b = arctantbl[idx+1];
    ang += a + (((b - a) * f) >> 8);
  }
  if (neg)
    ang = -ang;
  return __SSAT(ang/128, 16);
}

void
fm_demod(int16_t *src, int16_t *dst, size_t len)
{
    int32_t *s = __SIMD32(src);
    int32_t *dst32 = __SIMD32(dst);
	unsigned int i;
	uint32_t x0 = fm_demod_state.last;
    q15_t v;

    disp_fetch_samples(B_CAPTURE, BT_C_INTERLEAVE, src, NULL, len);
    
	for (i = 0; i < len; i += 2) {
        uint32_t x1 = *s++;
        v = atan_2iq(x0, x1);
        *dst32++ = __PKHBT(v, v, 16);
		x0 = x1;
	}
	fm_demod_state.last = x0;

    disp_fetch_samples(B_PLAYBACK, BT_R_INTERLEAVE, dst, NULL, len);
}

// state variables for stereo separation
stereo_separate_state_t stereo_separate_state;

#define IF_RATE 192.0
#define PHASESTEP_NCO19KHz 	((19.0*65536.0*65536.0)/IF_RATE)

void
stereo_separate_init(void)
{
	stereo_separate_state.phase_accum = 0;
	stereo_separate_state.phase_step_default = PHASESTEP_NCO19KHz;
	stereo_separate_state.phase_step = stereo_separate_state.phase_step_default;

	stereo_separate_state.corr = 0;
	stereo_separate_state.integrator = 0;
	stereo_separate_state.sdi = 0;
	stereo_separate_state.sdq = 0;
}


// src: baseband signal (real)
// dest: 38kHz shifted subchannel (real)
void
stereo_separate(int16_t *src, int16_t *dest, int32_t length)
{
	int i;
	int32_t di = 0;
	int32_t dq = 0;
	uint32_t phase_accum = stereo_separate_state.phase_accum;
	uint32_t phase_step = stereo_separate_state.phase_step;

	int32_t corr = 0;

    // PLL 19kHz, 38kHz frequency shift
	for (i = 0; i < length; i++) {
		uint32_t cs = cos_sin(phase_accum >> 16);
		int16_t s = cs & 0xffff;
		int16_t c = cs >> 16;

        // sin(2t) = 2sin(t)cos(t)
		int16_t ss = (int32_t)(c * s) >> (16 - 2);

        // frequency shift
		int32_t x = src[i];
		dest[i] = (ss * x) >> (16 - 1);
        //src[i] = src[i] / 2;
        
        // correlate 19kHz pilot carrier
		di += (c * x) >> 16;
		dq += (s * x) >> 16;
		phase_accum += phase_step;
	}
    stereo_separate_state.phase_accum = phase_accum;

	// averaging correlation
	di = (stereo_separate_state.sdi * 15 + di) / 16;
	dq = (stereo_separate_state.sdq * 15 + dq) / 16;
	stereo_separate_state.sdi = di;
	stereo_separate_state.sdq = dq;
	if (di > 0) {
		corr = 1024 * dq / di;
		//corr += stereo_separate_state.corr;
		if (corr > 4095)
			corr = 4095;
		else if (corr < -4095)
			corr = -4095;
	} else {
		if (dq > 0)
			corr = 4095;
		else if (dq < 0)
			corr = -4095;
        //phase_step = stereo_separate_state.phase_step_default;
	}
    
	if (corr != 0) {
        // for monitoring
		stereo_separate_state.corr = corr;
		stereo_separate_state.corr_ave = (stereo_separate_state.corr_ave * 15 + corr) / 16;

		int32_t d = stereo_separate_state.corr_ave - corr;
		int32_t sd = (stereo_separate_state.corr_std * 15 + d * d) / 16;
		if (sd > 32767)
			sd = 32767;
		stereo_separate_state.corr_std = sd;

        // feedback phase step
        phase_step = stereo_separate_state.phase_step_default
          - stereo_separate_state.integrator - corr * 128;
        //phase_step += -stereo_separate_state.corr_ave;
		stereo_separate_state.phase_step = phase_step;
        if (stereo_separate_state.corr_std < 100)
          stereo_separate_state.integrator += stereo_separate_state.corr_ave;
	}
}

void
stereo_matrix(int16_t *s1, int16_t *s2, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		uint32_t x1 = *s1;
		uint32_t x2 = *s2;
		uint32_t l = __QADD16(x1, x2);
		uint32_t r = __QSUB16(x1, x2);
		*s1++ = l;
		*s2++ = r;
	}
}

void
stereo_matrix2(int16_t *s1, int16_t *s2, int16_t *dst, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		uint32_t x1 = *s1++ / 2;
		uint32_t x2 = *s2++ / 2;
		uint32_t l = __QADD16(x1, x2);
		uint32_t r = __QSUB16(x1, x2);
		*dst++ = l;
		*dst++ = r;
	}
}

void
stereo_matrix3(int16_t *s1, int16_t *s2, int16_t *dst, int len)
{
	int i;
	for (i = 0; i < len; i += 4) {
        int32_t x1 = *s1++ / 4;
        int32_t x2 = *s2++ / 4;
		int32_t l = __QADD16(x1, x2);
		int32_t r = __QSUB16(x1, x2);
        x1 = *s1++ / 4;
        x2 = *s2++ / 4;
		int32_t l0 = __QADD16(x1, x2);
		int32_t r0 = __QSUB16(x1, x2);
        l = __QADD16(l, l0);
        r = __QADD16(r, r0);
        x1 = *s1++ / 4;
        x2 = *s2++ / 4;
        l0 = __QADD16(x1, x2);
        r0 = __QSUB16(x1, x2);
        l = __QADD16(l, l0);
        r = __QADD16(r, r0);
        x1 = *s1++ / 4;
        x2 = *s2++ / 4;
        l0 = __QADD16(x1, x2);
        r0 = __QSUB16(x1, x2);
        l = __QADD16(l, l0);
        r = __QADD16(r, r0);
		*dst++ = l;
		*dst++ = r;
		*dst++ = l;
		*dst++ = r;
		*dst++ = l;
		*dst++ = r;
		*dst++ = l;
		*dst++ = r;
	}
}

static inline void
fm_adj_filter(int16_t *src, size_t len)
{
    int32_t *s = __SIMD32(src);
	unsigned int i;

    uint32_t x1 = fm_demod_state.pre1;
    uint32_t x2 = fm_demod_state.pre2;
    uint32_t zero = 0;
    uint32_t k12 = 0x5ae1eccd;
    //uint32_t k12 = 0x51ecea3d;
    
	for (i = 0; i < len; i += 2) {
        int32_t x0 = *s;
        int32_t acc_i = 0;
        uint32_t i12 = __PKHBT(x2, x1, 16);
        uint32_t i0_ = __PKHBT(zero, x0, 16);
        acc_i = __SMLAD(k12, i12, acc_i);
        acc_i = __SMLADX(k12, i0_, acc_i);

        int32_t acc_q = 0;
        uint32_t q12 = __PKHTB(x1, x2, 16);
        uint32_t q0_ = __PKHTB(x0, zero, 16);
        acc_q = __SMLAD(k12, q12, acc_q);
        acc_q = __SMLADX(k12, q0_, acc_q);
#if 1
        acc_i = __SSAT(acc_i / 8192, 16);
        acc_q = __SSAT(acc_q / 8192, 16);
        *s++ = __PKHBT(acc_i, acc_q, 16);
#else
        *s++ = __PKHTB(acc_q, acc_i, 16);
#endif
        
        x2 = x1;
        x1 = x0;
    }
    fm_demod_state.pre1 = x1;
    fm_demod_state.pre2 = x2;
}

void
fm_demod0(int16_t *src, int16_t *dst, size_t len)
{
    int32_t *s = __SIMD32(src);
	uint32_t x0 = fm_demod_state.last;
	unsigned int i;

	for (i = 0; i < len; i += 2) {
        uint32_t x1 = *s++;
        *dst++ = atan_2iq(x0, x1);
		x0 = x1;
	}
	fm_demod_state.last = x0;
}

void
fm_demod_stereo(int16_t *src, int16_t *dst, size_t len)
{
  // apply frequency response adjustment
  fm_adj_filter(src, len);
   
  disp_fetch_samples(B_CAPTURE, BT_C_INTERLEAVE, src, NULL, len);
  fm_demod0(src, buffer[0], len);
  disp_fetch_samples(B_IF1, BT_REAL, buffer[0], NULL, len/2);
  stereo_separate(buffer[0], buffer2[0], len/2);
  disp_fetch_samples(B_IF2, BT_REAL, buffer2[0], NULL, len/2);
  stereo_matrix2(buffer[0], buffer2[0], dst, len/2);
  //stereo_matrix3(buffer[0], buffer2[0], dst, len/2);
  disp_fetch_samples(B_PLAYBACK, BT_R_INTERLEAVE, dst, NULL, len);
}

#if 0

void
fir_filter_stereo()
{
	const uint32_t *coeff;
	const uint16_t *src1 = (const uint16_t *)RESAMPLE_STATE;
	const uint16_t *src2 = (const uint16_t *)RESAMPLE2_STATE;
	const uint32_t *s1, *s2;
	int32_t tail = RESAMPLE_BUFFER_SIZE;
	int32_t idx = resample_state.index;
	int32_t acc1, acc2;
	int i, j;
	int cur = audio_state.write_current;
	uint16_t *dest = (uint16_t *)AUDIO_BUFFER;
	float val1 = resample_state.deemphasis_value;
	float val2 = resample_state.deemphasis_value2;

	while (idx < tail) {
		coeff = (uint32_t*)resample_fir_coeff[idx % 2];
		acc1 = 0;
		acc2 = 0;
		s1 = (const uint32_t*)&src1[idx >> 1];
		s2 = (const uint32_t*)&src2[idx >> 1];
		for (j = 0; j < RESAMPLE_NUM_TAPS / 2; j++) {
			uint32_t x1 = *s1++;
			uint32_t x2 = *s2++;
			//uint32_t l = __SADD16(x1, x2);
			//uint32_t r = __SSUB16(x1, x2);
			acc1 = __SMLAD(x1, *coeff, acc1);
			acc2 = __SMLAD(x2, *coeff, acc2);
			coeff++;
		}

		// deemphasis with time constant
		val1 = (float)acc1 * resample_state.deemphasis_rest + val1 * resample_state.deemphasis_mult;
		val2 = (float)acc2 * resample_state.deemphasis_rest + val2 * resample_state.deemphasis_mult;
		int32_t left = val1 + val2;
		int32_t right = val1 - val2;
		dest[cur++] = left >> (16 - RESAMPLE_GAINBITS);
		dest[cur++] = right >> (16 - RESAMPLE_GAINBITS);
		//dest[cur++] = __SSAT((int32_t)(val1-val2) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = __SSAT((int32_t)(val1) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = __SSAT((int32_t)(val1) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = 0;
		cur %= AUDIO_BUFFER_SIZE / 2;
		audio_state.write_total += 2;
		idx += 13; /* 2/13 decimation: 2 samples per loop */
	}

	resample_state.deemphasis_value = val1;
	resample_state.deemphasis_value2 = val2;
	audio_state.write_current = cur;
	resample_state.index = idx - tail;
	uint32_t *state = (uint32_t *)RESAMPLE_STATE;
	src1 = &src1[tail / sizeof(*src1)];
	for (i = 0; i < RESAMPLE_STATE_SIZE / sizeof(uint32_t); i++) {
		//*state++ = *src1++;
	    __asm__ volatile ("ldr r0, [%0], #+4\n" : : "r" (src1) : "r0");
	    __asm__ volatile ("str r0, [%0], #+4\n" : : "r" (state) : "r0");
	}
	state = (uint32_t *)RESAMPLE2_STATE;
	src2 = &src2[tail / sizeof(*src2)];
	for (i = 0; i < RESAMPLE_STATE_SIZE / sizeof(uint32_t); i++) {
		//*state++ = *src2++;
	    __asm__ volatile ("ldr r0, [%0], #+4\n" : : "r" (src2) : "r0");
	    __asm__ volatile ("str r0, [%0], #+4\n" : : "r" (state) : "r0");
	}
}

#endif

void
dsp_init(void)
{
  stereo_separate_init();
}
