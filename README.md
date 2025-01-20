# EVDK5

## Performance measurements

scaleFast(src, dst);
3550 us

scale(src, dst);
5860 us

clearUint8Image(dst);
910 us

clearUint8Image_cm33(dst);
50 us

convolve(src_int16, dst_int16, msk_int16);
41 ms

convolveFast(src_int16, dst_int16, msk_int16);
8490 us

mean(src, dst, 3);
42940 us

meanFast(src, dst);
8990 us

threshold2Means(src, dst, 1);
4940 us

thresholdOtsu(src, dst, 1);
3080 us