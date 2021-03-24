const unsigned int revSampleRate = 22050;
const unsigned int revSampleCount = 6392;
const signed char revSamples[] = {//0
0, -1, -2, -3, -3, -3, 0, 2, 4, 5, 8, 10, 13, 15, 16, 17, //16
19, 20, 22, 23, 24, 24, 24, 25, 25, 25, 24, 23, 22, 21, 21, 20, //32
19, 19, 19, 20, 21, 22, 23, 23, 26, 28, 31, 35, 38, 40, 41, 40, //48
38, 36, 33, 28, 24, 21, 17, 16, 14, 11, 10, 9, 8, 7, 5, 3, //64
2, 0, 0, 0, 0, 1, 2, 3, 4, 6, 10, 12, 13, 14, 14, 14, //80
12, 10, 8, 6, 5, 0, -4, -8, -12, -15, -17, -19, -20, -21, -21, -21, //96
-22, -23, -24, -24, -24, -25, -26, -27, -26, -25, -24, -23, -21, -22, -23, -23, //112
-25, -25, -26, -27, -28, -30, -32, -36, -38, -40, -39, -37, -36, -33, -31, -28, //128
-26, -25, -22, -19, -16, -14, -12, -10, -6, -3, -1, 0, 1, 3, 4, 4, //144
4, 5, 6, 8, 9, 10, 11, 13, 15, 19, 21, 22, 24, 24, 23, 20, //160
16, 13, 9, 6, 3, 3, 5, 6, 10, 13, 17, 20, 21, 21, 20, 19, //176
15, 12, 10, 8, 8, 9, 10, 11, 13, 15, 16, 18, 17, 16, 15, 13, //192
9, 2, -2, -7, -11, -14, -17, -17, -16, -14, -11, -8, -4, -3, -2, -3, //208
-5, -7, -12, -14, -15, -15, -13, -9, -6, -3, -1, 0, 2, 1, -1, -3, //224
-7, -10, -14, -20, -23, -26, -28, -27, -24, -21, -17, -13, -10, -8, -6, -7, //240
-9, -13, -16, -20, -23, -24, -24, -23, -20, -14, -9, -6, -2, 1, 2, 1, //256
-2, -5, -9, -14, -18, -23, -24, -23, -21, -20, -17, -15, -14, -13, -13, -13, //272
-16, -18, -21, -25, -30, -32, -34, -33, -32, -30, -28, -26, -24, -22, -20, -18, //288
-15, -13, -12, -12, -13, -13, -13, -12, -10, -8, -5, -2, 3, 6, 8, 10, //304
9, 8, 5, 2, -2, -4, -6, -8, -7, -6, -5, -5, -5, -6, -8, -10, //320
-13, -16, -19, -24, -28, -31, -33, -35, -37, -37, -36, -36, -35, -35, -34, -33, //336
-31, -29, -26, -23, -19, -17, -15, -13, -12, -9, -6, -3, -1, 0, 1, 1, //352
0, -1, -2, -3, -3, -4, -4, -5, -5, -5, -4, -3, -3, -4, -5, -5, //368
-8, -9, -10, -12, -13, -16, -19, -20, -22, -23, -24, -25, -25, -26, -28, -31, //384
-35, -38, -39, -38, -37, -36, -33, -32, -31, -30, -30, -30, -30, -29, -28, -27, //400
-27, -27, -28, -28, -27, -25, -24, -24, -24, -25, -26, -28, -30, -32, -33, -34, //416
-34, -35, -36, -37, -37, -37, -36, -35, -34, -35, -36, -36, -39, -41, -44, -46, //432
-45, -46, -47, -46, -47, -46, -46, -45, -45, -44, -43, -43, -43, -44, -44, -44, //448
-43, -43, -41, -39, -38, -35, -33, -31, -29, -28, -28, -27, -27, -26, -26, -25, //464
-24, -23, -22, -21, -20, -17, -10, -6, -2, 2, 5, 9, 12, 14, 15, 17, //480
19, 22, 24, 27, 29, 29, 28, 27, 25, 23, 19, 15, 10, 7, 4, 2, //496
1, -2, -5, -7, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, //512
-1, -2, -3, -3, -4, -3, -2, -1, -1, -2, -3, -4, -6, -8, -12, -14, //528
-16, -19, -22, -23, -25, -24, -24, -24, -23, -23, -23, -23, -24, -23, -22, -21, //544
-22, -23, -23, -25, -28, -31, -33, -35, -37, -39, -40, -42, -42, -42, -42, -44, //560
-49, -52, -57, -60, -62, -63, -61, -60, -58, -56, -55, -54, -53, -51, -48, -45, //576
-41, -35, -30, -26, -23, -22, -21, -18, -15, -13, -9, -5, 1, 6, 9, 10, //592
12, 12, 11, 11, 10, 10, 10, 10, 11, 12, 13, 13, 13, 14, 13, 11, //608
9, 8, 6, 4, 4, 3, 3, 2, 2, 2, 3, 5, 6, 8, 9, 9, //624
7, 5, 4, 2, 0, -1, -2, -2, -1, -1, 2, 4, 6, 9, 9, 8, //640
7, 5, 3, 2, 2, 4, 6, 8, 12, 16, 19, 22, 23, 24, 24, 23, //656
21, 19, 17, 16, 15, 15, 15, 16, 17, 18, 18, 17, 17, 16, 14, 13, //672
12, 10, 8, 6, 4, 3, 0, -3, -4, -4, -4, -4, -2, 1, 2, 3, //688
2, 1, 0, -1, -1, -1, -1, 1, 4, 5, 5, 5, 5, 6, 6, 7, //704
8, 8, 8, 8, 6, 4, 3, 1, -1, -3, -3, -3, -3, -3, -3, -3, //720
-4, -5, -6, -8, -9, -9, -8, -6, -5, -3, -2, -2, -2, -2, -2, -2, //736
-1, -1, -1, -1, 0, 0, 1, 1, 0, 0, -1, -1, -3, -5, -6, -6, //752
-6, -5, -4, -3, -2, 0, 3, 5, 6, 7, 8, 8, 8, 8, 7, 6, //768
5, 3, 2, 1, 0, 0, 2, 6, 9, 12, 15, 17, 19, 19, 20, 21, //784
21, 23, 26, 27, 28, 29, 28, 28, 28, 29, 30, 30, 31, 32, 33, 33, //800
33, 34, 34, 35, 34, 33, 33, 32, 32, 32, 33, 33, 33, 33, 31, 27, //816
24, 21, 18, 17, 16, 16, 17, 19, 19, 20, 23, 24, 24, 26, 26, 26, //832
26, 27, 28, 28, 28, 28, 27, 25, 25, 25, 25, 27, 28, 30, 32, 33, //848
33, 34, 32, 31, 30, 27, 27, 26, 26, 28, 28, 29, 31, 31, 29, 27, //864
24, 21, 15, 12, 9, 6, 4, 3, 2, 1, 1, 1, 0, 0, -1, -3, //880
-4, -7, -10, -12, -14, -15, -18, -20, -22, -21, -19, -17, -15, -11, -5, -2, //896
1, 3, 5, 7, 8, 8, 8, 7, 7, 7, 7, 7, 8, 9, 10, 13, //912
14, 15, 17, 17, 19, 21, 23, 24, 26, 28, 33, 35, 37, 39, 41, 42, //928
43, 43, 43, 45, 46, 48, 48, 48, 47, 46, 47, 49, 51, 53, 54, 55, //944
56, 57, 57, 57, 58, 57, 58, 58, 57, 58, 57, 58, 57, 58, 57, 57, //960
58, 57, 58, 57, 56, 55, 54, 53, 51, 50, 48, 46, 45, 41, 40, 39, //976
38, 38, 38, 37, 35, 33, 29, 26, 22, 18, 15, 12, 8, 4, -1, -4, //992
-6, -7, -7, -6, -7, -8, -9, -10, -12, -13, -15, -17, -18, -19, -21, -23, //1008
-24, -24, -24, -25, -27, -30, -31, -31, -32, -33, -33, -32, -31, -29, -28, -26, //1024
-22, -18, -13, -9, -5, -1, 0, 1, 1, 1, 2, 3, 4, 6, 7, 9, //1040
10, 13, 15, 16, 19, 21, 24, 25, 25, 27, 29, 29, 30, 30, 30, 30, //1056
30, 30, 32, 33, 35, 36, 37, 38, 37, 36, 35, 34, 34, 33, 32, 31, //1072
31, 30, 29, 31, 31, 31, 32, 31, 29, 28, 26, 26, 26, 26, 27, 26, //1088
25, 24, 21, 19, 16, 14, 12, 9, 7, 3, 1, -1, -2, -3, -3, -4, //1104
-5, -5, -4, -3, -2, 0, 1, 3, 4, 4, 7, 10, 12, 14, 17, 19, //1120
21, 22, 23, 23, 25, 26, 26, 24, 21, 19, 15, 12, 9, 6, 4, 3, //1136
1, 0, 0, -1, -2, -5, -8, -12, -14, -14, -13, -11, -6, -2, 1, 4, //1152
7, 9, 9, 9, 9, 9, 9, 9, 8, 6, 4, 3, 1, -1, -2, -2, //1168
-1, 0, 1, 5, 7, 7, 6, 5, 1, -3, -5, -8, -9, -8, -7, -5, //1184
-5, -6, -7, -9, -12, -13, -14, -15, -15, -15, -16, -17, -18, -19, -19, -19, //1200
-20, -19, -18, -17, -16, -14, -14, -14, -15, -15, -16, -17, -18, -19, -19, -19, //1216
-18, -18, -17, -16, -15, -13, -10, -8, -8, -7, -7, -8, -9, -10, -10, -10, //1232
-9, -8, -6, -4, -3, -2, -1, -2, -3, -4, -5, -6, -8, -9, -11, -12, //1248
-13, -13, -11, -8, -5, -1, 2, 5, 7, 8, 7, 6, 5, 3, 1, 1, //1264
2, 2, 4, 9, 11, 13, 13, 13, 12, 9, 7, 5, 4, 5, 5, 6, //1280
5, 4, 3, 3, 0, 0, 0, 1, 2, 2, 3, 2, 1, -1, -6, -14, //1296
-20, -25, -29, -31, -32, -31, -30, -30, -30, -31, -33, -36, -39, -41, -42, -43, //1312
-44, -44, -44, -43, -43, -42, -39, -38, -36, -35, -34, -33, -32, -32, -32, -32, //1328
-33, -35, -34, -33, -32, -30, -26, -22, -18, -16, -14, -11, -6, -3, 0, 3, //1344
4, 6, 7, 7, 6, 5, 4, 3, 3, 3, 3, 2, 1, -1, -3, -5, //1360
-6, -8, -8, -7, -7, -6, -5, -5, -5, -4, -4, -3, 0, 1, 4, 4, //1376
5, 4, 3, 1, -2, -4, -6, -7, -8, -8, -7, -6, -5, -5, -7, -11, //1392
-14, -16, -19, -21, -22, -24, -25, -25, -25, -24, -22, -17, -14, -9, -5, -3, //1408
-1, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -15, -16, -17, -17, //1424
-18, -20, -21, -23, -23, -24, -25, -27, -28, -30, -30, -29, -28, -24, -22, -21, //1440
-21, -21, -24, -26, -29, -31, -31, -30, -28, -26, -24, -23, -23, -23, -22, -21, //1456
-20, -19, -19, -20, -20, -21, -21, -21, -20, -19, -17, -14, -12, -10, -6, -4, //1472
-2, 0, 1, 1, 1, 0, -1, -2, -2, -2, -1, -1, 0, 0, 0, 0, //1488
0, 0, 1, 2, 2, 2, 1, -1, -2, -3, -3, -2, -2, -2, -2, -1, //1504
-1, 1, 2, 3, 5, 8, 11, 12, 12, 13, 13, 14, 17, 19, 20, 19, //1520
18, 13, 9, 6, 2, 1, 0, 0, 0, 1, 2, 4, 5, 5, 6, 6, //1536
6, 6, 6, 6, 7, 8, 9, 9, 9, 9, 9, 10, 11, 11, 13, 14, //1552
14, 13, 12, 11, 10, 9, 9, 8, 8, 8, 9, 11, 12, 13, 16, 19, //1568
21, 23, 24, 25, 27, 28, 28, 29, 28, 28, 27, 27, 28, 27, 27, 26, //1584
25, 22, 19, 14, 10, 5, 2, 0, -1, -2, -3, -5, -7, -8, -9, -9, //1600
-9, -8, -8, -7, -7, -7, -9, -8, -7, -6, -4, -2, 0, 0, -1, -2, //1616
-3, -3, -4, -5, -6, -8, -10, -10, -11, -11, -11, -13, -13, -12, -11, -9, //1632
-7, -4, 2, 4, 6, 8, 9, 10, 10, 10, 9, 8, 7, 6, 5, 4, //1648
3, 2, 1, -1, -3, -5, -7, -8, -9, -11, -11, -11, -10, -9, -8, -4, //1664
-1, 3, 7, 10, 12, 13, 13, 13, 14, 15, 16, 18, 19, 20, 22, 24, //1680
29, 31, 33, 35, 34, 33, 32, 31, 31, 31, 31, 32, 32, 32, 32, 33, //1696
35, 38, 40, 42, 44, 46, 47, 47, 46, 44, 42, 38, 33, 30, 28, 26, //1712
25, 24, 23, 21, 19, 16, 13, 11, 10, 9, 8, 5, 2, -2, -5, -7, //1728
-8, -9, -9, -9, -9, -10, -11, -12, -15, -17, -19, -21, -21, -22, -21, -20, //1744
-18, -17, -16, -14, -13, -13, -13, -12, -12, -12, -12, -12, -12, -12, -12, -10, //1760
-9, -8, -5, -5, -5, -5, -6, -7, -8, -7, -6, -4, -2, 0, 4, 7, //1776
11, 13, 14, 15, 14, 13, 11, 8, 6, 5, 3, 0, -2, -3, -3, -2, //1792
-1, -2, -4, -7, -11, -14, -19, -22, -23, -24, -24, -23, -23, -23, -23, -23, //1808
-24, -26, -29, -30, -32, -34, -37, -40, -41, -43, -44, -45, -46, -44, -40, -36, //1824
-32, -27, -24, -18, -16, -15, -15, -15, -17, -19, -20, -20, -22, -24, -27, -28, //1840
-29, -32, -34, -36, -40, -41, -43, -45, -46, -48, -48, -48, -46, -43, -39, -33, //1856
-29, -26, -22, -19, -18, -16, -16, -17, -18, -19, -19, -19, -18, -16, -15, -14, //1872
-12, -11, -10, -9, -9, -9, -9, -9, -9, -9, -9, -9, -9, -9, -9, -9, //1888
-8, -7, -6, -5, -4, -3, -2, -1, -1, -1, -2, -3, -3, -3, -3, -3, //1904
-4, -4, -4, -4, -4, -5, -6, -9, -11, -13, -14, -14, -14, -13, -12, -11, //1920
-10, -10, -10, -12, -14, -16, -19, -21, -24, -25, -25, -25, -25, -26, -27, -28, //1936
-29, -31, -33, -33, -34, -34, -34, -34, -34, -34, -32, -30, -29, -27, -26, -25, //1952
-25, -24, -22, -21, -20, -17, -15, -14, -13, -13, -12, -12, -13, -14, -14, -14, //1968
-13, -13, -12, -10, -9, -7, -7, -8, -9, -12, -14, -16, -15, -14, -12, -11, //1984
-11, -11, -11, -12, -13, -13, -15, -14, -12, -10, -9, -7, -4, -3, -4, -5, //2000
-5, -5, -3, -1, 1, 3, 4, 4, 5, 3, 2, 2, 2, 4, 7, 9, //2016
12, 14, 16, 17, 16, 15, 14, 12, 10, 6, 3, 0, -1, -3, -4, -5, //2032
-4, -4, -3, -3, -6, -8, -10, -13, -16, -18, -18, -18, -18, -19, -19, -23, //2048
-25, -28, -32, -34, -36, -37, -36, -35, -33, -32, -30, -28, -26, -24, -23, -22, //2064
-21, -19, -16, -14, -11, -8, -6, -5, -5, -7, -9, -10, -11, -11, -11, -10, //2080
-10, -9, -9, -8, -7, -7, -7, -9, -10, -10, -11, -11, -11, -10, -9, -8, //2096
-6, -6, -5, -5, -4, -3, -2, -2, -3, -4, -4, -3, -2, 0, 2, 2, //2112
2, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2, 4, 7, 11, 17, 21, //2128
25, 27, 29, 30, 33, 34, 35, 38, 40, 42, 44, 43, 42, 43, 43, 43, //2144
43, 44, 44, 44, 42, 40, 37, 34, 30, 26, 23, 22, 22, 23, 24, 24, //2160
24, 23, 22, 20, 19, 16, 12, 10, 8, 6, 4, 1, -3, -5, -8, -8, //2176
-8, -7, -5, -3, -2, 0, 1, 0, -2, -4, -7, -9, -11, -11, -11, -10, //2192
-9, -7, -6, -7, -8, -8, -8, -10, -10, -11, -12, -13, -13, -14, -14, -14, //2208
-14, -13, -12, -8, -6, -5, -5, -6, -8, -12, -14, -15, -17, -19, -19, -19, //2224
-18, -17, -16, -15, -13, -11, -10, -9, -8, -8, -10, -9, -8, -7, -5, -2, //2240
0, 2, 3, 4, 6, 8, 9, 10, 11, 12, 12, 14, 16, 17, 19, 21, //2256
24, 26, 28, 30, 33, 35, 37, 38, 39, 39, 39, 38, 35, 34, 33, 33, //2272
34, 37, 39, 42, 44, 46, 47, 48, 47, 45, 43, 41, 40, 39, 38, 37, //2288
37, 37, 37, 36, 33, 31, 27, 23, 19, 18, 18, 18, 18, 18, 17, 17, //2304
18, 19, 19, 21, 22, 21, 21, 21, 21, 23, 25, 26, 27, 29, 30, 31, //2320
31, 31, 33, 33, 33, 32, 32, 31, 30, 30, 29, 29, 31, 32, 34, 34, //2336
33, 31, 28, 26, 25, 22, 20, 19, 18, 18, 18, 21, 23, 24, 26, 26, //2352
26, 24, 23, 21, 19, 18, 15, 11, 7, 3, 0, -1, 0, 3, 8, 12, //2368
17, 20, 21, 21, 21, 21, 22, 24, 25, 26, 26, 26, 25, 26, 26, 26, //2384
26, 25, 23, 21, 19, 18, 17, 16, 14, 12, 11, 11, 11, 12, 17, 20, //2400
22, 24, 24, 22, 20, 17, 16, 15, 15, 14, 13, 13, 13, 15, 16, 18, //2416
18, 19, 19, 19, 18, 19, 21, 22, 23, 25, 26, 27, 28, 28, 29, 30, //2432
31, 30, 28, 26, 23, 20, 18, 18, 19, 20, 22, 27, 30, 32, 34, 37, //2448
38, 40, 41, 41, 40, 38, 37, 36, 35, 33, 32, 32, 31, 30, 29, 27, //2464
26, 23, 19, 16, 12, 8, 6, 3, 3, 4, 5, 6, 8, 9, 9, 8, //2480
7, 6, 6, 3, 2, 1, 0, -1, -2, -3, -2, -2, -1, 1, 4, 5, //2496
6, 8, 10, 10, 10, 9, 8, 8, 8, 10, 11, 13, 16, 18, 19, 21, //2512
20, 19, 18, 15, 14, 13, 14, 15, 18, 22, 29, 33, 38, 44, 47, 49, //2528
51, 49, 47, 45, 43, 41, 40, 39, 38, 37, 36, 33, 31, 29, 27, 26, //2544
24, 22, 20, 18, 17, 15, 13, 14, 15, 18, 20, 23, 25, 26, 27, 27, //2560
28, 27, 24, 21, 18, 13, 9, 3, 0, -2, -3, -3, -3, -4, -4, -5, //2576
-6, -6, -8, -11, -13, -15, -18, -20, -21, -21, -19, -17, -15, -12, -9, -7, //2592
-7, -7, -7, -9, -10, -11, -12, -12, -12, -11, -11, -11, -11, -11, -11, -10, //2608
-9, -8, -7, -7, -8, -12, -14, -17, -19, -20, -17, -12, -7, -1, 3, 7, //2624
10, 8, 6, 2, -1, -3, -5, -5, -4, -3, -1, 0, 1, 2, 2, 3, //2640
3, 1, 0, -2, -3, -4, -3, -2, -1, 2, 5, 5, 5, 3, 3, 3, //2656
3, 4, 3, 2, 1, 0, -2, -4, -6, -7, -7, -6, -6, -6, -7, -9, //2672
-10, -12, -15, -21, -23, -26, -28, -28, -28, -26, -23, -20, -17, -16, -17, -19, //2688
-20, -22, -21, -20, -18, -18, -18, -18, -21, -25, -27, -28, -30, -32, -32, -35, //2704
-36, -37, -40, -42, -46, -50, -51, -52, -53, -53, -50, -47, -43, -40, -38, -37, //2720
-37, -37, -39, -41, -42, -43, -45, -44, -43, -41, -39, -38, -36, -35, -34, -33, //2736
-33, -32, -33, -33, -33, -33, -34, -34, -34, -34, -35, -36, -37, -37, -38, -37, //2752
-36, -33, -28, -24, -21, -19, -17, -15, -12, -9, -7, -3, 3, 10, 14, 18, //2768
20, 22, 22, 20, 18, 17, 17, 17, 19, 23, 25, 29, 32, 33, 34, 32, //2784
29, 28, 28, 28, 29, 29, 28, 27, 26, 23, 21, 19, 17, 16, 14, 11, //2800
9, 7, 6, 3, 0, -6, -9, -12, -15, -16, -16, -15, -14, -14, -15, -15, //2816
-17, -18, -19, -20, -22, -22, -24, -25, -27, -28, -29, -29, -28, -27, -25, -24, //2832
-24, -25, -26, -27, -30, -32, -33, -35, -36, -36, -37, -38, -39, -40, -39, -38, //2848
-36, -34, -33, -32, -33, -35, -38, -42, -44, -44, -43, -41, -39, -37, -35, -35, //2864
-34, -34, -33, -32, -30, -28, -26, -24, -21, -20, -19, -19, -20, -20, -19, -18, //2880
-16, -15, -14, -13, -14, -15, -16, -17, -17, -16, -12, -8, -2, 2, 5, 9, //2896
11, 12, 13, 14, 14, 16, 17, 18, 21, 24, 27, 30, 30, 31, 31, 30, //2912
30, 31, 31, 31, 31, 30, 28, 25, 23, 21, 20, 20, 21, 21, 21, 21, //2928
20, 19, 16, 13, 8, 4, 0, -4, -4, -3, -2, 1, 3, 4, 3, 1, //2944
-1, -3, -6, -9, -11, -13, -14, -16, -17, -19, -20, -22, -22, -23, -23, -24, //2960
-24, -24, -24, -25, -26, -26, -26, -26, -27, -27, -25, -22, -19, -15, -11, -9, //2976
-8, -8, -9, -9, -12, -15, -16, -19, -21, -20, -18, -16, -14, -12, -11, -13, //2992
-14, -15, -16, -16, -16, -14, -13, -12, -11, -11, -11, -11, -11, -12, -12, -11, //3008
-8, -6, -5, -5, -5, -7, -9, -10, -10, -9, -8, -7, -4, -2, -1, -1, //3024
-2, -6, -9, -13, -17, -20, -22, -24, -23, -21, -20, -16, -13, -9, -7, -6, //3040
-5, -4, -1, 2, 4, 8, 11, 14, 17, 18, 19, 20, 20, 21, 23, 23, //3056
23, 22, 22, 22, 22, 23, 23, 23, 24, 27, 29, 31, 33, 34, 35, 35, //3072
33, 30, 28, 26, 23, 21, 20, 18, 17, 16, 15, 14, 13, 12, 11, 8, //3088
6, 5, 4, 4, 5, 6, 7, 7, 6, 4, 3, 3, 3, 3, 4, 4, //3104
4, 3, 1, 0, -1, -1, 0, 2, 4, 6, 8, 11, 14, 15, 17, 18, //3120
18, 18, 17, 17, 17, 17, 18, 18, 17, 17, 16, 15, 14, 13, 14, 15, //3136
16, 18, 19, 20, 21, 21, 24, 27, 31, 32, 32, 33, 32, 31, 28, 26, //3152
24, 24, 24, 25, 24, 23, 22, 20, 19, 18, 17, 17, 17, 17, 16, 16, //3168
15, 14, 13, 13, 12, 13, 14, 15, 17, 19, 21, 20, 18, 16, 14, 12, //3184
11, 12, 13, 16, 18, 22, 24, 28, 32, 35, 38, 40, 40, 38, 35, 31, //3200
28, 25, 24, 23, 23, 25, 28, 30, 31, 31, 32, 31, 27, 24, 22, 21, //3216
21, 23, 26, 29, 32, 34, 36, 39, 40, 42, 42, 43, 42, 40, 38, 35, //3232
33, 29, 26, 24, 23, 23, 24, 26, 28, 28, 27, 26, 25, 23, 20, 18, //3248
17, 17, 18, 21, 23, 24, 26, 26, 26, 28, 29, 31, 31, 32, 33, 33, //3264
33, 33, 32, 31, 31, 30, 30, 30, 29, 29, 28, 27, 26, 26, 25, 24, //3280
23, 23, 22, 21, 20, 19, 19, 19, 19, 19, 19, 18, 17, 16, 16, 16, //3296
16, 17, 17, 17, 17, 16, 15, 13, 11, 9, 8, 6, 3, 1, 0, 0, //3312
-1, -2, -3, -3, -3, -2, -1, 1, 1, 1, 1, 1, 1, 3, 4, 6, //3328
7, 9, 11, 13, 14, 13, 11, 10, 6, 4, 2, 0, -1, -3, -3, -2, //3344
-2, -1, -1, -1, 0, 0, -1, -2, -3, -6, -9, -11, -11, -11, -11, -12, //3360
-13, -13, -14, -14, -14, -14, -14, -15, -17, -20, -25, -27, -29, -31, -34, -35, //3376
-37, -39, -39, -40, -40, -42, -42, -43, -44, -45, -46, -46, -46, -45, -44, -43, //3392
-43, -42, -41, -40, -40, -40, -41, -43, -46, -49, -54, -58, -63, -67, -70, -72, //3408
-74, -75, -78, -80, -82, -83, -84, -86, -87, -88, -88, -88, -88, -88, -87, -86, //3424
-86, -85, -84, -81, -80, -80, -79, -78, -75, -74, -72, -71, -70, -69, -68, -67, //3440
-65, -62, -60, -58, -56, -56, -56, -58, -60, -65, -67, -68, -69, -69, -68, -66, //3456
-64, -62, -61, -62, -62, -63, -64, -64, -63, -61, -57, -55, -53, -54, -54, -55, //3472
-54, -54, -52, -49, -47, -46, -45, -47, -49, -50, -51, -52, -52, -51, -50, -48, //3488
-46, -45, -43, -42, -41, -41, -41, -41, -42, -42, -43, -43, -42, -42, -43, -43, //3504
-43, -40, -38, -37, -36, -35, -35, -35, -36, -37, -38, -40, -42, -46, -47, -49, //3520
-51, -52, -53, -54, -53, -51, -50, -49, -48, -46, -45, -43, -41, -40, -38, -37, //3536
-35, -34, -33, -32, -30, -29, -28, -26, -25, -24, -24, -25, -25, -27, -29, -35, //3552
-37, -38, -37, -36, -35, -34, -34, -33, -33, -34, -35, -35, -35, -34, -33, -32, //3568
-32, -32, -32, -34, -36, -35, -33, -30, -27, -25, -22, -21, -20, -19, -18, -17, //3584
-16, -14, -11, -9, -5, -1, 3, 8, 9, 9, 10, 10, 10, 11, 12, 13, //3600
14, 14, 17, 19, 22, 25, 28, 30, 32, 33, 32, 30, 29, 26, 25, 24, //3616
24, 24, 24, 23, 21, 20, 20, 20, 21, 21, 21, 21, 20, 19, 17, 16, //3632
16, 17, 18, 18, 19, 21, 22, 23, 23, 22, 22, 22, 23, 23, 24, 26, //3648
27, 29, 29, 32, 34, 40, 44, 48, 50, 52, 53, 54, 55, 55, 56, 56, //3664
57, 57, 58, 59, 60, 61, 62, 62, 60, 58, 53, 47, 44, 42, 41, 41, //3680
40, 38, 35, 32, 29, 25, 22, 19, 17, 17, 19, 20, 22, 23, 22, 22, //3696
22, 22, 24, 25, 28, 30, 32, 33, 32, 30, 27, 24, 20, 16, 15, 16, //3712
17, 20, 25, 31, 34, 36, 38, 39, 40, 39, 39, 39, 41, 43, 46, 47, //3728
47, 48, 48, 48, 48, 48, 49, 50, 52, 53, 54, 53, 51, 50, 50, 49, //3744
50, 52, 54, 57, 58, 59, 60, 59, 57, 55, 53, 49, 46, 44, 42, 39, //3760
36, 34, 34, 34, 34, 35, 38, 41, 43, 44, 43, 44, 45, 45, 46, 47, //3776
49, 52, 56, 60, 63, 67, 71, 74, 75, 76, 77, 78, 81, 82, 84, 85, //3792
86, 86, 86, 85, 84, 82, 79, 77, 73, 72, 71, 70, 68, 65, 61, 58, //3808
56, 54, 52, 49, 47, 44, 43, 42, 42, 44, 44, 45, 46, 45, 44, 42, //3824
40, 38, 36, 35, 32, 31, 29, 28, 27, 25, 21, 18, 15, 14, 14, 15, //3840
18, 20, 24, 27, 29, 31, 31, 30, 29, 28, 27, 28, 29, 30, 32, 32, //3856
32, 30, 27, 25, 21, 18, 16, 15, 14, 12, 11, 8, 2, -1, -4, -6, //3872
-6, -8, -8, -7, -7, -7, -6, -6, -6, -6, -6, -5, -4, -2, 0, 2, //3888
4, 7, 9, 10, 11, 11, 11, 11, 12, 13, 14, 15, 15, 15, 15, 15, //3904
15, 16, 16, 18, 19, 20, 22, 22, 22, 23, 22, 22, 22, 22, 24, 26, //3920
28, 30, 32, 33, 33, 33, 33, 33, 33, 32, 31, 30, 28, 26, 24, 23, //3936
22, 21, 20, 19, 18, 15, 12, 9, 6, 3, 1, -1, -1, -2, -3, -5, //3952
-5, -6, -5, -4, -3, -3, 0, 1, 1, 1, 0, -1, -5, -7, -9, -12, //3968
-13, -13, -12, -11, -9, -9, -9, -12, -15, -17, -20, -22, -23, -26, -26, -25, //3984
-25, -25, -27, -28, -28, -27, -26, -25, -25, -24, -22, -21, -20, -18, -18, -17, //4000
-17, -17, -17, -17, -18, -19, -20, -22, -24, -27, -29, -29, -29, -28, -26, -23, //4016
-22, -21, -21, -21, -22, -22, -22, -22, -22, -21, -19, -18, -18, -18, -18, -18, //4032
-20, -21, -22, -24, -26, -28, -29, -29, -29, -29, -29, -28, -26, -24, -22, -20, //4048
-18, -14, -11, -9, -8, -7, -7, -8, -10, -11, -14, -18, -22, -24, -23, -21, //4064
-20, -18, -15, -13, -12, -12, -15, -19, -21, -24, -26, -27, -26, -25, -23, -21, //4080
-20, -20, -20, -22, -24, -26, -29, -31, -33, -34, -35, -35, -36, -37, -39, -39, //4096
-38, -37, -35, -32, -31, -30, -30, -30, -30, -33, -34, -36, -36, -35, -34, -30, //4112
-27, -25, -24, -23, -23, -23, -24, -24, -24, -25, -25, -24, -23, -22, -22, -22, //4128
-24, -25, -26, -25, -24, -21, -19, -17, -16, -16, -17, -23, -26, -29, -30, -29, //4144
-28, -26, -25, -23, -22, -21, -21, -22, -23, -25, -27, -27, -29, -29, -29, -30, //4160
-30, -30, -35, -36, -39, -41, -43, -45, -45, -46, -45, -45, -45, -45, -47, -50, //4176
-51, -53, -54, -55, -56, -56, -55, -53, -52, -52, -52, -54, -56, -58, -59, -58, //4192
-59, -60, -59, -58, -57, -57, -55, -53, -51, -49, -47, -46, -44, -43, -43, -42, //4208
-40, -38, -36, -34, -31, -28, -28, -28, -28, -29, -31, -31, -31, -33, -33, -32, //4224
-31, -29, -26, -22, -18, -13, -10, -8, -6, -5, -4, -3, -2, -2, -2, -3, //4240
-3, -6, -6, -7, -6, -5, -5, -2, -1, 0, 3, 5, 8, 9, 10, 9, //4256
8, 7, 4, 3, 3, 2, 2, 3, 3, 3, 1, -2, -5, -11, -14, -16, //4272
-18, -18, -19, -18, -17, -17, -17, -17, -17, -17, -16, -16, -17, -18, -21, -22, //4288
-23, -24, -24, -24, -25, -25, -25, -25, -25, -27, -29, -31, -33, -35, -36, -35, //4304
-34, -33, -32, -31, -30, -28, -26, -24, -23, -20, -18, -17, -17, -17, -17, -16, //4320
-15, -15, -14, -14, -13, -13, -13, -12, -11, -11, -10, -9, -9, -9, -9, -8, //4336
-7, -4, -1, 1, 2, 2, 2, 0, 0, 0, 1, 2, 6, 9, 11, 12, //4352
14, 14, 15, 15, 16, 16, 16, 17, 19, 20, 19, 18, 15, 8, 3, 0, //4368
-3, -4, -4, -2, 0, 1, 2, 2, 2, 1, 1, 0, -1, -1, 0, 2, //4384
3, 4, 6, 6, 7, 5, 3, 1, 1, 1, 1, 1, 1, 0, -1, -1, //4400
-1, -1, 0, 0, 0, 0, 0, 1, 2, 4, 5, 6, 5, 4, 3, 1, //4416
1, 1, 1, 0, -1, -3, -4, -5, -5, -4, -3, -2, -1, 0, 1, 2, //4432
4, 7, 9, 10, 11, 14, 16, 22, 25, 29, 33, 35, 35, 34, 32, 31, //4448
31, 31, 31, 31, 31, 30, 30, 30, 30, 31, 31, 32, 33, 34, 34, 33, //4464
32, 31, 31, 30, 29, 29, 29, 30, 31, 33, 34, 34, 34, 33, 32, 31, //4480
29, 28, 27, 24, 20, 17, 15, 12, 10, 8, 5, 4, 3, 2, 0, -4, //4496
-8, -12, -16, -21, -24, -27, -28, -30, -32, -34, -37, -41, -42, -42, -40, -38, //4512
-37, -35, -34, -33, -33, -33, -35, -36, -36, -37, -38, -39, -38, -37, -36, -35, //4528
-35, -35, -35, -35, -34, -33, -30, -26, -23, -21, -17, -13, -9, 0, 6, 12, //4544
15, 17, 22, 24, 27, 29, 30, 31, 32, 33, 33, 33, 34, 36, 37, 37, //4560
39, 42, 44, 46, 47, 48, 47, 46, 45, 45, 45, 46, 49, 51, 55, 57, //4576
60, 63, 65, 68, 69, 69, 69, 69, 69, 68, 66, 65, 62, 59, 57, 53, //4592
52, 51, 49, 48, 47, 45, 42, 40, 38, 36, 33, 29, 25, 22, 20, 19, //4608
19, 20, 21, 22, 24, 26, 28, 28, 26, 24, 20, 16, 9, 4, 2, 0, //4624
-2, -3, -4, -4, -4, -4, -4, -3, -2, -1, -1, -1, -1, -3, -4, -4, //4640
-4, -3, -3, -2, -2, -1, 1, 2, 2, 3, 2, 1, -1, -3, -6, -8, //4656
-11, -12, -14, -15, -15, -14, -13, -12, -11, -10, -9, -9, -9, -10, -10, -10, //4672
-10, -9, -8, -6, -2, 1, 3, 6, 8, 10, 10, 9, 7, 5, 4, 3, //4688
4, 5, 6, 8, 11, 13, 15, 16, 17, 17, 16, 15, 15, 16, 17, 19, //4704
22, 23, 24, 25, 25, 25, 27, 27, 28, 28, 27, 27, 25, 24, 22, 20, //4720
19, 17, 18, 19, 19, 21, 22, 25, 25, 25, 24, 21, 18, 15, 13, 12, //4736
12, 12, 13, 13, 13, 13, 12, 9, 5, 2, 0, -3, -5, -7, -7, -6, //4752
-5, -4, -2, 2, 5, 7, 9, 11, 13, 13, 12, 11, 10, 9, 7, 5, //4768
5, 6, 7, 8, 12, 14, 15, 15, 14, 13, 9, 7, 5, 2, 2, 2, //4784
3, 4, 5, 7, 10, 13, 14, 14, 13, 12, 9, 6, 4, 2, 0, 0, //4800
0, -1, -2, -3, -3, -2, 1, 3, 4, 5, 5, 5, 2, 1, 0, 0, //4816
0, -1, 0, 0, 1, 1, 1, 0, 0, -1, -2, -4, -5, -8, -9, -9, //4832
-8, -7, -6, -4, -2, -1, -1, -2, -3, -4, -6, -6, -7, -7, -5, -3, //4848
-1, 0, 2, 2, 2, 1, -1, -2, -3, -4, -5, -5, -5, -7, -9, -12, //4864
-14, -15, -16, -18, -18, -19, -20, -21, -23, -25, -29, -30, -31, -31, -30, -29, //4880
-28, -27, -26, -26, -26, -28, -29, -30, -31, -33, -35, -35, -36, -36, -35, -35, //4896
-35, -36, -36, -36, -35, -34, -34, -34, -34, -33, -33, -33, -33, -33, -32, -31, //4912
-29, -28, -27, -27, -26, -26, -26, -26, -27, -28, -31, -33, -34, -36, -35, -33, //4928
-32, -30, -29, -28, -28, -28, -28, -29, -31, -31, -32, -31, -31, -30, -28, -26, //4944
-24, -22, -20, -17, -14, -12, -10, -9, -8, -7, -5, -4, -3, -2, -1, -1, //4960
1, 3, 4, 5, 5, 5, 4, 3, 2, 0, -2, -3, -4, -4, -4, -3, //4976
1, 3, 5, 7, 7, 6, 1, -3, -7, -11, -13, -16, -16, -16, -16, -16, //4992
-16, -18, -20, -22, -25, -29, -34, -37, -38, -40, -41, -41, -41, -40, -38, -36, //5008
-34, -31, -29, -27, -26, -26, -26, -28, -29, -31, -32, -34, -34, -36, -36, -35, //5024
-34, -33, -30, -30, -30, -30, -32, -33, -35, -36, -36, -35, -33, -32, -28, -25, //5040
-22, -20, -19, -19, -20, -21, -22, -24, -24, -24, -22, -20, -18, -15, -13, -9, //5056
-6, -4, -1, 2, 4, 6, 7, 7, 7, 7, 7, 7, 8, 9, 10, 11, //5072
11, 12, 13, 13, 12, 11, 8, 7, 5, 5, 5, 7, 9, 10, 12, 14, //5088
16, 17, 18, 18, 17, 17, 17, 14, 14, 13, 13, 14, 15, 15, 16, 16, //5104
16, 16, 18, 19, 20, 20, 20, 21, 21, 22, 22, 22, 24, 25, 26, 25, //5120
24, 23, 22, 20, 19, 19, 18, 17, 16, 16, 15, 15, 15, 15, 15, 15, //5136
15, 15, 15, 16, 15, 14, 13, 12, 12, 11, 11, 12, 13, 14, 14, 14, //5152
13, 12, 11, 11, 11, 11, 11, 12, 13, 12, 12, 11, 11, 11, 11, 13, //5168
16, 18, 19, 20, 20, 20, 21, 22, 24, 27, 30, 36, 38, 41, 43, 45, //5184
47, 50, 53, 54, 54, 53, 52, 51, 50, 50, 49, 47, 45, 43, 42, 41, //5200
39, 38, 37, 36, 35, 35, 34, 34, 37, 38, 40, 40, 41, 40, 40, 40, //5216
39, 39, 38, 37, 35, 32, 30, 27, 26, 24, 21, 19, 18, 16, 13, 11, //5232
11, 12, 12, 12, 13, 14, 15, 16, 16, 16, 16, 17, 17, 17, 19, 23, //5248
26, 28, 30, 32, 33, 36, 37, 38, 40, 41, 43, 45, 45, 46, 46, 46, //5264
46, 46, 46, 45, 44, 43, 42, 40, 38, 37, 37, 36, 36, 38, 38, 39, //5280
40, 42, 42, 42, 43, 43, 44, 46, 48, 49, 50, 50, 49, 47, 45, 44, //5296
44, 44, 43, 42, 40, 38, 36, 33, 31, 31, 31, 32, 34, 39, 40, 41, //5312
43, 45, 46, 48, 50, 51, 52, 53, 54, 53, 53, 52, 50, 47, 44, 41, //5328
39, 37, 35, 32, 30, 27, 24, 22, 19, 18, 15, 15, 14, 13, 12, 11, //5344
11, 10, 10, 9, 8, 8, 9, 10, 12, 13, 14, 15, 15, 16, 16, 16, //5360
16, 15, 16, 16, 17, 19, 20, 22, 23, 23, 25, 25, 25, 25, 24, 22, //5376
21, 17, 14, 11, 9, 8, 8, 9, 11, 13, 14, 15, 15, 14, 12, 10, //5392
9, 6, 3, 0, -3, -4, -6, -6, -5, -4, -3, -4, -5, -6, -10, -12, //5408
-14, -16, -16, -17, -16, -15, -14, -13, -12, -11, -9, -7, -6, -6, -7, -11, //5424
-13, -17, -22, -25, -28, -29, -29, -28, -26, -25, -24, -24, -25, -26, -27, -30, //5440
-32, -33, -35, -35, -36, -35, -32, -30, -27, -24, -22, -19, -17, -16, -16, -17, //5456
-17, -18, -18, -19, -18, -17, -15, -13, -10, -7, -4, -1, 2, 2, 2, 2, //5472
1, 1, 0, -1, 0, 1, 2, 5, 8, 10, 11, 11, 11, 10, 9, 8, //5488
8, 8, 7, 6, 5, 5, 5, 6, 8, 10, 12, 13, 12, 11, 9, 9, //5504
8, 7, 6, 5, 4, 2, 1, 0, -1, -1, -1, -2, -3, -4, -6, -10, //5520
-13, -16, -18, -20, -23, -24, -25, -27, -28, -30, -33, -37, -40, -43, -46, -49, //5536
-54, -56, -58, -59, -59, -60, -61, -61, -62, -64, -65, -66, -68, -71, -73, -77, //5552
-80, -85, -88, -90, -93, -96, -100, -103, -106, -108, -112, -115, -119, -122, -123, -125, //5568
-127, -127, -125, -122, -118, -114, -111, -106, -103, -101, -99, -98, -98, -98, -96, -95, //5584
-92, -89, -83, -79, -74, -71, -68, -66, -64, -62, -60, -58, -56, -53, -49, -46, //5600
-43, -39, -35, -29, -23, -18, -12, -7, -3, 4, 8, 11, 14, 17, 19, 22, //5616
23, 23, 25, 25, 24, 24, 24, 24, 24, 25, 27, 27, 27, 26, 23, 21, //5632
18, 17, 16, 16, 17, 20, 22, 25, 28, 31, 33, 38, 39, 41, 42, 43, //5648
42, 37, 34, 31, 28, 24, 20, 18, 16, 14, 13, 12, 12, 12, 12, 11, //5664
8, 5, -1, -4, -5, -5, -3, 1, 4, 5, 5, 4, 3, 0, -2, -4, //5680
-5, -6, -8, -14, -20, -25, -30, -35, -39, -39, -40, -39, -38, -38, -37, -36, //5696
-36, -36, -36, -37, -38, -38, -37, -35, -33, -29, -25, -23, -21, -18, -17, -16, //5712
-17, -17, -17, -19, -19, -20, -19, -18, -17, -15, -12, -10, -8, -7, -6, -4, //5728
-2, -1, -1, -2, -2, -5, -5, -5, -3, 2, 8, 16, 21, 25, 28, 31, //5744
32, 34, 35, 35, 36, 38, 41, 43, 44, 45, 47, 48, 49, 50, 51, 53, //5760
54, 55, 56, 55, 55, 54, 53, 55, 56, 58, 60, 61, 62, 62, 61, 58, //5776
55, 51, 47, 42, 39, 37, 36, 36, 36, 37, 39, 39, 40, 39, 38, 37, //5792
35, 33, 31, 28, 27, 27, 28, 28, 28, 29, 29, 28, 28, 27, 26, 24, //5808
21, 19, 17, 15, 12, 8, 6, 5, 5, 5, 6, 6, 6, 7, 6, 6, //5824
4, 3, 2, 0, 0, 0, 2, 3, 4, 5, 7, 7, 6, 5, 3, 0, //5840
-3, -8, -10, -13, -16, -18, -20, -19, -17, -15, -13, -11, -9, -9, -9, -9, //5856
-8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -4, -3, -2, -3, -3, -4, //5872
-7, -10, -15, -23, -28, -32, -35, -37, -42, -43, -44, -45, -46, -48, -51, -51, //5888
-52, -53, -54, -54, -53, -52, -53, -52, -51, -49, -47, -44, -41, -37, -36, -33, //5904
-32, -31, -29, -27, -25, -22, -21, -21, -20, -19, -15, -11, -8, -4, 0, 4, //5920
9, 11, 13, 14, 14, 14, 16, 17, 20, 23, 25, 28, 29, 30, 31, 33, //5936
34, 36, 37, 37, 37, 38, 38, 38, 37, 37, 37, 37, 38, 39, 40, 40, //5952
40, 40, 40, 40, 41, 42, 43, 44, 46, 47, 46, 45, 43, 42, 41, 40, //5968
39, 38, 37, 36, 34, 33, 32, 31, 30, 27, 25, 22, 20, 19, 17, 17, //5984
16, 17, 17, 17, 18, 18, 17, 16, 16, 17, 18, 18, 19, 19, 19, 17, //6000
14, 12, 7, 3, -1, -7, -10, -14, -18, -21, -23, -28, -30, -33, -35, -37, //6016
-39, -39, -41, -40, -40, -39, -37, -36, -34, -33, -32, -31, -32, -33, -33, -35, //6032
-36, -38, -37, -36, -36, -35, -35, -34, -32, -30, -28, -25, -21, -18, -17, -16, //6048
-14, -11, -7, -4, -1, 3, 8, 11, 17, 20, 23, 26, 29, 32, 34, 36, //6064
38, 39, 40, 41, 41, 40, 39, 38, 37, 35, 34, 33, 32, 31, 28, 26, //6080
24, 22, 21, 21, 21, 22, 25, 29, 32, 35, 40, 43, 46, 49, 51, 53, //6096
53, 53, 52, 50, 48, 45, 42, 40, 38, 36, 34, 33, 33, 32, 32, 31, //6112
29, 28, 28, 28, 28, 29, 31, 31, 31, 32, 32, 33, 33, 34, 34, 33, //6128
32, 28, 26, 25, 24, 24, 23, 22, 19, 16, 14, 10, 8, 5, 3, 3, //6144
4, 4, 6, 5, 5, 4, 2, 0, -3, -4, -5, -4, -3, -3, -3, -4, //6160
-6, -7, -8, -8, -6, -4, -3, -3, -3, -5, -6, -7, -9, -10, -12, -13, //6176
-13, -14, -14, -14, -14, -13, -13, -13, -13, -14, -18, -20, -21, -22, -22, -21, //6192
-20, -18, -17, -16, -14, -11, -10, -9, -9, -11, -12, -12, -12, -11, -10, -10, //6208
-10, -11, -13, -14, -15, -14, -11, -9, -7, -6, -5, -5, -4, -4, -4, -5, //6224
-5, -6, -7, -9, -10, -11, -13, -17, -20, -24, -27, -29, -31, -33, -33, -35, //6240
-35, -34, -34, -34, -33, -31, -29, -26, -24, -22, -21, -20, -17, -14, -9, -6, //6256
-2, 1, 4, 6, 6, 5, 3, 2, 0, -4, -5, -6, -5, -4, -4, -4, //6272
-5, -6, -6, -8, -10, -12, -13, -13, -15, -15, -15, -15, -14, -14, -13, -13, //6288
-13, -13, -12, -12, -12, -12, -13, -14, -15, -18, -21, -27, -31, -35, -38, -40, //6304
-41, -42, -42, -43, -44, -45, -47, -48, -47, -46, -45, -44, -41, -39, -36, -34, //6320
-32, -31, -30, -30, -30, -30, -32, -34, -36, -39, -42, -45, -48, -51, -52, -53, //6336
-53, -52, -50, -49, -47, -45, -43, -42, -40, -37, -35, -34, -32, -32, -31, -29, //6352
-28, -27, -24, -24, -25, -26, -27, -29, -30, -31, -31, -30, -30, -30, -31, -31, //6368
-30, -29, -28, -26, -25, -23, -22, -20, -19, -18, -17, -17, -15, -13, -11, -8, //6384
-5, -3, -1, 0, 2, 2, 1, 0, };
