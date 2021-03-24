const unsigned int revSampleRate = 22050;
const unsigned int revSampleCount = 4396;
const signed char revSamples[] = {
0, 3, 8, 9, 10, 10, 14, 15, 16, 17, 20, 25, 32, 38, 41, 41,
40, 38, 38, 37, 37, 37, 39, 45, 46, 48, 51, 54, 57, 61, 64, 66, 
67, 66, 68, 69, 69, 66, 62, 58, 57, 57, 58, 57, 55, 52, 50, 51, 
53, 52, 54, 58, 62, 65, 67, 68, 65, 62, 63, 64, 63, 59, 58, 57, 
56, 57, 57, 58, 61, 64, 67, 69, 69, 67, 63, 60, 59, 58, 56, 54, 
53, 51, 50, 51, 51, 51, 51, 50, 47, 45, 43, 36, 31, 27, 26, 23, 
20, 20, 23, 27, 29, 31, 35, 34, 30, 25, 21, 16, 11, 11, 12, 14, 
16, 19, 27, 31, 33, 32, 31, 26, 21, 17, 12, 7, 4, 8, 12, 15, 
15, 17, 17, 13, 11, 9, 5, 1, -7, -12, -16, -18, -16, -13, -11, -11, 
-10, -10, -11, -13, -14, -15, -16, -17, -17, -19, -20, -19, -16, -14, -15, -21, 
-25, -28, -30, -30, -29, -29, -27, -24, -20, -17, -17, -17, -18, -22, -26, -32, 
-38, -40, -42, -43, -45, -50, -54, -58, -59, -56, -53, -49, -47, -45, -44, -45, 
-46, -40, -34, -28, -26, -29, -38, -40, -38, -34, -28, -20, -12, -9, -8, -9, 
-10, -11, -10, -9, -9, -10, -10, -10, -12, -12, -11, -12, -15, -23, -28, -29, 
-28, -25, -21, -13, -9, -7, -4, -1, 4, 4, 0, -5, -8, -7, -4, -1, 
2, 3, 2, 2, 5, 8, 10, 14, 20, 23, 19, 15, 11, 8, 6, 9, 
12, 13, 15, 13, 10, 5, 4, 6, 9, 11, 11, 8, 5, 2, 0, -1, 
6, 13, 16, 14, 11, 11, 11, 13, 20, 26, 28, 18, 7, -4, -13, -18, 
-19, -14, -9, -5, -3, 0, 5, 7, 6, 4, 1, -3, -11, -14, -13, -12, 
-11, -10, -12, -16, -19, -20, -19, -16, -12, -9, -8, -8, -9, -12, -13, -14, 
-15, -16, -16, -14, -10, -8, -4, 1, 2, -1, -4, -4, -1, 2, 6, 9, 
11, 15, 17, 17, 15, 13, 12, 12, 13, 17, 20, 25, 30, 35, 40, 45, 
43, 40, 35, 31, 28, 28, 31, 34, 34, 31, 21, 14, 9, 9, 12, 14, 
15, 17, 19, 18, 15, 12, 10, 8, 6, 5, 4, 5, 7, 9, 12, 12, 
10, 6, 4, 3, 2, 1, 1, 3, 5, 8, 14, 19, 24, 22, 18, 13, 
8, 6, 9, 11, 13, 14, 14, 13, 10, 9, 9, 7, 4, 3, 3, 4, 
7, 10, 15, 18, 17, 15, 13, 13, 15, 15, 15, 16, 19, 21, 22, 20, 
18, 17, 14, 11, 7, 5, 6, 9, 12, 16, 18, 20, 21, 20, 19, 18, 
17, 16, 15, 12, 12, 15, 17, 16, 11, 6, -1, -3, -4, -3, -2, 0, 
2, 2, 1, -1, -4, -6, -9, -9, -8, -4, 0, 6, 9, 10, 11, 11, 
11, 13, 11, 9, 7, 7, 7, 6, 6, 7, 8, 6, -1, -3, -3, -2, 
-2, -2, -2, -5, -10, -14, -17, -21, -25, -27, -30, -31, -29, -25, -24, -23, 
-21, -20, -21, -26, -30, -35, -39, -42, -42, -38, -34, -31, -27, -23, -20, -20, 
-21, -21, -20, -18, -19, -19, -18, -18, -19, -23, -25, -26, -26, -26, -25, -21, 
-20, -19, -18, -14, -11, -11, -11, -10, -7, -6, -7, -7, -7, -7, -8, -8, 
-6, -7, -11, -16, -19, -18, -19, -20, -22, -25, -27, -28, -27, -25, -23, -21, 
-21, -23, -28, -33, -37, -38, -36, -36, -38, -43, -47, -48, -47, -46, -46, -49, 
-52, -54, -54, -53, -50, -47, -47, -48, -49, -49, -48, -49, -49, -51, -51, -52, 
-50, -49, -49, -52, -55, -54, -52, -48, -43, -39, -38, -38, -41, -44, -49, -48, 
-46, -44, -39, -32, -26, -23, -20, -18, -17, -15, -16, -18, -21, -23, -25, -26, 
-26, -25, -23, -20, -15, -7, -2, 3, 7, 10, 12, 12, 10, 6, 1, -3, 
-8, -9, -8, -4, 0, 2, 5, 6, 7, 10, 14, 20, 27, 27, 26, 26, 
26, 24, 23, 26, 32, 38, 43, 47, 46, 43, 39, 35, 30, 26, 25, 24, 
23, 22, 27, 32, 37, 41, 46, 50, 54, 54, 53, 52, 53, 55, 56, 56, 
57, 57, 58, 58, 58, 57, 58, 57, 56, 55, 54, 55, 56, 56, 58, 61, 
65, 67, 67, 64, 59, 59, 58, 56, 52, 51, 51, 53, 56, 58, 59, 57, 
56, 56, 54, 51, 49, 48, 47, 46, 48, 53, 58, 61, 61, 58, 54, 49, 
41, 37, 34, 32, 31, 31, 35, 36, 34, 29, 21, 14, 5, 3, 2, 2, 
1, 1, 0, -1, -1, 0, 1, 6, 8, 7, 3, -3, -5, -4, -2, 1, 
4, 4, -1, -6, -9, -11, -9, -5, 6, 12, 12, 9, 2, -7, -16, -18, 
-17, -15, -15, -18, -22, -25, -27, -28, -26, -19, -15, -15, -17, -17, -18, -21, 
-24, -23, -21, -19, -22, -24, -25, -26, -27, -26, -23, -23, -25, -29, -33, -37, 
-38, -34, -30, -26, -25, -26, -27, -27, -27, -28, -27, -23, -20, -17, -16, -14, 
-11, -7, -6, -6, -5, -4, -4, -7, -8, -7, -4, -1, 0, -2, -4, -8, 
-11, -14, -18, -19, -18, -15, -15, -17, -20, -25, -29, -33, -34, -29, -25, -24, 
-26, -29, -36, -39, -40, -41, -39, -37, -33, -30, -29, -29, -28, -26, -26, -27, 
-26, -26, -26, -27, -26, -24, -23, -22, -19, -19, -20, -21, -20, -18, -15, -9, 
-9, -10, -13, -18, -28, -32, -35, -33, -28, -23, -15, -11, -9, -8, -8, -8, 
-9, -10, -10, -10, -9, -4, 1, 5, 10, 14, 16, 18, 16, 16, 15, 15, 
15, 15, 15, 15, 15, 17, 16, 14, 14, 15, 17, 20, 28, 33, 38, 40, 
41, 39, 33, 30, 28, 24, 20, 12, 8, 5, 5, 7, 10, 14, 15, 16, 
15, 14, 13, 12, 9, 8, 7, 7, 7, 8, 10, 11, 13, 15, 22, 26, 
30, 33, 34, 31, 23, 18, 12, 7, 2, -2, -3, -4, -5, -5, -2, 6, 
12, 16, 17, 15, 11, 3, -4, -10, -14, -18, -19, -17, -16, -15, -13, -10, 
-3, 3, 7, 7, 6, 4, 4, 4, 4, 4, 4, 3, 3, 4, 5, 7, 
7, 5, 6, 8, 10, 12, 12, 13, 12, 10, 7, 4, -3, -9, -16, -21, 
-26, -30, -32, -28, -21, -15, -12, -10, -9, -8, -8, -8, -8, -12, -15, -17, 
-16, -16, -16, -15, -14, -14, -15, -15, -15, -14, -13, -11, -10, -9, -10, -11, 
-12, -12, -15, -17, -19, -19, -18, -18, -17, -14, -11, -10, -9, -8, -8, -9, 
-11, -12, -15, -17, -18, -22, -24, -24, -23, -20, -15, -13, -14, -15, -14, -12, 
-10, -9, -7, -6, -5, -5, -7, -9, -10, -9, -6, -2, 1, 5, 9, 10, 
9, 6, 5, 3, 2, 2, 2, 0, -2, -3, -3, -1, 2, 3, 1, -5, 
-12, -19, -27, -30, -33, -33, -33, -32, -33, -34, -37, -42, -46, -48, -48, -49, 
-50, -51, -51, -51, -51, -52, -53, -55, -56, -55, -54, -52, -50, -48, -47, -47, 
-48, -49, -49, -49, -46, -41, -38, -36, -34, -33, -32, -30, -29, -28, -27, -24, 
-23, -23, -21, -17, -14, -9, -7, -6, -6, -7, -9, -10, -11, -9, -4, 2, 
8, 10, 12, 14, 14, 15, 20, 24, 27, 30, 32, 34, 34, 34, 34, 34, 
34, 35, 35, 33, 31, 30, 32, 33, 33, 33, 32, 31, 28, 29, 31, 35, 
39, 43, 48, 50, 52, 53, 53, 52, 51, 51, 53, 55, 57, 58, 59, 58, 
55, 51, 49, 49, 50, 49, 47, 43, 39, 36, 36, 38, 43, 47, 51, 51, 
53, 55, 57, 58, 58, 56, 52, 47, 43, 39, 36, 35, 35, 37, 38, 42, 
44, 46, 45, 43, 39, 34, 32, 32, 33, 35, 35, 30, 26, 22, 18, 14, 
12, 12, 13, 14, 16, 16, 19, 22, 25, 24, 19, 10, 6, 5, 5, 7, 
12, 17, 17, 17, 18, 17, 16, 13, 10, 8, 7, 7, 6, 2, -1, -4, 
-6, -6, -6, -5, -4, -3, -3, -2, 0, 2, 4, 5, 5, 6, 8, 10, 
11, 12, 15, 19, 21, 23, 21, 15, 10, 4, 2, 0, -1, -3, -7, -10, 
-11, -12, -13, -15, -17, -17, -18, -17, -16, -15, -14, -12, -11, -11, -11, -11, 
-11, -10, -9, -8, -6, -2, 0, 2, 3, 3, 1, -1, -1, -1, 0, 0, 
-1, -2, -4, -4, -3, -2, -1, -2, -4, -5, -7, -10, -16, -18, -20, -22, 
-25, -28, -30, -31, -31, -30, -27, -21, -17, -16, -16, -16, -18, -20, -22, -25, 
-25, -26, -25, -25, -26, -27, -30, -34, -43, -50, -55, -57, -56, -54, -49, -48, 
-47, -46, -46, -49, -54, -56, -53, -51, -50, -47, -45, -43, -41, -38, -33, -31, 
-29, -28, -30, -33, -36, -37, -39, -38, -35, -30, -23, -19, -16, -13, -13, -13, 
-13, -14, -15, -16, -18, -20, -17, -13, -8, -5, -5, -9, -14, -20, -26, -31, 
-33, -33, -33, -32, -31, -30, -26, -24, -21, -17, -13, -9, -7, -7, -8, -10, 
-10, -9, -7, -6, -7, -11, -14, -20, -23, -25, -24, -20, -16, -8, -4, 0, 
3, 2, -5, -12, -18, -22, -24, -22, -15, -11, -7, -5, -4, -4, -6, -8, 
-9, -10, -10, -11, -11, -11, -11, -12, -13, -11, -8, -8, -8, -8, -7, -5, 
-3, 1, 5, 8, 11, 15, 19, 21, 22, 21, 20, 18, 17, 17, 20, 23, 
26, 30, 35, 37, 39, 41, 43, 45, 43, 41, 38, 38, 37, 34, 32, 31, 
29, 26, 26, 26, 28, 33, 40, 42, 44, 46, 45, 44, 41, 40, 38, 35, 
30, 22, 16, 13, 14, 18, 23, 29, 30, 28, 23, 16, 9, 5, 7, 10, 
11, 9, 2, -3, -5, -7, -9, -8, -8, -10, -13, -15, -14, -11, -9, -10, 
-11, -12, -13, -13, -16, -19, -20, -19, -15, -12, -9, -6, -4, -4, -6, -12, 
-15, -15, -15, -17, -16, -12, -6, 0, 5, 7, 6, 4, 4, 5, 8, 10, 
9, 6, 5, 5, 7, 10, 13, 19, 24, 27, 32, 39, 42, 40, 36, 33, 
32, 32, 31, 30, 33, 40, 52, 57, 58, 56, 53, 48, 38, 33, 30, 27, 
24, 22, 25, 28, 31, 34, 36, 37, 38, 41, 44, 46, 45, 39, 34, 27, 
20, 14, 12, 14, 16, 16, 15, 15, 15, 17, 18, 17, 14, 6, -11, -20, 
-25, -27, -29, -30, -30, -32, -35, -35, -31, -26, -25, -25, -24, -25, -30, -41, 
-50, -57, -62, -63, -59, -52, -43, -33, -24, -18, -12, -12, -13, -14, -14, -13, 
-8, -5, -5, -6, -8, -11, -12, -12, -9, -2, 6, 20, 25, 26, 26, 27, 
29, 27, 22, 21, 23, 26, 26, 25, 27, 33, 42, 52, 65, 69, 67, 59, 
47, 38, 30, 27, 23, 17, 7, -6, -10, -8, -4, -1, 2, 5, 6, 4, 
-2, -11, -21, -36, -48, -58, -66, -71, -76, -78, -78, -77, -74, -72, -69, -67, 
-69, -72, -74, -74, -77, -84, -92, -100, -106, -115, -119, -119, -118, -116, -110, -97, 
-90, -86, -83, -79, -74, -70, -72, -75, -77, -78, -78, -77, -71, -63, -53, -45, 
-37, -36, -37, -40, -44, -48, -46, -41, -35, -31, -27, -23, -22, -23, -25, -25, 
-25, -28, -29, -28, -26, -25, -26, -30, -32, -31, -28, -25, -20, -16, -16, -22, 
-29, -35, -35, -31, -24, -17, -9, 9, 24, 36, 43, 44, 40, 25, 14, 6, 
2, 3, 7, 16, 19, 18, 18, 23, 43, 58, 68, 70, 66, 58, 44, 36, 
30, 26, 26, 26, 33, 38, 40, 39, 40, 47, 49, 49, 48, 46, 42, 34, 
30, 27, 25, 24, 24, 25, 27, 28, 30, 32, 34, 37, 41, 45, 49, 50, 
47, 42, 38, 37, 39, 43, 51, 58, 63, 67, 70, 75, 77, 78, 79, 79, 
79, 77, 79, 83, 86, 83, 78, 72, 73, 77, 84, 93, 102, 105, 105, 99, 
89, 77, 67, 65, 66, 69, 69, 66, 58, 56, 60, 67, 76, 84, 85, 80, 
72, 66, 62, 58, 52, 44, 34, 25, 17, 7, 4, 4, 8, 14, 23, 26, 
27, 26, 23, 20, 11, 5, -3, -12, -21, -29, -33, -33, -30, -24, -18, -15, 
-15, -15, -15, -14, -11, -7, -8, -11, -14, -16, -14, -13, -13, -15, -17, -18, 
-13, -5, 7, 19, 24, 24, 17, 11, 5, 1, -2, -7, -8, -8, -9, -10, 
-9, 1, 7, 9, 8, 7, 6, 2, -2, -10, -19, -30, -43, -47, -49, -47, 
-42, -32, -16, -11, -11, -13, -16, -21, -31, -36, -40, -44, -48, -53, -54, -53, 
-53, -53, -52, -47, -42, -40, -39, -37, -36, -39, -44, -48, -55, -64, -74, -76, 
-74, -71, -65, -55, -47, -45, -42, -38, -35, -38, -53, -62, -65, -63, -58, -55, 
-54, -55, -55, -53, -51, -49, -48, -49, -52, -55, -56, -56, -57, -59, -62, -66, 
-66, -64, -59, -56, -54, -53, -55, -59, -62, -62, -61, -61, -62, -65, -68, -70, 
-73, -71, -65, -60, -60, -63, -69, -74, -75, -74, -69, -60, -52, -46, -45, -46, 
-47, -47, -44, -43, -43, -45, -46, -45, -42, -37, -28, -19, -11, -3, 1, 2, 
1, -3, -5, -5, 0, 6, 11, 15, 17, 17, 12, 6, 3, 4, 11, 15, 
19, 22, 25, 27, 32, 33, 34, 36, 39, 43, 45, 43, 41, 40, 39, 41, 
45, 51, 57, 60, 62, 66, 68, 71, 73, 74, 73, 68, 63, 59, 53, 49, 
49, 52, 57, 61, 63, 67, 73, 77, 78, 78, 80, 81, 78, 77, 75, 76, 
77, 75, 74, 73, 71, 69, 67, 63, 58, 55, 54, 55, 56, 56, 56, 55, 
53, 51, 49, 48, 50, 52, 51, 52, 57, 61, 62, 58, 53, 48, 46, 43, 
39, 34, 29, 22, 19, 18, 17, 16, 13, 6, 0, -7, -13, -16, -18, -19, 
-16, -15, -17, -20, -24, -24, -24, -23, -19, -15, -14, -17, -23, -26, -27, -24, 
-20, -19, -21, -26, -34, -43, -42, -36, -28, -17, -7, -1, -2, -3, -3, -3, 
-3, -5, -9, -14, -21, -26, -30, -31, -30, -28, -26, -26, -28, -25, -22, -19, 
-16, -14, -16, -18, -19, -19, -18, -19, -20, -18, -16, -14, -16, -20, -23, -26, 
-28, -27, -20, -14, -9, -5, -4, -5, -11, -14, -18, -21, -22, -21, -19, -18, 
-19, -22, -26, -33, -35, -34, -32, -30, -26, -23, -23, -23, -26, -30, -34, -43, 
-51, -62, -71, -76, -76, -76, -77, -76, -71, -64, -52, -41, -28, -15, -5, 1, 
5, 4, 1, -3, -8, -9, -8, -9, -10, -10, -8, -4, 1, 8, 15, 19, 
19, 21, 20, 18, 15, 13, 16, 20, 22, 24, 23, 20, 15, 10, 8, 7, 
4, 1, 4, 7, 11, 18, 24, 29, 27, 24, 22, 21, 19, 21, 24, 24, 
21, 17, 15, 13, 13, 14, 18, 21, 28, 31, 36, 41, 46, 50, 52, 52, 
51, 50, 50, 49, 45, 40, 35, 31, 31, 37, 42, 47, 49, 47, 47, 52, 
55, 55, 52, 50, 48, 44, 38, 32, 28, 25, 19, 14, 13, 13, 13, 12, 
11, 12, 13, 14, 14, 14, 14, 13, 12, 11, 10, 13, 16, 17, 16, 16, 
17, 21, 23, 27, 31, 34, 36, 35, 34, 34, 36, 38, 39, 38, 36, 35, 
36, 34, 28, 26, 25, 24, 24, 25, 26, 23, 17, 12, 9, 4, -2, -8, 
-9, -8, -6, -9, -15, -20, -24, -28, -32, -31, -30, -30, -30, -29, -27, -25, 
-23, -23, -25, -24, -22, -22, -24, -26, -28, -33, -35, -37, -37, -38, -39, -39, 
-37, -35, -33, -31, -30, -29, -28, -28, -28, -30, -31, -28, -23, -17, -11, -8, 
-8, -9, -10, -12, -13, -12, -11, -10, -9, -8, -8, -9, -8, -6, -3, 0, 
2, 4, 4, 4, 4, 3, 1, -2, -4, -6, -6, -5, -3, -1, -2, -5, 
-9, -12, -17, -19, -18, -16, -14, -11, -9, -7, -6, -6, -6, -8, -7, -3, 
2, 5, 7, 9, 9, 8, 6, 3, -1, -3, -3, -3, -1, 2, 5, 5, 
3, 1, -2, -5, -7, -4, -2, 0, 2, 4, 5, 5, 6, 6, 6, 5, 
3, 1, 0, -2, -2, 0, 1, 2, 3, 4, 5, 7, 9, 11, 13, 13, 
13, 13, 11, 6, 3, 2, 1, -1, -3, -6, -8, -12, -14, -15, -14, -12, 
-9, -4, -1, 2, 4, 4, 2, -3, -5, -8, -11, -13, -12, -12, -14, -15, 
-14, -11, -4, -1, 0, 0, -1, -1, -2, -3, -4, -6, -9, -13, -13, -12, 
-11, -8, -6, -2, 1, 6, 10, 12, 13, 8, 5, 0, -7, -13, -21, -21, 
-19, -18, -18, -20, -23, -24, -26, -28, -27, -27, -26, -26, -25, -25, -26, -25, 
-24, -23, -23, -26, -28, -30, -31, -34, -37, -39, -37, -35, -35, -36, -36, -36, 
-38, -39, -37, -33, -27, -19, -11, -8, -6, -3, -3, -5, -5, -4, -1, 2, 
4, 5, 2, 0, -3, -8, -10, -4, 1, 7, 15, 24, 33, 33, 30, 24, 
16, 7, -3, -8, -12, -13, -13, -14, -15, -14, -15, -17, -18, -15, -12, -10, 
-8, -6, -6, -5, -2, -1, -1, 0, 1, 3, 1, -1, -4, -7, -9, -9, 
-9, -9, -9, -9, -12, -16, -19, -18, -17, -15, -13, -10, -8, -5, -2, -1, 
1, 5, 12, 18, 21, 24, 26, 28, 28, 29, 33, 40, 44, 46, 47, 46, 
43, 38, 32, 25, 21, 19, 24, 29, 32, 34, 36, 35, 33, 30, 28, 27, 
26, 26, 26, 26, 23, 17, 11, 8, 8, 6, 3, 0, 1, 1, 2, 6, 
11, 16, 22, 24, 28, 30, 31, 32, 34, 35, 33, 31, 28, 23, 20, 19, 
21, 23, 25, 26, 24, 21, 20, 20, 22, 19, 15, 12, 9, 6, 0, -2, 
-2, -1, 2, 5, 8, 7, 7, 7, 7, 8, 6, 3, -2, -5, -6, -8, 
-11, -15, -20, -26, -31, -35, -34, -30, -24, -20, -15, -12, -10, -7, -2, 3, 
8, 10, 11, 15, 19, 23, 25, 22, 19, 14, 10, 8, 6, 7, 7, 8, 
11, 18, 22, 25, 28, 30, 29, 23, 18, 13, 9, 6, 5, 7, 10, 14, 
18, 20, 20, 20, 20, 21, 21, 18, 11, 7, 5, 2, 1, -1, -1, 2, 
7, 11, 13, 16, 18, 19, 19, 18, 16, 12, 7, 1, -4, -7, -9, -12, 
-15, -20, -24, -27, -30, -32, -33, -31, -31, -31, -33, -35, -38, -41, -43, -43, 
-43, -42, -41, -43, -46, -51, -52, -52, -51, -52, -52, -50, -47, -46, -46, -47, 
-47, -43, -38, -34, -31, -28, -23, -19, -18, -17, -16, -16, -16, -16, -16, -15, 
-15, -14, -13, -11, -8, -2, 4, 9, 10, 11, 11, 11, 11, 12, 13, 14, 
14, 14, 16, 17, 18, 20, 22, 22, 24, 22, 19, 15, 11, 7, 1, -1, 
-2, -4, -5, -2, 0, 1, 1, 3, 6, 10, 11, 11, 10, 9, 9, 11, 
14, 17, 19, 20, 17, 16, 15, 15, 15, 14, 11, 10, 10, 13, 17, 20, 
21, 19, 15, 13, 11, 10, 10, 9, 9, 9, 12, 16, 19, 22, 24, 25, 
25, 23, 21, 19, 17, 16, 16, 17, 17, 15, 12, 11, 12, 13, 14, 13, 
12, 11, 10, 8, 3, -2, -5, -3, 0, 2, 3, 2, 0, -3, -5, -6, 
-7, -8, -7, -5, -4, -2, 2, 7, 9, 6, 2, 1, 1, 1, 3, 6, 
6, 4, 1, -2, -3, -4, -4, -4, -4, -3, 0, 2, 5, 5, 5, 4, 
2, -3, -9, -15, -25, -29, -28, -22, -16, -12, -10, -10, -8, -7, -6, -5, 
-2, -3, -6, -8, -7, -4, -2, -1, 0, 2, 2, -2, -4, -3, -2, 0, 
2, 4, 6, 6, 6, 2, -6, -12, -17, -21, -27, -32, -35, -33, -32, -32, 
-33, -32, -34, -36, -38, -38, -39, -42, -41, -40, -40, -42, -43, -45, -44, -41, 
-38, -35, -33, -34, -34, -34, -34, -36, -39, -41, -41, -38, -34, -31, -28, -25, 
-22, -20, -19, -17, -19, -20, -21, -20, -19, -17, -13, -10, -10, -12, -16, -22, 
-26, -31, -34, -35, -35, -35, -34, -32, -30, -28, -24, -22, -21, -20, -21, -25, 
-32, -35, -36, -36, -38, -38, -38, -38, -40, -40, -40, -40, -39, -39, -39, -39, 
-40, -39, -38, -38, -35, -31, -28, -27, -28, -29, -30, -32, -37, -38, -36, -31, 
-27, -22, -18, -14, -12, -13, -15, -15, -14, -13, -12, -7, -1, 9, 15, 21, 
26, 28, 26, 18, 11, 5, 0, -2, 0, 4, 7, 9, 12, 15, 20, 23, 
24, 26, 26, 25, 25, 25, 25, 25, 27, 27, 24, 22, 22, 25, 28, 35, 
41, 46, 50, 49, 45, 41, 40, 39, 36, 31, 25, 25, 27, 28, 29, 31, 
34, 36, 38, 37, 39, 40, 40, 41, 41, 42, 41, 43, 44, 47, 49, 50, 
52, 53, 52, 53, 55, 57, 57, 54, 50, 47, 45, 43, 42, 43, 45, 43, 
41, 38, 34, 33, 33, 35, 39, 41, 41, 38, 34, 29, 26, 32, 35, 37, 
40, 44, 49, 55, 57, 58, 55, 50, 45, 43, 41, 39, 38, 37, 37, 36, 
36, 35, 33, 32, 29, 29, 31, 33, 32, 28, 26, 26, 25, 23, 22, 17, 
11, 6, 4, 5, 10, 16, 19, 20, 18, 18, 19, 21, 22, 24, 26, 27, 
26, 25, 25, 25, 24, 23, 22, 19, 16, 14, 13, 8, 2, -6, -12, -15, 
-15, -14, -14, -14, -14, -18, -24, -33, -39, -43, -45, -45, -44, -42, -37, -30, 
-25, -21, -17, -15, -16, -21, -26, -29, -31, -32, -34, -33, -32, -29, -23, -19, 
-18, -20, -22, -23, -22, -23, -25, -24, -23, -24, -26, -27, -27, -28, -30, -30, 
-30, -32, -32, -31, -29, -29, -30, -29, -29, -30, -28, -25, -20, -18, -18, -22, 
-25, -28, -32, -35, -36, -34, -31, -29, -26, -22, -19, -19, -20, -21, -24, -26, 
-28, -27, -25, -23, -18, -13, -13, -15, -19, -22, -25, -29, -29, -28, -27, -27, 
-29, -31, -31, -30, -30, -30, -30, -29, -28, -26, -23, -20, -17, -16, -17, -21, 
-26, -29, -31, -30, -27, -24, -20, -15, -11, -10, -11, -12, -12, -7, -4, -1, 
3, 6, 8, 11, 9, 6, 1, -2, -2, -4, -3, -1, 2, 3, 2, 2, 
1, -1, -6, -12, -27, -37, -43, -45, -46, -46, -47, -47, -46, -44, -42, -41, 
-42, -41, -39, -37, -35, -35, -35, -36, -35, -31, -27, -19, -15, -9, -4, 2, 
7, 9, 8, 8, 9, 11, 10, 8, 6, 6, 6, 8, 8, 6, 4, 3, 
2, -5, -10, -17, -22, -26, -31, -36, -37, -37, -34, -29, -22, -12, -8, -5, 
-3, -4, -7, -9, -13, -18, -24, -30, -36, -35, -31, -25, -17, -7, 7, 13, 
15, 15, 17, 16, 9, -2, -12, -19, -20, -14, -9, -4, 0, 2, 4, 5, 
5, 7, 11, 15, 14, 10, 10, 14, 21, 28, 31, 28, 24, 20, 19, 19, 
20, 22, 25, 30, 41, 56, 61, 62, 58, 51, 42, 34, 30, 25, 19, 16, 
15, 18, 24, 32, 38, 40, 37, 31, 23, 18, 18, 20, 17, 14, 13, 13, 
9, 2, 0, 1, 4, 9, 16, 23, 22, 19, 15, 10, 3, -4, -4, -5, 
-6, -8, -3, 9, 22, 32, 35, 32, 20, 10, -3, -11, -14, -10, -6, -5, 
-2, 3, 11, 23, 27, 32, 35, 35, 33, 29, 26, 25, 26, 30, 37, 45, 
48, 50, 51, 52, 55, 60, 64, 66, 66, 64, 59, 56, 55, 56, 55, 52, 
45, 41, 38, 35, 31, 28, 31, 37, 41, 42, 41, 36, 29, 19, 11, 6, 
2, -2, -2, -1, -1, -2, -5, -5, -4, -3, -2, -2, -8, -14, -21, -28, 
-32, -31, -26, -22, -16, -11, -7, -8, -9, -9, -9, -12, -16, -18, -18, -18, 
-17, -16, -13, -9, -7, -8, -11, -10, -1, 4, 8, 8, 5, 0, -5, -3, 
4, 12, 19, 20, 17, 14, 13, 13, 13, 16, 19, 22, 23, 21, 15, 4, 
-7, -18, -25, -30, -33, -35, -38, -43, -54, -67, -82, -85, -85, -86, -89, -91, 
-87, -85, -87, -92, -97, -100, -102, -105, -113, -123, -127, -122, -113, -103, -90, -80, 
-76, -76, -76, -74, -75, -78, -84, -86, -85, -80, -73, -67, -66, -67, -67, -65, 
-64, -66, -70, -73, -74, -76, -79, -75, -68, -62, -60, -61, -58, -47, -43, -42, 
-44, -46, -48, -50, -49, -47, -44, -38, -28, -22, -20, -19, -19, -18, -12, -7, 
-3, -5, -8, -12, -15, -13, -10, -4, 4, 13, 14, 12, 10, 9, 10, 20, 
29, 35, 40, 42, 45, 46, 50, 54, 55, 56, 59, 66, 76, 84, 92, 97, 
96, 92, 88, 85, 81, 77, 76, 74, 72, 70, 66, 59, 55, 53, 56, 59, 
64, 73, 75, 75, 73, 71, 67, 68, 70, 72, 73, 73, 70, 69, 69, 69, 
67, 64, 63, 63, 63, 63, 64, 67, 66, 61, 54, 50, 47, 47, 48, 47, 
49, 51, 54, 55, 59, 66, 71, 73, 68, 62, 57, 52, 49, 45, 38, 36, 
36, 39, 42, 44, 48, 51, 49, 43, 37, 34, 34, 35, 35, 33, 32, 31, 
31, 29, 28, 31, 36, 44, 47, 49, 48, 45, 41, 36, 29, 22, 18, 16, 
16, 15, 17, 22, 28, 33, 31, 28, 28, 28, 28, 26, 24, 21, 15, 10, 
4, -7, -11, -14, -15, -14, -14, -16, -18, -19, -18, -15, -14, -13, -12, -12, 
-12, -14, -18, -21, -20, -18, -14, -9, -7, -6, -7, -10, -13, -16, -19, -23, 
-24, -22, -16, -11, -11, -14, -18, -23, -32, -36, -39, -41, -43, -44, -45, -42, 
-39, -37, -36, -33, -28, -26, -25, -25, -25, -28, -30, -36, -45, -55, -64, -72, 
-72, -68, -62, -57, -53, -52, -53, -53, -56, -59, -66, -73, -78, -81, -82, -81, 
-75, -71, -71, -71, -73, -78, -85, -89, -91, -91, -91, -86, -78, -69, -62, -58, 
-57, -57, -59, -61, -62, -62, -61, -62, -61, -58, -52, -48, -47, -48, -48, -51, 
-55, -59, -60, -55, -51, -48, -47, -48, -54, -61, -65, -66, -65, -64, -62, -60, 
-59, -59, -58, -58, -57, -58, -58, -59, -60, -60, -58, -56, -56, -55, -50, -45, 
-41, -39, -35, -32, -27, -22, -17, -13, -10, -8, -3, 0, };
