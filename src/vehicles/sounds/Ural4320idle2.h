const unsigned int sampleRate = 22050;
const unsigned int sampleCount = 3281;
const signed char samples[] = {
0, -2, -4, -3, -1, 1, 2, 2, 2, 3, 4, 4, 7, 8, 9, 11,
14, 17, 18, 18, 17, 14, 10, 3, 1, 2, 8, 15, 23, 33, 38, 40, 
39, 33, 22, 16, 10, 7, 6, 6, 6, 9, 12, 13, 12, 9, 5, 2, 
0, -4, -8, -8, -4, 5, 15, 24, 32, 40, 42, 40, 33, 23, 12, -1, 
-4, -5, -4, -3, -2, -4, -6, -8, -8, -7, 0, 5, 11, 15, 17, 17, 
16, 15, 15, 17, 21, 24, 25, 24, 21, 18, 14, 8, 7, 8, 10, 13, 
15, 17, 17, 16, 13, 9, -1, -6, -6, -4, -1, 2, 4, 5, 7, 9, 
13, 16, 23, 28, 30, 31, 28, 18, 12, 4, -2, -7, -11, -14, -15, -16, 
-17, -20, -20, -12, -3, 9, 20, 28, 35, 35, 34, 32, 27, 19, 7, 0, 
-6, -9, -11, -8, -3, 5, 12, 15, 17, 11, 6, 3, 3, 6, 11, 18, 
23, 29, 34, 37, 36, 35, 30, 22, 14, 5, -6, -7, -3, 3, 9, 15, 
27, 33, 39, 46, 54, 61, 61, 56, 48, 40, 32, 22, 19, 20, 23, 26, 
28, 24, 18, 12, 7, 2, 1, 5, 9, 15, 20, 25, 32, 36, 38, 39, 
38, 33, 19, 9, 0, -6, -8, -7, -5, -3, -1, 0, 1, -1, -5, -11, 
-17, -23, -28, -31, -30, -29, -28, -28, -27, -28, -30, -36, -44, -49, -56, -60, 
-62, -62, -57, -51, -47, -47, -48, -48, -46, -45, -46, -47, -47, -47, -45, -42, 
-41, -41, -43, -43, -44, -43, -40, -37, -34, -30, -27, -27, -29, -30, -30, -27, 
-21, -19, -19, -20, -21, -24, -29, -31, -31, -31, -32, -33, -32, -29, -23, -16, 
-11, -2, 1, 3, 2, -1, -7, -9, -9, -10, -10, -11, -10, -9, -8, -7, 
-7, -10, -15, -15, -12, -6, 3, 15, 19, 19, 16, 12, 6, -5, -11, -16, 
-17, -16, -16, -14, -11, -6, -1, 2, 7, 9, 10, 11, 13, 16, 19, 19, 
18, 17, 15, 13, 14, 15, 15, 15, 14, 9, 4, -1, -2, -1, 5, 19, 
30, 38, 41, 40, 36, 32, 31, 29, 25, 19, 11, 7, 6, 6, 8, 14, 
21, 25, 29, 31, 32, 34, 32, 30, 27, 28, 30, 36, 43, 48, 50, 47, 
39, 25, 16, 10, 5, -1, -5, -9, -9, -8, -4, 5, 21, 30, 37, 40, 
41, 38, 30, 22, 15, 8, 3, 1, 1, 4, 9, 14, 18, 15, 11, 5, 
1, -1, -2, 5, 12, 20, 29, 36, 41, 40, 36, 33, 30, 28, 26, 23, 
18, 13, 9, 5, 4, 7, 12, 19, 27, 39, 45, 47, 44, 38, 29, 16, 
9, 5, 5, 5, 8, 13, 17, 23, 28, 32, 37, 42, 47, 50, 50, 48, 
44, 40, 39, 36, 32, 26, 17, 12, 9, 7, 6, 6, 7, 9, 11, 13, 
13, 11, 10, 6, 1, -4, -7, -7, -4, -1, 3, 6, 4, 0, -5, -11, 
-16, -20, -23, -23, -22, -20, -20, -21, -23, -24, -24, -23, -21, -20, -21, -21, 
-20, -18, -18, -19, -21, -21, -22, -24, -25, -30, -34, -37, -40, -43, -47, -48, 
-48, -45, -42, -39, -35, -33, -30, -26, -23, -21, -22, -23, -24, -24, -26, -32, 
-37, -42, -44, -45, -44, -42, -44, -47, -52, -56, -59, -60, -59, -56, -52, -45, 
-32, -25, -21, -18, -19, -20, -22, -24, -26, -27, -27, -26, -26, -29, -35, -41, 
-47, -53, -56, -57, -57, -53, -46, -36, -29, -24, -17, -8, 2, 4, 0, -8, 
-16, -24, -33, -34, -34, -31, -27, -22, -14, -10, -4, 1, 7, 15, 20, 24, 
26, 27, 29, 37, 43, 46, 46, 42, 37, 28, 21, 16, 14, 15, 21, 28, 
35, 42, 47, 50, 52, 52, 52, 52, 51, 50, 51, 52, 51, 49, 48, 50, 
55, 59, 59, 56, 48, 37, 32, 32, 35, 39, 44, 49, 51, 50, 47, 44, 
42, 44, 46, 48, 49, 47, 44, 42, 39, 36, 33, 29, 21, 17, 17, 18, 
20, 25, 29, 32, 32, 29, 25, 20, 18, 17, 14, 11, 6, 0, -2, -2, 
-2, 1, 5, 9, 12, 13, 13, 12, 13, 15, 17, 19, 18, 13, 2, -4, 
-8, -9, -9, -8, -8, -9, -10, -10, -10, -8, -6, -4, -1, -1, -2, -5, 
-6, -6, -3, 2, 5, 5, 4, 1, -2, -5, -9, -12, -15, -18, -22, -28, 
-31, -32, -32, -32, -30, -30, -29, -26, -24, -21, -19, -17, -19, -22, -24, -29, 
-37, -43, -47, -48, -46, -42, -37, -37, -40, -46, -53, -56, -54, -49, -44, -38, 
-33, -27, -24, -21, -18, -15, -12, -9, -7, -8, -10, -14, -18, -21, -25, -28, 
-33, -41, -51, -56, -58, -56, -51, -42, -27, -18, -12, -9, -7, -6, -6, -8, 
-11, -15, -19, -25, -26, -26, -22, -17, -11, 2, 13, 25, 32, 35, 35, 31, 
30, 27, 23, 20, 12, 7, 3, -3, -10, -16, -21, -23, -24, -23, -20, -16, 
-7, -4, -1, 2, 2, -2, -7, -14, -22, -30, -36, -42, -43, -41, -37, -32, 
-30, -30, -29, -26, -24, -22, -22, -22, -24, -28, -30, -29, -21, -15, -9, -7, 
-10, -16, -30, -39, -47, -53, -56, -58, -57, -56, -52, -45, -36, -29, -29, -29, 
-29, -26, -23, -21, -21, -21, -20, -20, -18, -16, -12, -9, -9, -14, -24, -29, 
-30, -29, -29, -24, -19, -13, -8, -5, -4, -4, -4, -4, -5, -8, -13, -15, 
-14, -13, -11, -9, -5, -1, 2, 4, 6, 6, 4, 3, 4, 7, 11, 14, 
16, 18, 21, 24, 27, 29, 26, 19, 14, 8, 3, 6, 10, 13, 15, 16, 
19, 21, 19, 16, 17, 18, 18, 16, 12, 9, 7, 8, 9, 11, 14, 20, 
27, 33, 42, 47, 54, 60, 63, 61, 61, 60, 59, 56, 52, 46, 42, 37, 
32, 30, 29, 27, 27, 27, 25, 24, 24, 22, 21, 21, 22, 25, 25, 25, 
25, 26, 26, 26, 26, 28, 31, 32, 32, 25, 18, 13, 10, 9, 11, 18, 
24, 30, 34, 37, 38, 34, 32, 27, 20, 11, -7, -18, -28, -32, -28, -20, 
-5, 3, 10, 14, 15, 11, 5, -3, -8, -11, -12, -15, -14, -9, -4, -2, 
-3, -3, 1, 8, 14, 14, 4, -3, -6, -5, -5, -8, -10, -8, -6, -5, 
-7, -11, -18, -22, -26, -29, -31, -28, -25, -25, -25, -24, -22, -21, -22, -24, 
-26, -29, -29, -28, -27, -23, -19, -15, -9, -7, -7, -6, -3, 0, 2, -3, 
-10, -18, -27, -34, -37, -38, -38, -39, -39, -35, -30, -23, -15, -5, 6, 17, 
17, 10, 1, -8, -15, -23, -26, -28, -28, -26, -26, -25, -24, -20, -13, -8, 
-5, -7, -9, -9, -6, -3, 1, 7, 16, 24, 29, 31, 33, 35, 36, 34, 
30, 25, 21, 15, 10, 9, 10, 7, 4, 3, 5, 9, 14, 15, 14, 14, 
16, 20, 24, 28, 33, 38, 41, 37, 24, 15, 7, 2, 2, 7, 9, 10, 
10, 8, 5, -4, -11, -16, -15, -10, 5, 16, 25, 29, 29, 27, 17, 8, 
-1, -7, -7, -5, 2, 7, 14, 23, 33, 44, 48, 50, 51, 47, 38, 22, 
14, 10, 8, 8, 11, 19, 24, 25, 24, 21, 16, 16, 17, 21, 26, 32, 
38, 42, 46, 48, 45, 40, 32, 27, 22, 16, 10, 4, 7, 16, 28, 42, 
53, 65, 70, 72, 73, 70, 63, 50, 42, 37, 31, 24, 16, 15, 20, 28, 
37, 45, 53, 57, 60, 61, 58, 51, 40, 33, 29, 27, 24, 14, 8, 4, 
5, 7, 7, 9, 14, 23, 37, 50, 56, 58, 57, 55, 55, 53, 48, 43, 
37, 28, 17, 5, -7, -9, -9, -6, 0, 5, 6, 4, 1, 1, 5, 12, 
13, 11, 5, -3, -14, -34, -47, -55, -57, -57, -58, -62, -63, -60, -56, -52, 
-50, -51, -52, -53, -53, -54, -57, -58, -56, -55, -54, -57, -59, -57, -52, -44, 
-36, -28, -29, -35, -44, -57, -73, -97, -109, -112, -108, -100, -82, -69, -57, -49, 
-44, -39, -35, -38, -45, -55, -63, -67, -65, -62, -58, -54, -52, -56, -64, -74, 
-82, -87, -89, -87, -82, -77, -69, -58, -49, -36, -30, -26, -25, -28, -42, -56, 
-69, -80, -86, -84, -75, -63, -47, -29, -11, 2, 16, 22, 23, 19, 9, -13, 
-29, -41, -49, -54, -57, -53, -45, -35, -27, -22, -18, -12, -8, -4, -2, -1, 
-1, 2, 9, 17, 27, 32, 30, 24, 17, 8, 0, -5, -6, -1, 9, 23, 
38, 57, 65, 70, 72, 71, 67, 60, 57, 55, 55, 54, 54, 56, 57, 56, 
51, 47, 48, 56, 66, 72, 75, 73, 67, 62, 59, 58, 57, 58, 59, 62, 
64, 63, 62, 63, 66, 70, 74, 76, 76, 74, 70, 65, 60, 56, 57, 61, 
65, 68, 69, 70, 72, 72, 68, 65, 60, 54, 42, 33, 25, 19, 18, 21, 
23, 25, 27, 29, 33, 36, 35, 30, 27, 26, 27, 28, 25, 20, 17, 14, 
9, 4, 1, -1, -5, -10, -23, -31, -36, -34, -25, -12, 6, 17, 23, 26, 
25, 14, 1, -16, -31, -44, -54, -63, -66, -65, -60, -51, -42, -32, -28, -25, 
-24, -27, -37, -46, -56, -64, -72, -76, -76, -71, -62, -54, -48, -46, -48, -52, 
-59, -64, -67, -61, -51, -44, -38, -38, -42, -53, -61, -66, -68, -68, -64, -58, 
-56, -59, -64, -68, -67, -63, -59, -52, -47, -45, -47, -52, -54, -50, -43, -36, 
-33, -36, -42, -47, -50, -45, -38, -29, -19, -11, -7, -8, -11, -14, -19, -24, 
-29, -31, -30, -25, -18, -10, 2, 8, 14, 16, 15, 10, 2, -1, -3, -4, 
-6, -5, -1, 2, 2, 1, -2, -2, 2, 9, 15, 19, 24, 26, 26, 25, 
25, 23, 21, 20, 19, 20, 22, 26, 34, 36, 35, 30, 21, 13, 6, 7, 
13, 20, 27, 33, 35, 34, 30, 23, 16, 5, -1, -8, -15, -19, -22, -20, 
-15, -7, 1, 7, 14, 14, 13, 10, 9, 10, 11, 12, 12, 11, 11, 11, 
10, 8, 4, 1, -3, -5, -3, 1, 7, 12, 15, 16, 13, 5, -5, -15, 
-25, -35, -37, -35, -29, -23, -16, -13, -11, -8, -4, 1, 6, 7, 6, 5, 
2, -3, -13, -19, -25, -30, -33, -29, -20, -10, -2, 0, -4, -18, -27, -33, 
-33, -30, -25, -15, -12, -11, -13, -16, -16, -13, -8, -4, -1, 1, 1, 1, 
2, 3, 3, 3, -1, -4, -6, -7, -8, -10, -9, -7, -1, 9, 17, 25, 
27, 26, 23, 20, 14, 13, 15, 18, 23, 29, 36, 40, 41, 41, 40, 35, 
24, 19, 18, 20, 21, 22, 23, 26, 32, 37, 42, 43, 43, 45, 48, 51, 
51, 48, 49, 51, 52, 50, 44, 40, 36, 34, 32, 32, 32, 33, 33, 28, 
23, 18, 12, 11, 14, 23, 35, 52, 61, 67, 69, 69, 66, 61, 56, 51, 
44, 38, 32, 31, 32, 34, 35, 34, 32, 31, 29, 25, 22, 20, 18, 16, 
12, 7, 4, 4, 6, 8, 8, 7, 5, -2, -10, -19, -27, -30, -30, -29, 
-28, -26, -22, -17, -10, -2, 1, 3, 2, -4, -20, -31, -41, -50, -57, -62, 
-63, -65, -68, -69, -69, -66, -62, -59, -57, -58, -59, -57, -56, -52, -49, -48, 
-49, -51, -52, -53, -54, -56, -60, -68, -74, -80, -85, -88, -83, -74, -64, -53, 
-41, -31, -21, -20, -22, -25, -29, -32, -35, -35, -35, -37, -39, -46, -50, -52, 
-53, -52, -50, -46, -40, -31, -21, -10, 4, 12, 18, 22, 22, 16, 1, -9, 
-19, -27, -31, -33, -29, -27, -27, -28, -29, -28, -24, -17, -6, 6, 19, 33, 
37, 38, 36, 33, 31, 26, 24, 23, 21, 22, 22, 20, 16, 10, 7, 8, 
15, 21, 27, 33, 39, 43, 45, 45, 42, 39, 34, 26, 20, 15, 12, 10, 
12, 21, 27, 31, 33, 35, 36, 33, 30, 26, 25, 27, 32, 35, 37, 37, 
36, 37, 39, 41, 41, 41, 39, 37, 32, 29, 26, 23, 21, 22, 25, 27, 
27, 27, 27, 28, 30, 31, 31, 31, 31, 29, 28, 29, 30, 33, 40, 43, 
44, 40, 34, 26, 14, 10, 9, 14, 22, 30, 41, 46, 50, 51, 51, 45, 
39, 33, 28, 25, 22, 20, 20, 20, 21, 24, 34, 43, 50, 55, 55, 51, 
40, 32, 26, 22, 20, 20, 24, 28, 32, 37, 41, 45, 47, 47, 45, 43, 
41, 34, 27, 21, 18, 18, 22, 35, 41, 44, 43, 37, 25, 19, 16, 17, 
22, 29, 44, 51, 52, 48, 41, 35, 27, 21, 15, 9, 7, 8, 10, 13, 
18, 24, 31, 39, 40, 33, 23, 13, 1, -16, -26, -32, -36, -37, -40, -42, 
-43, -42, -38, -31, -19, -14, -12, -13, -18, -26, -36, -40, -43, -44, -45, -45, 
-46, -46, -44, -44, -43, -45, -48, -49, -49, -49, -47, -40, -35, -32, -33, -39, 
-54, -66, -76, -82, -85, -83, -75, -70, -66, -62, -54, -48, -43, -44, -45, -46, 
-47, -44, -42, -39, -37, -39, -45, -55, -62, -67, -69, -69, -65, -60, -57, -55, 
-52, -50, -42, -31, -21, -14, -13, -18, -31, -39, -46, -51, -52, -51, -50, -48, 
-45, -42, -37, -29, -24, -18, -13, -9, -10, -17, -23, -28, -30, -28, -17, -7, 
3, 10, 13, 12, 2, -6, -12, -16, -15, -11, 0, 9, 16, 20, 20, 19, 
21, 24, 26, 27, 28, 31, 35, 40, 46, 52, 59, 64, 62, 56, 45, 34, 
21, 18, 20, 26, 35, 44, 54, 56, 53, 50, 45, 45, 54, 65, 77, 86, 
89, 84, 76, 69, 63, 58, 57, 54, 49, 41, 33, 28, 28, 35, 44, 55, 
64, 70, 72, 68, 62, 55, 50, 46, 38, 33, 29, 26, 27, 28, 29, 30, 
33, 38, 44, 51, 54, 53, 50, 45, 37, 22, 9, -4, -14, -19, -20, -15, 
-10, -7, -5, -1, 7, 12, 13, 13, 10, 7, 5, 3, 2, 1, -1, -3, 
-5, -8, -14, -21, -26, -28, -29, -29, -29, -29, -28, -26, -27, -30, -31, -29, 
-20, -14, -11, -14, -21, -25, -26, -25, -27, -31, -37, -46, -59, -66, -72, -74, 
-71, -60, -52, -45, -39, -35, -35, -36, -37, -36, -33, -30, -25, -19, -18, -19, 
-25, -32, -44, -51, -57, -62, -62, -59, -49, -41, -36, -33, -32, -33, -33, -34, 
-34, -33, -30, -24, -21, -22, -26, -31, -36, -38, -37, -38, -40, -45, -48, -47, 
-43, -38, -32, -30, -33, -39, -44, -49, -52, -53, -51, -46, -41, -35, -30, -26, 
-21, -22, -23, -24, -24, -20, -17, -18, -21, -25, -25, -21, -19, -16, -14, -13, 
-14, -23, -29, -34, -36, -34, -27, -24, -21, -20, -21, -23, -22, -17, -9, 0, 
5, 6, 7, 7, 4, -2, -8, -14, -13, -11, -7, -4, -1, 4, 7, 8, 
8, 8, 8, 6, 4, 1, -2, -2, 5, 15, 25, 34, 38, 38, 37, 36, 
35, 35, 34, 36, 36, 33, 27, 17, 3, -17, -25, -26, -24, -18, -9, 13, 
29, 41, 49, 52, 51, 50, 46, 40, 30, 19, 4, -1, -1, 2, 7, 12, 
17, 17, 15, 11, 5, -3, -3, 0, 6, 10, 10, 2, -4, -9, -14, -17, 
-16, -12, -10, -11, -12, -13, -14, -15, -18, -21, -23, -24, -25, -27, -28, -26, 
-20, -14, -10, -12, -16, -24, -30, -32, -31, -25, -17, -11, -9, -15, -20, -24, 
-28, -29, -26, -20, -12, -5, -2, 0, -3, -6, -9, -10, -9, -4, 2, 0, 
-6, -14, -20, -23, -20, -13, -5, 6, 15, 25, 26, 25, 26, 28, 30, 25, 
17, 10, 3, -3, -5, -6, -1, 7, 16, 25, 30, 30, 28, 26, 24, 24, 
30, 36, 42, 45, 46, 46, 44, 42, 40, 41, 45, 48, 47, 43, 39, 35, 
30, 18, 12, 11, 14, 19, 24, 24, 22, 17, 13, 9, 13, 21, 31, 41, 
48, 50, 43, 36, 31, 24, 18, 8, 3, 0, -2, -3, -3, 6, 17, 28, 
38, 45, 48, 50, 48, 44, 39, 33, 24, 17, 9, 0, -8, -15, -21, -21, 
-17, -8, 2, 9, 12, 14, 16, 18, 17, 12, 8, 7, 5, -2, -12, -23, 
-23, -19, -13, -7, -4, 0, 2, 7, 13, 21, 31, 33, 31, 26, 20, 12, 
-5, -17, -27, -31, -31, -30, -35, -39, -40, -37, -35, -36, -35, -29, -20, -12, 
-8, -8, -11, -19, -29, -38, -40, -36, -30, -25, -21, -18, -13, -11, -11, -11, 
-8, -3, -2, -10, -23, -35, -41, -44, -44, -40, -34, -24, -17, -18, -27, -37, 
-42, -43, -41, -37, -36, -34, -32, -30, -30, -32, -36, -41, -42, -38, -25, -16, 
-9, -4, 0, 1, -3, -10, -18, -24, -24, -19, -17, -16, -16, -16, -16, -17, 
-16, -13, -8, -1, 4, 8, 7, 6, 3, 4, 10, 16, 20, 20, 19, 17, 
15, 16, 18, 21, 28, 34, 40, 41, 41, 40, 38, 39, 41, 42, 43, 43, 
41, 37, 32, 26, 20, 14, 10, 7, 10, 18, 29, 38, 47, 51, 53, 53, 
52, 50, 46, 45, 43, 45, 47, 50, 54, 55, 56, 54, 49, 42, 39, 39, 
43, 50, 57, 60, 63, 65, 67, 68, 66, 62, 61, 63, 67, 71, 71, 66, 
59, 51, 45, 39, 32, 29, 28, 31, 38, 47, 64, 73, 80, 83, 83, 78, 
71, 65, 61, 61, 65, 68, 65, 60, 58, 57, 58, 63, 65, 67, 68, 67, 
58, 47, 38, 33, 32, 35, 42, 45, 46, 45, 43, 39, 31, 27, 24, 20, 
17, 9, 4, 0, -2, -1, 2, 6, 5, 3, 2, -1, -9, -18, -28, -37, 
-45, -52, -59, -61, -59, -55, -50, -44, -37, -35, -34, -34, -33, -36, -42, -50, 
-62, -74, -83, -94, -97, -95, -88, -78, -70, -63, -65, -71, -78, -86, -99, -105, 
-110, -113, -115, -117, -119, -120, -119, -118, -118, -120, -121, -121, -121, -120, -118, -111, 
-105, -101, -99, -96, -94, -95, -97, -100, -103, -105, -105, -106, -107, -110, -112, -113, 
-107, -100, -93, -86, -80, -78, -80, -86, -92, -97, -100, -102, -98, -93, -85, -76, 
-69, -56, -46, -39, -36, -36, -39, -45, -50, -54, -60, -63, -64, -58, -50, -40, 
-30, -21, -12, -8, -9, -11, -13, -13, -8, -3, 4, 12, 19, 24, 31, 36, 
40, 44, 47, 46, 42, 36, 30, 26, 26, 34, 43, 52, 60, 66, 73, 82, 
83, 80, 76, 73, 70, 68, 66, 66, 72, 80, 92, 97, 100, 99, 98, 98, 
97, 96, 94, 92, 91, 94, 96, 95, 93, 89, 86, 85, 84, 84, 84, 87, 
93, 94, 93, 93, 94, 97, 103, 106, 109, 111, 114, 118, 123, 126, 127, 126, 
121, 105, 91, 79, 69, 63, 60, 63, 71, 79, 86, 91, 93, 96, 98, 100, 
98, 93, 81, 69, 55, 42, 32, 30, 42, 53, 62, 67, 67, 65, 59, 50, 
38, 25, 14, 3, -2, -5, -8, -8, -8, -6, -6, -8, -10, -12, -15, -19, 
-23, -29, -33, -36, -38, -38, -41, -45, -50, -53, -54, -58, -62, -67, -71, -74, 
-76, -73, -68, -60, -51, -43, -44, -51, -60, -70, -80, -92, -102, -109, -112, -111, 
-105, -92, -83, -71, -61, -54, -49, -51, -56, -62, -69, -79, -94, -103, -110, -113, 
-115, -113, -98, -85, -73, -66, -64, -67, -72, -77, -83, -88, -88, -77, -69, -63, 
-61, -59, -55, -50, -47, -46, -48, -50, -51, -54, -58, -62, -62, -60, -50, -44, 
-40, -40, -42, -47, -51, -53, -54, -53, -48, -35, -24, -16, -10, -5, -1, 3, 
2, -5, -14, -27, -43, -50, -52, -49, -42, -32, -16, -8, -4, -3, -2, -1, 
0, };
