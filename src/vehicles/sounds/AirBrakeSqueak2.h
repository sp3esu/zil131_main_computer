const unsigned int brakeSampleRate = 22050;
const unsigned int brakeSampleCount = 8192;
const signed char brakeSamples[] = {
0, -1, -1, -1, -1, -1, -1, -1, 0, -1, -1, 0, 0, 0, 1, 0,
0, -1, -1, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 
0, -1, -1, -1, 0, -1, -1, -1, 0, 0, 1, 2, 2, 1, 1, 1, 
1, 1, 0, -2, -2, -3, -5, -6, -6, -3, -3, -4, -4, -3, -1, -1, 
1, 4, 4, 4, 2, 3, 3, 2, 1, 0, 2, 1, -1, -3, -4, -4, 
-3, -1, 1, 1, -1, -2, 0, 2, 4, 3, 0, -3, -3, -2, -1, -2, 
-2, -3, -2, 0, 1, 2, -1, -2, -1, -1, -1, -1, 1, 4, 3, -1, 
-2, -1, -1, -2, -2, -2, -2, 2, 6, 11, 14, 13, 10, 4, 1, -3, 
-7, -11, -17, -21, -20, -16, -12, -8, -2, 1, 4, 9, 14, 18, 15, 13, 
13, 11, 4, -5, -7, -6, -6, -9, -10, -8, -5, -5, -7, -7, -4, -2, 
-1, 3, 7, 5, -1, 3, 7, 5, -2, -8, -11, -9, -3, 3, 4, 2, 
3, 5, 5, 4, 3, 3, 4, 6, 8, 9, 8, 5, 0, -5, -8, -8, 
-8, -7, -9, -8, -3, 1, 1, -2, -4, -4, -4, -7, -11, -11, -7, 1, 
6, 6, 7, 6, 5, 7, 11, 12, 9, 8, 6, 2, -1, 0, -1, -5, 
-11, -14, -16, -18, -14, -9, -8, -9, 3, 11, 13, 12, 8, 3, 4, 7, 
5, -3, -15, -18, -14, -13, -11, -8, -2, 4, 10, 17, 21, 21, 19, 16, 
14, 13, 10, 4, -2, -8, -12, -15, -19, -18, -14, -14, -10, -3, 2, 7, 
14, 11, 6, 3, 3, 2, -11, -21, -23, -21, -22, -16, -5, 5, 8, 10, 
19, 25, 16, 7, 5, 4, 0, 4, 8, 8, 5, -2, -14, -19, -18, -12, 
-3, 3, 2, 3, 6, 6, 1, -5, 0, 2, 1, 3, 5, 5, 14, 24, 
22, 10, -6, -27, -28, -16, -3, 1, 2, -2, -10, -14, -6, 4, 1, 1, 
3, -1, -8, -8, 4, 8, 7, 10, 16, 7, -6, -16, -19, -20, -16, -12, 
-11, -5, 1, 4, 4, 9, 10, 11, 17, 19, 12, 13, 17, 22, 19, 0, 
-30, -33, -31, -29, -24, -12, 4, 9, 11, 10, 5, 1, 0, -6, -9, -7, 
-8, -10, -4, 13, 29, 29, 19, 6, 9, 17, 19, 15, 11, 2, -9, -12, 
-17, -32, -47, -42, -32, -22, -11, -3, 9, 14, 9, 5, 12, 18, 12, 6, 
7, 14, 20, 28, 31, 27, 10, -7, -17, -22, -28, -28, -17, -13, -17, -14, 
-2, 5, 1, -4, -13, -8, 4, 14, 18, 15, 7, -1, 2, 6, 0, -7, 
-7, -9, -10, -6, 3, 13, 13, 7, 7, 7, -4, -11, -12, -10, -10, -15, 
-21, -22, -15, -2, 12, 17, 14, 14, 14, 11, 12, 16, 18, 13, -1, -19, 
-30, -29, -32, -30, -15, -5, -5, 5, 14, 12, 13, 28, 42, 38, 31, 24, 
20, 14, 2, -7, -14, -27, -42, -46, -38, -30, -20, -21, -26, -11, 2, 12, 
20, 23, 25, 23, 19, 20, 13, -7, -14, -3, -6, -16, -16, -5, 2, 2, 
9, 18, 18, 9, 5, 5, -2, -12, -23, -26, -23, -16, -4, 2, 9, 22, 
24, 12, 0, -5, -4, 4, 13, 21, 20, 0, -16, -7, -18, -29, -20, -4, 
1, 3, 9, 5, 3, 10, 7, -3, -8, -15, -28, -34, -42, -45, -28, -11, 
-9, 22, 49, 46, 28, 33, 61, 68, 49, 34, 26, 19, 1, -18, -38, -45, 
-43, -42, -40, -41, -41, -30, -15, -10, -7, -6, -6, -1, 0, 9, 27, 34, 
30, 17, 15, 23, 7, 0, 3, 2, -4, -17, -11, -1, 5, 8, 0, -6, 
-16, -23, -18, -10, 9, 23, 23, 7, -3, -2, -18, -22, -12, 2, 15, 20, 
11, -8, -28, -36, -30, -28, -26, -10, 8, 9, 1, 13, 26, 34, 37, 33, 
28, 19, 2, -5, -6, -14, -12, 4, 0, -14, -22, -22, -4, 2, -3, -2, 
16, 27, 22, 19, 14, 13, 17, 8, -10, -17, -25, -49, -54, -36, -40, -34, 
-13, 9, 17, 9, 15, 17, 0, -7, 14, 9, -7, -2, 10, 9, -6, -1, 
8, 7, 13, 40, 24, 5, 11, 17, 3, -13, -22, -40, -36, -15, -3, -19, 
-17, 8, 11, -12, -15, 9, 21, 15, 11, 19, 16, 10, 12, 14, 8, -9, 
-9, -9, -12, -11, -4, 8, 3, -16, -26, -26, -27, -26, -14, 5, 23, 27, 
24, 25, 12, 3, -1, 0, 22, 24, 13, -1, -7, -3, 11, 5, -7, -6, 
0, -11, -24, -24, -11, 2, -4, -27, -29, -21, -17, -13, -14, -26, -19, -5, 
8, 20, 37, 36, 22, 8, 12, 23, 34, 25, -1, -18, -17, -10, -37, -46, 
-17, 6, -15, -46, -15, 9, 15, 30, 43, 43, 34, 30, 36, 42, 27, -7, 
-13, -25, -40, -35, 0, 0, -25, -31, -1, 22, 4, -13, -9, 1, -1, -10, 
-19, -23, -31, -23, -11, -6, -15, -24, -15, -4, 6, 14, 17, 22, 20, 23, 
36, 42, 28, 6, -6, -5, -3, -3, -4, -1, -6, -17, -12, -5, -2, 0, 
2, -11, -13, -3, -5, -11, -10, 5, 1, 2, 8, 4, -6, -9, -5, 0, 
12, 22, 1, -2, 6, 3, -2, -10, -24, -29, -35, -32, -22, -2, 11, 3, 
19, 40, 36, 19, 39, 45, 31, 21, 11, -6, -17, -21, -21, -22, -24, -42, 
-46, -29, -14, -6, -1, 6, 21, 25, 14, 3, 18, 24, 27, 31, 26, 18, 
7, 0, -9, -26, -37, -29, -33, -41, -32, -19, -25, -29, -4, 8, 0, 8, 
29, 19, 11, 22, 41, 37, -3, -4, 13, 30, 26, 1, -24, -19, -16, -21, 
-15, 4, 13, -3, -15, -11, 10, 18, 5, 2, 6, 16, 21, -2, -21, -30, 
-29, -38, -44, -32, -24, -17, -14, -19, 1, 33, 33, 18, 32, 46, 34, 42, 
35, 13, -4, -10, -7, -1, -1, -20, -34, -8, 13, 18, 15, 6, 0, -1, 
-8, -26, -40, -28, -15, -3, 3, 5, 4, 6, 16, 17, 10, -2, -6, -1, 
7, 7, -1, -12, -13, -8, -19, -37, -35, -3, 9, -7, 1, 12, 23, 16, 
10, 21, 12, 10, 17, 15, 1, 0, -1, -6, -21, -24, -1, 9, 3, -16, 
-23, -9, 12, 11, 5, -4, -11, 4, 12, 9, -6, -18, -9, 11, 7, -5, 
-11, -7, 1, 2, 7, 6, -5, 1, 16, 15, 5, -6, 0, 9, -5, -14, 
-16, -13, -11, -14, -11, -8, 0, 4, 9, 25, 9, 15, 25, 8, -10, -10, 
2, -13, -41, -31, -1, 1, 5, 15, 6, 1, 18, 18, 16, 5, -3, 0, 
-3, -11, -29, -22, -8, -22, -15, 2, -5, -20, -17, 3, 14, 24, 26, 27, 
39, 51, 32, 8, 10, 10, -19, -43, -53, -57, -49, -31, -18, -28, -27, -13, 
-2, 4, 23, 31, 48, 53, 40, 44, 55, 47, 28, 10, -6, -21, -25, -27, 
-26, -36, -48, -48, -29, -8, -6, -12, -8, 1, -5, -12, 6, 29, 25, 14, 
3, 4, 10, 3, 34, 47, 19, -13, -17, -2, -7, -11, -16, -20, -18, -5, 
-1, -5, -12, -17, -4, 5, 4, 7, -11, -18, 15, 15, 7, 10, 26, 44, 
29, 12, 4, 5, -4, -30, -50, -45, -23, -14, -17, 3, 22, 14, -11, -11, 
3, 20, 0, -32, -39, -14, 15, 24, 39, 26, -23, -36, 22, 24, 2, -10, 
-11, -15, -8, 16, 37, 36, 16, 5, 26, 35, 29, 7, -13, -37, -37, -23, 
-26, -48, -63, -12, 7, 9, 7, 5, 19, 30, 37, 36, 17, -9, -5, 12, 
9, -2, -12, -11, -11, -14, -11, -25, -37, -29, -9, 12, 28, 20, -10, -20, 
3, 20, 16, 3, 7, 0, -4, 7, 11, 4, 10, 39, 58, 54, 25, 23, 
28, -5, -45, -52, -34, -33, -42, -45, -55, -67, -49, -7, -20, -34, -19, 5, 
24, 46, 38, 33, 28, 21, 36, 47, 44, 40, 22, -5, -25, -9, 0, -23, 
-51, -53, -11, -21, -51, -37, 9, 48, 51, 39, 15, 0, 8, 8, 16, -5, 
-18, -12, -3, 3, -7, -6, -6, -25, -51, 2, 25, -5, -34, -31, 16, 18, 
7, -9, -34, -25, 13, 25, 35, 21, 24, 48, 57, 24, -15, -9, 9, 2, 
-12, -30, -32, -32, -26, -37, -52, -52, -39, -13, 0, 13, 34, 28, 2, -12, 
57, 73, 46, 34, 24, 18, 10, 4, -10, -28, -40, -42, -34, -38, -26, -11, 
-22, -34, -20, 1, 11, -10, -5, 23, 15, 14, 31, 47, 45, 25, 8, 22, 
34, 33, 24, 2, -25, -37, -31, -40, -51, -20, -6, -27, -18, 14, 19, 5, 
-16, -26, -30, -8, 4, 3, 7, 28, 39, 0, -26, -17, 10, 7, -10, 3, 
22, 19, 19, 14, -27, -37, -8, 32, 33, 1, -25, 6, 19, 18, 20, 22, 
-6, -9, 6, -18, -23, 2, -11, -19, -13, -12, -15, -1, -11, -40, -43, 4, 
50, 37, 15, -10, -25, -35, -11, 2, 1, -7, -18, 1, 38, 21, 14, 28, 
26, 44, 51, 19, -19, -31, -23, -3, -11, -29, -42, -45, -35, -1, -6, -31, 
-23, 11, 23, 15, 26, 21, 3, 3, 26, 26, 16, 7, 7, 24, 13, 9, 
0, -3, -14, -35, -36, -52, -35, -13, -9, -34, -23, -5, -19, -20, 11, 35, 
18, 17, 27, 23, 6, 17, 7, -6, 20, 28, 3, 12, 21, 8, -7, 16, 
59, 31, -22, -51, -45, -28, -41, -34, -23, -19, -6, -9, -10, -7, 12, 29, 
14, -17, -15, -1, 9, 12, 9, 7, 7, -2, -6, -14, -36, -34, -19, -4, 
0, 18, 27, 15, 9, 15, 37, 60, 31, -2, -20, -14, -6, 17, 18, -7, 
-6, 2, -17, -24, -24, -19, -13, 0, 3, -5, -6, -12, -9, 1, 12, 7, 
-8, 12, 42, 28, 5, -13, -21, -23, -29, -53, -49, -38, -35, -26, -19, -13, 
-3, 3, -4, 13, 47, 28, 23, 35, 49, 43, 31, 33, 19, 3, -4, 0, 
7, 2, 7, 7, -13, -21, -5, 6, 3, -2, 16, 24, 9, -5, -27, -42, 
-64, -66, -74, -67, -24, -11, -5, 4, 2, -8, -4, 12, 37, 51, 44, 36, 
42, 66, 51, 26, 4, -11, 16, 11, -39, -65, -47, -29, -46, -63, -38, -7, 
6, 17, 30, 29, 28, 26, 20, 15, 7, 1, -20, -35, -9, 16, -20, -29, 
-12, 2, -13, -18, 41, 41, 4, -12, 5, 12, 2, -6, 0, 6, 13, 9, 
1, -6, -14, -12, -34, -37, -32, -35, -33, 0, 22, -16, -24, -8, 16, 35, 
9, 4, 20, 52, 66, 11, -1, 35, 21, -24, -16, 4, -21, -27, -6, -18, 
-40, -59, -42, -17, -6, 4, 4, 13, 0, 4, 20, 26, 14, -3, 9, 19, 
11, -7, -13, -10, -4, 22, 33, 19, 3, -3, -11, -30, -28, 5, 27, 26, 
27, 26, -2, -25, -19, -3, -15, -32, -7, 14, 0, -48, -35, 8, -2, -15, 
-27, -19, 4, 15, 9, 9, 7, 16, 27, 12, 6, -15, -19, -6, -5, 9, 
16, 24, 16, 5, 12, 33, 11, -14, 2, 11, -10, -6, 14, 6, -18, -24, 
-17, -20, -44, -42, -1, 25, -6, 5, 26, 26, 13, -36, -46, -34, -24, -5, 
-4, -37, -18, -6, 4, 27, 53, 53, 55, 62, 44, 20, 15, 24, 21, -4, 
-39, -49, -25, -30, -32, -26, -25, -44, -53, -38, -10, -1, -20, 4, 27, 43, 
59, 35, 6, -4, 12, 13, -17, -37, -34, -3, 19, 28, 21, 3, 45, 73, 
75, 62, 46, 33, -11, -22, -27, -49, -68, -68, -65, -60, -36, -13, -7, 4, 
-7, -9, 23, 30, 4, 5, 37, 45, 26, 26, 29, 7, -18, -29, -36, -50, 
-32, -25, -44, -49, -22, 15, 46, 25, -1, 24, 62, 32, 7, 37, 54, 21, 
10, 1, -3, -7, -7, 6, 1, -22, -33, -25, -38, -34, -3, -21, -41, -61, 
-46, -22, 8, 29, 39, 31, -3, 2, 22, 7, 13, 42, 36, -22, -15, -12, 
-13, -24, -18, -5, -10, 1, -2, 17, 24, 9, 9, 7, 13, 20, 15, -33, 
-17, 33, 47, 49, 21, -16, -41, -40, -38, -30, -14, -28, -24, -10, -2, -33, 
-46, -28, -10, 7, 7, -21, -13, -4, 3, 22, 69, 69, 51, 46, 36, -4, 
-33, 8, 25, 20, 17, 5, -62, -63, -11, 17, 18, -13, -6, -10, -23, -21, 
-34, -28, -10, 24, 36, 15, -11, -14, 1, 14, 38, 56, 29, -15, -30, -30, 
-12, 24, 27, 2, -4, 1, -2, -19, -48, -30, -33, -57, -54, -25, -43, -33, 
29, 70, 68, 43, 44, 47, 28, 9, 17, 40, 39, 37, 29, 10, -39, -44, 
-50, -62, -35, -1, 2, -11, -7, -10, -30, -36, -32, -10, -3, -12, -18, -14, 
-12, 1, 12, 4, 21, 39, 23, -3, -2, 40, 64, 45, 44, 50, 15, -41, 
-28, -21, -28, 4, 43, 14, -45, -37, -52, -63, -51, -30, 2, 14, 6, -6, 
2, -10, -1, 16, 15, 10, 6, 24, 25, 23, 48, 67, 77, 41, 7, -12, 
-49, -69, -22, 4, -4, -19, -38, -49, -33, -39, -52, -49, -22, -21, -23, -18, 
-18, -1, 24, 54, 75, 78, 44, 4, 9, 77, 101, 92, 70, 41, -19, -68, 
-102, -98, -55, -12, -15, -22, -38, -53, -44, -30, -20, -12, -1, -16, -32, 7, 
4, -7, 26, 69, 73, 40, 30, 21, -2, -1, 52, 71, 35, 11, -2, -21, 
-74, -74, -29, -2, -12, -9, 6, -4, -14, -12, -11, -37, -24, -7, -26, -32, 
-27, -11, 2, 12, 31, 25, -25, -53, -16, 31, 71, 101, 117, 47, -10, 3, 
16, -10, -42, -10, 24, 10, -21, -45, -44, -18, 2, -11, -29, -25, -6, 3, 
0, 4, 11, 18, 52, 64, 12, -31, -24, 25, 42, 20, 9, 23, 22, -52, 
-95, -95, -73, -48, 3, 18, 10, -3, -6, 0, 23, 45, 51, 41, 20, -1, 
-19, 3, 30, 34, 32, 8, 1, -2, -26, -40, -5, 63, 43, 7, -16, -29, 
-53, -80, -76, -40, 0, 13, 26, 38, 29, 22, 22, 20, 1, -5, -26, -41, 
-37, -35, -45, -49, -15, 19, 8, -23, 9, 33, 53, 86, 109, 120, 83, 37, 
0, -29, -53, -41, -31, -38, -39, -38, -56, -57, -46, -40, -30, -3, 26, 22, 
24, 8, -28, -13, 19, 15, -6, -5, -4, -36, -28, 25, 80, 93, 71, 31, 
2, -26, -27, -3, 5, -7, -6, -1, -13, -36, -39, -19, -7, 1, 14, 16, 
-20, -33, -43, -52, -35, -4, 11, 19, 11, 6, 15, 58, 81, 95, 106, 99, 
58, -21, -54, -37, -1, -7, -47, -67, -65, -59, -63, -68, -39, -17, -16, -17, 
-13, -4, -14, -19, -6, 9, 13, 55, 76, 51, 21, 20, 46, 88, 85, 85, 
75, 41, 2, -64, -70, -57, -39, -25, -45, -88, -127, -111, -59, -13, 3, -19, 
-18, 14, 27, -9, -9, 20, 50, 75, 85, 49, 18, -6, 5, 43, 71, 75, 
55, 19, -8, -18, -22, -27, -31, -30, -37, -65, -86, -82, -69, -32, -7, -12, 
-25, -19, -6, 4, 2, 2, 34, 62, 55, 31, 20, 2, 3, 20, 45, 77, 
98, 70, 9, -37, -57, -63, -28, -8, -14, -32, -42, -45, -39, -19, -9, -13, 
-12, -4, -17, -24, -7, 9, 14, 34, 55, 58, 36, 19, 23, 19, 23, 48, 
69, 67, 39, -38, -70, -80, -87, -77, -20, -3, -22, -55, -70, -55, -18, -4, 
9, 27, 25, -6, -10, 31, 50, 55, 68, 68, 47, 20, 3, 4, 24, 44, 
47, 60, 42, -15, -69, -85, -76, -59, -38, -23, -15, -48, -69, -52, -28, -18, 
4, 26, 39, 36, 19, 0, -1, 27, 37, 22, 14, -3, -16, -7, 11, 41, 
74, 73, 52, 34, 18, -15, -49, -28, -7, -1, -16, -42, -71, -84, -76, -53, 
-33, -20, -6, -7, -2, 9, 8, 1, 29, 69, 85, 57, 8, -8, 19, 55, 
66, 67, 66, 27, 5, -26, -47, -43, -24, -15, -15, -26, -51, -76, -56, -35, 
-36, -38, -22, 2, 6, -8, -20, -19, 0, 50, 61, 44, 28, 27, 29, 28, 
36, 54, 75, 71, 26, -41, -48, -33, -22, -20, -19, -33, -44, -57, -74, -69, 
-25, -2, 1, 0, 10, 18, -1, -11, 14, 55, 69, 44, 17, 10, 14, 4, 
-3, 37, 60, 51, 29, 2, -23, -40, -43, -38, -32, -26, -48, -62, -52, -22, 
-4, -27, -22, 21, 20, -15, -25, -4, 18, 15, 26, 52, 63, 30, 3, 13, 
40, 53, 60, 68, 52, 19, -11, -35, -45, -25, -28, -42, -57, -71, -76, -55, 
-41, -43, -45, -35, -11, -15, -13, 6, 26, 26, 27, 62, 78, 53, 30, 35, 
32, 36, 61, 75, 75, 47, 10, -30, -52, -48, -44, -52, -60, -70, -82, -89, 
-60, -34, -33, -38, -20, 13, 24, 21, 32, 48, 61, 43, 39, 34, 26, 26, 
15, 0, 22, 53, 60, 48, 34, -7, -23, -32, -53, -61, -21, -22, -39, -48, 
-44, -27, -20, -27, -19, -4, -6, -11, -10, -7, 2, 15, 25, 32, 18, -3, 
2, 20, 33, 56, 58, 51, 52, 51, 20, -28, -22, -16, -7, -11, -69, -63, 
-27, -22, -58, -79, -39, -21, -14, -14, 2, 26, 12, 20, 30, 40, 54, 39, 
25, 26, 42, 45, 28, 38, 36, 2, -29, -41, -47, -44, -42, -44, -41, -29, 
-11, -16, -22, -22, -18, -9, 0, 1, 3, 6, 11, 13, 3, 19, 47, 49, 
28, 13, 17, 24, 42, 66, 79, 63, 16, -26, -31, -14, -45, -77, -76, -55, 
-45, -63, -69, -46, -31, -23, -15, -14, -1, 8, 0, 3, 21, 6, 5, 34, 
52, 49, 39, 24, 28, 37, 58, 75, 76, 50, 19, -5, -26, -43, -63, -63, 
-44, -38, -57, -62, -28, -41, -59, -41, -1, 22, 4, -6, 3, 23, 44, 54, 
40, 29, 25, 15, 9, 24, 33, 38, 42, 41, 8, -41, -41, -52, -64, -55, 
-31, -24, -37, -46, -24, -1, -20, -29, -3, 34, 42, 30, 24, 35, 50, 63, 
74, 54, 24, 0, -4, -2, -1, 15, 27, 29, -13, -72, -83, -76, -86, -83, 
-65, -42, -17, -4, 5, 4, -10, -20, 20, 44, 40, 15, 8, 25, 31, 23, 
28, 52, 65, 32, 11, 7, 10, 22, 34, 35, 39, 22, -8, -27, -46, -73, 
-57, -43, -48, -53, -42, -21, -33, -61, -44, -6, -8, -12, 4, 17, 25, 14, 
7, 37, 77, 65, 16, 22, 44, 45, 47, 46, 52, 45, 13, -8, -33, -60, 
-52, -15, -11, -39, -54, -48, -32, -42, -44, -21, -4, 2, -5, -14, -13, -8, 
1, 50, 75, 50, 9, 11, 36, 23, 28, 41, 43, 42, 25, -25, -49, -58, 
-57, -62, -69, -49, -34, -30, -16, -4, -8, 14, 26, 17, 26, 35, -1, -12, 
10, 44, 60, 42, 24, 17, 20, 12, 1, 36, 55, 44, 5, -20, -14, -19, 
-37, -66, -89, -80, -47, -55, -59, -47, -47, -47, 0, 38, 55, 58, 62, 48, 
10, 10, 18, 30, 59, 75, 46, 25, 10, 8, 33, 40, 20, 5, -11, -38, 
-67, -62, -33, -26, -55, -80, -51, -43, -47, -47, -43, -24, -1, -8, 7, 20, 
5, -3, 44, 79, 85, 71, 75, 75, 21, -13, 3, 28, 41, 24, -5, -18, 
-24, -43, -44, -46, -66, -64, -43, -36, -27, -6, 14, 16, 10, 13, 10, 6, 
6, -1, -5, 10, 20, 25, 27, 26, 23, 27, 23, 8, 7, 22, 23, -13, 
-16, -13, -22, -31, -60, -76, -52, -17, -19, -26, -4, -2, -2, 8, 27, 48, 
32, -11, -19, 12, 26, 24, 39, 47, 40, 24, 15, 41, 37, 13, 11, 26, 
20, -34, -69, -76, -53, -48, -82, -68, -44, -58, -81, -67, -20, 16, 40, 35, 
44, 61, 30, 32, 48, 47, 45, 54, 55, 53, 36, 15, 15, 19, 11, 8, 
1, -9, -18, -41, -60, -76, -80, -79, -58, -40, -33, -24, -5, 7, 16, 34, 
37, 30, 23, 12, -22, -29, 8, 58, 76, 37, 7, 9, 18, 11, -1, -16, 
0, 36, 48, 21, -10, -27, -25, -24, -39, -60, -33, 1, -1, -1, 9, -9, 
-9, 5, -5, -15, -12, -15, -7, 2, 5, 24, 25, 24, 54, 45, 16, 6, 
-9, -10, 31, 52, 25, -12, -22, -22, -39, -51, -46, -38, -33, -45, -52, -35, 
-1, 36, 44, 36, 28, 28, 41, 57, 62, 25, -18, -6, 18, -10, -24, -22, 
-24, -39, -43, -4, 8, -1, -11, -13, -1, 5, 5, 7, 0, -22, -34, -24, 
-10, 6, 22, 20, -20, -26, 1, 30, 28, 10, 29, 46, 41, 14, 7, 57, 
56, 17, -20, -27, -20, -42, -43, -26, -35, -52, -41, -32, -22, -21, -32, -21, 
18, 34, 33, 19, 14, 15, -1, -13, -2, 23, 55, 75, 47, 21, 20, 23, 
9, -17, -32, -38, -36, -34, -32, -28, -36, -48, -38, -19, -20, -18, -7, 21, 
43, 32, 14, 28, 28, 2, -13, 0, 24, 12, 3, 33, 47, 9, -7, -16, 
-1, 21, 27, 25, 23, 9, -19, -29, -26, -33, -47, -63, -81, -104, -96, -50, 
3, 22, -1, -3, 33, 28, 15, 13, 40, 108, 103, 62, 45, 67, 85, 52, 
19, 14, 17, -14, -65, -72, -44, -52, -70, -53, -18, -20, -37, -56, -63, -57, 
-11, 20, 32, 36, 30, 11, -15, -32, -38, -11, 21, 35, 41, 51, 60, 47, 
32, 47, 37, 14, 4, 9, 15, -23, -60, -35, 9, 6, -24, -12, 12, -1, 
-50, -68, -17, -3, -10, -12, -13, -23, -16, 7, 10, 9, 22, 27, 17, -5, 
-5, 22, 31, 33, 36, 21, -2, -15, -16, -15, -18, -37, -45, -35, -38, -25, 
-16, -9, 28, 69, 81, 44, -1, -4, 7, -5, -23, -21, -10, -1, -5, -21, 
-19, -8, -20, -38, -10, 45, 26, -4, 9, 48, 38, -27, -12, 18, 9, -27, 
-17, 28, 28, -6, -16, 2, 20, -5, -35, -30, 2, -5, -49, -45, -2, 9, 
-10, -2, 4, 8, 19, 22, 14, 8, 12, 7, -20, -32, 24, 43, 14, -25, 
-36, -21, -8, -31, -24, 11, 15, 4, 14, 16, 11, 9, 20, 46, 39, 19, 
11, 10, 7, 11, 11, 9, 1, -15, -22, -31, -46, -47, -40, -21, -13, -55, 
-72, -37, -1, -2, -27, -27, -20, -6, 6, 8, 21, 29, 39, 57, 61, 50, 
60, 61, 52, 52, 43, 2, -4, 16, 32, 19, 7, 14, -21, -60, -77, -81, 
-80, -76, -65, -48, -27, -13, -18, -12, 6, 3, -18, -15, 0, -4, 7, 13, 
11, 35, 34, 21, 32, 49, 33, 31, 47, 22, 1, 5, 5, 9, 26, 33, 
31, 32, 2, -26, -35, -29, -25, -33, -65, -64, -51, -58, -66, -45, -11, 2, 
10, 7, 6, 23, 41, 57, 75, 79, 48, 9, 29, 37, 11, -1, -1, -26, 
-45, -48, -46, -30, 7, 8, -14, -45, -47, -11, 16, 19, 20, 14, 8, -27, 
-26, 12, 34, 20, -1, 3, 20, 20, 9, 14, 34, 20, -10, -17, -13, -14, 
-30, -44, -34, -19, -18, -15, -1, 20, 31, 20, -1, -21, -6, 4, 0, 0, 
3, 11, -2, -13, -5, 1, 1, -9, -8, 12, 18, -1, -7, 16, 19, 16, 
20, 28, 14, -10, -10, 8, 11, -16, -51, -40, -33, -32, -17, 7, 25, 30, 
37, 37, 32, 22, 20, 25, 26, 11, -12, -19, -16, -23, -39, -47, -42, -54, 
-61, -41, -2, 17, -4, 12, 33, 34, 22, 1, 0, 2, -6, -4, 12, 20, 
-6, -2, 6, 13, 25, 26, 20, 17, 23, 18, 1, 15, 34, 25, -11, -33, 
-32, -53, -65, -54, -25, -8, -30, -29, -12, 5, 25, 31, 9, 5, 8, 8, 
-6, -5, 12, 7, 6, 21, 33, 32, 27, 27, 28, 24, -2, -9, 17, 12, 
4, 18, 19, -14, -44, -49, -40, -29, -37, -51, -57, -53, -34, -20, -19, -1, 
21, 25, 15, 26, 39, 46, 51, 37, 24, 26, 15, -7, -28, -22, 4, -10, 
-29, -23, -21, -15, 36, 47, 31, -5, -32, -19, 15, 4, -24, -26, 5, 12, 
-29, -31, -14, -5, -9, -8, 2, -3, -26, -23, 14, 53, 35, 6, -1, -4, 
-9, 6, 13, 7, -2, -7, 4, 23, 16, -7, -1, 20, -7, -22, -10, -5, 
-5, 14, 29, 16, 3, -10, -15, -6, -18, -36, -33, -19, -12, -15, -5, -5, 
-17, -16, -7, 8, 12, 7, -14, -29, -9, 0, 10, 28, 45, 47, 45, 47, 
28, 9, 10, 12, 0, 6, 21, 17, -13, -40, -30, -22, -26, -32, -30, -41, 
-65, -57, -15, 11, 15, 18, 14, 11, 15, 13, 6, 3, -2, -17, -35, -34, 
7, 23, 20, 22, 35, 35, 17, 17, 31, 43, 59, 50, 37, 36, 22, -9, 
-41, -52, -54, -73, -81, -68, -36, -32, -47, -53, -33, -6, 6, 19, 24, 15, 
12, 11, 18, 38, 57, 54, 32, 21, 15, 1, 3, 16, 14, -2, 6, 19, 
18, 10, 10, 2, -13, -20, -25, -41, -46, -43, -40, -31, -28, -26, -11, 9, 
16, 13, 23, 25, 19, 13, 2, 5, 13, 7, 12, 20, 19, 13, 3, -5, 
-4, -16, -33, -36, -24, -12, -11, -7, -3, -4, -16, -10, 9, 25, 30, 19, 
-2, 6, 27, 42, 33, 23, 39, 37, 18, 3, -2, 9, 12, -2, -32, -57, 
-61, -60, -58, -49, -37, -24, -13, -16, -15, 2, 21, 36, 43, 35, 8, -15, 
-9, 1, 7, 15, 28, 32, 15, -2, 1, 4, 13, 19, 17, -6, -24, -16, 
7, 15, 4, -12, -21, -34, -41, -18, 10, -3, -18, -22, -9, -3, -1, 20, 
36, 34, 31, 32, 18, 3, -2, 6, 9, -7, -12, -17, -15, -9, -12, -15, 
-9, -3, -10, -21, -20, -16, -32, -36, -21, -14, -8, -10, -10, 10, 25, 27, 
33, 27, 22, 23, 22, 29, 36, 17, 10, 21, 18, 5, 13, 26, 27, 2, 
-29, -53, -54, -62, -69, -65, -50, -32, -36, -39, -29, -12, 26, 51, 62, 52, 
33, 25, 30, 43, 46, 39, 37, 37, 25, 17, 19, 12, -12, -56, -72, -75, 
-68, -46, -11, 8, -1, -2, 5, 0, -25, -38, -20, 7, 23, 5, -21, 0, 
32, 47, 33, 20, 20, 11, -1, -7, 8, 32, 23, -1, -14, 2, 17, 6, 
4, 2, -10, -19, -14, -1, 3, -1, -13, -23, -32, -38, -45, -45, -34, -21, 
-15, -13, -10, -7, 7, 33, 48, 38, 31, 35, 31, 15, 26, 35, 22, 0, 
-15, -20, -21, -26, -34, -29, -22, -32, -23, 2, 18, 18, 18, 38, 57, 40, 
15, 24, 16, -14, -24, -19, -19, -24, -28, -25, -25, -24, -15, -5, -9, -9, 
-3, 10, 27, 17, -16, -30, -12, 11, 13, -17, -37, -45, -26, 2, 8, 14, 
32, 38, 18, 10, 55, 61, 37, 19, 20, 24, 18, 2, -19, -32, -28, -29, 
-39, -43, -36, -28, -27, -5, 8, -4, -26, -34, -30, -22, -31, -36, -21, 5, 
24, 22, 23, 35, 58, 69, 48, 46, 51, 46, 29, 16, 22, 19, 13, 5, 
-17, -42, -38, -41, -61, -75, -75, -64, -60, -57, -49, -34, -11, 12, 2, 6, 
27, 32, 38, 51, 61, 58, 37, 25, 52, 65, 54, 35, 19, 0, -6, -13, 
-13, -4, 3, -1, -21, -38, -37, -39, -58, -80, -78, -73, -58, -41, -31, -9, 
18, 34, 39, 38, 42, 48, 49, 48, 47, 52, 45, 14, -16, -21, 6, 39, 
21, -13, -27, -15, -2, 7, -10, -20, -4, -3, -39, -79, -60, -34, -21, -21, 
-38, -35, -24, -23, -19, 0, 33, 48, 58, 62, 56, 57, 77, 70, 54, 41, 
26, -30, -61, -52, -34, -33, -33, -29, -31, -35, -31, -11, 6, -4, -10, 7, 
28, 27, -17, -22, -6, -1, -13, -18, -8, -15, -21, -2, 31, 36, -11, -12, 
5, 16, 28, 49, 45, 20, 0, 1, 18, 2, -34, -47, -48, -49, -41, -18, 
-6, 1, 5, 10, 25, 28, 30, 40, 43, 31, 8, 2, -5, -10, -10, -14, 
-43, -70, -72, -37, 0, -17, -30, -9, 12, 14, 8, 16, 35, 43, 36, 28, 
28, 28, 25, 12, 4, -2, -25, -12, 2, -1, 4, 16, 15, 0, -5, 2, 
0, -17, -27, -35, -48, -58, -54, -45, -30, -15, -10, -18, -19, 18, 21, -1, 
-10, 9, 48, 56, 59, 63, 59, 48, 32, 25, 12, -3, -4, -3, -30, -34, 
-21, -5, -1, -15, -2, 10, 0, -11, -10, -23, -43, -52, -50, -48, -49, -35, 
-21, -9, 5, 19, 31, 44, 36, 14, 15, 36, 63, 55, 38, 36, 41, 36, 
29, 29, 8, -20, -33, -36, -40, -42, -37, -42, -61, -64, -57, -51, -42, -29, 
-13, -9, -1, 17, 29, 33, 56, 72, 57, 41, 45, 44, 23, 7, -5, -18, 
-27, -19, -10, -10, -4, 3, -3, -20, -1, 17, 9, -4, -12, -18, -16, -12, 
-4, -4, -12, -22, -29, -30, -17, 4, 9, 4, 11, 23, 28, 23, 0, -8, 
2, 25, 40, 12, -7, -11, -10, -6, -10, -29, -26, -10, -8, -30, -37, 5, 
22, 21, 15, 24, 30, 22, 30, 30, 13, -3, -4, -5, -15, -16, -15, -21, 
-38, -42, -36, -29, -24, -5, 9, 8, 6, 12, 24, 35, 22, 17, 22, 9, 
-6, 17, 30, 21, -6, -30, -23, -5, -1, -3, 1, 6, -1, 7, 23, 30, 
27, 19, 0, -20, -53, -70, -55, -32, -30, -18, 3, 10, 2, 4, 20, 25, 
11, -1, -9, -12, -9, -12, 3, 40, 57, 33, 21, 33, 45, 35, -3, -12, 
-12, -12, -13, -9, 21, 31, 16, -14, -35, -39, -44, -61, -78, -74, -55, -29, 
-18, -4, 8, 12, 17, 27, 23, 14, 12, 18, 44, 62, 68, 62, 51, 46, 
43, 35, 24, 3, -27, -50, -50, -36, -35, -37, -26, -13, -13, -22, -30, -29, 
-28, -35, -49, -51, -38, -27, -14, 6, 27, 30, 29, 33, 47, 55, 61, 63, 
51, 28, 18, 38, 41, 19, 1, 3, -6, -32, -57, -59, -38, -14, -32, -52, 
-40, -14, 0, -9, -16, -8, 1, 2, 7, 2, -4, -9, -6, 6, 3, -6, 
-6, 7, 22, 33, 46, 42, 33, 27, 18, 1, -8, -8, -9, -26, -48, -58, 
-39, -7, 10, -1, -6, 14, 11, 9, 22, 36, 24, 17, 17, 13, 10, 4, 
-13, -20, -29, -33, -32, -36, -30, -26, -31, -25, -1, 26, 19, 6, 6, 6, 
1, 13, 26, 21, 6, -1, -1, 2, 13, 23, 25, 19, 11, 5, 0, 9, 
22, 26, 11, -8, -23, -19, -17, -38, -61, -59, -52, -38, -22, -5, 2, 15, 
31, 35, 15, -11, 7, 22, 17, 15, 26, 39, 32, 28, 32, 24, -20, -30, 
-10, 3, -14, -35, -16, 1, -2, -6, -6, -9, -17, -29, -33, -21, -10, -15, 
-12, 6, 15, 8, 4, 13, 19, 23, 17, 7, 10, 17, 32, 42, 23, 4, 
22, 30, 19, 5, -12, -33, -44, -43, -36, -31, -30, -27, -18, -11, -8, -1, 
0, -21, -20, -8, -1, 2, 13, 30, 23, 18, 22, 27, 25, 20, 23, 27, 
16, 0, -12, -15, -5, 19, 28, 11, -9, -11, -16, -15, -4, 7, -9, -25, 
-26, -22, -24, -19, -13, -12, -11, -5, 8, 4, -15, -20, -13, -9, -7, 10, 
16, 1, -8, -4, 17, 24, 33, 43, 33, 18, 32, 43, 36, 17, -7, -34, 
-33, -28, -17, -9, -28, -50, -31, -13, -6, 2, 15, 9, 1, 6, 15, 16, 
-13, -23, -19, -14, -15, -15, 3, 10, 17, 18, 16, 23, 33, 37, 27, 10, 
4, 2, 10, 23, 21, 0, -25, -34, -32, -41, -41, -28, -23, -22, -7, 11, 
15, 15, 22, 21, 7, -12, -20, -14, -13, -16, -6, 5, -1, -13, -10, -5, 
-2, -1, 4, 21, 41, 48, 34, 13, -3, 18, 31, 21, 9, -3, -17, -15, 
-2, 10, 10, 5, 1, 8, 4, -4, -11, -33, -47, -33, -27, -43, -57, -30, 
-20, -18, -5, 10, 12, 5, 15, 42, 59, 50, 34, 32, 25, 16, 18, 20, 
18, 14, 1, -19, -33, -25, -8, -19, -30, -18, 5, 16, -14, -32, -16, 9, 
6, -19, -28, -34, -26, -26, -36, -18, -8, -7, -5, -1, 8, 32, 42, 46, 
54, 55, 30, 29, 46, 51, 33, 7, -1, 10, -4, -29, -37, -40, -60, -57, 
-38, -30, -27, -3, -3, 0, 15, 20, 6, -8, -2, -5, -20, -33, -29, -8, 
-13, -9, 1, 4, 21, 34, 38, 39, 47, 59, 51, 34, 25, 28, 21, -7, 
-20, -39, -56, -58, -41, -20, -21, -20, -17, -10, 3, 8, -4, -16, -5, 10, 
-15, -32, -33, -20, -6, -5, -14, -1, 19, 25, 33, 49, 35, 18, 16, 17, 
14, -11, -22, -15, -2, -10, -24, -8, 4, 6, 6, 12, 27, 29, 9, -3, 
7, 22, 8, -8, -6, 2, -11, -36, -44, -30, -29, -42, -49, -45, -24, -15, 
-15, -5, 19, 43, 38, 25, 25, 34, 31, 14, 14, 24, 30, 30, 19, -8, 
-18, -25, -23, -9, 9, 5, 0, -4, -1, 15, 21, 6, -17, -37, -46, -48, 
-59, -59, -38, -16, -2, 10, 4, -3, 4, 29, 51, 42, 15, 5, 26, 54, 
56, 36, 35, 32, 14, -10, -42, -55, -59, -48, -27, -24, -36, -27, -20, -21, 
-14, -1, 14, 8, -2, -3, 7, 11, 13, 32, 49, 50, 37, 18, 6, -7, 
-6, 10, 25, 11, -7, -18, -14, 1, -11, -36, -45, -48, -61, -66, -45, -33, 
-25, -4, 35, 52, 36, 28, 29, 31, 31, 24, 8, 2, 5, 0, -17, -32, 
-25, -25, -28, -27, -22, -10, 14, 34, 42, 44, 50, 47, 30, 24, 39, 40, 
-2, -15, -12, -19, -43, -71, -62, -54, -58, -56, -39, -7, 14, 25, 18, 14, 
25, 42, 43, 30, 11, 7, 6, -11, -23, -31, -32, -20, -20, -16, 4, 12, 
13, 28, 42, 43, 32, 12, 0, 4, 0, -12, -17, -17, -18, -3, 7, -1, 
-12, -7, 2, 10, 10, -5, -21, -20, -9, -16, -26, -24, -22, -26, -26, -15, 
-6, 1, 11, 21, 33, 28, 14, 4, 6, 28, 33, 25, 23, 31, 32, 9, 
-1, -8, -18, -27, -31, -19, 4, 18, 8, -4, -1, -5, -7, 7, 19, 4, 
-46, -55, -57, -62, -60, -46, -32, -27, -20, -4, 15, 54, 84, 97, 72, 38, 
36, 41, 24, 16, 14, 4, -6, -3, 1, -2, -11, -20, -21, -14, -15, -20, 
-23, -29, -20, -9, -8, -9, -9, -9, -13, -30, -44, -25, 8, 17, 12, 7, 
-5, -14, 0, 29, 30, 19, 3, -3, 7, 23, 31, 35, 52, 60, 31, 15, 
-4, -27, -32, -26, -45, -51, -41, -36, -36, -20, -10, -13, -10, 9, 23, 15, 
-4, -15, -15, -6, 11, 13, 3, 8, 13, 5, 10, 28, 39, 39, 20, 4, 
22, 15, -14, -16, 8, 10, -40, -56, -35, -4, 1, -10, -1, 17, 24, 9, 
-10, -8, 3, 2, -6, -16, -24, -24, -28, -35, -27, -3, 11, 7, 8, 16, 
26, 30, 26, 31, 34, 22, 8, 4, 12, 11, -2, -9, 4, 15, -10, -23, 
-15, -12, -12, -15, -19, -11, -6, -21, -31, -15, -1, 10, 16, 2, -23, -31, 
-36, -30, -25, -20, -5, 13, 26, 33, 48, 41, 16, 11, 21, 22, 18, 30, 
35, 29, 28, 26, 14, -10, -23, -31, -35, -32, -25, -25, -26, -26, -20, -19, 
-27, -10, 6, 1, -11, -17, -25, -16, 2, 6, -2, 1, -2, 5, 37, 63, 
55, 28, 24, 25, 18, 9, 5, -5, -17, -30, -35, -29, -32, -38, -28, -14, 
-2, 2, 6, 19, 27, 20, 12, 20, 16, -1, -6, -8, -10, -15, -14, -4, 
6, 10, 4, -4, -1, 3, 12, 15, 6, 5, 14, 11, 4, -3, -17, -13, 
-5, -16, -42, -48, -25, -9, 9, 20, 15, 2, 13, 31, 33, 20, 7, 13, 
10, -2, -9, -13, -12, -18, -22, -14, -5, -1, -4, -17, -17, -10, 3, 18, 
14, 5, 9, 13, 12, 13, 23, 20, 11, 4, 6, 2, -5, -4, -4, -11, 
-20, -9, 10, 10, -8, -21, -17, 5, 7, -12, -28, -32, -43, -43, -28, -14, 
0, 16, 19, 15, 15, 17, 12, 5, 25, 37, 36, 30, 31, 27, 14, 7, 
4, 0, -2, -5, -9, -13, -18, -23, -21, -21, -34, -44, -45, -34, -13, -10, 
-9, -4, -1, 3, 16, 16, 14, 15, 12, 11, 15, 20, 25, 25, 29, 23, 
14, 12, 13, 6, -12, -27, -32, -41, -36, -24, -20, -9, 0, -3, -5, 6, 
37, 28, -1, -20, -19, -1, 0, -14, -15, -7, 1, 2, 2, -5, -9, 2, 
18, 31, 17, -2, 10, 36, 38, 13, 13, 19, 5, -23, -45, -48, -32, -37, 
-51, -44, -7, 17, 18, 8, 6, 16, 30, 21, 12, 3, -3, 5, 5, -11, 
-23, -20, -9, 3, 5, 0, -8, -8, -2, 8, 18, 28, 32, 40, 58, 29, 
-19, -37, -25, -22, -60, -73, -59, -34, -13, -3, 1, 12, 27, 38, 44, 54, 
45, 20, 4, 6, 9, -3, -8, -18, -34, -39, -30, -17, -14, -6, -4, -13, 
-11, -1, 5, 3, 2, 10, 12, -4, -12, -3, 6, 5, -2, -4, 0, 7, 
-2, -16, -2, 9, 7, 15, 36, 48, 35, 22, 20, 22, 10, -30, -47, -46, 
-35, -26, -18, -18, -24, -26, -19, -5, 11, 14, 19, 25, 21, 12, 21, 22, 
6, -8, -20, -41, -34, -6, 12, 12, -1, -36, -36, 3, 44, 52, 38, 6, 
-8, -11, -4, 8, 10, -2, -13, -15, -16, -18, 4, 14, 12, 12, 17, 9, 
-3, 6, 10, 1, -8, -3, 11, 12, -11, -38, -46, -21, -11, -16, -11, -4, 
-9, -1, 23, 50, 55, 36, 8, -5, -18, -19, -2, 11, -3, -16, -33, -43, 
-37, -10, 3, 11, 19, 13, -4, 7, 27, 35, 33, 24, 13, 11, 6, -2, 
-11, -23, -23, -18, -21, -20, -7, -2, -5, 10, 27, 34, 35, 27, -5, -13, 
-9, -6, -7, -18, -32, -39, -37, -34, -27, -2, 15, 28, 31, 21, 16, 28, 
31, 25, 19, 19, -4, -34, -53, -50, -34, -23, -13, 0, 14, 10, -7, -2, 
40, 54, 55, 42, 18, -5, -3, 4, 5, -2, -14, -39, -56, -59, -50, -40, 
-32, -25, -16, -1, 24, 48, 42, 25, 21, 21, 16, 11, 2, -9, -21, -21, 
-13, -6, 18, 30, 26, 11, -4, 1, 13, 26, 45, 52, 25, -27, -39, -37, 
-30, -28, -39, -64, -60, -40, -26, -17, 18, 37, 39, 36, 34, 30, 13, 3, 
8, 23, 33, 34, 24, -4, -38, -53, -43, -6, 3, 0, -4, -11, -16, 5, 
30, 41, 25, 1, -33, -46, -46, -41, -30, -13, -3, -15, -18, 2, 22, 28, 
37, 36, 28, 24, 28, 30, 15, 2, 8, 24, 33, 26, 1, -27, -45, -59, 
-64, -28, -1, -2, -17, -20, -8, 1, 18, 34, 30, 11, -9, -4, 11, 23, 
14, -14, -44, -52, -44, -21, -7, -12, -5, 15, 32, 41, 43, 34, 10, -21, 
-34, -16, 13, 13, -8, -17, -19, -22, 7, 44, 61, 46, 24, 12, 7, 9, 
17, 19, 3, -25, -56, -59, -49, -36, -29, -31, -34, -29, -11, 12, 24, 23, 
30, 35, 31, 28, 25, 7, -10, -21, -21, -8, 10, 1, -15, -30, -33, -16, 
21, 33, 24, 4, 1, 23, 46, 39, 31, 26, 12, -24, -22, -15, -22, -34, 
-36, -37, -38, -31, -24, -21, -23, -25, -16, 2, 21, 35, 19, -6, -16, -8, 
11, 33, 31, 3, -20, -25, -18, 24, 56, 69, 58, 33, 11, 9, 22, 23, 
7, -11, -27, -47, -50, -43, -37, -34, -37, -41, -29, -11, -2, -7, 1, 25, 
40, 46, 46, 29, -7, -19, -21, -11, 8, 21, 7, -9, -14, -11, 3, 26, 
21, 11, 4, 1, 1, 3, 3, -1, -4, -2, -11, -21, -19, -14, -14, -21, 
-23, -11, -1, -4, -6, -10, -19, 2, 33, 47, 37, -3, -18, -19, -11, 10, 
31, 29, 15, -2, -15, -11, 0, 15, 10, -1, -3, -3, 4, 12, 15, 3, 
-15, -23, -26, -25, -22, -28, -38, -39, -13, 9, 19, 23, 29, 26, 17, 18, 
26, 24, 11, -10, -18, -25, -24, -18, -11, -4, -12, -21, -22, -13, 19, 30, 
27, 22, 22, 26, 27, 10, -10, -11, 0, -2, -9, 3, 11, 13, 7, -9, 
-10, -12, -20, -29, -32, -39, -39, -32, -23, -14, -11, -8, 0, 5, 15, 31, 
40, 27, 17, 19, 29, 38, 30, 20, 15, 17, 16, 5, -6, -22, -31, -33, 
-26, -23, -23, -15, -13, -18, -25, -30, -16, -3, 0, 0, 16, 29, 27, 27, 
33, 36, 10, -13, -14, -10, -7, 4, 15, -3, -21, -20, -9, 4, -3, -17, 
-23, -19, -9, 7, 13, 16, 14, 6, 3, 0, -5, -6, -8, -15, -20, -11, 
1, 4, 3, 8, 20, 22, 19, 13, 4, -4, -18, -21, -22, -27, -26, 5, 
21, 17, 11, 18, 25, 18, 10, 0, -5, 2, 14, 20, 16, 4, -13, -20, 
-13, -15, -24, -35, -38, -29, -20, -18, -9, 1, 2, -1, -10, -13, 4, 30, 
40, 20, 15, 20, 17, 9, 8, 9, 4, -4, -4, 11, 21, 2, -2, 5, 
11, 8, -2, -2, -2, -4, -12, -19, -21, -31, -39, -36, -38, -35, -13, 1, 
2, 5, 15, 16, -3, -12, -1, 13, 11, 5, 13, 26, 34, 24, 11, 11, 
9, 1, -4, 1, 8, -4, -16, -11, 3, 12, 13, 14, 14, 12, 8, 0, 
-4, -5, -15, -31, -44, -46, -42, -43, -45, -41, -23, 1, 0, -3, 8, 29, 
40, 42, 47, 41, 26, 20, 24, 17, 8, 5, 5, 8, 18, 17, 3, -12, 
-13, -10, -18, -26, -23, -15, -13, -26, -29, -24, -20, -16, -15, -15, -13, -12, 
-10, -7, 1, 7, 8, 8, 10, 15, 26, 29, 27, 21, 14, 10, 2, -8, 
-15, -7, 10, 12, 5, 2, 3, 11, 16, 13, 11, 9, 7, 1, -6, -9, 
-16, -27, -30, -30, -38, -45, -41, -22, -6, -7, -11, 0, 11, 17, 18, 18, 
28, 30, 25, 19, 15, 9, 4, -3, -11, -12, -5, 8, 5, -3, -8, -13, 
-14, 0, 14, 15, 9, 12, 19, 13, 6, 0, -11, -16, -18, -27, -35, -30, 
-18, -16, -14, -2, 8, 12, 16, 26, 23, 12, 6, 7, 4, -6, -4, -8, 
-15, -13, -7, -12, -16, -12, -5, -2, 4, 12, 17, 21, 21, 21, 26, 26, 
19, 7, 1, 0, -12, -32, -40, -28, -19, -22, -23, -22, -17, -12, -7, 15, 
21, 16, 13, 13, 10, 7, 2, -2, -3, -3, 6, 10, 6, 6, 15, 19, 
6, -4, -2, -1, -10, -13, -7, -6, -10, -15, -11, 2, 10, 7, -1, -1, 
4, 11, 6, 1, -2, -8, -11, -12, -11, -9, -10, -11, -5, -1, -6, -6, 
3, 12, 14, 14, 15, 15, 14, 9, 1, -8, -17, -20, -17, -12, -11, -10, 
-3, 4, 4, 0, 2, 7, 6, 0, 1, 8, 11, 6, 7, 12, 14, 15, 
15, 9, -9, -28, -25, -20, -19, -16, -6, 7, 13, 16, 12, 8, 10, 9, 
1, 4, 9, 7, 2, 4, 1, -11, -20, -23, -29, -27, -21, -18, -16, -17, 
-15, -2, 8, 15, 27, 29, 18, 11, 17, 22, 13, -1, 0, 5, 11, 8, 
0, -3, 0, -3, -10, -12, -18, -23, -21, -13, -2, 9, 11, 3, 1, 9, 
14, 9, -4, -2, 6, 10, 1, -16, -9, 2, 5, -1, -11, -20, -14, -3, 
-3, -8, -7, -5, -9, -10, -3, 5, 11, 16, 21, 20, 16, 13, 13, 18, 
14, 4, -3, -6, -5, 2, 3, -5, -4, -6, -21, -29, -30, -26, -15, -10, 
-16, -16, -1, 15, 17, 15, 17, 15, 11, 6, 3, 2, 1, -3, -4, 1, 
2, -3, -5, -2, 1, -2, -10, -7, -4, -7, -15, -10, 3, 9, 9, 5, 
3, 1, 5, 11, 13, 11, 14, 14, 6, -5, -8, 1, 11, 1, -12, -16, 
-12, -9, -3, 7, 3, -15, -21, -3, 1, -2, -1, -1, -6, -18, -16, -6, 
4, 12, 13, 5, 5, 8, 5, 2, 13, 15, 10, 0, -9, -10, -8, -10, 
-14, -13, -6, -2, 2, 5, 4, 4, 8, 11, 7, 4, 3, 1, -1, 0, 
3, 1, -5, -8, -8, -17, -20, -10, 0, 0, -4, 2, 13, 18, 11, 1, 
-3, -3, -10, -17, -10, 1, 10, 12, 10, 5, 2, 10, 12, 4, -1, -6, 
-14, -10, -1, 7, 12, 8, -1, 0, 5, 2, -2, -3, -5, -4, -1, -4, 
-11, -13, -4, -5, -11, -13, -11, -14, -21, -19, -12, -2, 8, 5, 4, 11, 
18, 20, 20, 24, 25, 19, 11, 9, 13, 14, 5, -4, -7, -9, -17, -19, 
-12, -8, -10, -15, -23, -20, -11, 0, 5, 1, -3, -6, -10, -12, -8, 2, 
4, 5, 4, 4, 8, 15, 15, 13, 12, 10, 1, 3, 10, 11, 7, 2, 
-7, -2, 3, -2, -9, -10, -4, -5, -9, -11, -9, -12, -13, -10, -9, -6, 
1, 5, 6, 8, 12, 12, 11, 4, -7, -14, -14, -14, -11, -9, -6, 0, 
3, 5, 7, 5, 3, 5, 8, 9, 7, 5, 2, -1, -3, -3, -1, 1, 
3, 1, -4, -1, 3, 4, 4, 4, 3, 5, 8, 7, 1, -5, -10, -13, 
-16, -19, -17, -9, -5, -5, 1, 4, 4, 4, 5, 6, 7, 9, 7, -1, 
1, 7, 5, -5, -13, -13, -13, -8, 0, 2, 2, 10, 12, 11, 7, 2, 
-1, 2, 2, -2, -2, 3, 3, -1, -6, -9, -11, -18, -23, -17, -8, -6, 
-6, 5, 13, 14, 11, 10, 11, 13, 11, 7, 3, 0, 2, 5, 7, 5, 
-3, -8, -6, -4, -5, -7, -7, -6, -1, 0, 1, 5, 6, 2, 0, -1, 
-1, -1, -2, -8, -7, -2, -2, -8, -10, -10, -10, -8, -3, -1, -4, -3, 
0, 7, 10, 12, 19, 19, 13, 4, 3, 5, -1, -7, -10, -8, -7, -17, 
-18, -10, 1, 7, 5, -1, -1, 1, 5, 5, 2, 1, 0, -2, -2, -1, 
5, 6, 5, 0, -3, -3, -5, -3, -4, -7, -6, -5, -3, 0, 4, 6, 
6, 5, 4, 6, 6, 5, 4, 2, -3, -8, -9, -8, -8, -5, -3, -4, 
-5, -5, -2, -1, 0, 5, 7, 3, -2, -4, -3, 0, 2, 2, 4, 2, 
-2, -2, 2, 2, -2, -6, -3, 3, 1, -5, -5, -4, -2, 0, 2, 3, 
6, 7, 4, -2, -7, -4, 1, 2, 1, 1, 4, 5, 3, 0, 0, -5, 
-8, -8, -5, -3, -3, -1, 0, 1, 1, 2, 2, 2, 0, -4, -5, -5, 
-3, 0, 2, 2, 2, 1, 3, 6, 5, 4, 2, 0, -1, 0, 2, 1, 
-3, -7, -9, -8, -8, -8, -7, 0, 2, 1, 1, 4, 8, 4, 2, 3, 
2, -1, -2, 0, 1, -2, -5, -4, 4, 4, 2, 0, -2, -4, -6, -6, 
-6, -6, -5, -5, -4, -2, 0, 4, 9, 9, 6, 4, 3, 2, 0, -1, 
-1, -1, 0, 0, -1, 0, 0, 0, -1, -3, -5, -5, -4, -1, -1, -3, 
0, 0, -2, -5, -7, -3, 0, 2, 2, -1, -2, 2, 4, 4, 3, 2, 
2, 4, 2, -3, -2, 2, 1, -3, -3, 0, 3, 3, -2, -4, -3, -2, 
-3, -6, -7, -5, -3, -1, 0, 1, 2, 1, -2, -3, -3, -2, -4, -4, 
-2, 0, 2, 5, 7, 7, 7, 5, 4, 2, 1, 2, 2, -1, -5, -5, 
-6, -7, -7, -7, -6, -4, -2, -1, 1, 6, 7, 5, 3, 4, 4, 1, 
1, 1, 0, -2, -6, -7, -5, -4, -6, -8, -5, -4, -3, -1, 1, 3, 
3, 3, 4, 3, 3, 5, 5, 3, 2, 3, 3, 2, 1, -1, -2, -2, 
-3, -6, -6, -5, -3, -4, -4, -4, -3, 0, 2, 2, 1, 0, 0, -1, 
-1, -1, -3, -1, 1, 1, 0, -1, 0, 1, 1, 1, 1, 0, 1, 1, 
1, 0, -1, 0, 1, 0, -1, 0, -1, -2, -3, -3, -2, -1, 0, 0, 
0, -1, -2, -2, 1, 1, 1, 1, 1, 2, 1, 0, -2, -4, -3, -4, 
-5, -4, -2, -2, -2, -1, -1, 0, 3, 3, 2, 1, 2, 2, 1, 0, 
1, 2, 3, 3, 2, 1, 0, -1, -1, 0, 0, -3, -4, -3, -3, -4, 
-4, -2, -1, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 
2, 2, 2, 2, 1, 0, 0, 0, -1, -2, -2, -1, -1, -2, -2, 0, 
0, -1, -1, -1, -1, -1, -1, -1, 0, 0, -1, 0, 0, 0, 0, 0, 
0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 
0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
-1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, };
