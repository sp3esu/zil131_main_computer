const unsigned int sampleRate = 22050;
const unsigned int sampleCount = 3449;
const signed char samples[] = {
0, -1, -4, -5, -6, -7, -3, 2, 4, 3, 4, 7, 16, 12, 9, 13,
10, -11, -14, -10, -20, -32, -33, -17, -8, -10, -9, 2, 18, 31, 25, 16, 
11, 4, -10, -6, 4, 13, 9, -2, 4, 18, 33, 41, 46, 53, 62, 60, 
51, 41, 33, 14, -5, -18, -16, -9, -6, -1, 4, 7, 6, -1, -5, -2, 
-4, -7, -3, 6, 14, 16, 11, 0, -6, -7, -4, 5, 22, 29, 20, 11, 
12, 5, -12, -21, -24, -29, -21, -4, 15, 28, 40, 43, 31, 24, 25, 28, 
27, 19, 27, 39, 45, 45, 41, 35, 24, 13, 5, 3, 12, 24, 31, 30, 
30, 35, 31, 18, 3, -4, -2, 14, 20, 18, 15, 16, 15, 10, 6, -5, 
-13, -8, 2, 10, 11, 17, 26, 32, 33, 26, 15, 6, -4, -11, -7, -10, 
-16, -17, -17, -21, -36, -54, -63, -63, -45, 1, 19, 24, 19, 9, 2, -9, 
-16, -18, -9, 3, -1, -12, -22, -26, -17, -4, 4, 5, 2, 2, 13, 23, 
32, 38, 40, 41, 45, 57, 64, 62, 54, 41, 22, -2, -8, -9, -12, -21, 
-35, -55, -54, -47, -49, -57, -76, -86, -90, -87, -78, -67, -60, -59, -54, -48, 
-43, -37, -30, -18, 1, 15, 25, 33, 32, 31, 34, 30, 22, 8, -5, -12, 
-9, 7, 12, 0, 19, 47, 73, 76, 62, 59, 37, 42, 59, 58, 41, 10, 
11, 35, 48, 40, -3, -15, -2, -2, -18, -37, -59, -66, -52, -28, -11, -41, 
-61, -61, -58, -50, -39, -39, -53, -67, -60, -50, -71, -108, -127, -117, -100, -88, 
-87, -72, -62, -55, -35, -7, 27, 7, -12, 2, 24, 40, 54, 73, 72, 53, 
44, 31, 17, 17, 29, 43, 34, 33, 54, 78, 82, 62, 40, 35, 34, 31, 
19, 9, -3, -12, -13, -12, -11, -10, 2, 11, 10, 16, 27, 17, -6, -21, 
-22, -30, -36, -27, -33, -37, -30, -9, -11, -25, -29, -35, -27, -10, 3, -1, 
-18, -30, -26, -13, -14, -10, -6, -8, -8, 2, 6, -1, -6, -11, -10, 11, 
17, 22, 25, 30, 32, 33, 41, 42, 34, 33, 38, 28, 14, 0, -9, -4, 
11, 20, 25, 19, 4, -3, 12, 12, 2, -10, -23, -29, -18, -17, -26, -29, 
-33, -47, -53, -54, -60, -70, -76, -67, -49, -38, -30, -25, -18, -2, 19, 33, 
36, 33, 41, 58, 63, 56, 51, 44, 18, -1, -16, -24, -21, -14, -12, -8, 
-2, -1, -10, -23, -29, -34, -39, -42, -41, -24, 0, 12, 8, 4, 11, 16, 
13, 7, -4, -11, -9, 4, 17, 13, -4, -7, 0, 1, -10, -31, -46, -54, 
-52, -43, -28, -17, -16, -14, -8, -2, 1, 9, 31, 34, 27, 7, -6, -7, 
-14, -11, -5, 1, -5, -16, -23, -30, -31, -25, -20, -19, -19, -18, -8, 11, 
30, 40, 26, 9, 10, 31, 53, 57, 32, -3, -31, -46, -17, 10, 21, 12, 
-12, -24, -4, 14, 14, 12, 23, 40, 61, 57, 34, 4, -16, -4, 27, 57, 
70, 68, 60, 40, 36, 37, 30, 20, 17, 24, 22, 17, 24, 43, 60, 53, 
44, 50, 66, 73, 57, 43, 40, 43, 45, 35, 14, 12, 16, 23, 30, 48, 
62, 65, 54, 39, 27, 14, 10, 4, -8, -19, -20, -5, 2, 0, -9, -17, 
-26, -37, -43, -43, -44, -49, -47, -30, -17, -9, -2, 1, -3, -8, -15, -24, 
-32, -24, -18, -16, -10, -1, 10, 14, 3, -3, 2, 7, 7, 0, -11, -16, 
-11, -1, 2, -8, -17, -14, -4, 10, 12, 6, 6, 12, 21, 6, -23, -43, 
-48, -42, -33, -7, 7, 2, -8, -3, 12, 22, 15, 6, -3, 1, 19, 23, 
20, 10, 2, -5, -2, 13, 27, 33, 23, 2, -29, -30, -17, -8, -14, -32, 
-25, -5, 14, 30, 36, 22, 11, 15, 25, 18, -3, -1, 18, 29, 24, 16, 
4, -6, -1, 1, 1, 0, -21, -20, -3, 19, 22, 13, 17, 9, -20, -41, 
-37, -20, -10, -14, -25, -34, -48, -53, -52, -44, -27, -10, 16, 34, 13, 5, 
24, 50, 44, 15, -2, 1, 1, -11, -11, -7, -1, 5, 12, 10, -24, -17, 
3, -10, -43, -58, -46, -28, -14, -16, -28, -40, -24, -8, -7, -11, -11, -5, 
-6, -9, -9, -8, -11, -21, -34, -29, 3, 12, -18, -25, -24, -23, -23, -10, 
-3, -1, 2, 11, 13, 5, 10, 7, -4, -11, -9, 0, -4, -12, -16, -28, 
-30, -18, -22, -14, 8, 20, 8, -12, -15, -2, 8, 10, 6, -10, -26, -31, 
-28, -14, -11, -8, -10, -17, -18, -30, -35, -30, -20, -2, 17, 10, -13, -25, 
-18, -1, 5, -5, -17, -21, -19, -17, -14, -24, -31, -18, 8, 25, 18, 7, 
-3, 0, 9, 17, 29, 39, 38, 33, 25, 2, -1, 6, 7, -4, -7, 8, 
8, 10, 18, 17, 13, 20, 23, 6, -22, -31, -8, 0, 1, 4, 11, 13, 
2, -4, -9, -14, -11, 1, 3, 9, 23, 33, 27, 5, 0, 1, 3, 0, 
-10, -17, -17, -4, 12, 17, 9, 2, 5, 6, 9, 6, -15, -25, -36, -39, 
-29, -12, 0, -6, -12, -8, 0, 12, 22, 34, 29, 7, -8, 1, 13, 16, 
14, 14, 25, 31, 23, 13, 8, 9, 18, 14, 10, 7, 4, 11, 20, 14, 
7, 13, 27, 27, 14, 1, -7, -4, 7, 19, 6, -15, -28, -32, -31, -18, 
-8, -5, -10, -18, -21, -20, -23, -19, -8, -6, -36, -56, -50, -31, -13, 4, 
4, -11, -23, -26, -25, -11, 9, 20, 21, 22, 24, 14, 5, 7, 19, 22, 
19, 6, -3, -10, -20, -23, -11, -3, -3, -6, -8, -14, -20, -13, -8, -10, 
-10, -8, -7, -9, -10, -11, -7, 11, 20, 21, 21, 18, 12, 6, 7, 10, 
17, 26, 30, 19, 9, 1, -1, -1, 11, 30, 42, 40, 27, 13, 2, -1, 
-5, -8, 0, 16, 23, 13, -1, -7, -5, -8, -11, -16, -29, -46, -51, -39, 
-39, -33, -12, 14, 26, 23, 22, 26, 28, 22, 18, 15, 17, 24, 34, 41, 
44, 48, 52, 49, 42, 40, 34, 26, 18, 10, 8, 20, 39, 48, 34, 9, 
-10, -10, 3, 16, 24, 30, 23, 11, -3, -15, -21, -23, -15, -9, -4, 3, 
10, 19, 21, 15, 6, 2, 11, 31, 34, 31, 34, 42, 49, 46, 35, 30, 
32, 38, 43, 42, 42, 41, 36, 29, 18, -6, -25, -22, -3, 7, -2, 13, 
26, 26, 19, 5, -8, -33, -45, -45, -46, -25, -29, -27, -12, -8, -10, -25, 
-30, -24, -37, -48, -28, -20, -23, -27, -25, -15, -21, -27, -28, -36, -42, -40, 
-37, -35, -41, -35, -20, -11, -28, -22, -1, 5, 5, -21, -27, -33, -41, -37, 
-38, -46, -68, -64, -31, -15, -31, -44, -12, 4, -7, -12, -23, -41, -44, -38, 
-43, -57, -52, -33, -17, -13, -20, -21, -8, -9, -17, -29, -34, -31, -30, -24, 
-23, -30, -35, -18, -7, -14, -16, -4, -2, -2, 4, 7, -8, -23, 4, 16, 
11, 1, 6, 9, -11, -15, -26, -31, -27, -22, -15, 0, 13, 20, 26, -6, 
-18, -13, -5, -1, -4, 10, 19, 33, 38, 32, 25, 17, 11, 5, 8, 17, 
23, 35, 42, 44, 43, 37, 19, 11, 5, -7, -5, 3, 2, 4, 13, 24, 
27, 16, 7, 6, 7, 8, 17, 33, 26, 19, 30, 42, 32, 18, 9, 1, 
-5, -9, -15, -8, 5, 20, 22, 4, -13, -7, 2, 5, 0, -9, -6, -5, 
-16, -21, -5, 31, 38, 35, 36, 39, 38, 26, 17, 16, 11, 4, 10, 15, 
17, 16, 15, 5, -13, -8, -3, -4, -17, -27, -25, -23, -18, -21, -36, -55, 
-55, -52, -53, -54, -46, -38, -43, -46, -37, -22, -19, -37, -46, -36, -20, -14, 
-7, 7, 22, 30, 34, 25, -15, -29, -27, -13, 0, 4, 9, 20, 37, 47, 
47, 39, 37, 34, 30, 33, 39, 44, 43, 44, 56, 68, 69, 57, 44, 41, 
51, 60, 56, 48, 40, 26, 5, -5, 16, 27, 18, -3, -24, -42, -42, -24, 
1, 11, -6, -37, -42, -48, -60, -62, -48, -35, -28, -15, -6, -12, -43, -55, 
-48, -37, -26, -19, -18, -13, 3, 20, 21, 10, 5, 11, 15, 16, 13, -2, 
-9, -16, -26, -31, -21, 14, 21, 15, 11, 12, 16, 13, 3, -13, -17, -4, 
5, 3, 0, 1, 12, 28, 30, 10, -10, -12, -4, -3, -14, -16, -15, -12, 
-1, 3, -3, -7, -13, -17, -12, -2, -6, -10, -5, 2, -6, -25, -17, 3, 
19, 30, 43, 47, 50, 47, 37, 31, 27, 19, 3, -10, -11, -2, 19, 29, 
36, 33, 31, 39, 49, 53, 49, 42, 35, 35, 33, 28, 29, 36, 46, 47, 
52, 55, 48, 31, 8, 5, 4, -3, -9, -3, 4, 0, -2, 0, 5, 7, 
3, -5, -7, -7, -8, -5, -3, -5, -11, -13, -5, 10, 12, 8, 15, 31, 
31, 6, -7, 9, 26, 29, 28, 38, 52, 52, 46, 54, 67, 59, 44, 35, 
40, 25, 7, 5, 20, 32, 21, 10, 20, 21, 0, -8, 9, 10, -10, -29, 
-31, -32, -44, -44, -45, -40, -28, -19, -21, -40, -52, -53, -46, -43, -62, -66, 
-57, -39, -37, -52, -51, -49, -44, -38, -28, -29, -25, -2, 8, -12, -38, -38, 
-31, -31, -36, -43, -32, -22, -19, -31, -48, -59, -58, -48, -46, -54, -63, -61, 
-39, -30, -32, -36, -36, -50, -55, -56, -59, -61, -53, -16, -5, -8, -15, -20, 
-34, -58, -50, -36, -34, -35, -19, -18, -31, -47, -62, -73, -51, -35, -32, -27, 
-13, 15, 9, -4, -5, 2, 5, -5, 1, 15, 26, 27, 29, 54, 64, 63, 
48, 36, 33, 26, 25, 33, 42, 42, 41, 39, 32, 32, 47, 68, 69, 52, 
43, 54, 70, 80, 69, 43, 26, 26, 42, 62, 60, 45, 21, 4, -4, -2, 
-7, -18, -22, -13, 2, 13, 28, 31, 20, 13, 2, -13, -24, -19, -11, -12, 
-4, 18, 34, 23, -1, -16, -5, 12, 26, 38, 42, 25, 7, 3, 7, -1, 
-14, -23, -27, -26, -18, -14, -26, -37, -41, -42, -37, -28, -17, -11, -13, -17, 
-17, -16, -10, -8, -5, -2, 9, 8, -15, -33, -40, -32, -20, -22, -34, -42, 
-36, -22, -18, -18, -19, -21, -22, -18, -13, -19, -34, -49, -48, -37, -19, -20, 
-33, -45, -53, -51, -35, -20, -10, -9, -6, -2, -8, -9, -3, 6, 7, 10, 
27, 46, 56, 54, 49, 37, 26, 19, 6, -8, -18, -8, 2, 4, 0, -1, 
-3, -6, -4, 9, 19, -3, -31, -44, -36, -25, -21, -13, -2, 9, 12, 14, 
15, 5, -4, -5, 4, 12, 6, 3, 1, 1, 4, 12, 40, 56, 61, 65, 
74, 79, 70, 58, 46, 40, 37, 50, 67, 75, 67, 42, 19, 12, 12, 11, 
4, -11, -26, -16, 9, 19, 9, -5, -6, 4, 7, -2, -21, -35, -27, -12, 
8, 24, 25, 22, 32, 31, 14, -15, -41, -58, -51, -34, -19, -11, -3, 24, 
41, 43, 35, 21, 13, 35, 62, 81, 74, 55, 37, 33, 32, 28, 24, 25, 
25, 27, 28, 21, 3, -14, -17, -17, -20, -20, -23, -33, -42, -56, -64, -61, 
-49, -38, -38, -46, -64, -83, -103, -116, -105, -92, -82, -74, -52, -36, -20, -2, 
9, 12, 17, 23, 28, 27, 12, -11, -9, -5, -9, -27, -47, -53, -33, -9, 
-1, -3, 3, 20, 18, 15, 15, 18, 34, 46, 51, 51, 52, 59, 74, 72, 
61, 54, 45, 32, 17, 25, 43, 55, 65, 72, 58, 36, 5, -19, -41, -53, 
-39, -37, -23, -19, -32, -18, -3, 10, 8, -9, 0, 0, -20, -17, 2, 11, 
4, 4, 22, 35, 31, 34, 44, 39, 32, 26, 20, -14, -33, -27, -6, -2, 
-12, -17, -18, -3, -11, -28, -30, -36, -18, 0, 8, 0, -28, 2, 43, 53, 
47, 46, 44, 35, 24, 11, 1, 4, 19, 15, 2, 1, 3, -20, -34, -51, 
-62, -57, -45, -33, -41, -55, -50, -37, -34, -50, -61, -60, -54, -46, -41, -36, 
-26, -21, -20, -11, -7, -29, -46, -39, -29, -24, -12, 8, 19, 4, -8, -25, 
-29, -23, -16, -8, -7, -19, -35, -41, -47, -50, -30, -19, -10, -6, 0, 5, 
14, 12, -15, -37, -43, -38, -31, -31, -42, -56, -46, -28, -37, -48, -51, -50, 
-50, -42, -32, -22, -19, -29, -42, -64, -74, -63, -41, -27, -15, -7, -8, -16, 
-15, -13, -7, 14, 22, 9, -8, -9, 18, 33, 37, 27, 25, 26, 16, 17, 
24, 32, 34, 32, 34, 34, 34, 33, 29, 13, 4, 14, 33, 43, 58, 70, 
75, 64, 39, 6, -30, -22, -11, -6, -6, -4, 2, 0, 1, 4, 9, 7, 
-5, 2, 26, 48, 60, 61, 57, 43, 28, 25, 34, 19, -5, -10, 8, 29, 
41, 44, 40, 21, -6, -20, -2, 16, 31, 43, 47, 45, 24, 8, 5, 11, 
24, 42, 40, 35, 32, 23, 22, 43, 46, 42, 35, 23, 14, 10, 18, 28, 
31, 19, -4, 6, 13, 7, 1, 0, 1, -1, -1, -4, -9, 2, 12, 15, 
7, -5, -19, -41, -46, -49, -43, -24, -5, 2, -3, -1, 10, 14, -4, -15, 
-12, 3, 19, 24, 5, -7, -7, -4, -1, -1, 0, -1, -2, 1, 0, -10, 
-13, -8, -8, -10, -10, -17, -17, -6, 7, 4, -9, -19, -24, -27, -22, -16, 
-14, -15, -15, -14, -14, -11, -6, -16, -27, -39, -45, -29, 16, 34, 28, 12, 
0, -13, -13, -11, -10, -11, -7, 9, 23, 28, 18, 4, -11, -25, -32, -43, 
-50, -46, -13, 13, 22, 15, 3, -9, -24, -27, -25, -24, -31, -36, -8, 20, 
31, 19, -9, -31, -14, 13, 26, 21, 17, 26, 37, 42, 40, 34, 26, 4, 
-4, 2, 15, 28, 34, 36, 36, 29, 24, 19, 10, 7, 11, 16, 18, 16, 
12, 23, 42, 55, 53, 34, 28, 28, 31, 34, 41, 62, 64, 48, 36, 42, 
54, 45, 28, 22, 23, 16, 13, 20, 25, 24, 15, 2, -3, 4, -1, -19, 
-25, 4, 22, 21, 7, 2, 2, -12, -4, -4, -5, -6, -22, -22, -14, -17, 
-33, -36, -3, 8, -12, -24, -6, 13, 10, -4, -10, -10, -19, -34, -52, -49, 
-31, -10, 1, -22, -37, -35, -25, -23, -38, -35, -23, -22, -35, -48, -41, -23, 
-6, 6, 4, -7, -15, 0, 11, 8, 6, 23, 24, -17, -49, -54, -27, 12, 
33, 12, -16, -27, 0, 59, 49, 14, -11, -6, 9, 19, 16, 3, -7, -15, 
-18, -10, -11, -22, -21, -2, 2, -11, -10, 3, 3, -8, -9, 3, 10, 19, 
31, 31, -7, -35, -46, -52, -52, -33, -21, -5, 8, 14, 6, -16, -16, -9, 
-5, -15, -26, -29, -37, -45, -50, -52, -46, -22, -11, -20, -29, -27, 1, 30, 
54, 52, 30, 14, 8, 8, 17, 18, -2, -20, -10, 5, 12, 9, 8, 1, 
-4, -6, -1, 12, 21, 15, 6, 0, 0, 6, 13, 7, -2, -8, -7, -10, 
-14, -19, -31, -34, -27, -11, 0, 16, 27, 19, 5, -8, -17, -28, -35, -39, 
-48, -59, -61, -55, -35, -18, -9, 1, 17, 26, 23, 17, 4, 7, 22, 26, 
19, 8, -9, -9, 5, 17, 15, 11, 10, 10, 11, 4, -4, -1, 14, 21, 
9, -18, -31, -20, -6, 8, 11, 7, -12, -24, -31, -36, -37, -38, -29, -22, 
-20, -20, -20, -17, -15, -26, -29, -25, -19, -8, -8, -13, -20, -23, -15, -14, 
-17, -4, 24, 40, 29, -3, 1, 18, 22, 9, -2, 4, 13, 22, 35, 43, 
39, 38, 34, 26, 25, 31, 30, 32, 35, 32, 28, 21, 26, 30, 25, 16, 
3, -12, -12, -1, 18, 31, 35, 33, 29, 28, 35, 46, 56, 58, 54, 45, 
37, 34, 41, 43, 37, 29, 24, 15, 11, 7, -2, -7, -6, -3, 0, 4, 
6, 8, 16, 24, 22, 26, 36, 36, 33, 38, 41, 40, 34, 28, 28, 33, 
37, 34, 28, 26, 29, 20, 5, -8, -16, -18, -10, -3, -2, -2, 6, 11, 
1, -8, -15, -19, -15, -11, -18, -19, -13, -7, -11, -28, -38, -31, -18, -10, 
-11, -6, 0, -1, -13, -27, -39, -42, -45, -49, -47, -36, -32, -25, -11, -6, 
-18, -40, -37, -34, -37, -45, -53, -49, -41, -34, -30, -33, -42, -40, -31, -25, 
-26, -28, -19, -17, -15, -14, -16, -17, -29, -35, -37, -32, -26, -22, -16, -11, 
-4, -3, -10, -9, -1, 8, 17, 19, 10, 8, 20, 27, 26, 25, 19, 18, 
20, 19, 17, 19, 15, 9, 4, 8, 13, 4, 1, 8, 16, 13, 13, 19, 
11, 2, 0, 11, 21, 22, 29, 36, 44, 41, 48, 42, 23, 22, 16, 18, 
20, 2, 3, 8, 7, -7, -39, -30, -29, -52, -53, -26, -12, 1, 8, 13, 
8, 3, 7, -1, -7, -2, -2, 5, -5, -13, -1, 23, 22, 5, 11, 19, 
20, 17, 10, 8, 2, 10, 25, 29, -8, -28, -15, -5, -25, -44, -13, 4, 
-13, -25, -21, -19, -16, -11, -12, -26, -37, -22, -12, -10, -15, -12, 6, 2, 
-13, -26, -33, -38, -45, -44, -42, -39, -30, -18, 8, 9, 3, 8, 3, -5, 
3, 5, -11, -35, -28, -3, -13, -26, -20, 1, 3, 13, 29, 34, 25, 7, 
5, 3, 8, 19, 27, 17, -12, -4, 6, 7, -2, -8, -8, -2, 2, -11, 
-26, -40, -42, -40, -31, -21, -21, -11, -4, -6, -12, -13, -8, -21, -37, -43, 
-34, -14, 8, 13, 11, 8, 7, 13, 22, 26, 28, 20, 7, 4, 5, 5, 
17, 30, 25, -7, -20, -26, -29, -23, -12, -2, 3, 3, 5, 3, 4, 9, 
12, 16, 17, 16, 4, -5, -14, -20, -14, -3, 2, -1, -11, -23, -28, -23, 
-17, -9, -7, -10, -3, 40, 67, 75, 65, 48, 34, 38, 51, 48, 32, 19, 
2, -3, -4, -1, 1, -13, -47, -43, -25, -16, -17, -19, -21, -32, -50, -61, 
-66, -77, -83, -85, -77, -59, -44, -27, -26, -31, -29, -14, -2, -7, -9, -5, 
-1, 5, 11, 6, 3, 12, 29, 49, 58, 51, 44, 46, 62, 84, 84, 69, 
70, 82, 94, 103, 101, 91, 74, 56, 38, 31, 35, 29, 9, -12, -25, -31, 
-30, -31, -31, -24, -11, -20, -48, -80, -95, -84, -41, -18, -6, -5, -6, -24, 
-50, -65, -62, -49, -42, -38, -33, -21, -1, 18, 29, 13, -1, 8, 31, 50, 
59, 61, 61, 54, 55, 66, 78, 76, 63, 50, 48, 51, 43, 31, 16, 11, 
16, 17, 14, 13, 11, 11, 21, 44, 50, 40, 20, 4, 0, 0, -10, -19, 
-20, -11, 9, 20, 18, 6, -5, -10, -13, -19, -34, -43, -38, -22, -1, -5, 
-18, -26, -21, -2, 12, 24, 24, 14, 9, 20, 43, 62, 67, 59, 50, 60, 
68, 74, 79, 70, 49, 39, 35, 32, 24, 14, 17, 24, 31, 36, 36, 36, 
27, 13, -1, -13, -20, -25, -30, -27, -14, -4, -7, -6, 9, 18, 16, 4, 
-9, -15, -16, -17, -16, -13, -14, -19, -29, -46, -62, -67, -53, -52, -65, -71, 
-65, -53, -32, -36, -49, -56, -47, -32, -24, -14, 10, 40, 48, 29, 23, 27, 
20, 15, 12, 18, 19, -3, -19, -15, -9, -29, -44, -26, -3, -7, -25, -7, 
8, 5, -10, -34, -32, -38, -39, -33, -41, -51, -45, -37, -46, -62, -62, -35, 
-24, -32, -33, -24, -8, -13, -26, -21, -16, -20, -19, -18, -26, -38, -24, -7, 
-29, -46, -27, 1, -5, -43, -57, -46, -16, 8, 1, -50, -67, -57, -32, -16, 
-16, -37, -38, -19, -1, 8, 7, 7, 10, 21, 30, 28, 4, -7, -6, -11, 
-15, -7, 20, 18, 1, -3, -5, -17, -20, -16, -15, -25, -31, 5, 32, 50, 
57, 40, 12, -12, -10, 2, 17, 33, 51, 51, 34, 31, 58, 88, 99, 93, 
85, 64, 50, 57, 58, 54, 51, 50, 42, 33, 36, 47, 63, 65, 55, 52, 
61, 53, 35, 19, 12, 15, 14, 15, 14, 18, 33, 39, 38, 30, 27, 23, 
17, 15, 9, 6, 2, -14, -20, -21, -29, -38, -34, -23, -22, -30, -38, -38, 
-43, -50, -40, -21, -9, -16, -61, -83, -90, -86, -74, -68, -68, -66, -70, -77, 
-71, -51, -35, -41, -43, -44, -42, -31, -41, -57, -59, -46, -35, -30, -27, -23, 
-19, -16, -17, -26, -37, -46, -49, -49, -56, -63, -68, -66, -55, -46, -25, -14, 
-13, -16, -24, -28, -13, 4, 10, 8, 0, };
