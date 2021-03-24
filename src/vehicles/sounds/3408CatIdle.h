const unsigned int sampleRate = 22050;
const unsigned int sampleCount = 4963;
const signed char samples[] = {
0, -7, -9, -11, -14, -13, -8, -8, -7, -5, -6, -7, -2, 0, 1, 0,
-1, -4, -7, -7, -6, -4, 0, 0, -3, -7, -6, 0, 9, 27, 39, 44, 
46, 45, 44, 42, 37, 28, 18, 10, -2, -8, -11, -11, -12, -11, -12, -12, 
-11, -12, -13, -14, -12, -9, -7, -6, -9, -15, -18, -15, -9, -5, 0, 0, 
-5, -6, -5, -5, -7, -8, -9, -11, -10, -5, 6, 11, 14, 16, 20, 15, 
6, 3, 4, 3, 0, -2, -1, -2, -5, -4, -2, 0, 2, 3, 6, 5, 
-1, -11, -17, -21, -21, -19, -13, -5, 4, 13, 18, 18, 13, 5, -4, -11, 
-13, -13, -15, -15, -15, -17, -17, -9, -1, 5, 10, 14, 17, 15, 14, 15, 
15, 14, 10, 0, -5, -9, -10, -7, -3, -1, -3, -2, 0, -5, -16, -19, 
-20, -23, -20, -13, -8, 2, 10, 14, 18, 24, 26, 21, 14, 10, 7, 3, 
2, 2, 4, 3, -2, -4, -5, -10, -12, -8, -3, 3, 8, 8, 6, 3, 
3, 8, 12, 14, 18, 20, 16, 8, -2, -8, -11, -15, -16, -16, -18, -20, 
-18, -15, -12, -10, -10, -9, -7, -5, 0, 6, 11, 13, 18, 21, 20, 16, 
13, 11, 10, 9, 10, 13, 13, 7, 2, -5, -15, -28, -40, -52, -49, -41, 
-31, -19, -9, 2, 12, 19, 25, 31, 31, 26, 19, 13, 9, 5, 2, 0, 
4, 9, 11, 12, 4, 3, 3, 0, -9, -17, -20, -22, -21, -18, -15, -11, 
-7, -5, -3, 1, 5, 6, 8, 6, 4, 4, 1, -1, -3, -4, -4, -4, 
-2, -3, -9, -14, -16, -17, -15, -10, -4, 2, 6, 7, 7, 6, 3, -4, 
-11, -15, -14, -11, -7, -4, 5, 11, 14, 14, 13, 9, 1, 2, 8, 12, 
15, 14, 7, 3, -1, 0, 1, 4, 11, 19, 22, 21, 19, 9, 5, 1, 
-3, -5, -5, 0, 6, 10, 13, 16, 11, 2, -5, -13, -22, -29, -35, -29, 
-21, -18, -16, -13, -13, -14, -17, -18, -19, -19, -8, 3, 9, 11, 7, 11, 
10, 8, 13, 17, 19, 17, 15, 14, 13, 12, 8, 10, 14, 14, 12, 13, 
18, 17, 16, 16, 15, 16, 20, 20, 21, 23, 24, 18, 11, 9, 4, -3, 
-9, -12, -10, -15, -19, -19, -20, -7, -3, -3, -4, -10, -21, -28, -28, -30, 
-38, -38, -22, -12, -3, 3, 8, 10, -4, -15, -25, -27, -21, -18, -16, -12, 
-1, 17, 31, 42, 44, 53, 64, 70, 73, 64, 43, 19, -5, -27, -46, -45, 
-38, -36, -43, -57, -66, -52, -27, -2, 14, 3, -23, -44, -48, -35, -14, 14, 
25, 23, 20, 8, -14, -33, -41, -37, -21, -2, 27, 33, 26, 3, -20, -36, 
-48, -31, -17, -15, -15, -21, -26, -21, -16, -8, 0, 9, 10, 4, -3, 6, 
24, 45, 60, 72, 84, 97, 104, 100, 91, 85, 79, 71, 53, 26, -15, -57, 
-89, -99, -91, -87, -81, -69, -59, -52, -41, -29, -18, -5, 6, 7, 5, 8, 
13, 7, -3, -2, 3, 13, 23, 34, 46, 61, 58, 42, 19, -11, -53, -75, 
-89, -90, -79, -55, -10, 10, 28, 43, 50, 50, 26, -3, -27, -45, -57, -55, 
-47, -33, -11, 13, 22, 8, 3, 4, 18, 44, 69, 91, 92, 80, 60, 38, 
9, 0, -9, -11, -9, -9, -13, -17, -18, -21, -28, -33, -32, -33, -35, -32, 
-30, -21, -7, 5, 4, 0, 4, 6, 12, 23, 26, 26, 39, 43, 44, 45, 
39, 26, -7, -30, -48, -62, -67, -63, -47, -38, -31, -23, -13, 3, 9, 16, 
21, 24, 26, 30, 29, 11, -17, -41, -61, -85, -91, -81, -58, -32, -8, 0, 
4, 9, 12, 20, 44, 57, 61, 63, 66, 59, 29, 8, -13, -32, -44, -34, 
-20, -9, -3, 4, 8, 10, 15, 17, 10, 4, 6, 4, -1, -4, -8, -12, 
-12, -15, -26, -38, -47, -49, -36, -30, -23, -12, 0, 6, 17, 29, 34, 41, 
54, 64, 67, 70, 73, 72, 66, 57, 52, 47, 36, 15, -5, -20, -26, -30, 
-39, -47, -41, -31, -20, -15, -11, 0, 29, 50, 67, 74, 71, 63, 57, 60, 
63, 60, 45, 18, -4, -20, -25, -30, -38, -48, -54, -56, -59, -60, -57, -57, 
-57, -57, -54, -47, -48, -54, -59, -59, -51, -42, -36, -36, -36, -30, -27, -24, 
-25, -29, -28, -22, -10, 20, 38, 52, 61, 61, 54, 44, 38, 28, 18, 16, 
23, 33, 46, 53, 53, 48, 36, 25, 16, 11, 5, -5, -13, -12, -11, -8, 
-5, -11, -9, -8, -10, -12, -10, -10, -15, -19, -21, -27, -32, -27, -24, -21, 
-16, -14, -12, -7, 3, 11, 15, 18, 13, 8, 5, 0, -4, -3, 9, 18, 
26, 33, 34, 27, 22, 18, 20, 31, 42, 49, 44, 33, 19, 7, -1, -11, 
-13, -10, -3, 6, 20, 21, 18, 14, 7, -1, -7, -9, -12, -15, -19, -22, 
-20, -22, -26, -25, -19, -7, 4, 15, 17, 13, 10, 5, 1, -5, -11, -21, 
-34, -56, -65, -69, -75, -77, -64, -46, -29, -17, -12, -11, -12, -6, 6, 19, 
32, 42, 43, 36, 30, 31, 38, 52, 54, 52, 48, 39, 25, -4, -24, -40, 
-51, -58, -61, -56, -51, -41, -33, -32, -19, -21, -25, -20, -13, -6, 12, 20, 
25, 27, 26, 23, 22, 22, 19, 18, 15, -1, -8, -11, -7, 0, 3, 6, 
4, -4, -13, -24, -34, -32, -25, -10, 7, 20, 39, 43, 38, 27, 14, 4, 
-7, -16, -29, -42, -56, -72, -70, -59, -38, -12, 7, 25, 37, 47, 52, 53, 
48, 41, 40, 36, 27, 16, 7, 6, 1, -2, 3, 12, 24, 23, 15, 3, 
-7, -19, -33, -34, -33, -31, -30, -28, -23, -16, -8, -2, 2, 7, 10, 10, 
6, 0, 0, 7, 9, 10, 15, 23, 25, 19, 10, 6, 9, 16, 28, 34, 
30, 22, 15, 12, 20, 30, 33, 30, 28, 21, 11, 1, -10, -17, -15, -3, 
-4, -14, -25, -37, -48, -61, -67, -64, -49, -30, 0, 8, 6, 0, -9, -16, 
-13, 0, 14, 30, 44, 51, 40, 16, -2, -1, 11, 32, 34, 22, 2, -14, 
-25, -26, -13, 9, 23, 18, -18, -52, -81, -100, -108, -105, -81, -60, -45, -41, 
-48, -56, -51, -34, -9, 19, 40, 63, 67, 51, 29, 13, 0, -1, 0, -4, 
-4, -9, -18, -22, -15, 5, 30, 44, 51, 52, 50, 49, 44, 40, 41, 43, 
52, 62, 74, 82, 65, 47, 29, 16, 9, 5, 6, -1, -17, -35, -52, -65, 
-61, -56, -48, -40, -34, -17, -3, 5, 4, -1, 3, 8, 9, 11, 16, 19, 
20, 15, 7, -7, -18, -19, -9, 2, 7, 0, -11, -22, -26, -21, -13, -5, 
3, 4, 0, -2, -2, 1, 9, 21, 24, 23, 21, 15, 8, 3, 0, -1, 
5, 17, 24, 26, 27, 28, 26, 19, 17, 18, 20, 20, 9, -26, -56, -76, 
-81, -75, -60, -40, -27, -14, -1, 13, 29, 38, 48, 53, 53, 50, 41, 25, 
4, -12, -21, -27, -27, -20, -17, -27, -33, -29, -23, -16, -17, -27, -38, -52, 
-56, -57, -52, -38, -21, 9, 23, 29, 26, 19, 23, 36, 48, 55, 58, 53, 
31, 14, -2, -12, -14, -12, -10, -16, -20, -19, -16, -17, -13, -1, 11, 23, 
28, 16, 4, -5, -8, -11, -9, -4, -5, -7, -15, -26, -38, -31, -13, 8, 
24, 35, 33, 22, 10, -2, -10, -12, 1, 16, 21, 9, -8, -26, -30, -26, 
-15, -5, -1, -7, -13, -13, -10, -11, -9, -1, 3, 7, 12, 16, 8, 3, 
3, 5, 9, 11, 3, -8, -18, -28, -33, -31, -19, -7, 8, 24, 30, 27, 
20, 10, 2, -1, 2, 9, 7, 2, -4, -5, -4, 8, 19, 27, 29, 24, 
18, 17, 16, 14, 10, 7, 7, 8, 5, -2, -13, -30, -34, -38, -36, -29, 
-19, -4, 3, 4, -2, -13, -26, -45, -52, -52, -47, -40, -30, -24, -16, -7, 
2, 10, 10, 10, 11, 11, 14, 18, 16, 13, 7, 2, 1, 7, 9, 8, 
4, 0, -5, -4, -5, -9, -7, -3, -1, 6, 10, 12, 16, 22, 28, 22, 
16, 11, 5, 1, 5, 11, 17, 17, 9, -2, -21, -27, -31, -35, -42, -54, 
-56, -54, -43, -24, -10, -4, -7, -14, -18, -16, -10, 5, 19, 34, 48, 57, 
67, 63, 54, 46, 38, 31, 16, 6, 6, 8, 3, -7, -13, -6, 3, 5, 
8, 19, 19, 10, -5, -15, -14, 1, 9, 17, 28, 34, 33, 22, 11, 2, 
-5, -11, -14, -3, 7, 6, -1, -3, 0, -1, -5, -10, -14, -19, -28, -31, 
-33, -30, -24, -21, -19, -16, -8, 0, 7, 24, 35, 45, 50, 48, 34, 17, 
-2, -16, -23, -21, -17, -9, 3, 11, 14, 11, 1, -3, -6, -9, -10, -9, 
-9, -10, -18, -22, -17, -11, -6, -1, 4, 9, 10, 12, 15, 15, 12, 14, 
20, 23, 20, 12, 8, -2, -25, -29, -29, -33, -29, -22, -21, -15, -13, -13, 
-11, -17, -22, -20, -15, -16, -17, -9, -6, -6, -12, -19, -24, -21, -15, -7, 
13, 28, 35, 30, 17, 3, -14, -24, -32, -23, -12, -8, 1, 2, -2, -5, 
-10, -6, -2, 16, 24, 26, 28, 22, 14, 2, -4, -11, -13, -11, -11, -15, 
-16, -13, -10, -5, 2, -2, 4, 9, 14, 25, 28, 31, 31, 30, 26, 22, 
35, 42, 48, 48, 38, 3, -22, -34, -40, -34, -22, -5, 3, -7, -29, -47, 
-54, -56, -47, -37, -25, -10, 0, 14, 33, 44, 53, 54, 41, 29, 19, 18, 
22, 32, 33, 32, 30, 27, 24, 12, -1, -5, -2, 3, 6, 4, 7, 17, 
25, 23, 5, -11, -24, -42, -57, -56, -39, -15, 6, 16, 17, 18, 18, 9, 
3, 1, -3, -13, -19, -21, -25, -29, -28, -29, -32, -33, -28, -19, -12, -12, 
-12, -4, -1, 6, 1, -18, -39, -58, -62, -56, -24, 5, 27, 41, 43, 36, 
17, 2, -13, -24, -27, -17, -1, 14, 15, 3, -8, -18, -21, -19, -15, -10, 
2, 16, 26, 40, 48, 51, 43, 33, 22, 10, -2, -11, -20, -34, -40, -38, 
-41, -39, -35, -29, -17, -12, -3, 16, 16, 10, 6, 0, -6, -11, -15, -10, 
-7, -2, 4, 2, 6, 12, 21, 29, 36, 32, 21, 15, 11, 9, 17, 27, 
33, 36, 39, 38, 32, 30, 23, 16, 9, 7, 13, 16, 17, 12, 1, -17, 
-29, -38, -46, -48, -45, -40, -36, -39, -43, -48, -61, -64, -59, -45, -31, -15, 
2, 11, 23, 38, 56, 67, 63, 56, 56, 56, 56, 58, 54, 50, 42, 33, 
26, 17, 12, 3, -7, -19, -40, -46, -49, -56, -62, -69, -71, -64, -55, -45, 
-32, -18, -4, -2, -8, -11, -7, 7, 18, 27, 29, 28, 30, 34, 34, 33, 
29, 23, 15, -6, -22, -34, -43, -46, -42, -43, -41, -39, -37, -30, -27, -27, 
-23, -20, -20, -16, -4, 1, 2, 3, 7, 22, 33, 44, 55, 64, 71, 66, 
57, 44, 34, 28, 16, 3, -10, -23, -33, -41, -46, -45, -41, -38, -40, -43, 
-43, -39, -34, -25, -19, -11, 2, 5, 5, 4, 0, 1, 10, 25, 41, 58, 
78, 110, 118, 107, 91, 74, 55, 25, 8, -3, -9, -15, -35, -53, -67, -73, 
-70, -66, -55, -47, -41, -36, -29, -11, 7, 28, 48, 62, 70, 63, 51, 43, 
38, 35, 33, 33, 28, 21, 15, 4, -8, -14, -21, -31, -44, -56, -67, -73, 
-79, -81, -76, -66, -48, -35, -20, -1, 20, 44, 55, 59, 57, 56, 51, 42, 
30, 16, 4, -8, -19, -31, -34, -34, -35, -38, -38, -32, -28, -24, -23, -25, 
-24, -24, -28, -27, -23, -16, 4, 19, 35, 47, 58, 72, 77, 79, 78, 76, 
67, 50, 36, 20, 8, -1, -10, -20, -26, -28, -31, -34, -28, -24, -20, -17, 
-17, -18, -13, -9, -4, 5, 13, 17, 22, 27, 29, 26, 21, 11, 1, -6, 
-6, -5, -3, 7, 12, 15, 13, 5, -4, -22, -29, -34, -37, -38, -34, -28, 
-18, -7, 2, 11, 24, 35, 45, 51, 56, 52, 39, 31, 18, 6, -6, -24, 
-29, -33, -38, -39, -39, -43, -39, -32, -22, -10, 0, 2, 0, -6, -12, -16, 
-16, -14, -9, -2, 3, 9, 16, 15, 20, 25, 25, 17, -1, -20, -33, -37, 
-30, -16, -8, -3, -2, -3, -8, -11, -5, 4, 18, 28, 28, 24, 17, 12, 
7, 2, 0, -1, -4, -11, -20, -24, -25, -30, -39, -45, -47, -40, -30, -18, 
-6, 4, 8, 11, 10, 7, 2, -1, -2, -5, -7, -10, -14, -17, -14, -9, 
0, 10, 19, 26, 32, 32, 30, 26, 18, 9, -7, -14, -17, -17, -9, 0, 
-3, -8, -19, -29, -35, -34, -25, -13, -4, 4, 10, 15, 15, 13, 9, 5, 
3, 6, 10, 12, 8, 2, -8, -7, -4, 4, 18, 34, 58, 63, 58, 43, 
21, -9, -22, -29, -28, -20, -11, -1, -1, -7, -11, -5, 4, 12, 8, -6, 
-23, -35, -38, -28, -17, -7, -1, 3, 4, 6, 3, -4, -7, -3, 2, 2, 
-1, -4, -3, 7, 13, 17, 15, 9, 5, 9, 10, 14, 22, 27, 30, 28, 
22, 16, 13, 8, -6, -11, -17, -20, -16, -8, -4, -6, -5, 1, 5, -2, 
-3, 4, 11, 18, 21, 14, 9, -5, -23, -33, -29, -6, 5, 4, -11, -30, 
-59, -72, -71, -54, -20, 19, 54, 58, 50, 36, 14, -20, -54, -51, -31, -3, 
25, 46, 45, 36, 24, 15, 16, 31, 31, 23, 8, -8, -20, -37, -43, -42, 
-35, -28, -30, -28, -23, -17, -13, -18, -26, -29, -30, -26, -15, 2, 26, 44, 
57, 56, 50, 37, 31, 34, 44, 56, 65, 56, 27, -4, -27, -37, -34, -12, 
5, 15, 13, 0, -28, -39, -40, -40, -38, -25, -7, -6, -8, -8, -4, 3, 
23, 37, 45, 49, 44, 15, -3, -8, -8, -6, 0, 4, -2, -15, -26, -28, 
-22, -17, -9, -1, 3, 5, 1, 3, 6, 7, 12, 18, 12, 4, -1, -4, 
-4, 1, 1, -3, -6, -6, -8, -14, -15, -12, -12, -14, -13, -11, -5, 4, 
9, 12, 12, 7, 4, 5, 10, 15, 17, 17, 16, 13, 5, -4, -7, -4, 
5, 15, 19, 16, 4, -12, -34, -54, -63, -62, -55, -51, -50, -44, -35, -19, 
-3, 17, 40, 55, 55, 43, 27, 16, 9, 9, 19, 24, 26, 23, 16, 10, 
4, -1, -4, -2, -3, -19, -27, -29, -22, -15, -10, -9, -10, -15, -24, -29, 
-30, -27, -17, -4, 5, 6, -2, -8, -14, -15, -12, -10, -9, -5, -4, -7, 
-4, 4, 14, 18, 25, 32, 34, 27, 18, 7, -1, -5, -4, -3, -1, -2, 
-7, -12, -16, -20, -18, -10, -1, 4, 3, -6, -14, -23, -28, -28, -18, -12, 
-5, 4, 12, 14, 11, 5, -1, -4, -4, 7, 14, 19, 22, 21, 15, 2, 
-6, -10, -8, -2, 14, 17, 11, 4, 0, -2, 0, 6, 11, 11, 9, 2, 
-15, -21, -21, -15, -10, -6, 3, 13, 19, 18, 11, -3, -7, -14, -17, -11, 
-1, 13, 20, 25, 26, 26, 26, 25, 27, 29, 27, 22, 14, 6, -5, -9, 
-8, -9, -14, -18, -23, -26, -25, -20, -21, -23, -21, -20, -20, -19, -17, -10, 
-1, 4, 8, 3, -2, -5, -5, 3, 22, 29, 33, 26, 14, 3, -7, -4, 
5, 11, 13, 14, 2, -10, -19, -20, -17, -16, -20, -27, -29, -25, -18, -8, 
-5, 2, 12, 21, 29, 31, 27, 19, 5, -9, -18, -17, -11, -3, 3, 7, 
3, -3, -12, -23, -27, -25, -15, -8, -3, -4, -9, -21, -24, -18, -6, 6, 
12, 12, 11, 15, 20, 24, 30, 34, 40, 43, 45, 46, 45, 36, 22, 2, 
-13, -19, -18, -8, 3, 10, 15, 11, 5, 0, -7, -16, -18, -10, -11, -14, 
-21, -31, -34, -29, -29, -32, -34, -31, -21, -16, -13, -14, -18, -18, -5, 4, 
14, 24, 32, 39, 35, 17, -2, -20, -29, -34, -31, -25, -16, -3, 9, 10, 
3, -6, -10, -8, -4, 3, 5, 10, 21, 29, 30, 24, 21, 18, 16, 17, 
5, -8, -18, -19, -14, -8, 1, 5, 7, 6, 2, -10, -19, -26, -28, -28, 
-28, -29, -30, -27, -20, -10, 0, 8, 5, 8, 14, 13, 6, 5, 6, 9, 
11, 10, 11, 11, 13, 18, 21, 14, 2, -4, -9, -13, -16, -22, -21, -14, 
-5, 1, 4, 9, 12, 18, 21, 20, 18, 11, 5, 3, 2, 0, 0, 3, 
8, 10, 6, 3, 2, 5, 3, -1, -3, -4, -3, -7, -14, -17, -12, 4, 
16, 23, 25, 27, 26, 17, 10, 3, -6, -14, -11, -1, 10, 16, 15, 10, 
-1, -4, -6, -7, -8, -8, -7, -12, -17, -15, -12, -12, -16, -17, -15, -8, 
0, 8, 14, 12, 3, -6, -13, -8, 3, 11, 15, 17, 11, 6, 0, -12, 
-25, -32, -31, -26, -17, -6, 7, 16, 27, 27, 22, 19, 17, 9, 7, 4, 
-5, -16, -26, -30, -20, -15, -5, 13, 29, 39, 29, 15, 1, -15, -31, -22, 
-3, 17, 30, 32, 23, 9, -7, -15, -17, -24, -34, -34, -31, -22, -10, 5, 
11, 11, 8, 7, 11, 9, 10, 10, 7, 3, 2, 7, -1, -7, -1, 5, 
2, -6, -16, -25, -33, -41, -41, -26, -9, 6, 17, 23, 17, 4, -12, -21, 
-25, -16, -8, -11, -22, -29, -30, -37, -39, -32, -18, -1, 30, 43, 48, 47, 
37, 28, 24, 30, 36, 38, 37, 31, 18, 20, 29, 36, 32, 24, 13, 0, 
-15, -31, -42, -55, -58, -51, -40, -33, -30, -30, -24, -12, 1, 19, 36, 43, 
47, 46, 41, 30, 20, 11, -2, -14, -25, -30, -34, -40, -43, -38, -28, -9, 
-1, 6, 15, 21, 22, 6, -6, -15, -29, -42, -51, -57, -57, -56, -50, -41, 
-20, -4, 13, 27, 43, 59, 64, 54, 41, 31, 22, 15, 2, -11, -18, -22, 
-26, -29, -23, -15, -12, -10, -8, -7, -7, -8, -11, -17, -20, -16, -12, -14, 
-20, -21, -10, -1, 10, 20, 27, 34, 29, 18, 6, -5, -17, -24, -22, -14, 
0, 9, 18, 32, 36, 39, 38, 34, 27, 18, 15, 17, 21, 19, 15, 3, 
-5, -9, -11, -5, 13, 30, 48, 61, 70, 74, 66, 54, 36, 12, -7, -21, 
-22, -28, -41, -54, -63, -55, -43, -34, -28, -29, -34, -54, -61, -60, -47, -22, 
-1, 6, 10, 7, -2, -12, -19, -11, 1, 15, 29, 39, 48, 51, 47, 39, 
35, 43, 50, 52, 46, 39, 36, 33, 33, 33, 38, 44, 45, 35, 31, 25, 
14, 3, -8, -20, -34, -49, -66, -82, -105, -110, -108, -104, -102, -101, -91, -80, 
-70, -66, -60, -40, -24, -10, 3, 12, 17, 25, 30, 33, 39, 51, 66, 85, 
90, 85, 76, 64, 38, 19, 7, 3, 2, 2, 2, 1, -2, -4, -5, -9, 
-12, -13, -13, -9, -4, -1, 2, 7, 12, 14, 17, 20, 16, 10, 7, 8, 
13, 12, 5, -1, -4, 0, 6, 6, 6, 5, -1, -14, -37, -46, -44, -37, 
-31, -22, -10, -5, 0, -1, -2, -1, -1, -2, -5, -3, 5, 27, 35, 35, 
31, 19, -7, -16, -15, -7, 1, 5, 5, 2, 1, 2, 12, 28, 38, 31, 
21, 15, 15, 23, 34, 44, 47, 47, 41, 25, 18, 6, -8, -19, -27, -36, 
-38, -43, -53, -67, -77, -76, -71, -63, -53, -37, -15, -7, -4, -5, -7, -10, 
-6, -1, 1, 0, -5, -18, -24, -23, -16, -8, -2, -2, -4, -6, -4, 7, 
18, 28, 30, 27, 25, 26, 27, 30, 35, 38, 35, 27, 7, -2, -6, -3, 
7, 16, 19, 17, 9, -6, -22, -35, -34, -29, -26, -29, -36, -61, -71, -71, 
-63, -47, -29, -10, -2, 10, 23, 33, 37, 37, 33, 24, 12, 1, -15, -26, 
-34, -36, -32, -29, -20, -11, -1, 11, 20, 23, 23, 26, 37, 53, 73, 97, 
103, 102, 95, 78, 56, 16, -9, -34, -55, -64, -70, -73, -71, -66, -63, -57, 
-47, -47, -49, -49, -47, -41, -34, -22, -6, 9, 23, 38, 44, 55, 60, 51, 
41, 25, 18, 14, 6, -1, -6, -9, -12, -14, -13, -9, -6, -5, -8, -16, 
-19, -14, -3, 6, 20, 35, 46, 64, 70, 70, 69, 59, 39, 3, -28, -57, 
-80, -91, -95, -87, -76, -64, -54, -45, -30, -22, -16, -10, -2, 7, 18, 24, 
30, 36, 43, 50, 62, 69, 74, 73, 66, 44, 27, 11, -5, -17, -30, -36, 
-32, -32, -32, -34, -35, -30, -28, -27, -23, -18, -11, -5, 3, 17, 35, 50, 
59, 55, 48, 37, 28, 24, 24, 25, 20, 8, -4, -18, -18, -13, -7, 2, 
9, 8, 5, 1, -1, 1, 1, -2, -2, -7, -11, -19, -33, -34, -37, -41, 
-43, -48, -56, -54, -49, -44, -37, -32, -25, -14, -4, 4, 15, 27, 30, 34, 
37, 45, 58, 82, 98, 103, 102, 92, 62, 39, 17, 3, -10, -24, -41, -53, 
-58, -61, -64, -65, -67, -67, -67, -65, -60, -48, -40, -28, -10, 8, 18, 30, 
29, 25, 25, 24, 25, 25, 16, 8, -3, -15, -28, -31, -32, -31, -30, -27, 
-10, 2, 13, 23, 30, 33, 34, 31, 31, 35, 38, 44, 43, 41, 39, 37, 
30, 17, 6, -6, -15, -21, -24, -19, -16, -15, -17, -26, -32, -29, -28, -27, 
-23, -17, -16, -16, -14, -11, -8, -5, -2, -1, 3, 8, 11, 15, 16, 14, 
14, 16, 18, 24, 22, 20, 23, 24, 20, 12, 8, 11, 13, 10, 0, -5, 
-9, -18, -23, -26, -36, -38, -35, -24, -11, -4, 2, 5, 2, -7, -18, -31, 
-33, -33, -30, -21, -12, -2, 2, 1, 0, 2, 4, 8, 13, 18, 22, 27, 
30, 35, 39, 40, 47, 50, 45, 37, 25, 13, 5, -7, -13, -18, -26, -41, 
-57, -70, -70, -62, -51, -41, -30, -16, -8, -1, 5, 11, 23, 34, 42, 47, 
49, 50, 44, 38, 28, 14, 0, -9, -18, -20, -25, -31, -33, -33, -30, -26, 
-19, -7, 4, 11, 15, 18, 16, 13, 11, 3, 4, 5, 5, 8, 9, 7, 
8, 11, 13, 8, -8, -18, -25, -29, -35, -35, -29, -25, -19, -10, -6, -8, 
-5, -3, 2, 6, 8, 13, 11, 7, 1, -9, -16, -12, -8, -6, -2, 5, 
13, 13, 12, 8, 5, 1, -12, -14, -10, -5, -1, 3, 6, 8, 6, 4, 
4, -3, -6, -9, -16, -19, -18, -11, 1, 6, 7, 15, 20, 12, 3, -1, 
0, 1, 9, 14, 17, 16, 12, 8, 14, 17, 21, 22, 17, 13, 5, -6, 
-20, -35, -41, -35, -30, -24, -21, -16, -8, -1, 6, 12, 15, 18, 16, 12, 
11, 12, 12, 9, 8, 9, 11, 11, 5, 0, -6, -9, -7, -11, -22, -27, 
-33, -35, -34, -32, -25, -12, -7, 1, 12, 20, 22, 28, 29, 24, 21, 20, 
21, 20, 21, 24, 30, 31, 13, 8, 10, 8, -1, -11, -19, -22, -25, -17, 
-6, -5, 0, 5, 6, 13, 15, 13, 14, 7, -3, -7, -9, -16, -25, -33, 
-39, -43, -34, -27, -24, -22, -24, -16, -8, -13, -9, -4, 2, 5, 1, -2, 
-6, -11, -15, -11, -4, 5, 8, 7, 2, -4, 1, 10, 12, 12, 19, 28, 
31, 32, 30, 27, 25, 17, 16, 21, 15, 8, -2, -7, -8, -13, -10, -2, 
-1, 2, 3, 5, 2, 2, 0, -3, -6, -21, -26, -21, -26, -19, -13, -11, 
-5, -1, -3, 0, -3, -8, -3, -6, -8, -3, 2, -3, -13, -21, -22, -12, 
-7, -8, -2, 6, 4, 3, 8, 19, 24, 20, 23, 19, 15, 15, 5, 6, 
9, 7, 1, 3, 13, 19, 24, 26, 21, 16, 4, -2, 2, -2, -3, 5, 
15, 26, 31, 33, 26, 17, 14, 12, 6, -13, -23, -28, -34, -41, -43, -36, 
-33, -33, -37, -40, -28, -20, -10, -5, -8, -10, -8, -1, 4, 5, 7, 8, 
10, 9, 15, 21, 18, 6, 2, 6, 11, 18, 27, 23, 19, 24, 19, 2, 
-4, -18, -30, -33, -37, -34, -18, -14, -10, -13, -19, -17, -6, 9, 18, 26, 
43, 58, 75, 71, 61, 46, 18, -13, -37, -56, -60, -64, -54, -8, 22, 47, 
67, 69, 42, -9, -48, -78, -73, -60, -56, -52, -54, -52, -38, -18, 3, 0, 
-15, -21, -4, 35, 96, 118, 127, 104, 46, -16, -35, -46, -57, -68, -72, -82, 
-79, -68, -50, -10, 65, 99, 112, 120, 124, 111, 61, 26, -8, -54, -95, -99, 
-55, -25, 7, 31, 45, 61, 47, 17, -18, -41, -41, -20, -15, -20, -36, -53, 
-61, -55, -39, -25, -12, 9, 43, 60, 58, 52, 43, 17, -38, -68, -88, -89, 
-72, -54, -47, -42, -20, 7, 26, 64, 97, 110, 100, 83, 67, 43, 23, -4, 
-26, -35, -37, -42, -42, -44, -52, -56, -55, -42, -21, 0, 18, 24, 11, -3, 
-3, 13, 37, 63, 76, 63, 47, 23, -7, -41, -47, -42, -28, -18, -24, -41, 
-59, -76, -85, -82, -64, -19, 21, 59, 88, 106, 98, 79, 65, 44, 24, 15, 
12, 9, -5, -21, -35, -46, -39, -26, -8, 7, 10, -10, -26, -36, -43, -45, 
-38, -10, 10, 27, 39, 43, 42, 46, 44, 34, 24, 15, 3, -6, -19, -31, 
-39, -47, -63, -63, -50, -31, -7, 24, 41, 52, 50, 44, 32, -1, -18, -28, 
-34, -33, -34, -38, -38, -46, -55, -56, -26, 1, 20, 27, 28, 24, 6, -3, 
-10, -13, -4, 16, 45, 55, 59, 56, 42, 18, 9, 1, -10, -24, -34, -48, 
-54, -52, -49, -40, -27, -16, -9, 1, 11, 25, 53, 61, 62, 67, 74, 76, 
69, 62, 53, 36, 17, 1, -19, -19, -14, -17, -28, -40, -42, -44, -35, -19, 
-4, 15, 19, 10, -4, -14, -21, -25, -26, -32, -33, -33, -38, -42, -47, -48, 
-45, -47, -41, -18, 7, 33, 57, 72, 83, 85, 77, 65, 56, 38, 24, 16, 
12, 6, -4, -14, -14, -19, -34, -51, -65, -63, -53, -44, -28, -12, -4, -2, 
-4, -4, 0, 8, 21, 16, 11, 16, 20, 16, 5, 4, 4, 2, 0, -1, 
-5, -9, -15, -24, -30, -25, -16, -2, 12, 23, 30, 34, 37, 34, 31, 29, 
33, 36, 37, 37, 30, 19, 2, -7, -6, -1, 3, 3, 4, 4, 0, -4, 
-3, -3, 0, 5, 4, 0, -8, -25, -36, -46, -54, -59, -52, -44, -33, -16, 
1, 15, 16, 8, -2, -9, -12, -11, 0, 10, 18, 21, 25, 27, 18, 5, 
-7, -20, -30, -29, -23, -20, -18, -16, -10, -2, 3, 5, 5, 9, 20, 28, 
36, 41, 42, 37, 22, 15, 11, 8, 8, 10, 8, 3, -5, -14, -20, -35, 
-46, -47, -44, -38, -28, -10, -2, 1, 1, -1, -6, -14, -10, -1, 9, 14, 
15, 14, 14, 9, 0, 1, 12, 14, 12, 11, 11, 11, 11, 10, 8, 7, 
8, 5, -1, -4, -1, 4, 9, 20, 27, 31, 29, 21, 11, -2, -8, -17, 
-27, -32, -33, -35, -43, -51, -59, -61, -51, -37, -18, 5, 29, 42, 35, 25, 
20, 17, 7, -5, -5, -7, -13, -23, -34, -41, -39, -33, -29, -24, -9, 2, 
8, 7, 11, 20, 30, 38, 48, 51, 53, 51, 41, 36, 31, 26, 20, 11, 
6, -2, -13, -23, -29, -32, -34, -35, -36, -33, -23, -4, 4, 7, 8, 7, 
0, -11, -20, -28, -37, -41, -45, -39, -23, -4, 8, 13, 23, 30, 30, 22, 
12, 5, 2, 0, -2, -4, -1, 4, 3, 5, 16, 28, 37, 40, 35, 34, 
35, 30, 25, 26, 28, 28, 23, 16, 8, 6, 4, 0, -5, -11, -18, -26, 
-34, -43, -51, -52, -53, -59, -65, -65, -61, -52, -43, -30, -15, 1, 16, 28, 
26, 23, 18, 10, 12, 16, 21, 31, 34, 31, 32, 30, 28, 30, 35, 38, 
34, 27, 13, -5, -26, -50, -55, -52, -43, -32, -22, -10, -3, 3, 6, 9, 
11, 7, 0, -7, -14, -21, -22, -19, -17, -13, -7, -3, 1, 2, 3, 8, 
17, 35, 44, 54, 63, 65, 58, 36, 16, -2, -14, -21, -24, -25, -22, -22, 
-22, -23, -28, -28, -25, -22, -16, -7, 3, 7, 8, 12, 16, 21, 28, 32, 
42, 51, 51, 46, 32, 15, 1, -9, -14, -19, -19, -21, -27, -35, -41, -43, 
-39, -30, -19, -7, 6, 11, 12, 10, 6, 1, -7, -12, -15, -14, -15, -14, 
-8, -5, -3, 0, 3, 9, 10, 8, 8, 7, 9, 15, 18, 23, 24, 15, 
6, -3, -7, -8, -10, -15, -24, -26, -26, -23, -15, -3, 19, 26, 22, 15, 
8, 3, -2, -4, -5, -7, -14, -24, -25, -22, -17, -17, -19, -16, -11, -6, 
-4, 0, 7, 15, 15, 15, 17, 20, 24, 25, 28, 34, 39, 40, 36, 28, 
21, 17, 11, -5, -14, -20, -23, -26, -27, -23, -21, -21, -21, -21, -23, -22, 
-18, -13, -11, -7, 3, 8, 13, 15, 14, 9, -3, -8, -9, -7, -7, -8, 
-12, -13, -14, -14, -10, 3, 9, 14, 19, 20, 20, 25, 27, 28, 29, 30, 
26, 19, 15, 12, 8, 4, -1, -7, -13, -14, -15, -20, -32, -37, -37, -36, 
-35, -34, -31, -27, -25, -23, -18, -2, 9, 18, 22, 24, 25, 22, 19, 17, 
15, 11, 6, -1, -2, -3, -6, -8, -11, -13, -12, -8, -4, 0, 9, 9, 
5, 0, -2, 0, 4, 6, 8, 8, 9, 7, 3, 0, -1, -3, -5, -7, 
-7, -6, 0, };
