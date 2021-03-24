const unsigned int knockSampleRate = 22050;
const unsigned int knockSampleCount = 1630; // 6530 240
const signed char knockSamples[] = {
2, 0, -11, -21, -29, -26, -7, -6, -20, -39, -50, -50, -46, -41, -36, -35, 
-29, -12, -7, -15, -29, -32, -28, -12, -8, -14, -17, -16, -14, -27, -39, -46, 
-46, -42, -39, -39, -39, -44, -50, -54, -42, -25, -20, -28, -43, -59, -72, -68, 
-57, -50, -49, -58, -62, -61, -56, -49, -48, -52, -49, -52, -64, -72, -60, -34, 
-40, -60, -67, -51, -32, -44, -49, -32, -9, -4, -33, -47, -44, -31, -15, -2, 
0, -6, -12, -19, -26, -29, -19, 2, 17, 7, -15, -25, -20, -8, 5, 7, 
4, -5, -16, -19, -20, -29, -42, -41, -40, -40, -38, -38, -45, -53, -51, -33, 
-11, 1, -5, -8, -11, -24, -42, -48, -31, -22, -26, -32, -32, -39, -54, -64, 
-62, -53, -45, -26, -9, -5, -11, -14, 2, 14, 14, -1, -19, -31, -53, -60, 
-54, -48, -47, -51, -44, -31, -31, -44, -47, -12, 14, 21, 10, -9, -20, -20, 
-19, -23, -35, -42, -31, -19, -13, -16, -25, -32, -30, -29, -25, -23, -23, -20, 
-12, -13, -24, -43, -53, -49, -37, -27, -35, -46, -42, -23, -16, -20, -26, -28, 
-25, -12, 0, 3, -4, -7, -5, -6, -9, -15, -23, -23, -14, -12, -5, 2, 
4, 0, -11, -19, -24, -26, -28, -20, -9, -4, -2, -2, 0, 8, 8, 0, 
-11, -21, -19, 8, 18, 12, -5, -21, -23, -20, -13, -7, -9, -12, -8, 2, 
14, 18, 13, 4, -6, -12, -15, -5, 11, 16, 24, 34, 31, 27, 31, 41, 
44, 46, 46, 49, 62, 63, 58, 60, 67, 77, 72, 62, 68, 86, 90, 85, 
105, 101, 73, 52, 49, 70, 107, 88, 60, 52, 58, 56, 35, 20, 29, 47, 
47, 22, 11, 22, 40, 46, 39, 32, 21, 21, 33, 38, 31, 21, 21, 41, 
63, 70, 61, 52, 50, 57, 60, 55, 60, 75, 87, 87, 80, 53, 48, 65, 
77, 78, 70, 58, 51, 40, 36, 42, 45, 28, 9, 4, 12, 18, 7, 2, 
12, 33, 39, 30, 18, 28, 49, 56, 54, 58, 70, 77, 68, 45, 18, 4, 
13, 25, 26, 11, -5, 4, 12, 7, 3, -3, -7, -3, -6, -14, -22, -24, 
-25, -26, -28, -32, -33, -32, -34, -35, -38, -36, -37, -41, -27, -8, 1, -3, 
-6, 11, 16, 5, -9, -14, 0, 28, 23, 6, -5, -6, 4, 9, 8, 1, 
-7, -1, 16, 19, 19, 18, 19, 18, 11, 14, 10, 0, -4, -5, -6, -1, 
8, 11, 6, -2, 2, 7, 10, 8, 10, 27, 28, 21, 8, 1, 16, 23, 
20, 15, 12, 5, -2, 3, 11, 21, 31, 36, 33, 27, 23, 24, 21, 21, 
29, 37, 38, 31, 22, 17, 24, 28, 26, 16, 2, -2, -1, 8, 21, 27, 
25, 22, 17, 13, 9, -2, -18, -26, -27, -27, -34, -41, -42, -42, -39, -35, 
-36, -37, -35, -38, -36, -35, -32, -29, -34, -41, -52, -63, -60, -43, -29, -26, 
-34, -48, -56, -52, -43, -38, -41, -54, -71, -62, -46, -32, -30, -36, -35, -31, 
-29, -38, -45, -35, -16, -2, -3, -9, -17, -25, -23, -13, -10, -21, -44, -52, 
-43, -42, -48, -51, -60, -69, -81, -87, -74, -51, -24, -16, -20, -38, -58, -58, 
-41, -31, -31, -37, -40, -45, -52, -58, -64, -71, -82, -90, -86, -82, -85, -91, 
-83, -72, -69, -74, -79, -81, -84, -85, -82, -74, -65, -55, -48, -48, -48, -48, 
-48, -51, -49, -39, -29, -28, -24, -13, -11, -12, -20, -24, -6, 3, 2, -1, 
-4, -7, -4, -3, -3, -5, -13, -25, -33, -25, -13, -6, -9, -21, -15, -1, 
9, 7, -2, -13, -10, 1, -2, -15, -25, -36, -39, -37, -27, -13, -5, -6, 
-6, -12, -20, -21, -23, -20, -10, -7, -15, -22, -14, 2, 10, 2, -1, 5, 
-1, -15, -30, -38, -36, -20, -8, 6, 18, 20, 15, 10, 8, 6, 1, 0, 
5, 8, 13, 13, 12, 10, -2, -9, -7, 2, 8, 16, 30, 41, 45, 34, 
18, 1, 6, 16, 23, 20, 17, 18, 13, 15, 16, 15, 17, 21, 18, 16, 
10, 3, 12, 12, 7, 1, -8, -14, -13, -14, -12, -12, -16, -16, -4, 4, 
4, -1, -6, -15, -15, -3, 17, 36, 42, 26, 8, -2, 1, 11, 16, 8, 
13, 26, 37, 44, 40, 33, 34, 42, 46, 42, 30, 23, 16, 11, 11, 11, 
8, 11, 18, 22, 20, 20, 20, 23, 31, 30, 21, 7, 4, 9, 20, 27, 
30, 31, 35, 42, 50, 58, 79, 88, 92, 91, 85, 83, 87, 89, 86, 73, 
54, 41, 35, 31, 34, 46, 64, 78, 71, 62, 64, 72, 76, 63, 54, 50, 
49, 57, 68, 67, 61, 52, 42, 33, 25, 19, 22, 30, 32, 30, 26, 29, 
36, 37, 29, 16, 17, 18, 24, 28, 28, 26, 26, 27, 20, 14, 17, 40, 
53, 55, 49, 42, 40, 45, 45, 31, 16, 16, 35, 39, 31, 16, -1, -15, 
-23, -24, -26, -24, -14, 0, 2, 4, 4, 7, 11, 10, 7, 12, 21, 23, 
23, 29, 34, 34, 31, 29, 38, 51, 61, 65, 62, 57, 54, 47, 40, 37, 
36, 32, 18, 12, 10, 12, 17, 12, -1, -8, -10, -11, -11, 2, 9, 15, 
17, 13, 13, 8, 7, 17, 20, 18, 26, 35, 38, 37, 31, 24, 28, 30, 
36, 50, 66, 67, 59, 46, 36, 43, 49, 45, 31, 12, 1, -2, 2, 7, 
6, 8, 17, 19, -4, -15, -10, 9, 22, 18, -7, -17, -14, -6, 7, 11, 
-8, -31, -48, -49, -37, -9, -4, -8, -18, -18, -18, -35, -37, -30, -24, -32, 
-45, -60, -64, -61, -55, -51, -47, -47, -50, -50, -46, -38, -34, -31, -16, 1, 
8, -1, -15, -15, -12, -17, -33, -50, -48, -34, -18, -17, -31, -55, -68, -69, 
-61, -48, -38, -46, -59, -78, -95, -97, -70, -53, -51, -55, -63, -66, -58, -60, 
-60, -55, -44, -30, -18, -21, -26, -30, -31, -25, -25, -30, -36, -35, -26, -19, 
-26, -33, -34, -27, -9, 13, 10, -3, -14, -21, -27, -31, -28, -19, -11, -6, 
-23, -49, -67, -71, -58, -32, -22, -14, -7, -6, -12, -16, -16, -15, -14, -15, 
-21, -28, -23, -18, -20, -29, -33, -27, -24, -27, -26, -20, -1, 16, 24, 23, 
19, 21, 31, 37, 35, 19, -1, -22, -17, -7, -5, -10, -16, -28, -31, -24, 
-6, 17, 31, 22, 5, -7, -17, -26, -29, -25, -19, -17, -24, -30, -29, -25, 
-29, -39, -50, -53, -43, -35, -25, -17, -13, -16, -16, -11, -8, -7, -2, 3, 
-1, -6, -10, -16, -17, -13, -9, -6, -7, -14, -38, -49, -47, -33, -19, -13, 
-21, -33, -39, -37, -31, -25, -7, 13, 23, 12, -12, -36, -41, -42, -38, -25, 
-11, -9, -19, -28, -22, -11, -6, -9, -15, -20, -22, -24, -27, -26, -27, -35, 
-44, -49, -41, -27, -15, -15, -23, -26, -15, -10, -10, -10, -6, -4, -12, -23, 
-28, -20, -6, 6, 5, -2, -4, -8, -9, -2, 7, 13, 7, -4, -16, -12, 
-4, 4, 16, 24, 25, 19, 14, 18, 30, 52, 48, 28, 8, 0, 2, 9, 
16, 28, 36, 38, 34, 30, 35, 40, 44, 46, 42, 43, 48, 50, 44, 45, 
67, 61, 40, 33, 52, 74, 72, 65, 77, 101, 110, 90, 88, 101, 108, 107, 
106, 109, 101, 84, 69, 60, 55, 55, 63, 71, 71, 50, 20, 34, 56, 70, 
74, 74, 71, 62, 47, 37, 39, 43, 54, 45, 15, -9, -9, 24, 44, 55, 
59, 59, 56, 42, 37, 52, 76, 79, 57, 28, 31, 37, 34, 24, 6, 2, 
-2, -11, -17, -17, -4, 3, 6, -1, -10, -10, -7, -10, -12, -15, -24, -37, 
-45, -51, -48, -45, -46, -42, -28, -9, 1, -4, -11, -4, 4, 10, 18, 25, 
19, 4, -11, -19, -19, -12, -9, -10, -7, -5, -8, -21, -27, -25, -20, -21, 
-23, -24, -25, -23, -19, -18, -26, -36, -28, -16, -8, -3, 0, -2, -7, -9, 
-4, 2, 1, -11, -19, -17, -16, -13, 11, 24, 27, 24, 21, 26, 22, 14, 
13, 17, 18, 4, -8, -8, 2, 11, 13, 0, -17, -34, -41, -28, -2, 1, 
-10, -23, -27, -25, -11, 0, 10, 21, 24, 15, -4, -16, -23, -18, -13, -13, 
-15, -16, -14, -11, -3, 16, 25, 29, 27, 28, 34, 41, 36, 29, 23, 20, 
13, 3, -3, -4, -2, 2, 10, 20, 29, 38, 44, 41, 35, 32, 26, 18, 
15, 13, 7, -1, -10, -17, -23, -30, -26, -22, -21, -17, -12, -10, -9, -3, 
0, 0, -12, -28, -38, -43, -43, -40, -25, -22, -31, -46, -60, -63, -63, -51, 
-32, -13, 1, 9, 10, 10, 3, -4, -10, -15, -6, 2, 3, 3, -8, -15, 
-16, -13, -9, -10, -24, -24, -23, -31, -40, -36, -35, -44, -55, -67, -76, -88, 
-89, -85, -80, -77, -75, -75, -79, -81, -80, -68, -44, -19, -20, -32, -46, -54, 
-51, -46, -38, -27, -24, -30, -42, -53, -54, -50, -42, -29, -20, -23, -29, -35, 
-42, -53, -60, -74, -85, -89, -85, -69, -72, -83, -94, -100, -94, -80, -77, -76, 
-73, -68, -63, -71, -73, -58, -36, -23, -34, -46, -46, -40, -34, -24, 0, 4, 
1, -6, -15, -23, -19, -7, 8, 20, 22, 19, 11, 7, 10, 11, 9, -6, 
-15, -18, -20, -26, -41, -43, -38, -29, -25, -23, -15, -12, -15, -25, -29, -27, 
-15, -7, -9, -17, -21, -16, -3, 11, 19, 21, 16, 2, -9, -14, -6, 18, 
45, 52, 40, 25, 9, -1, -2, 9, 21, 22, 19, 19, 22, 24, 27, 34, 
41, 52, 53, 42, 29, 26, 29, 24, 21, 20, 25, 36, 36, 18, 5, 6, 
17, 29, 34, 26, 14, 7, 9, 13, 28, 41, 43, 39, 29, 21, 22, 28, 
42, 70, 97, 82, 65, 59, 71, 84, 75, 56, 43, 37, 33, 24, 14, 31, 
60, 74, 73, 64, 61, 67, 68, 57, 44, 40, 56, 61, 50, 36, 32, 40, 
53, 49, 46, 49, 60, 72, 72, 68, 64, 71, 80, 71, 52, 41, 45, 54, 
66, 68, 60, 49, 43, 43, 44, 46, 45, 47, 48, 50, 55, 43, 33, 27, 
28, 28, 8, 0, 3, 17, 29, 26, 10, -4, -6, -7, 0, 20, 36, 48, 
42, 26, 5, -15, -10, 0, 4, -1, -6, -4, 3, 3, -4, -9, -7, 5, 
15, 14, 9, 8, 23, 43, 58, 59, 52, 55, 63, 66, 56, 40, 36, 53, 
61, 57, 41, 25, 21, 20, 20, 14, 7, 5, 1, 1, 6, 12, 20, 23, 
8, -12, -29, -35, -33, -16, -14, -21, -34, -46, -48, -42, -34, -28, -21, -11, 
1, 8, 5, -5, -10, 4, 22, 22, 14, 7, 9, 14, 9, 0, -9, -11, 
-4, 4, 17, 13, -5, -24, -35, -39, -35, -29, -29, -38, -45, -58, -66, -62, 
-41, -20, -16, -18, -19, -17, -5, 5, 7, 9, 4, 0, 4, 14, 31, 29, 
21, 11, 4, 8, 33, 46, 52, 55, 48, 31, 26, 29, 39, 43, 40, 21, 
15, 17, 12, 5, 3, 4, 5, 9, 8, 6, 2, 0, 2, 3, -1, -9, 
-26, -39, -37, -25, -9, -4, -28, -37, -38, -32, -23, -22, -32, -33, -23, -12, 
-7, -9, -12, -7, 5, 13, 13, 12, 14, 14, 11, 4, -3, 4, 16, 15, 
8, 2, -11, -9, 0, 7, -3, -43, -64, -62, -39, -21, -21, -56, -80, -79, 
-62, -35, -13, -22, -37, -47, -51, -48, -41, -31, -22, -21, -29, -44, -58, -59, 
-52, -42, -37, -33, -25, -19, -16, -18, -28, -42, -35, -27, -19, -21, -42, -70, 
-95, -110, -106, -90, -63, -35, -38, -53, -69, -84, -98, -101, -90, -69, -54, -57, 
-91, -115, -126, -127, -119, -102, -83, -72, -58, -45, -41, -43, -31, -16, -8, -7, 
-8, 3, 9, 9, -8, -35, -55, -61, -47, -37, -42, -50, -52, -44, -32, -16, 
0, 5, 1, -6, -13, -15, -8, 3, -7, -29, -45, -46, -42, -46, -43, -32, 
-21, -15, -12, 1, 12, 21, 17, 3, -6, -4, 8, 15, 18, 6, -36, -51, 
-56, -47, -28, -8, 3, 1, 3, 9, 14, 12, 0, 3, 10, 12, 8, -9, 
-19, -23, -23, -22, -19, -6, 1, 3, 2, 0, -2, 0, 3, 12, 20, 24, 
27, 17, 0, -4, 6, 19, 32, 20, -10, -42, -57, -43, -22, -3, 8, 12, 
11, 5, -4, -13, -23, -30, -28, -22, -20, -17, -15, -8, 8, 8, -1, -8, 
1, 17, 30, 35, 33, 28, 21, 14, 13, 10, 11, 10, -5, -18, -20, -19, 
-18, -15, -4, 19, 30, 31, 26, 16, 7, 2, 9, 27, 37, 35, 35, 29, 
20, 24, 36, 47, 62, 66, 64, 56, 50, 57, 73, 78, 72, 58, 45, 40, 
45, 51, 51, 44, 37, 34, 42, 51, 53, 49, 48, 43, 38, 32, 31, 39, 
41, 34, 20, 8, 10, 19, 38, 46, 37, 26, 27, 35, 47, 53, 51, 48, 
46, 30, 21, 17, 21, 24, 17, 7, 3, -6, -10, -1, 13, 24, 20, 6, 
-4, -2, 3, -1, -12, -20, -19, -10, 3, 5, 6, 9, 9, 3, 5, 15, 
27, 33, 34, 26, 19, 19, 29, 35, 35, 24, 12, 7, 14, 23, 28, 22, 
9, -4, -10, -11, -6, 4, 18, 27, 22, 8, -20, -34, -33, -14, 4, 6, 
1, 1, 6, 8, 1, -16, -14, 9, 37, 47, 39, 34, 37, 41, 39, 30, 
29, 34, 35, 27, 21, 22, 27, 35, 45, 48, 43, 31, 13, 14, 25, 37, 
41, 37, 36, 27, 14, -1, -8, 4, 22, 37, 39, 31, 19, 8, 10, 16, 
21, 18, 7, 12, 18, 25, 36, 45, 42, 34, 27, 23, 28, 39, 54, 53, 
46, 37, 35, 48, 54, 55, 50, 40, 33, 30, 38, 49, 57, 60, 58, 38, 
25, 27, 41, 57, 64, 52, 27, 12, 18, 30, 49, 50, 48, 44, 40, 40, 
23, 16, 17, 18, 15, -1, -7, 0, 13, 22, 22, 8, 3, 1, 2, -5, 
-22, -24, -25, -28, -38, -51, -60, -65, -66, -65, -58, -50, -54, -64, -68, -61, 
-49, -41, -38, -35, -34, -37, -47, -52, -47, -30, -21, -26, -28, -23, -11, -12, 
-17, -18, -11, -4, -2, 1, -7, -16, -18, -17, -17, -27, -41, -56, -73, -73, 
-64, -48, -41, -45, -47, -51, -59, -68, -74, -71, -62, -56, -53, -52, -51, -46, 
-59, -79, -93, -90, -65, -48, -36, -40, -55, -68, -68, -58, -50, -43, -38, -29, 
-25, -31, -45, -62, -65, -60, -65, -70, -74, -77, -78, -72, -62, -44, -29, -32, 
-43, -46, -40, -42, -49, -51, -59, -65, -60, -44, -24, -5, -6, -15, -25, -36, 
-39, -37, -11, 7, 10, -3, -18, -18, -12, -2, 2, -7, -12, -11, -17, -20, 
-17, -17, -11, 0, -2, -6, -8, -4, 3, 4, 13, 35, 60, 66, 43, 28, 
22, 28, 33, 24, 15, 17, 25, 29, 29, 19, 13, 13, 21, 38, 50, 50, 
45, 34, 18, 6, 13, 20, 26, 23, 8, -6, -10, -15, -16, -7, 8, 23, 
27, 17, 1, -10, -16, -29, -37, -37, -28, -20, -17, -10, -7, -1, 9, 15, 
14, -2, -19, -32, -37, -33, -22, -20, -23, -22, -18, -20, -26, -24, -21, -17, 
-19, -24, -23, -22, -23, -27, -32, -39, -39, -39, -40, -32, -19, 3, 3, -11, 
-22, -28, -29, -14, -5, -1, -1, 5, 7, 1, 1, 0, -7, 0, 17, 16, 
11, 5, 13, 22, 17, 22, 34, 36, 35, 16, 6, 7, 3, 0, 5, 7, 
8, 11, 13, 23, 30, 13, 5, 10, 15, 21, 25, 25, 26, 17, 6, 8, 
16, 24, 31, 30, 22, 15, 26, 46, 56, 49, 41, 37, 32, 26, 28, 41, 
57, 75, 67, 47, 32, 22, 12, 13, 14, 28, 53, 68, 63, 53, 50, 53, 
60, 66, 71, 70, 57, 48, 44, 45, 47, 51, 56, 61, 63, 52, 47, 51, 
60, 68, 66, 53, 46, 39, 35, 33, 24, 20, 20, 22, 27, 32, 40, 45, 
43, 28, 4, -14, -7, 21, 43, 49, 46, 32, 20, 12, 8, 15, 28, 38, 
33, 24, 22, 21, 18, 23, 35, 42, 39, 32, 12, 5, 7, 7, 4, -4, 
-14, -12, 0, 8, 3, -4, -10, -6, -3, -8, -13, -15, -14, -10, -4, -2, 
-2, -1, 1, 1, -5, -15, -21, -13, -2, 10, 14, 6, -8, -10, -8, -9, 
-12, -14, -13, -12, -13, -13, -13, -11, -11, -17, -17, -12, -3, 15, 17, 22, 
28, 24, 16, 10, 11, 13, 13, 14, 19, 18, 9, -2, -8, -7, -6, -11, 
-14, -9, 6, 16, 10, 0, -9, -17, -22, -17, -15, -16, -21, -32, -42, -47, 
-42, -32, -24, -24, -30, -45, -50, -46, -40, -42, -53, -54, -48, -43, -43, -40, 
-28, -25, -22, -22, -29, -36, -37, -32, -27, -23, -19, -16, -18, -30, -46, -56, 
-51, -26, -14, -13, -18, -18, -14, -17, -23, -19, -11, -7, -12, -17, -16, -14, 
-11, -15, -34, -38, -28, -12, -6, -15, -34, -38, -33, -27, -20, -26, -35, -38, 
-41, -47, -45, -25, -20, -28, -37, -32, -25, -27, -34, -38, -29, -18, -12, 1, 
21, 32, 25, 13, -6, -15, -21, -21, -15, -15, -34, -51, -55, -51, -45, -39, 
-33, -26, -25, -24, -28, -40, -48, -56, -52, -42, -41, -45, -37, -29, -29, -35, 
-33, -22, -14, -10, -7, -2, -2, -2, -2, -5, -8, -10, -20, -36, -54, -71, 
-79, -60, -31, -9, -7, -15, -21, -17, -11, 1, 17, 36, 48, 34, 18, 10, 
11, 12, 6, 5, 4, 13, 26, 30, 37, 41, 51, 58, 57, 49, 35, 16, 
-1, -11, -15, -11, -5, -3, 1, 9, 10, 1, -7, -16, -16, -11, -1, 5, 
5, 2, -5, -17, -41, -53, -58, -45, -27, -22, -28, -32, -34, -38, -42, -44, 
-49, -55, -56, -45, -30, -25, -40, -51, -53, -55, -56, -55, -50, -41, -45, -59, 
-74, -67, -50, -31, -19, -18, -25, -32, -36, -46, -61, -65, -45, -31, -31, -49, 
-65, -67, -50, -37, -33, -29, -23, -5, 1, -2, -9, -14, -13, -5, -1, 6, 
15, 24, 34, 38, 36, 31, 28, 30, 49, 63, 69, 70, 69, 67, 68, 66, 
54, 28, 6, 0, 2, 8, 11, 11, 7, 2, -3, -10, -11, -1, 15, 22, 
21, 22, 26, 36, 56, 66, 72, 79, 82, 77, 74, 79, 91, 100, 102, 99, 
92, 88, 83, 80, 84, 90, 89, 79, 69, 69, 72, 58, 43, 42, 49, 53, 
54, 56, 54, 51, 54, 56, 44, 43, 51, 64, 72, 74, 74, 83, 88, 79, 
66, 52, 60, 77, 89, 88, 82, 84, 86, 82, 74, 70, 75, 79, 64, 45, 
30, 23, 19, 15, 15, 20, 31, 41, 48, 48, 44, 37, 29, 21, 19, 16, 
13, 14, 19, 23, 18, 2, -10, -8, 3, 14, 10, 3, -1, -1, -3, -3, 
5, 11, 10, -2, -29, -43, -51, -48, -49, -58, -61, -55, -45, -33, -31, -37, 
-32, -32, -41, -44, -34, -20, -20, -24, -29, -34, -37, -32, -22, -14, -7, -6, 
-14, -24, -24, -24, -26, -20, -8, -2, 0, -9, -23, -35, -44, -34, -21, -19, 
-18, -20, -27, -29, -28, -22, -16, -8, -4, 2, 5, 1, -7, -15, -4, 7, 
13, 16, 5, -4, -7, -1, 5, 6, 11, 25, 46, 55, 40, 13, -12, -6, 
11, 18, 14, 13, 25, 34, 29, 16, 11, 35, 45, 37, 26, 16, 15, 26, 
21, 10, 4, 6, 10, 13, 18, 23, 28, 30, 10, -7, -12, -5, 10, 22, 
32, 33, 24, 5, -15, -25, -25, -26, -26, -18, -1, 0, -19, -32, -26, -3, 
25, 48, 46, 40, 40, 45, 49, 43, 29, 21, 21, 22, 4, -7, -6, 2, 
0, -11, -21, -31, -36, -26, -10, 8, 6, -10, -33, -47, -45, -34, -38, -52, 
-58, -59, -54, -49, -61, -74, -74, -63, -57, -68, -78, -77, -70, -71, -82, -74, 
-58, -50, -51, -58, -57, -45, -35, -31, -31, -24, -24, -34, -52, -68, -76, -73, 
-74, -82, -87, -80, -62, -44, -52, -60, -63, -58, -49, -49, -44, -43, -50, -62, 
-79, -74, -61, -55, -58, -58, -54, -58, -67, -67, -58, -48, -48, -52, -54, -52, 
-48, -45, -37, -30, -28, -28, -19, -10, -5, -4, -9, -21, -27, -26, -24, -24, 
-26, -32, -43, -53, -60, -58, -45, -21, -21, -29, -37, -31, -11, -4, -13, -20, 
-22, -23, -32, -45, -44, -38, -36, -34, -25, -21, -20, -19, -15, -9, -11, -19, 
-25, -22, -16, -10, -14, -28, -34, -26, -16, -18, -22, -22, -17, -15, -16, -19, 
-17, -14, -13, -16, -19, -6, 3, 1, -8, -11, 5, 19, 30, 32, 29, 27, 
21, 18, 18, 23, 27, 30, 31, 28, 22, 13, 5, 5, 8, 7, 1, -9, 
-11, 2, 12, 17, 16, 13, 18, 33, 33, 32, 33, 31, 25, 23, 30, 44, 
53, 48, 32, 29, 29, 30, 35, 44, 57, 60, 58, 48, 33, 24, 25, 24, 
18, 10, 4, -2, -3, 3, 16, 28, 32, 30, 37, 47, 53, 56, 45, 34, 
29, 28, 31, 31, 38, 45, 49, 57, 71, 90, 91, 83, 72, 64, 60, 51, 
46, 42, 40, 42, 50, 57, 53, 45, 34, 30, 34, 32, 31, 29, 23, 17, 
14, 20, 24, 26, 24, 21, 18, 21, 28, 36, 47, 47, 35, 22, 17, 26, 
35, 48, 56, 54, 43, 35, 33, 37, 38, 34, 30, 32, 33, 25, 17, 8, 
0, -5, -11, -9, -1, 7, 9, 6, 2, 4, 9, 6, -2, -4, 0, 3, 
-4, -17, -31, -35, -23, -9, 1, 6, 9, 20, 22, 22, 18, 5, -7, -5, 
1, 1, -3, -1, 8, 11, 13, 15, 21, 27, 16, 7, -1, -2, 0, -7, 
-15, -16, -13, -11, -13, -21, -14, 0, 7, 3, -8, -8, 2, 14, 21, 21, 
25, 20, 16, 18, 16, 11, 4, 10, 21, 28, 31, 19, 11, 16, 30, 43, 
51, 53, 46, 37, 29, 27, 36, 51, 50, 41, 35, 37, 52, 63, 67, 62, 
52, 43, 41, 42, 47, 53, 61, 70, 67, 54, 43, 40, 40, 48, 55, 52, 
42, 31, 26, 25, 23, 23, 29, 41, 47, 28, 9, -6, -12, -2, 13, 8, 
2, 3, 3, 6, 13, 10, 2, 1, 10, 21, 20, 17, 27, 39, 42, 36, 
28, 18, 12, 12, 18, 26, 23, 17, 15, 13, 13, 15, 24, 37, 38, 25, 
0, -9, -7, -7, -12, -22, -43, -56, -69, -75, -71, -62, -51, -47, -46, -49, 
-50, -48, -42, -36, -31, -29, -41, -69, -84, -82, -67, -61, -63, -64, -68, -72, 
-74, -66, -40, -23, -10, -3, -4, -17, -33, -27, -18, -22, -35, -29, -3, 4, 
-5, -24, -34, -42, -42, -37, -39, -39, -30, -28, -36, -44, -48, -50, -54, -60, 
-68, -73, -71, -65, -58, -58, -61, -55, -38, -27, -31, -36, -39, -35, -29, -30, 
-38, -41, -44, -38, -26, -10, -6, -18, -32, -34, -27, -15, -6, 5, 17, 18, 
-16, -39, -41, -26, -16, -14, -23, -35, -41, -35, -25, -19, -16, -17, -17, -16, 
-20, -35, -35, -33, -38, -48, -57, -70, -71, -73, -72, -65, -61, -64, -62, -55, 
-44, -40, -33, -17, -5, 0, -6, -22, -33, -30, -29, -31, -29, -20, -1, 6, 
6, -3, -14, -18, -6, 2, -6, -22, -32, -25, -19, -18, -20, -19, -12, -8, 
-7, -12, -21, -29, -38, -30, -25, -27, -30, -33, -32, -31, -31, -32, -36, -41, 
-38, -25, -19, -21, -24, -15, -7, -9, -19, -27, -27, -10, 1, 9, 14, 17, 
26, 38, 41, 36, 19, 7, 17, 28, 33, 35, 34, 34, 31, 24, 18, 15, 
20, 25, 22, 15, 8, 5, -1, -17, -24, -23, -16, -10, -12, -12, -5, -10, 
-1, 19, 30, 30, 29, 37, 44, 48, 50, 45, 52, 62, 67, 65, 56, 60, 
64, 60, 59, 77, 57, 31, 38, 55, 53, 22, 15, 34, 68, 67, 30, 31, 
62, 78, 53, 7, -14, -1, 0, -12, -6, 20, 60, 65, 49, 28, 18, 22, 
42, 46, 36, 20, 5, -1, 8, 12, 15, 19, 23, 34, 35, 28, 10, 2, 
12, 38, 36, 22, 9, 5, 19, 8, -4, -5, -3, 5, 4, 3, 13, 24, 
25, 13, -1, -7, -8, -3, 8, 24, 22, 5, -16, -26, -21, -1, 12, 20, 
24, 29, 31, 25, 20, 23, 34, 45, 45, 35, 26, 15, -2, -7, 14, 24, 
28, 27, 19, 16, 15, 14, 15, 21, 32, 32, 15, 1, -5, -3, 8, 18, 
15, 8, 2, 3, -6, -9, -11, -19, -20, -6, 5, 0, -1, 5, 5, -5, 
-26, -29, -21, -16, -17, -17, -14, -6, 4, 7, 12, 36, 42, 36, 35, 36, 
35, 31, 25, 27, 31, 30, 22, 13, 12, 19, 29, 32, 13, 6, 10, 15, 
20, 17, 7, 9, 18, 24, 21, 16, 14, 8, -1, -7, -7, 18, 32, 34, 
31, 27, 30, 43, 45, 37, 24, 19, 25, 26, 22, 18, 15, 9, 6, -1, 
-5, -4, 0, 5, 5, 2, -2, -2, 4, -5, -21, -30, -29, -25, -28, -50, 
-62, -60, -53, -47, -43, -39, -35, -37, -39, -32, -11, -4, -8, -14, -13, -11, 
-17, -29, -39, -44, -47, -50, -45, -37, -29, -25, -29, -31, -30, -29, -32, -43, 
-52, -51, -41, -34, -29, -28, -31, -36, -41, -44, -44, -37, -26, -22, -22, -24, 
-26, -31, -43, -56, -67, -75, -78, -81, -79, -62, -48, -50, -56, -75, -81, -81, 
-77, -63, -46, -24, -22, -23, -22, -30, -40, -35, -24, -13, -5, -2, -6, -11, 
-14, -17, -28, -45, -52, -46, -41, -42, -50, -50, -45, -41, -47, -61, -70, -87, 
-93, -89, -78, -62, -45, -23, -14, -10, -8, -9, -13, -17, -21, -16, -6, 3, 
7, 11, 15, 12, 7, -4, -11, -1, 12, 22, 23, 12, 4, -2, -11, -19, 
-16, -1, 7, 9, 4, 6, 16, 18, 13, 5, -6, -14, -29, -36, -37, -36, 
-30, -19, -11, -9, -13, -19, -24, -25, -25, -25, -26, -25, -17, -11, -13, -17, 
-25, -36, -38, -17, -1, 11, 15, 10, 1, -9, -7, -7, -5, -1, 13, 24, 
21, 16, 17, 16, 10, -7, -22, -25, -23, -17, -17, -13, 2, 15, 19, -3, 
-15, -18, -14, -7, -5, -2, -1, -6, -21, -31, -29, -19, -8, -3, 4, 11, 
9, 6, 9, 18, 26, 32, 26, 15, 3, -4, -6, -8, -5, -5, -6, -6, 
-5, 4, 9, 11, 11, 7, 5, 6, 7, 9, 9, 6, 4, -3, -6, 1, 
11, 21, 21, 18, 15, 12, 11, 12, 7, -1, -1, 4, 11, 16, 17, 14, 
5, -5, -8, 6, 17, 22, 25, 26, 25, 15, 8, 2, 0, 1, 7, 14, 
13, 5, 2, 0, -10, -13, -8, 7, 21, 26, 14, 2, 2, 13, 29, 52, 
62, 69, 70, 68, 65, 54, 58, 74, 89, 89, 76, 54, 50, 59, 77, 90, 
97, 95, 84, 67, 53, 45, 45, 51, 51, 46, 42, 37, 34, 32, 27, 24, 
25, 24, 17, 10, 7, 11, 15, 17, 12, 9, 13, 17, 12, 2, 14, 37, 
58, 67, 62, 53, 45, 43, 43, 46, 69, 79, 69, 56, 47, 44, 48, 53, 
58, 62, 64, 58, 54, 50, 46, 47, 43, 37, 33, 29, 26, 26, 35, 37, 
36, 32, 33, 45, 48, 35, 24, 22, 29, 39, 58, 59, 54, 51, 48, 46, 
48, 51, 55, 57, 50, 34, 23, 21, 23, 24, 27, 16, 6, -3, 0, 13, 
27, 27, 19, 8, 5, 6, -2, 7, 19, 24, 25, 27, 23, 21, 18, 20, 
28, 28, 34, 44, 42, 38, 35, 33, 29, 27, 30, 40, 47, 35, 25, 15, 
10, 12, 10, 4, -5, -11, -14, -14, 1, 7, 3, -6, -6, 2, 10, 9, 
8, 13, 11, -6, -13, -14, -8, 4, 17, 11, 9, 10, 5, -3, -12, -7, 
-6, -14, -20, -29, -40, -40, -36, -33, -35, -37, -52, -55, -53, -50, -45, -46, 
-55, -53, -54, -59, -53, -38, -33, -38, -45, -46, -46, -41, -37, -41, -48, -44, 
-26, -21, -25, -30, -37, -44, -48, -52, -61, -68, -65, -63, -74, -71, -57, -42, 
-39, -46, -45, -39, -26, -16, -22, -37, -40, -35, -29, -30, -31, -27, -23, -12, 
-3, -2, 6, 18, 24, 19, 5, -13, -35, -37, -26, -11, -3, -9, -23, -24, 
-25, -26, -25, -33, -41, -43, -37, -28, -22, -31, -44, -52, -55, -54, -50, -46, 
-44, -43, -45, -47, -36, -28, -27, -29, -30, -27, -12, -2, 3, 4, 5, 8, 
15, 12, 5, 2, 0, 2, -2, -7, 4, 32, 60, 65, 45, 26, 17, 21, 
33, 32, 27, 22, 9, -12, -42, -42, -32, -27, -25, -26, -16, -5, -4, -3, 
-2, 1, 4, -6, -16, -22, -18, -16, -23, -34, -48, -53, -43, -32, -36, -48, 
-54, -44, -26, -9, -4, -2, -3, -5, -6, -5, -5, -7, -6, -7, -19, -30, 
-38, -32, -18, -11, -21, -29, -28, -21, -14, -18, -19, -18, -21, -28, -38, -50, 
-50, -48, -46, -45, -53, -63, -64, -60, -56, -49, -54, -58, -60, -62, -63, -66, 
-68, -64, -64, -67, -73, -75, -72, -72, -68, -61, -51, -44, -44, -41, -29, -17, 
-17, -28, -30, -25, -15, -12, -18, -18, -13, -9, -11, -16, -16, -6, 17, 32, 
25, 8, 0, 3, 1, -1, -8, -26, -35, -40, -41, -34, -19, -11, -18, -21, 
-17, -12, -8, 10, 30, 45, 46, 41, 36, 31, 30, 36, 44, 48, 57, 70, 
90, 104, 102, 90, 81, 78, 77, 78, 75, 76, 70, 69, 76, 83, 79, 63, 
73, 90, 93, 76, 63, 87, 99, 93, 73, 58, 61, 62, 53, 41, 39, 47, 
50, 43, 41, 39, 42, 52, 58, 55, 52, 51, 54, 55, 52, 51, 53, 59, 
64, 52, 42, 44, 50, 60, 68, 71, 71, 66, 64, 59, 63, 82, 95, 84, 
58, 36, 27, 31, 33, 38, 42, 46, 47, 35, 18, 8, 6, 11, 10, 0, 
-13, -16, -6, 1, -7, -21, -35, -36, -24, -19, -16, -19, -29, -35, -27, -19, 
-14, -9, -4, -1, -4, -2, 1, 1, 7, 24, 22, 13, 8, 7, 3, -6, 
-11, -11, -5, 0, -1, -4, -5, -11, -25, -43, -56, -51, -40, -29, -26, -30, 
-29, -33, -41, -46, -45, -33, -5, -3, -13, -21, -20, -17, -13, -8, -6, 0, 
12, 22, 22, 24, 31, 38, 40, 41, 38, 26, 11, 9, 18, 23, 27, 25, 
23, 23, 13, 3, -6, -9, -7, 2, 13, 8, 1, 1, 9, 15, 1, -9, 
-9, -2, 2, 1, 4, 12, 17, 9, -4, -5, 9, 21, 25, 21, 12, 14, 
18, 19, 19, 20, 19, 13, 5, 1, 2, 5, 17, 22, 16, 5, -4, -1, 
9, 16, 12, 10, 11, 11, 17, 21, 25, 26, 27, 27, 25, 21, 13, 10, 
14, 13, 5, -6, -13, -12, -9, -15, -26, -33, -32, -29, -24, -14, -11, -17, 
-27, -45, -58, -64, -58, -49, -45, -45, -48, -52, -52, -46, -46, -51, -50, -45, 
-46, -42, -20, -15, -14, -14, -22, -33, -38, -32, -24, -27, -37, -36, -28, -24, 
-29, -38, -39, -35, -55, -81, -83, -66, -49, -40, -44, -46, -52, -56, -54, -57, 
-52, -43, -39, -42, -61, -70, -72, -67, -58, -50, -50, -57, -65, -66, -56, -38, 
-34, -41, -51, -62, -72, -78, -76, -69, -63, -61, -67, -82, -86, -90, -88, -76, 
-60, -57, -61, -65, -60, -54, -51, -58, -66, -69, -60, -47, -36, -37, -41, -43, 
-40, -37, -35, -27, -24, -21, -9, 19, 30, 29, 22, 17, 17, 22, 24, 19, 
14, 12, 2, -6, -8, -2, 6, 14, 13, 11, 13, 16, 20, 9, -6, -17, 
-31, -41, -43, -43, -39, -32, -24, -25, -28, -44, -49, -41, -28, -12, 5, 10, 
10, 3, -5, -11, -6, 10, 27, 33, 27, 21, 8, 4, 6, 10, 12, 16, 
20, 19, 14, 9, 5, 0, -1, -4, -6, -3, 1, -4, -10, -10, -5, 1, 
9, 15, 18, 18, 18, 15, 9, 5, 1, 1, -2, -2, 10, 9, -1, -14, 
-17, -12, -9, -12, -19, -22, -21, 6, 29, 44, 54, 58, 51, 30, 21, 30, 
38, 44, 46, 53, 55, 43, 41, 34, 49, 63, 67, 66, 62, 53, 43, 36, 
24, 28, 32, 43, 34, 21, 19, 20, 27, 37, 48, 67, 74, 72, 63, 42, 
43, 40, 40, 44, 60, 59, 48, 40, 29, 28, 37, 43, 45, 41, 42, 47, 
53, 65, 71, 65, 51, 38, 40, 39, 39, 46, 55, 66, 66, 54, 39, 31, 
31, 35, 35, 34, 33, 32, 38, 37, 40, 41, 30, 21, 21, 20, 11, 6, 
3, 14, 33, 39, 29, 14, 5, 8, 18, 29, 37, 44, 48, 40, 25, 13, 
8, 14, 22, 28, 36, 43, 51, 57, 56, 50, 41, 38, 43, 43, 34, 28, 
27, 19, 9, -2, -12, -14, -10, -8, -10, -9, -13, -15, -13, -10, -6, -11, 
-21, -36, -46, -49, -49, -43, -41, -39, -40, -42, -46, -46, -40, -29, -17, -7, 
1, -1, -4, -11, -22, -30, -31, -26, -19, -18, -20, -14, -10, -12, -15, -14, 
-12, -16, -16, -8, 3, 8, 15, 15, 6, -11, -30, -33, -7, 13, 19, 11, 
2, -7, -15, -21, -21, -4, 21, 41, 37, 34, 29, 20, 19, 21, 9, -1, 
3, 14, 17, 11, 4, 2, 5, 7, 3, 1, 7, 19, 23, 17, 10, 11, 
12, 8, 6, -1, -17, -22, -10, 1, -1, -18, -26, -18, -10, -16, -29, -32, 
-22, -17, -16, -15, -25, -41, -45, -34, -27, -23, -23, -20, -21, -27, -35, -39, 
-20, -5, 2, 6, 9, -3, -14, -11, 3, 19, 22, 11, 6, 0, -5, -9, 
-11, -5, 0, 4, 5, 4, -17, -36, -44, -36, -24, -25, -41, -53, -63, -71, 
-77, -75, -62, -51, -39, -32, -31, -34, -35, -35, -32, -27, -20, -15, -18, -24, 
-30, -30, -24, -29, -30, -27, -24, -23, -41, -51, -53, -42, -26, -19, -34, -57, 
-79, -95, -97, -92, -76, -65, -58, -52, -50, -51, -51, -46, -40, -35, -32, -36, 
-37, -35, -35, -48, -61, -61, -55, -48, -43, -39, -39, -45, -49, -45, -35, -22, 
-7, -1, 6, 3, -10, -28, -48, -66, -70, -68, -70, -70, -57, -37, -22, -21, 
-24, -8, 10, 32, 49, 51, 29, 10, -8, -21, -18, -11, -14, -19, -22, -21, 
-14, 2, 21, 26, 29, 37, 38, 10, 0, 6, 22, 26, 10, -12, -21, -23, 
-30, -43, -44, -31, -18, -9, -9, -5, 14, 21, 18, 6, -2, 6, 26, 29, 
23, 12, 1, -8, -14, -19, -26, -29, -24, -8, -3, 0, 2, 7, 11, 4, 
0, -2, 0, 9, 15, 2, -12, -26, -37, -43, -34, -26, -29, -32, -29, -20, 
-3, 6, 14, 18, 21, 17, 5, 4, 11, 18, 11, -4, -4, 4, 17, 25, 
27, 14, 4, 2, 2, 1, 2, 12, 18, 20, 17, 7, -4, -4, 0, 0, 
-4, -3, 1, 2, -2, -8, -6, -2, -1, 0, 3, 7, 17, 40, 45, 33, 
15, 9, 13, 18, 20, 22, 30, 37, 36, 26, 17, 24, 40, 56, 63, 55, 
48, 40, 32, 29, 28, 28, 29, 24, 22, 28, 31, 34, 34, 43, 51, 41, 
27, 22, 32, 47, 50, 37, 33, 29, 19, 8, 12, 21, 19, 9, 3, 7, 
23, 25, 22, 19, 20, 23, 23, 19, 20, 22, 21, 18, 19, 25, 27, 25, 
16, -1, -1, 5, 13, 19, 20, 20, 24, 29, 26, 21, 15, 12, 15, 21, 
24, 20, 12, 11, 14, 14, 7, 2, 5, 15, 28, 35, 31, 14, 1, -4, 
-4, -7, -12, -12, -9, -13, -24, -36, -37, -9, 12, 25, 26, 14, -10, -14, 
-7, 9, 32, 50, 43, 27, 9, -7, -13, -11, 1, 15, 30, 45, 61, 72, 
68, 61, 53, 51, 56, 56, 46, 35, 28, 32, 39, 38, 30, 23, 22, 25, 
16, 2, -2, 2, 14, 23, 20, 14, 5, 0, -3, -10, -7, 3, 7, 5, 
7, 11, 17, 19, 18, 12, 13, 29, 43, 53, 55, 52, 40, 36, 29, 19, 
14, 13, 24, 33, 40, 43, 45, 48, 45, 47, 53, 66, 76, 78, 70, 59, 
46, 29, 24, 41, 58, 53, 28, 4, -9, -1, 8, 17, 24, 23, 16, 12, 
10, 3, -2, 1, 26, 32, 25, 6, -9, -18, -23, -15, -12, -4, 2, 14, 
20, 22, 20, 16, 19, 14, 2, -12, -20, -21, -19, -19, -10, -6, -10, -16, 
-29, -32, -34, -30, -22, -14, -7, -4, -8, -15, -17, -15, -15, -19, -21, -24, 
-26, -16, -8, -4, -5, -5, -4, 2, 11, 17, 19, 13, 0, -14, -20, -16, 
-10, -7, -11, -29, -55, -74, -74, -58, -28, -26, -42, -61, -72, -74, -77, -81, 
-82, -72, -56, -49, -52, -55, -60, -65, -69, -71, -66, -53, -41, -34, -35, -46, 
-53, -54, -54, -54, -51, -55, -69, -84, -85, -73, -55, -51, -52, -53, -52, -54, 
-57, -49, -37, -24, -16, -21, -26, -29, -26, -27, -30, -24, -20, -18, -21, -21, 
-9, 1, 2, -15, -36, -49, -36, -24, -17, -18, -18, -7, -4, -13, -19, -26, 
-29, -28, -32, -37, -30, -16, -3, 0, -9, -16, -21, -11, 2, 4, 1, -2, 
6, 14, 14, 8, 0, -1, -1, -2, -3, -2, 4, 9, 8, 3, -1, 0, 
6, 15, 17, 20, 17, 18, 22, 27, 28, 20, 10, 1, 1, 6, 14, 29, 
23, 16, 17, 22, 19, 12, 8, 5, 2, -5, -19, -27, -31, -30, -26, -20, 
-8, -7, -10, -10, -4, 4, 2, -2, -8, -10, -6, -10, -8, -2, 2, 6, 
12, 14, 4, -11, -20, -22, -21, -16, -14, -9, 4, 17, 11, -9, -27, -34, 
-35, -34, -31, -27, -21, -12, -3, -1, -4, -6, -6, -3, -1, 7, 20, 27, 
22, 16, -3, -21, -24, -18, -10, -2, 7, 26, 39, 24, 5, 2, 6, 4, 
1, 1, 9, 18, 11, 9, 1, -13, -14, -5, 4, 14, 19, 23, 31, 40, 
44, 39, 32, 33, 46, 37, 16, 10, 22, 42, 45, 29, 11, 2, 3, 0, 
-7, -4, 6, 19, 26, 29, 30, 23, 14, 15, 18, 16, 17, 13, 11, 17, 
19, 17, 18, 16, 15, 19, 28, 38, 43, 39, 35, 34, 29, 25, 19, 20, 
28, 31, 22, 10, 7, 11, 15, 18, 18, 20, 33, 47, 52, 43, 34, 34, 
39, 40, 35, 26, 23, 21, 26, 37, 50, 46, 35, 27, 24, 26, 33, 42, 
48, 48, 46, 39, 20, 15, 23, 34, 37, 20, 7, 5, 8, 4, 0, 7, 
15, 15, 6, 1, 19, 36, 40, 29, 16, 15, 22, 11, 2, 4, 9, 11, 
14, 20, 27, 26, 19, 15, 22, 30, 31, 29, 32, 42, 38, 30, 28, 30, 
30, 28, 19, 8, 1, -2, 9, 17, 24, 28, 27, 19, 8, 14, 31, 48, 
50, 43, 32, 16, 1, -11, -17, -8, 0, 1, -3, -5, -6, -10, -12, -12, 
-7, -2, };
