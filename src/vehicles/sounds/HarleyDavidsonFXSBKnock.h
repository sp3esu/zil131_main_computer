const unsigned int knockSampleRate = 22050;
const unsigned int knockSampleCount = 4413;
const signed char knockSamples[] = {
0, 0, 0, 1, 1, 1, 1, 1, 0, -1, -2, 0, -1, -1, 0, 0, 
0, -13, -17, -13, 1, 11, 13, -3, -12, -14, -43, -53, -23, -1, 31, 18, 
7, -8, -64, -32, -41, -13, 27, -1, -7, -31, -20, -15, -20, 20, -13, -35, 
-57, -29, 24, 32, 48, -15, -74, -77, -50, 45, 31, 26, 8, -73, -115, -56, 
71, 90, 57, 75, -23, -108, -101, -21, 32, 36, 27, 28, -10, -29, 19, 54, 
36, -11, -26, -43, -59, 4, 22, 83, 77, 28, 25, -68, -58, -60, -43, -9, 
-23, 38, 16, 34, 3, -88, -56, -73, 6, 64, 26, 52, 2, -36, 36, 34, 
32, 104, 53, 54, -15, -74, 15, 28, 83, 121, 91, 116, 74, 19, 48, 49, 
97, 125, 101, 76, 19, 10, 81, 94, 115, 89, 19, 28, 24, 13, 111, 113, 
109, 112, 66, 59, 29, -18, 0, -1, 3, 49, 37, 47, 65, 49, 15, -2, 
-4, -18, -26, -15, 3, 41, 49, 35, 30, -7, -55, -42, -48, -34, -2, 1, 
4, -5, -18, -35, -43, -26, -29, -28, -24, -48, -64, -53, -55, -32, -33, -51, 
-57, -78, -69, -67, -69, -72, -61, -62, -80, -86, -86, -94, -93, -88, -77, -70, 
-84, -72, -68, -103, -107, -107, -111, -98, -96, -65, -77, -106, -79, -97, -80, -67, 
-86, -84, -98, -107, -80, -77, -53, -38, -70, -73, -85, -76, -74, -89, -63, -65, 
-60, -46, -38, -28, -34, -41, -57, -66, -75, -85, -53, -37, -27, -1, -1, -14, 
-30, -46, -28, -36, -55, -27, -46, -35, -11, -5, 27, 10, -2, -2, -21, -31, 
-22, -23, -11, -8, -1, 23, 23, 29, 14, -8, -12, -28, -20, 4, 19, 24, 
31, 25, 30, 23, 17, 25, -2, 13, 12, 4, 18, 15, 35, 30, 25, 32, 
13, 15, 21, 23, 34, 27, 43, 54, 52, 57, 49, 52, 51, 51, 52, 43, 
44, 53, 62, 75, 86, 89, 81, 74, 59, 62, 72, 74, 87, 100, 87, 86, 
76, 69, 70, 68, 89, 90, 84, 88, 81, 82, 86, 83, 78, 66, 64, 67, 
71, 86, 92, 81, 73, 72, 67, 63, 56, 51, 39, 46, 59, 63, 66, 58, 
52, 50, 48, 54, 45, 40, 27, 20, 36, 38, 44, 43, 37, 38, 24, 20, 
28, 19, 22, 25, 22, 29, 31, 29, 25, 13, 15, 14, 18, 24, 21, 17, 
18, 27, 30, 30, 30, 21, 15, 15, 23, 29, 25, 27, 27, 22, 26, 22, 
21, 21, 18, 17, 17, 20, 20, 19, 18, 9, 9, 7, 2, 2, -8, -3, 
-1, 2, 10, 2, -6, -15, -23, -22, -18, -19, -20, -19, -22, -25, -27, -30, 
-34, -33, -30, -33, -38, -35, -38, -36, -32, -36, -40, -48, -52, -48, -47, -48, 
-51, -57, -55, -54, -48, -37, -43, -44, -48, -53, -54, -56, -53, -55, -54, -50, 
-49, -49, -50, -54, -58, -58, -57, -53, -48, -48, -47, -43, -40, -40, -46, -51, 
-54, -58, -57, -59, -55, -52, -53, -53, -58, -61, -62, -63, -62, -58, -59, -62, 
-65, -68, -69, -75, -73, -69, -67, -61, -59, -63, -66, -69, -70, -67, -64, -60, 
-59, -60, -59, -60, -63, -62, -60, -58, -55, -55, -53, -51, -51, -50, -52, -48, 
-47, -42, -40, -42, -41, -39, -35, -30, -30, -30, -29, -28, -27, -26, -25, -26, 
-28, -26, -27, -24, -20, -20, -15, -13, -12, -7, -6, -5, 0, -1, 3, 5, 
3, 6, 4, 8, 11, 8, 8, 9, 14, 21, 23, 24, 24, 22, 22, 23, 
27, 30, 31, 32, 32, 35, 39, 42, 47, 48, 50, 53, 57, 60, 60, 63, 
64, 67, 70, 68, 69, 70, 74, 76, 76, 83, 84, 86, 88, 87, 87, 84, 
86, 89, 91, 92, 92, 93, 92, 92, 93, 93, 96, 96, 95, 96, 93, 89, 
90, 89, 90, 90, 88, 89, 86, 83, 82, 82, 81, 76, 74, 74, 73, 74, 
74, 69, 64, 60, 58, 56, 55, 54, 51, 49, 47, 46, 40, 37, 35, 35, 
36, 32, 27, 25, 24, 21, 18, 15, 14, 11, 9, 10, 6, 2, -2, -8, 
-9, -11, -14, -16, -22, -25, -28, -33, -32, -34, -38, -44, -47, -48, -53, -53, 
-57, -60, -65, -73, -71, -73, -77, -73, -75, -73, -72, -77, -75, -73, -74, -73, 
-76, -76, -73, -76, -72, -73, -73, -71, -74, -73, -71, -70, -66, -67, -70, -69, 
-68, -64, -62, -60, -58, -54, -51, -50, -48, -47, -45, -43, -44, -42, -41, -42, 
-40, -37, -34, -31, -29, -28, -28, -28, -28, -27, -24, -22, -12, -8, -7, -7, 
-12, -14, -10, -5, 0, 2, 2, 1, 0, 4, 7, 12, 16, 17, 19, 21, 
23, 28, 29, 33, 34, 33, 33, 30, 33, 32, 31, 35, 35, 36, 35, 34, 
35, 34, 34, 39, 41, 42, 42, 41, 41, 37, 33, 32, 31, 32, 32, 29, 
30, 32, 33, 34, 39, 39, 38, 40, 40, 39, 34, 34, 36, 36, 39, 39, 
35, 37, 35, 36, 38, 35, 32, 33, 33, 35, 39, 37, 37, 36, 36, 36, 
33, 32, 29, 27, 27, 25, 24, 23, 23, 23, 20, 19, 18, 18, 19, 19, 
18, 18, 15, 12, 8, 5, 4, 4, 2, 0, -3, -4, -6, -8, -7, -8, 
-7, -5, -5, -7, -10, -11, -13, -14, -14, -16, -16, -18, -18, -16, -13, -12, 
-15, -12, -11, -9, -7, -8, -8, -9, -10, -13, -16, -15, -16, -17, -16, -20, 
-19, -18, -18, -15, -16, -15, -14, -15, -16, -16, -19, -19, -22, -24, -23, -23, 
-20, -21, -21, -20, -22, -21, -23, -23, -25, -30, -27, -30, -27, -24, -24, -20, 
-17, -14, -12, -14, -14, -9, -16, -14, -11, -13, -6, -8, -7, 6, 1, -1, 
7, -10, -16, -15, -19, 4, -9, -34, -15, -10, 9, 11, -23, -13, -9, -8, 
9, -17, -61, -45, -27, 21, 60, -1, -37, -55, -51, -18, -10, 10, -19, -52, 
-29, 45, 19, -40, -94, -92, -22, -16, 81, -4, -59, -92, -64, 60, 5, 17, 
-55, -15, -51, -37, 64, -28, 22, -41, 52, 65, 26, -8, -26, -72, -94, 8, 
-18, 49, 14, 53, 64, 25, -17, -70, 27, 3, -43, -34, -45, 57, 80, 62, 
95, -52, -127, 5, 110, 93, 16, -84, -30, 27, 87, 120, 16, -5, -13, 85, 
110, 72, -47, -46, 42, 70, 116, 20, 15, 20, -15, 32, 81, 39, 67, 58, 
70, 125, 46, 51, 8, -26, 30, 5, 25, 50, 61, 87, 100, 78, 18, -16, 
0, 34, 38, 3, -27, 29, 39, 58, 50, 7, -18, -17, 51, 35, 2, -57, 
-55, -4, 12, 21, -18, 5, 16, -13, -9, -17, -27, -22, -5, -11, -31, -30, 
-32, -7, -9, -26, -45, -47, -24, -45, -21, -19, -42, -23, -28, -29, -52, -86, 
-75, -66, -48, -55, -70, -54, -75, -43, -16, -43, -67, -88, -84, -53, -46, -49, 
-64, -106, -113, -117, -93, -54, -38, -30, -67, -76, -93, -108, -92, -92, -84, -71, 
-73, -80, -81, -58, -56, -80, -87, -85, -100, -76, -51, -44, -45, -61, -73, -61, 
-53, -55, -49, -45, -71, -56, -39, -30, -25, -38, -35, -38, -35, -30, -37, -33, 
-39, -38, -14, -6, -10, -22, -38, -37, -33, -14, -8, -11, -8, -16, -13, -9, 
-19, -15, 0, -1, -5, 2, 13, 19, 19, 22, 9, 1, 11, 18, 27, 25, 
31, 44, 42, 35, 32, 45, 52, 62, 60, 55, 54, 52, 62, 66, 64, 59, 
55, 64, 74, 74, 79, 84, 79, 72, 75, 81, 84, 86, 82, 79, 82, 80, 
80, 76, 74, 80, 85, 82, 74, 73, 68, 72, 74, 72, 64, 55, 64, 66, 
70, 66, 61, 58, 56, 58, 54, 56, 54, 50, 56, 55, 52, 47, 52, 62, 
63, 64, 61, 58, 56, 56, 60, 62, 63, 61, 60, 59, 58, 62, 64, 63, 
54, 51, 50, 50, 56, 58, 61, 65, 61, 56, 50, 47, 45, 46, 46, 42, 
40, 45, 47, 44, 39, 29, 23, 26, 27, 28, 29, 27, 23, 21, 16, 10, 
7, 2, 3, 6, 0, -1, 0, -3, -8, -12, -19, -22, -21, -18, -12, -13, 
-18, -17, -17, -21, -25, -26, -27, -29, -29, -32, -32, -32, -34, -36, -36, -36, 
-37, -37, -37, -41, -42, -44, -48, -49, -50, -50, -49, -49, -50, -56, -60, -61, 
-63, -66, -67, -67, -67, -62, -62, -64, -67, -71, -72, -75, -77, -77, -74, -76, 
-78, -79, -81, -80, -78, -78, -76, -76, -76, -76, -76, -75, -77, -76, -75, -71, 
-68, -64, -63, -66, -64, -62, -62, -61, -64, -60, -56, -55, -56, -56, -52, -49, 
-49, -51, -55, -52, -47, -46, -45, -44, -42, -47, -49, -48, -47, -45, -46, -43, 
-42, -41, -40, -38, -36, -36, -33, -29, -27, -25, -25, -27, -24, -25, -25, -24, 
-23, -20, -19, -14, -4, 1, 4, 5, 10, 11, 10, 16, 19, 21, 24, 31, 
39, 41, 40, 41, 44, 49, 53, 58, 62, 62, 60, 65, 67, 66, 63, 64, 
71, 72, 75, 75, 76, 76, 73, 73, 72, 71, 72, 76, 79, 77, 76, 75, 
76, 73, 72, 73, 73, 76, 78, 75, 73, 69, 68, 66, 64, 64, 64, 63, 
59, 59, 59, 57, 57, 55, 53, 53, 52, 49, 46, 42, 36, 30, 29, 28, 
27, 27, 24, 21, 18, 15, 11, 6, 2, -6, -9, -11, -15, -17, -17, -20, 
-24, -27, -26, -32, -40, -39, -38, -38, -42, -47, -53, -53, -52, -54, -55, -57, 
-63, -62, -62, -60, -60, -65, -65, -65, -66, -67, -66, -63, -65, -65, -65, -64, 
-61, -58, -57, -58, -58, -56, -51, -50, -52, -50, -48, -46, -44, -41, -35, -29, 
-26, -24, -23, -21, -19, -18, -16, -15, -12, -6, -1, 1, 4, 13, 15, 18, 
21, 24, 27, 29, 32, 35, 40, 43, 44, 43, 43, 48, 51, 54, 60, 57, 
59, 61, 61, 64, 64, 64, 61, 64, 66, 62, 62, 62, 63, 61, 61, 58, 
56, 59, 56, 52, 48, 44, 42, 41, 40, 38, 38, 35, 34, 31, 25, 22, 
14, 15, 12, 7, 9, 7, 7, 5, 0, -4, -10, -14, -17, -18, -20, -24, 
-26, -25, -26, -27, -28, -31, -37, -40, -42, -41, -42, -46, -42, -40, -38, -39, 
-43, -43, -47, -48, -46, -50, -50, -47, -44, -40, -40, -43, -42, -41, -38, -35, 
-36, -35, -34, -31, -29, -26, -24, -23, -24, -22, -21, -19, -15, -15, -13, -11, 
-10, -8, -5, -2, -2, -2, -4, -4, 0, -2, 1, 2, 3, 7, 9, 10, 
10, 9, 9, 13, 16, 15, 13, 14, 15, 16, 21, 19, 22, 22, 18, 20, 
23, 24, 24, 24, 24, 27, 27, 29, 29, 29, 31, 31, 34, 31, 28, 30, 
30, 34, 35, 35, 34, 34, 36, 37, 35, 35, 35, 33, 34, 31, 31, 33, 
32, 33, 30, 26, 26, 27, 27, 27, 24, 24, 23, 23, 23, 21, 23, 20, 
15, 15, 13, 13, 14, 11, 10, 9, 9, 12, 11, 9, 6, 5, 4, 5, 
5, 4, 2, 3, 1, 0, 0, -2, -1, -3, -5, -5, -4, -4, -4, -6, 
-8, -9, -11, -11, -11, -11, -11, -10, -10, -11, -13, -13, -13, -13, -15, -16, 
-17, -17, -15, -15, -17, -18, -16, -16, -16, -18, -21, -19, -21, -19, -18, -21, 
-20, -19, -19, -18, -20, -21, -21, -22, -20, -19, -20, -18, -19, -18, -20, -20, 
-22, -26, -24, -25, -22, -20, -22, -20, -22, -21, -20, -23, -23, -22, -20, -17, 
-16, -16, -16, -18, -17, -13, -14, -17, -17, -17, -14, -13, -13, -13, -13, -13, 
-12, -9, -7, -7, -5, -6, -8, -8, -6, -4, -5, -6, -5, -2, 0, -2, 
-2, -2, -2, -1, 0, 2, 1, 2, 1, 2, -1, 0, 3, 3, 5, 3, 
2, 0, 0, 1, 1, 1, 1, 1, 2, 4, 6, 6, 3, 3, 2, 3, 
6, 4, 4, 4, 3, 4, 5, 5, 4, 3, 3, 4, 5, 4, 3, 2, 
2, 3, 2, 3, 2, 1, 1, 3, 5, 4, 4, 2, 1, 4, 1, 2, 
1, -1, 1, 5, 2, -1, -1, 2, 2, -1, -2, -4, -3, -3, -4, -4, 
-3, -5, -5, -3, -2, -1, -2, -3, -4, -5, -3, -3, -3, -5, -7, -4, 
-1, -2, -4, -5, -7, -4, -3, -5, -3, -4, -7, -3, -3, -4, -4, -3, 
0, -1, 0, -1, -1, 1, 0, 2, 2, 1, 1, -1, 1, 1, 1, 2, 
2, 5, 5, 6, 7, 7, 5, 3, 3, 5, 6, 6, 6, 8, 7, 5, 
6, 6, 7, 7, 6, 7, 6, 7, 9, 10, 10, 10, 8, 10, 9, 9, 
9, 7, 9, 7, 8, 10, 9, 8, 8, 7, 9, 9, 8, 11, 10, 10, 
11, 9, 7, 8, 8, 11, 9, 8, 9, 8, 8, 6, 7, 8, 8, 9, 
7, 8, 8, 8, 9, 9, 7, 5, 4, 3, 4, 4, 4, 3, 0, -2, 
-1, 1, 1, 2, 3, 4, 3, 4, 9, 11, 11, 11, 10, 11, 10, 7, 
4, 2, -1, -4, -2, 0, -2, -11, -12, -10, -7, -7, -12, -6, -7, -5, 
0, -6, -3, -17, -20, -12, -18, -20, -29, -28, -12, -1, 7, 10, -23, -49, 
-58, -55, -16, 11, 22, 15, -32, -21, -11, -15, -1, -21, -13, -24, 9, 19, 
1, 2, -36, -38, -30, -15, 10, 16, 20, 2, -24, -36, -24, -29, -3, 51, 
54, 59, 7, -48, -48, -90, -14, 40, 17, 42, 14, 43, 15, -26, -27, -33, 
-35, -51, 9, 4, 20, 32, -29, 20, -8, 15, 51, 1, -37, -50, -27, 23, 
60, 18, 26, -14, -27, 9, 29, 66, 41, 20, -47, -35, -15, 64, 120, 90, 
98, 57, 13, 28, 50, 71, 71, 14, 14, 46, 58, 103, 75, 49, 34, 4, 
62, 71, 61, 66, 38, 53, 68, 91, 111, 113, 97, 48, -5, 8, 46, 72, 
85, 67, 66, 36, 21, 58, 85, 85, 76, 57, -22, -23, -29, 0, 31, 12, 
-7, -36, -14, -2, -2, -7, -50, -51, -51, -37, -15, -17, -47, -68, -60, -48, 
-30, -43, -54, -86, -104, -96, -71, -45, -43, -47, -62, -75, -75, -65, -57, -61, 
-71, -92, -81, -69, -51, -42, -51, -73, -76, -76, -79, -63, -62, -78, -79, -80, 
-73, -64, -53, -59, -68, -77, -79, -72, -66, -71, -80, -72, -60, -53, -42, -53, 
-54, -65, -68, -46, -41, -46, -65, -70, -63, -39, -26, -27, -37, -49, -59, -65, 
-50, -39, -32, -29, -18, -19, -29, -30, -29, -28, -29, -29, -32, -43, -55, -52, 
-37, -18, -3, -8, -7, -13, -2, 2, -1, 4, -14, -22, -21, -16, 3, 6, 
3, 0, -3, 5, 6, 7, 13, 7, 4, 17, 28, 35, 31, 23, 25, 11, 
11, 32, 40, 38, 27, 17, 2, 1, 20, 31, 39, 42, 37, 27, 22, 28, 
35, 36, 34, 25, 27, 30, 37, 50, 49, 46, 42, 39, 54, 63, 75, 76, 
59, 54, 47, 46, 52, 56, 67, 65, 61, 71, 73, 74, 69, 63, 63, 60, 
62, 69, 64, 54, 51, 53, 61, 61, 69, 78, 65, 74, 68, 59, 50, 41, 
48, 39, 44, 58, 61, 54, 50, 40, 37, 34, 39, 62, 51, 43, 35, 39, 
37, 33, 47, 37, 33, 34, 47, 50, 36, 39, 37, 26, 40, 44, 42, 34, 
28, 33, 35, 39, 41, 34, 27, 17, 21, 24, 33, 48, 48, 49, 45, 44, 
44, 33, 31, 39, 38, 36, 34, 36, 32, 37, 43, 40, 33, 27, 18, 11, 
16, 25, 20, 18, 11, 11, 15, 13, 11, 6, -3, -19, -23, -21, -22, -10, 
-5, -6, -12, -19, -23, -32, -36, -41, -46, -42, -41, -40, -36, -31, -24, -25, 
-34, -46, -54, -61, -64, -56, -49, -42, -43, -45, -45, -48, -49, -54, -60, -60, 
-56, -57, -59, -62, -64, -64, -59, -57, -61, -65, -71, -75, -77, -73, -67, -65, 
-62, -63, -67, -72, -74, -75, -77, -76, -78, -81, -79, -78, -78, -76, -76, -77, 
-81, -83, -83, -78, -75, -77, -79, -78, -77, -78, -80, -84, -85, -84, -77, -74, 
-71, -71, -73, -71, -69, -67, -68, -67, -66, -66, -61, -57, -48, -40, -40, -44, 
-48, -50, -48, -47, -42, -35, -32, -28, -27, -27, -27, -22, -19, -20, -18, -19, 
-19, -18, -17, -15, -12, -11, -9, -9, -7, -5, -4, 4, 8, 11, 12, 12, 
11, 11, 11, 14, 15, 16, 18, 24, 27, 28, 34, 36, 43, 48, 49, 53, 
53, 51, 57, 59, 64, 67, 68, 67, 62, 66, 70, 77, 84, 90, 94, 96, 
99, 100, 99, 95, 93, 93, 94, 98, 100, 102, 107, 110, 113, 112, 112, 111, 
109, 107, 107, 105, 103, 101, 99, 95, 93, 90, 88, 87, 85, 87, 90, 92, 
92, 90, 88, 85, 83, 82, 77, 71, 69, 65, 63, 63, 62, 62, 60, 57, 
52, 48, 45, 41, 38, 38, 33, 33, 31, 26, 24, 18, 15, 9, 0, -3, 
-6, -10, -9, -13, -16, -21, -24, -24, -28, -32, -37, -40, -47, -51, -53, -55, 
-59, -64, -69, -75, -81, -83, -84, -83, -81, -84, -87, -90, -89, -85, -83, -83, 
-84, -87, -94, -94, -98, -97, -95, -96, -94, -90, -84, -81, -78, -77, -83, -79, 
-79, -80, -79, -81, -82, -79, -75, -75, -75, -73, -70, -65, -64, -63, -62, -61, 
-58, -55, -52, -51, -50, -54, -54, -50, -44, -37, -34, -37, -38, -35, -30, -25, 
-20, -15, -12, -10, -3, -2, -3, -3, -1, 4, 9, 15, 18, 13, 12, 14, 
15, 21, 22, 25, 28, 31, 38, 44, 49, 50, 48, 46, 43, 42, 41, 42, 
43, 44, 48, 46, 46, 47, 46, 47, 46, 45, 44, 41, 42, 41, 41, 41, 
42, 42, 41, 36, 31, 29, 29, 28, 26, 30, 30, 32, 36, 37, 38, 39, 
38, 36, 35, 34, 35, 37, 38, 38, 37, 36, 37, 37, 32, 30, 31, 34, 
36, 36, 36, 37, 38, 42, 41, 37, 36, 32, 30, 30, 27, 24, 20, 19, 
21, 19, 20, 19, 18, 18, 16, 17, 15, 11, 11, 8, 5, 5, 1, -2, 
-4, -6, -6, -6, -3, -3, -5, -7, -7, -5, -7, -8, -9, -14, -13, -13, 
-14, -16, -17, -15, -13, -11, -11, -14, -17, -16, -14, -13, -11, -8, -8, -8, 
-10, -11, -14, -16, -19, -20, -18, -17, -16, -17, -18, -19, -20, -21, -21, -21, 
-21, -22, -23, -23, -25, -25, -27, -28, -27, -28, -29, -31, -32, -32, -33, -31, 
-30, -29, -27, -29, -31, -32, -31, -29, -28, -28, -26, -24, -24, -24, -24, -26, 
-23, -22, -22, -21, -19, -18, -18, -16, -16, -15, -13, -15, -14, -13, -9, -7, 
-7, -4, -4, -4, -5, -3, -2, -4, -3, -3, -1, -1, -2, 0, -2, -5, 
-6, -6, -6, -2, 2, 2, 4, 3, -3, -4, -7, -8, -7, -5, -1, 1, 
1, 1, 4, 6, 6, 8, 9, 6, 5, 10, 11, 11, 12, 11, 11, 14, 
15, 15, 16, 17, 17, 18, 15, 16, 18, 18, 20, 20, 18, 18, 16, 14, 
15, 15, 16, 17, 18, 19, 19, 17, 16, 13, 11, 11, 8, 10, 8, 5, 
6, 2, 3, 2, 0, 1, -1, -2, -3, -5, -4, -5, -6, -5, -6, -8, 
-9, -10, -10, -9, -8, -10, -11, -11, -13, -12, -11, -9, -8, -11, -11, -12, 
-11, -10, -9, -8, -6, -5, -7, -9, -10, -10, -9, -9, -9, -7, -7, -7, 
-5, -6, -7, -4, -4, -4, -3, -4, -5, -5, -7, -7, -5, -4, -2, -3, 
-4, -4, -4, -2, -4, -6, -6, -5, -4, -3, -4, -5, -5, -6, -4, -3, 
-5, -5, -2, -3, -5, -4, -5, -4, -2, -2, -2, -1, -2, -2, -1, 0, 
0, 1, 2, 3, 3, 2, 2, 4, 5, 3, 3, 3, 3, 4, 6, 7, 
6, 6, 6, 6, 6, 7, 5, 5, 6, 7, 7, 6, 7, 7, 6, 4, 
4, 5, 5, 9, 7, 7, 8, 8, 6, 5, 5, 5, 7, 8, 7, 7, 
7, 8, 7, 5, 5, 5, 6, 7, 5, 5, 6, 6, 9, 9, 9, 13, 
12, 11, 11, 12, 11, 10, 10, 9, 10, 10, 5, 7, 9, 9, 13, 13, 
13, 16, 14, 12, 10, 8, 10, 12, 14, 11, 9, 12, 12, 13, 10, 8, 
7, 6, 5, 4, 7, 9, 9, 8, 6, 6, 6, 9, 9, 10, 9, 7, 
9, 9, 6, 4, 4, 5, 7, 8, 10, 10, 10, 10, 11, 12, 9, 7, 
4, 2, 3, 4, 5, 6, 7, 7, 8, 5, 4, 2, 0, 1, 1, 2, 
3, 4, 5, 2, 2, 2, 2, 2, 1, 2, 2, -2, -4, -6, -9, -7, 
-11, -11, -10, -8, -5, -4, -4, -7, -12, -17, -16, -14, -14, -12, -13, -15, 
-14, -16, -20, -22, -22, -23, -20, -18, -16, -15, -18, -18, -21, -23, -23, -22, 
-21, -18, -18, -19, -21, -22, -22, -24, -24, -22, -21, -18, -18, -18, -20, -23, 
-22, -22, -23, -21, -21, -21, -20, -20, -23, -21, -21, -19, -17, -19, -19, -21, 
-22, -22, -24, -24, -23, -21, -17, -16, -17, -20, -23, -23, -23, -24, -21, -19, 
-18, -15, -16, -17, -21, -24, -23, -20, -18, -16, -12, -12, -12, -10, -11, -10, 
-11, -12, -10, -8, -8, -6, -7, -8, -7, -5, -1, -1, 0, 0, 1, 3, 
3, 3, 3, 1, 3, 7, 7, 11, 13, 14, 18, 16, 15, 14, 11, 11, 
13, 16, 19, 21, 23, 24, 24, 26, 22, 23, 24, 23, 29, 30, 28, 31, 
31, 32, 33, 33, 34, 37, 39, 41, 45, 48, 47, 48, 49, 42, 43, 44, 
41, 41, 44, 46, 49, 52, 52, 51, 51, 47, 48, 46, 39, 39, 38, 41, 
44, 44, 47, 45, 44, 44, 41, 39, 38, 33, 29, 26, 25, 28, 29, 31, 
33, 32, 29, 26, 21, 18, 14, 14, 12, 8, 6, 1, 1, 0, -3, -4, 
-8, -10, -12, -11, -9, -11, -15, -18, -20, -23, -26, -29, -34, -38, -37, -36, 
-35, -33, -34, -35, -38, -39, -42, -48, -51, -56, -56, -55, -51, -50, -49, -49, 
-51, -51, -55, -58, -60, -63, -62, -60, -57, -54, -56, -56, -55, -58, -57, -60, 
-65, -66, -67, -66, -64, -64, -65, -65, -60, -56, -57, -57, -56, -54, -51, -47, 
-47, -48, -44, -38, -32, -28, -28, -29, -31, -31, -25, -18, -14, -10, -6, 4, 
10, 10, 10, 10, 9, 13, 19, 28, 34, 38, 39, 45, 46, 46, 49, 50, 
57, 59, 63, 71, 72, 74, 72, 69, 70, 70, 73, 77, 75, 77, 78, 80, 
83, 77, 75, 72, 70, 72, 76, 77, 74, 71, 70, 71, 70, 68, 64, 58, 
53, 52, 47, 46, 45, 43, 42, 38, 33, 31, 24, 21, 18, 13, 11, 5, 
4, 1, -3, -5, -10, -12, -13, -18, -23, -26, -28, -29, -31, -32, -37, -40, 
-41, -42, -45, -50, -53, -56, -58, -58, -59, -62, -65, -66, -66, -66, -65, -66, 
-68, -68, -69, -70, -70, -68, -64, -64, -66, -63, -62, -56, -54, -54, -55, -55, 
-50, -47, -44, -40, -36, -33, -30, -28, -27, -25, -21, -18, -12, -7, -3, 1, 
3, 8, 11, 13, 19, 23, 26, 32, 34, 37, 38, 38, 40, 42, 44, 45, 
48, 50, 55, 57, 55, 51, 50, 51, 50, 54, 55, 54, 51, 49, 47, 45, 
45, 45, 45, 40, 39, 36, 32, 31, 28, 25, 23, 22, 21, 16, 12, 4, 
1, 2, 2, -1, -5, -10, -12, -12, -13, -15, -18, -18, -20, -19, -18, -21, 
-24, -25, -25, -25, -25, -32, -33, -32, -31, -29, -27, -21, -20, -18, -21, -23, 
-22, -26, -23, -20, -17, -12, -9, -5, -6, -7, -6, -7, -5, -5, -5, -3, 
-4, -2, 2, 6, 7, 6, 6, 9, 12, 14, 14, 12, 10, 8, 11, 14, 
15, 13, 14, 16, 17, 21, 23, 21, 19, 17, 19, 19, 20, 23, 22, 24, 
23, 20, 22, 20, 21, 21, 20, 22, 23, 24, 24, 21, 21, 19, 19, 21, 
20, 21, 21, 21, 18, 17, 17, 16, 15, 14, 12, 13, 13, 10, 10, 9, 
7, 7, 6, 9, 7, 3, 2, 1, 0, -4, -4, -4, -4, -4, -6, -7, 
-6, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -16, -16, -19, -18, 
-16, -19, -20, -21, -22, -22, -23, -22, -21, -22, -22, -23, -23, -20, -22, -24, 
-24, -26, -25, -23, -23, -22, -22, -22, -19, -19, -18, -18, -17, -16, -17, -16, 
-17, -16, -14, -14, -12, -12, -12, -13, -12, -6, -5, -6, -7, -8, -8, -6, 
-6, -6, -6, -6, -4, -2, -2, -5, -5, -4, -3, -1, 0, 0, -1, -2, 
0, -2, 0, -1, 0, 3, 2, 3, 1, 1, 2, 3, 6, 6, 4, 3, 
3, 2, 1, 1, 2, 3, 6, 5, 5, 7, 4, 2, 2, 2, 6, 7, 
7, 9, 9, 10, 10, 7, 6, 6, 5, 6, 7, 7, 6, 6, 5, 6, 
6, 5, 5, 5, 7, 7, 7, 6, 1, 2, 3, 3, 4, 4, 5, 4, 
4, 3, 3, 2, 3, 3, 3, 5, 4, 4, 0, 0, 1, -2, -2, -1, 
-2, 0, 1, 1, -2, -4, -1, -1, -2, -3, -5, -3, -3, -6, -6, -5, 
-4, -4, -6, -4, -3, -3, -3, -2, -2, -3, -4, -2, -2, -3, -5, -6, 
-6, -7, -8, -7, -5, -7, -9, -6, -6, -5, -6, -5, -5, -5, -8, -10, 
-7, -6, -6, -6, -6, -5, -5, -6, -6, -6, -9, -9, -7, -6, -5, -7, 
-8, -5, -5, -6, -5, -5, -5, -6, -6, -5, -3, -3, -1, -2, -3, -1, 
0, 1, 2, 0, 2, 3, 5, 3, 1, 4, 4, 2, 0, 1, 3, 4, 
5, 5, 5, 5, 6, 5, 6, 7, 7, 8, 9, 8, 6, 7, 9, 10, 
5, 6, 7, 8, 8, 7, 12, 12, 9, 10, 9, 8, 8, 8, 9, 6, 
6, 6, 5, 7, 6, 3, 5, 8, 8, 7, 5, 6, 5, 4, 4, 4, 
3, 3, 3, 1, 2, 3, 2, 3, 1, 2, 2, 2, 3, -1, 0, 0, 
-2, -1, -1, -2, -1, -1, -1, -1, 1, 1, 0, -1, -4, -4, -3, -2, 
0, 1, 0, -1, -1, -3, -3, -3, -5, -4, -4, -3, -3, -4, -3, -2, 
-3, -1, -1, -1, 0, 2, 2, 2, 1, 1, 1, 0, 0, };
