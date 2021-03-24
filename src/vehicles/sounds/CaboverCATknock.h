const unsigned int knockSampleRate = 22050;
const unsigned int knockSampleCount = 1261;
const signed char knockSamples[] = {
0, -9, -15, -17, -21, -18, -13, -18, -22, -24, -28, -36, -41, -37, -36, -41,
-47, -43, -41, -42, -46, -53, -59, -63, -68, -71, -67, -59, -54, -52, -53, -56, 
-60, -63, -62, -62, -65, -67, -68, -63, -55, -54, -49, -34, -20, -15, -11, -10, 
-12, -15, -16, -17, -20, -25, -29, -31, -31, -22, -18, -21, -27, -30, -32, -31, 
-37, -47, -53, -54, -56, -56, -59, -67, -74, -81, -89, -90, -85, -81, -81, -82, 
-74, -71, -71, -70, -69, -65, -61, -59, -62, -63, -61, -60, -60, -57, -52, -53, 
-60, -69, -68, -71, -77, -73, -58, -55, -55, -53, -50, -52, -53, -46, -40, -41, 
-48, -53, -55, -56, -59, -66, -73, -82, -89, -95, -98, -95, -86, -78, -69, -58, 
-51, -50, -51, -51, -58, -68, -76, -76, -79, -78, -74, -73, -78, -78, -76, -77, 
-79, -79, -76, -75, -72, -64, -55, -47, -43, -45, -51, -56, -60, -65, -70, -72, 
-74, -76, -71, -64, -61, -58, -52, -43, -31, -24, -22, -21, -20, -22, -25, -27, 
-26, -21, -16, -12, -9, -11, -16, -20, -23, -25, -24, -21, -19, -15, -10, -5, 
1, 6, 9, 12, 13, 8, -10, -21, -26, -29, -31, -31, -37, -43, -44, -44, 
-42, -40, -39, -39, -35, -29, -32, -42, -42, -40, -40, -39, -37, -33, -29, -23, 
-22, -29, -33, -30, -28, -29, -27, -20, -12, -6, 0, 2, 5, 13, 18, 18, 
23, 33, 42, 45, 47, 48, 44, 39, 35, 28, 22, 20, 19, 17, 15, 8, 
0, -8, -15, -16, -11, -10, -11, -7, 0, 9, 15, 16, 18, 20, 20, 15, 
8, 7, 6, 1, -5, -6, -7, -13, -18, -23, -32, -42, -41, -39, -37, -31, 
-21, -14, -14, -11, -6, -5, -10, -13, -18, -25, -31, -38, -46, -49, -51, -49, 
-48, -50, -50, -46, -43, -43, -39, -27, -18, -11, -11, -14, -16, -18, -17, -14, 
-13, -11, -7, -7, -9, -13, -15, -12, -11, -6, 1, 3, 2, 2, 0, 0, 
6, 21, 35, 39, 41, 40, 31, 17, 7, 0, -2, -1, -1, -4, -7, -4, 
-1, 0, 2, 8, 17, 26, 31, 33, 30, 27, 25, 26, 26, 22, 17, 15, 
12, 9, 5, 0, -2, -2, 1, 4, 9, 17, 25, 28, 28, 28, 29, 29, 
25, 26, 34, 43, 48, 48, 41, 21, 14, 9, 7, 4, -5, -11, -9, 0, 
7, 9, 7, 8, 12, 14, 13, 16, 32, 39, 44, 53, 59, 62, 64, 59, 
49, 41, 36, 28, 21, 10, 1, -4, -9, -23, -31, -31, -24, -18, -12, -10, 
-13, -15, -11, -5, 7, 19, 29, 36, 41, 48, 52, 57, 63, 68, 70, 71, 
69, 65, 59, 58, 60, 58, 53, 52, 52, 43, 24, 18, 14, 6, -4, -14, 
-23, -20, -15, -17, -20, -19, -17, -18, -19, -16, -12, -4, 1, 13, 24, 28, 
24, 19, 17, 16, 17, 19, 24, 35, 38, 40, 41, 40, 35, 30, 27, 33, 
37, 37, 40, 40, 40, 42, 45, 45, 41, 38, 37, 37, 36, 41, 50, 58, 
63, 67, 70, 69, 61, 53, 51, 51, 48, 48, 49, 53, 58, 55, 49, 54, 
55, 56, 57, 56, 56, 56, 57, 60, 66, 77, 94, 100, 99, 94, 88, 76, 
68, 60, 53, 50, 46, 38, 37, 38, 41, 42, 41, 43, 46, 47, 46, 49, 
55, 57, 57, 57, 55, 48, 32, 15, -2, -5, 2, 5, 3, 12, 25, 34, 
38, 40, 41, 44, 48, 50, 47, 34, 28, 22, 13, 6, 8, 6, 4, 7, 
10, 8, 5, 6, 7, 9, 14, 22, 29, 34, 39, 42, 43, 40, 33, 27, 
21, 9, -4, -9, -8, -10, -8, -5, -1, 5, 10, 11, 5, 0, -5, -11, 
-14, -15, -17, -18, -8, -4, -13, -26, -31, -32, -38, -36, -25, -12, -8, -12, 
-7, -1, -2, -8, -14, -18, -20, -26, -35, -42, -45, -46, -53, -59, -63, -67, 
-67, -63, -56, -50, -49, -48, -40, -37, -36, -35, -32, -30, -34, -34, -32, -36, 
-42, -47, -48, -48, -45, -41, -38, -30, -30, -34, -36, -35, -34, -43, -49, -51, 
-50, -49, -54, -55, -53, -50, -48, -49, -61, -67, -69, -68, -65, -62, -58, -56, 
-49, -46, -52, -64, -68, -71, -78, -87, -94, -96, -93, -92, -91, -89, -87, -81, 
-77, -74, -65, -54, -47, -50, -53, -55, -57, -59, -61, -65, -68, -71, -76, -79, 
-77, -75, -74, -75, -72, -68, -70, -72, -72, -73, -73, -73, -71, -68, -68, -67, 
-65, -63, -63, -63, -60, -62, -67, -63, -59, -58, -59, -61, -66, -67, -68, -69, 
-68, -68, -72, -75, -78, -82, -85, -86, -87, -84, -74, -64, -57, -54, -52, -50, 
-44, -39, -40, -35, -29, -28, -32, -33, -33, -36, -38, -40, -44, -49, -56, -62, 
-72, -80, -84, -87, -83, -76, -74, -69, -55, -39, -32, -26, -28, -32, -31, -27, 
-24, -21, -20, -19, -16, -19, -26, -35, -43, -50, -57, -61, -66, -70, -69, -66, 
-61, -51, -42, -40, -43, -40, -38, -34, -28, -28, -34, -37, -34, -28, -24, -20, 
-16, -16, -19, -22, -27, -33, -35, -31, -26, -26, -26, -23, -25, -30, -37, -41, 
-39, -34, -28, -24, -17, -11, -11, -13, -14, -19, -24, -25, -24, -19, -20, -22, 
-21, -21, -24, -26, -28, -34, -42, -47, -47, -48, -49, -49, -47, -48, -53, -56, 
-60, -62, -65, -68, -63, -59, -59, -66, -72, -65, -62, -61, -56, -49, -43, -36, 
-31, -26, -21, -14, -4, 1, 9, 17, 24, 29, 33, 37, 42, 46, 48, 47, 
34, 22, 12, 0, -11, -15, -14, -14, -15, -18, -21, -25, -28, -27, -26, -29, 
-30, -19, -13, -11, -8, -7, -5, -4, -5, -7, -8, -10, -16, -22, -27, -33, 
-37, -33, -23, -23, -22, -14, -3, -2, -8, -8, -8, -13, -18, -21, -24, -26, 
-23, -20, -21, -27, -33, -37, -41, -44, -44, -42, -38, -38, -41, -41, -28, -29, 
-35, -35, -30, -28, -36, -41, -43, -43, -46, -40, -29, -20, -12, -4, 3, 3, 
1, 1, 4, 10, 14, 23, 29, 32, 33, 30, 22, 16, 12, 13, 15, 12, 
3, 0, 0, 0, -1, 1, 8, 15, 19, 22, 26, 30, 30, 32, 34, 34, 
30, 29, 29, 26, 21, 20, 20, 17, 14, 12, 8, 5, 0, -9, -15, -13, 
-8, -5, -1, 0, 0, 0, 2, 6, 12, 24, 33, 37, 42, 42, 42, 46, 
50, 50, 51, 58, 59, 57, 56, 58, 51, 36, 23, 15, 7, 0, -6, -6, 
-4, 0, 5, 11, 16, 11, 6, 7, 11, 17, 23, 29, 33, 33, 31, 33, 
36, 33, 28, 25, 27, 33, 34, 34, 39, 47, 49, 53, 61, 66, 64, 64, 
71, 74, 73, 68, 62, 58, 55, 55, 56, 59, 64, 74, 81, 86, 86, 85, 
90, 101, 102, 98, 94, 93, 91, 80, 72, 67, 65, 63, 58, 53, 49, 49, 
51, 50, 47, 51, 55, 55, 56, 56, 55, 54, 54, 53, 49, 43, 44, 47, 
51, 54, 59, 64, 72, 81, 85, 85, 90, 93, 92, 91, 92, 92, 83, 84, 
89, 89, 86, 83, 76, 71, 73, 72, 68, 72, 83, 95, 107, 118, 124, 127, 
124, 117, 107, 96, 86, 81, 77, 70, 61, 53, 43, 42, 45, 46, 46, 47, 
53, 58, 62, 63, 65, 67, 75, 84, 91, 94, 93, 87, 83, 80, 79, 77, 
71, 65, 66, 69, 67, 61, 55, 51, 46, 35, 26, 21, 15, 15, 18, 19, 
20, 18, 11, 10, 14, 12, 6, -1, -9, -15, -15, -7, 0, };
