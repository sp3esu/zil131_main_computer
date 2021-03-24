const unsigned int sampleRate = 22050;
const unsigned int sampleCount = 3449;
const signed char samples[] = {
0, -5, -6, -4, -7, -6, 4, 3, 0, 5, 7, 17, 4, 7, 22, 6, 
-12, -8, -1, -29, -38, -25, -22, -5, -19, -10, 7, 21, 31, 18, 16, 15, 
5, -14, 2, 7, 14, 4, -11, 5, 21, 36, 38, 43, 55, 61, 60, 47, 
41, 38, 23, -8, -18, -7, -5, -11, -4, 4, 7, 4, -4, -7, 3, -9, 
-9, 2, 8, 11, 16, 9, -5, -4, -3, -7, 7, 30, 26, 12, 9, 17, 
1, -22, -16, -20, -32, -18, 2, 16, 23, 39, 47, 24, 26, 31, 30, 25, 
14, 35, 44, 42, 43, 43, 35, 21, 15, 6, 6, 11, 27, 32, 23, 30, 
39, 35, 13, 1, 0, 2, 14, 18, 13, 14, 18, 15, 8, 11, -10, -16, 
0, 7, 8, 9, 20, 29, 29, 34, 22, 12, 9, -3, -11, -5, -12, -22, 
-14, -16, -19, -42, -58, -59, -63, -40, -3, 11, 19, 16, 5, 4, -3, -18, 
-17, -2, 8, -4, -12, -22, -25, -10, 0, -1, 4, -1, 0, 20, 24, 23, 
41, 37, 42, 47, 54, 66, 60, 52, 42, 22, 4, -3, -7, -11, -24, -37, 
-53, -47, -40, -58, -59, -70, -87, -87, -89, -76, -68, -66, -64, -53, -48, -47, 
-36, -35, -16, 7, 10, 23, 33, 29, 32, 37, 30, 17, 17, -8, -11, -7, 
16, 13, -20, 38, 48, 80, 71, 41, 79, 10, 60, 77, 41, 64, -12, 30, 
53, 42, 34, 1, -11, 19, -6, -33, -17, -74, -61, -43, -25, -5, -44, -62, 
-46, -64, -48, -36, -44, -60, -77, -44, -47, -65, -127, -124, -99, -105, -87, -103, 
-59, -66, -66, -25, -10, 32, -21, -18, 24, 30, 31, 55, 85, 65, 37, 53, 
47, 11, 27, 33, 52, 33, 34, 70, 82, 78, 47, 45, 40, 36, 37, 11, 
11, 8, -15, -6, -12, -10, -16, 10, 11, -1, 21, 33, 20, -14, -22, -9, 
-35, -42, -14, -45, -36, -30, 2, -5, -32, -15, -45, -19, -3, -8, -4, -28, 
-30, -19, -4, -23, -4, -4, -14, -7, -1, 8, -12, 0, -13, -12, 19, 9, 
24, 23, 30, 34, 27, 49, 40, 28, 36, 46, 20, 14, 2, -11, 4, 11, 
18, 26, 16, -3, -1, 19, 7, -4, -9, -24, -32, -7, -23, -35, -25, -31, 
-47, -53, -49, -64, -75, -76, -74, -43, -44, -33, -27, -27, 2, 23, 33, 29, 
30, 38, 66, 60, 48, 53, 47, 27, -2, -12, -23, -15, -12, -14, -7, -1, 
0, -19, -21, -29, -35, -39, -44, -38, -35, 8, 7, -3, 3, 11, 18, 7, 
7, -5, -12, -8, 7, 23, 5, -13, -5, 5, 0, -16, -38, -44, -49, -50, 
-44, -24, -17, -27, -15, -9, 1, -4, 9, 34, 28, 27, -3, -4, 4, -16, 
-7, -3, 4, -11, -24, -14, -35, -29, -21, -23, -21, -21, -20, -5, 15, 31, 
40, 17, 6, 16, 44, 58, 56, 22, -7, -27, -47, -19, 12, 14, 4, -24, 
-22, -2, 18, 1, 6, 34, 42, 56, 52, 24, 1, -13, -7, 38, 60, 62, 
62, 60, 43, 43, 43, 27, 18, 18, 29, 17, 14, 30, 49, 61, 43, 40, 
60, 74, 72, 58, 44, 45, 50, 49, 29, 17, 17, 20, 24, 29, 43, 65, 
63, 49, 36, 30, 20, 13, 5, -13, -20, -18, -7, 1, -5, -14, -17, -18, 
-43, -41, -39, -46, -52, -55, -21, -20, -15, 0, -3, -4, -10, -15, -24, -35, 
-24, -20, -19, -9, 0, 11, 17, -5, -2, 10, 6, 7, -2, -16, -14, -5, 
1, 3, -14, -19, -8, -3, 13, 15, 0, 12, 13, 26, 15, -33, -40, -40, 
-37, -34, -16, 9, -10, -14, 3, 20, 24, 8, 7, -4, 5, 22, 20, 19, 
5, 4, -5, -6, 19, 26, 33, 17, -4, -23, -23, -8, -9, -21, -34, -17, 
3, 11, 30, 34, 23, 9, 23, 36, 8, -15, -1, 26, 27, 16, 13, 16, 
-13, 14, -2, -2, 6, -27, -12, 1, 29, 12, 1, 25, 6, -34, -44, -19, 
-24, -7, -22, -31, -28, -56, -52, -54, -45, -21, -18, 23, 42, -8, 9, 40, 
60, 44, 5, 0, 16, 3, -26, -8, -9, 0, 5, 12, 14, -32, 1, 19, 
-26, -62, -53, -40, -27, -13, -25, -36, -44, -11, -6, -17, -14, -11, -1, -12, 
-8, -11, -7, -8, -22, -40, -31, 28, 4, -32, -17, -19, -22, -31, -8, -7, 
-3, 0, 16, 14, -5, 21, 4, -10, -10, -5, 3, -9, -16, -11, -34, -41, 
-7, -38, -12, 22, 15, 10, -21, -8, 10, 7, 5, 11, -17, -29, -24, -26, 
-14, -17, -7, -12, -24, -12, -26, -36, -25, -21, 0, 21, 12, -23, -22, -8, 
6, 5, -11, -18, -19, -14, -21, -12, -31, -36, -7, 15, 25, 12, 7, -6, 
8, 16, 6, 36, 42, 32, 31, 30, 4, 6, 15, 4, -12, -6, 15, -1, 
8, 25, 13, 5, 29, 24, -3, -32, -25, -1, -7, -5, 5, 14, 14, 0, 
-1, -8, -17, -8, 4, -3, 11, 27, 33, 23, 1, 9, 1, 7, -1, -18, 
-10, -21, 5, 16, 10, 12, -1, 13, 4, 9, 8, -18, -19, -40, -38, -22, 
-11, 0, -16, -14, -2, 3, 8, 22, 41, 24, -7, -6, 6, 15, 12, 11, 
14, 24, 32, 14, 12, 11, 11, 22, 8, 11, 7, 1, 16, 24, 6, 4, 
21, 36, 27, 11, 0, -6, 0, 10, 22, -4, -22, -22, -28, -29, -20, -8, 
-7, -15, -21, -20, -19, -28, -17, -2, -10, -30, -61, -37, -23, -19, 6, 6, 
-20, -22, -20, -25, -20, 16, 16, 12, 25, 27, 16, 3, 11, 27, 19, 14, 
13, -5, -6, -22, -24, -9, -6, -6, -8, -8, -16, -25, -6, -9, -16, -9, 
-6, -8, -11, -10, -12, -8, 11, 17, 17, 20, 16, 12, 7, 8, 12, 19, 
27, 30, 20, 8, 3, 1, 0, 5, 37, 40, 34, 22, 12, 8, 2, -5, 
-11, 4, 22, 22, 7, -4, -2, 2, -8, -12, -14, -33, -53, -46, -33, -47, 
-34, -6, 21, 19, 16, 23, 31, 30, 17, 21, 14, 18, 27, 37, 39, 40, 
49, 54, 48, 39, 48, 30, 29, 18, 9, 11, 12, 49, 45, 24, 2, -8, 
-6, 8, 15, 21, 32, 25, 9, -3, -14, -18, -22, -17, -10, -5, 4, 9, 
22, 22, 13, 5, 1, 20, 31, 30, 26, 37, 49, 50, 49, 32, 35, 34, 
43, 43, 37, 46, 41, 36, 27, 30, -16, -28, -12, 8, 8, -21, 33, 26, 
15, 23, -6, 10, -53, -39, -31, -62, -12, -52, -23, -2, -17, -11, -23, -30, 
-10, -51, -61, -1, -23, -24, -28, -28, -5, -24, -26, -20, -45, -42, -35, -44, 
-32, -55, -28, -15, -12, -30, -15, 17, -11, 9, -22, -13, -32, -50, -23, -48, 
-35, -90, -58, -4, -23, -51, -62, 16, -2, -29, -5, -11, -50, -36, -31, -49, 
-66, -56, -28, -19, -16, -31, -20, -2, -16, -19, -36, -33, -22, -37, -19, -24, 
-37, -37, -22, -4, -29, -17, 9, -6, -1, 9, 11, -20, -33, 15, 9, 5, 
-8, 17, 16, -26, 0, -36, -30, -19, -28, -15, 5, 13, 14, 35, -7, -8, 
-4, -4, 1, -16, 13, 12, 40, 36, 21, 38, 11, 18, 1, 12, 25, 11, 
44, 39, 42, 43, 38, 24, 16, 10, -16, 4, 9, -2, 5, 15, 28, 23, 
18, 4, 12, 10, 6, 21, 35, 17, 14, 43, 47, 31, 17, 14, 2, -5, 
-6, -17, -3, 7, 25, 19, -8, -11, 0, 4, 4, -4, -12, 1, -3, -23, 
-25, 6, 30, 29, 29, 38, 44, 37, 30, 17, 23, 11, -1, 15, 13, 15, 
16, 16, 4, -17, 2, 1, -9, -23, -29, -18, -27, -14, -22, -45, -51, -52, 
-48, -58, -58, -42, -37, -51, -49, -31, -18, -25, -37, -49, -28, -13, -23, -14, 
10, 24, 28, 33, 24, -15, -21, -19, -7, 2, -3, 10, 22, 44, 43, 43, 
41, 40, 37, 28, 38, 42, 45, 39, 45, 63, 71, 69, 55, 42, 44, 60, 
63, 56, 47, 44, 26, -1, -3, 17, 29, 8, -12, -22, -36, -40, -17, 7, 
8, -23, -37, -34, -50, -65, -64, -39, -38, -32, -9, -9, -15, -41, -51, -38, 
-37, -28, -19, -23, -12, 8, 25, 15, 0, 7, 14, 16, 16, 12, 0, -7, 
-16, -29, -31, -16, 13, 12, 7, 11, 16, 20, 14, 3, -19, -15, 10, 3, 
1, -2, 1, 18, 30, 36, -4, -11, -2, 1, -4, -18, -12, -14, -13, 4, 
5, -7, -6, -13, -18, -9, 1, -14, -12, 1, 7, -14, -32, -6, 8, 18, 
25, 40, 44, 49, 48, 33, 32, 33, 20, -2, -10, -6, 3, 16, 27, 36, 
29, 29, 38, 54, 52, 44, 44, 33, 38, 31, 26, 33, 40, 47, 44, 55, 
56, 48, 26, 9, 13, 7, -5, -14, 6, 8, -5, 0, 1, 8, 7, 1, 
-11, -3, -5, -11, -5, -4, -5, -17, -12, 1, 7, 10, 1, 19, 40, 35, 
-8, -8, 31, 27, 20, 25, 45, 60, 45, 40, 63, 76, 50, 44, 33, 55, 
35, 3, 13, 31, 39, 4, 7, 30, 24, -19, -8, 32, 12, -19, -32, -20, 
-28, -52, -38, -49, -40, -23, -24, -15, -55, -49, -47, -44, -40, -71, -60, -56, 
-30, -46, -65, -44, -55, -41, -42, -25, -28, -28, 14, 7, -30, -47, -29, -32, 
-35, -39, -47, -33, -21, -20, -42, -52, -56, -62, -41, -53, -60, -67, -60, -40, 
-32, -42, -39, -29, -54, -50, -57, -61, -65, -54, -15, -15, -15, -19, -16, -37, 
-65, -37, -31, -42, -43, -18, -23, -40, -49, -62, -78, -55, -35, -45, -27, -13, 
14, -5, -12, 4, 6, 6, -12, 10, 21, 25, 22, 27, 58, 59, 64, 38, 
37, 49, 19, 31, 39, 45, 35, 39, 39, 26, 35, 55, 77, 70, 44, 46, 
68, 77, 76, 66, 33, 30, 36, 51, 62, 57, 43, 17, 7, 1, 2, -8, 
-24, -21, -6, -1, 12, 34, 26, 11, 17, 11, -17, -25, -10, -6, -21, -12, 
28, 38, 8, -9, -10, 5, 17, 21, 40, 40, 29, 2, 10, 18, -9, -20, 
-15, -29, -25, -15, -15, -26, -40, -37, -43, -37, -25, -24, -9, -20, -19, -16, 
-17, -9, -10, -4, -4, 13, 18, -27, -32, -34, -27, -15, -22, -40, -44, -30, 
-15, -24, -24, -18, -22, -23, -18, -10, -23, -40, -54, -40, -33, -25, -25, -42, 
-42, -55, -49, -37, -19, -10, -18, -3, 3, -14, -9, 2, 10, 0, 3, 33, 
51, 53, 47, 56, 33, 28, 23, 5, -11, -19, 0, 5, 0, -6, 1, 2, 
-9, -3, 16, 24, -1, -36, -41, -24, -23, -28, -18, 1, 10, 6, 14, 18, 
8, -5, -3, 10, 13, 2, 6, 3, 2, 6, 11, 36, 54, 55, 65, 79, 
82, 71, 58, 47, 47, 38, 43, 75, 73, 63, 34, 17, 23, 12, 12, 4, 
-17, -28, -20, 20, 13, -2, -9, -9, 13, 3, -5, -27, -38, -22, -12, 11, 
25, 17, 18, 36, 29, 8, -23, -41, -50, -47, -26, -23, -16, -5, 18, 41, 
37, 32, 17, 15, 32, 69, 83, 62, 49, 47, 36, 38, 26, 23, 30, 25, 
30, 29, 17, 0, -17, -12, -17, -24, -17, -24, -33, -42, -61, -65, -57, -45, 
-41, -41, -49, -71, -82, -107, -119, -98, -95, -86, -81, -61, -41, -22, 0, 5, 
7, 13, 26, 27, 27, 5, -12, -1, -4, -10, -34, -52, -52, -25, -5, -10, 
-12, 6, 24, 10, 13, 19, 18, 30, 48, 48, 49, 54, 63, 76, 67, 56, 
60, 47, 29, 20, 32, 53, 49, 64, 80, 50, 39, -3, -13, -37, -66, -23, 
-55, -12, -19, -55, -12, -8, 13, 5, -27, 2, -2, -45, -2, 14, 4, 5, 
0, 38, 36, 17, 37, 54, 31, 34, 26, 25, -4, -32, -16, 7, -8, -29, 
-3, -27, 18, -24, -44, -10, -51, 3, -4, 4, -1, -47, 28, 56, 37, 33, 
54, 51, 32, 27, 10, 0, 11, 22, 10, -9, 11, 12, -24, -28, -53, -64, 
-48, -41, -36, -50, -65, -40, -29, -40, -51, -62, -56, -53, -46, -46, -37, -21, 
-26, -25, -7, -1, -42, -52, -23, -29, -30, -18, 16, 23, -19, -3, -14, -26, 
-17, -20, -3, -15, -12, -46, -33, -46, -60, -22, -29, -6, -13, 1, 7, 3, 
16, -35, -37, -33, -34, -32, -31, -48, -67, -32, -22, -48, -53, -48, -46, -56, 
-45, -32, -22, -20, -36, -45, -54, -79, -54, -33, -34, -25, -4, -11, -25, -9, 
-10, -22, 30, 17, -3, -12, -4, 19, 33, 34, 16, 31, 38, 9, 24, 29, 
34, 34, 27, 39, 33, 33, 35, 29, 20, 0, 26, 43, 34, 51, 72, 75, 
60, 34, 4, -28, -4, -8, -12, -9, -4, 7, -5, 4, 3, 12, 16, -15, 
12, 37, 47, 56, 56, 60, 38, 25, 33, 44, 28, -14, -4, 22, 34, 32, 
45, 39, 16, -13, -18, 4, 15, 29, 45, 42, 44, 34, 4, 14, 15, 26, 
44, 31, 36, 37, 17, 22, 49, 38, 40, 37, 20, 17, 13, 22, 31, 31, 
11, -10, 22, 15, -4, 1, 3, 2, -2, 0, -4, -14, 1, 13, 12, 1, 
-7, -17, -39, -41, -50, -42, -18, -3, -3, -7, 1, 16, 14, -6, -14, -8, 
11, 21, 21, 6, -5, 0, -3, 2, -6, -2, -2, -5, 6, 0, -11, -12, 
-1, -11, -13, -8, -18, -17, -1, 12, -4, -18, -11, -26, -27, -18, -16, -18, 
-15, -17, -12, -16, -12, 1, -26, -27, -40, -49, -18, 9, 32, 16, 3, 6, 
-9, -8, -9, -12, -12, -7, 6, 25, 28, 10, 2, -9, -22, -29, -48, -50, 
-44, -23, 16, 16, 6, 1, -8, -20, -26, -22, -23, -38, -40, -14, 25, 25, 
9, -18, -30, -1, 22, 19, 8, 17, 25, 41, 42, 36, 36, 28, 9, -2, 
9, 20, 29, 29, 37, 37, 25, 25, 22, 12, 8, 16, 17, 16, 16, 9, 
30, 49, 54, 46, 35, 31, 33, 32, 33, 43, 60, 62, 38, 35, 53, 59, 
38, 22, 28, 30, 12, 9, 27, 24, 22, 12, 1, -5, 15, -5, -31, -22, 
3, 24, 12, -5, 8, 12, -25, 14, -12, -8, 3, -37, -22, -9, -21, -44, 
-36, -2, 7, -36, -28, 17, 14, 6, -11, -6, -2, -25, -39, -46, -48, -22, 
-11, -1, -22, -37, -27, -20, -23, -56, -36, -21, -26, -42, -55, -27, -28, -2, 
5, -2, -12, -23, 13, 8, 0, 0, 37, 43, -41, -47, -44, -12, 21, 28, 
-3, -23, -22, 14, 68, 23, 0, -9, 10, 20, 9, 15, -5, -4, -16, -18, 
-6, -13, -31, -20, 13, 3, -21, -5, 14, -2, -18, -9, 8, 8, 18, 35, 
30, 1, -38, -33, -52, -54, -30, -28, -2, 7, 10, 1, -20, -7, -5, -3, 
-25, -28, -20, -41, -47, -50, -48, -59, -7, -15, -36, -28, -24, -4, 31, 59, 
42, 15, 25, 8, 9, 25, 18, -16, -22, -1, 9, 9, 2, 10, 3, -2, 
-8, 2, 18, 18, 12, 3, 5, 0, 12, 16, 5, -3, -10, 1, -13, -16, 
-21, -38, -30, -23, -12, -6, 22, 30, 6, 1, -2, -15, -34, -32, -36, -55, 
-54, -65, -53, -29, -18, -20, 2, 25, 24, 15, 19, 8, 10, 36, 20, 11, 
10, -6, -7, 16, 20, 2, 12, 8, 11, 13, 0, -6, -5, 22, 21, 0, 
-30, -26, -12, -6, 11, 6, 5, -9, -21, -29, -35, -36, -40, -31, -21, -24, 
-23, -21, -17, -10, -36, -25, -20, -23, -7, -14, -14, -24, -23, -6, -13, -23, 
2, 36, 40, 16, -5, 13, 30, 17, -3, -2, 10, 13, 20, 40, 44, 35, 
42, 35, 22, 28, 40, 27, 37, 38, 29, 29, 20, 34, 31, 21, 17, 1, 
-10, -8, 3, 23, 30, 30, 33, 29, 29, 39, 50, 53, 60, 52, 43, 39, 
37, 42, 45, 33, 28, 29, 18, 14, 8, -5, -6, -4, -2, -3, 5, 6, 
5, 19, 26, 16, 30, 42, 32, 28, 44, 42, 40, 34, 26, 31, 35, 39, 
31, 25, 31, 33, 17, 0, -6, -13, -20, -5, -2, -8, -4, 12, 15, -7, 
-6, -13, -20, -11, -8, -25, -19, -7, -9, -7, -38, -38, -20, -15, -11, -16, 
-2, 5, -6, -18, -29, -35, -41, -47, -50, -47, -35, -38, -26, -7, -7, -28, 
-46, -25, -36, -41, -48, -57, -48, -42, -36, -32, -37, -45, -38, -28, -24, -33, 
-31, -17, -21, -14, -15, -17, -14, -29, -33, -39, -27, -23, -32, -11, -15, -1, 
-5, -18, -8, 0, 9, 18, 17, 4, 5, 29, 27, 20, 28, 22, 19, 25, 
16, 15, 22, 18, 8, 4, 12, 18, 3, 0, 14, 22, 3, 12, 30, 2, 
4, -1, 20, 24, 9, 36, 32, 52, 32, 41, 40, 4, 44, 8, 19, 44, 
-16, 20, 10, 5, -7, -48, -7, -24, -82, -40, -19, -16, 4, 0, 17, 3, 
-5, 15, -11, -8, 8, -9, 8, -15, -17, 10, 31, 30, -10, 32, 19, 15, 
18, 6, 13, -7, 18, 33, 26, -2, -30, 8, 1, -46, -60, 15, 1, -39, 
-24, -8, -22, -15, -10, -10, -36, -42, -15, -15, -12, -24, -11, 12, -7, -20, 
-25, -29, -37, -47, -40, -44, -43, -26, -20, 4, -1, -8, 22, -1, -18, 14, 
3, -15, -52, -12, 14, -28, -29, -18, 22, -15, -4, 36, 27, 21, -4, 17, 
-6, 13, 21, 28, 13, -26, 17, 4, 4, -8, -8, -2, -1, 10, -25, -28, 
-32, -40, -39, -31, -15, -30, -17, -5, -12, -14, -13, 2, -33, -40, -40, -29, 
-6, 2, 7, 9, 7, 7, 19, 21, 25, 30, 17, 2, 7, 7, 1, 23, 
35, 18, -5, -15, -21, -32, -19, -9, -11, 4, -2, 8, -1, -1, 12, 7, 
20, 15, 15, 8, -4, -13, -22, -8, 1, -2, -2, -13, -26, -25, -23, -16, 
-10, -7, -16, -5, 36, 66, 67, 59, 45, 36, 39, 60, 41, 24, 23, 8, 
-1, -2, 1, 3, -19, -50, -30, -15, -22, -24, -21, -16, -37, -57, -59, -61, 
-76, -83, -87, -76, -54, -48, -35, -30, -39, -27, -4, -3, -14, -8, -4, 0, 
3, 12, 0, 2, 17, 33, 55, 55, 47, 45, 48, 70, 93, 85, 63, 82, 
90, 97, 102, 101, 90, 73, 59, 37, 37, 40, 27, 3, -15, -19, -27, -28, 
-34, -33, -22, -9, -27, -59, -84, -93, -74, -46, -23, -13, -12, -5, -14, -58, 
-61, -56, -43, -45, -48, -31, -21, 3, 19, 27, 16, -5, 21, 39, 51, 54, 
63, 66, 47, 59, 75, 75, 76, 59, 49, 58, 58, 45, 30, 13, 17, 24, 
17, 12, 15, 11, 8, 27, 42, 49, 34, 15, 7, 7, 9, -16, -21, -17, 
-7, 5, 20, 14, -2, -2, -8, -11, -20, -41, -44, -33, -17, -5, -11, -25, 
-25, -15, -5, 12, 25, 19, 7, 11, 17, 51, 64, 61, 55, 48, 63, 69, 
73, 84, 65, 52, 41, 39, 38, 21, 13, 18, 28, 32, 36, 34, 36, 35, 
9, 0, -12, -18, -18, -36, -24, -7, -6, -15, -14, 18, 18, 10, 2, -11, 
-10, -18, -17, -14, -15, -11, -22, -28, -53, -64, -62, -54, -54, -80, -70, -59, 
-54, -33, -45, -55, -57, -40, -33, -27, -15, 13, 50, 39, 24, 28, 39, 12, 
16, 18, 24, 27, -24, -17, -3, 0, -46, -50, -6, 5, -20, -40, 12, 8, 
-5, -14, -39, -16, -47, -42, -19, -54, -55, -43, -31, -55, -75, -51, -39, -24, 
-49, -32, -16, -16, -16, -41, -6, -13, -31, -16, -15, -31, -49, -7, 1, -32, 
-55, -5, 16, -26, -41, -57, -35, -6, 10, -11, -51, -61, -45, -22, -20, -25, 
-38, -34, -7, -2, 3, 1, 9, 11, 24, 33, 24, 9, -6, 5, -16, -18, 
-2, 19, 9, -11, 5, 0, -17, -18, -9, -15, -32, -37, 1, 31, 42, 62, 
27, 4, 0, -5, 8, 16, 34, 45, 51, 18, 35, 80, 91, 95, 87, 94, 
55, 51, 76, 56, 56, 51, 56, 38, 31, 40, 49, 72, 58, 47, 51, 72, 
46, 28, 22, 14, 22, 9, 18, 11, 21, 34, 39, 38, 24, 33, 25, 14, 
18, 6, 8, 3, -12, -18, -16, -33, -43, -26, -25, -23, -39, -39, -30, -49, 
-52, -38, -15, -11, -24, -55, -81, -80, -86, -70, -72, -77, -65, -76, -81, -72, 
-41, -34, -53, -38, -45, -45, -29, -49, -68, -53, -38, -38, -35, -27, -24, -19, 
-16, -21, -23, -41, -47, -46, -46, -55, -64, -71, -65, -50, -52, -34, -16, -21, 
-16, -29, -29, -14, 6, 19, 2, -2, 0, };
