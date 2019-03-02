/*
 * array.h
 *
 * Created : 23/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ARRAY_CONTAINER_H_
#define _ARRAY_CONTAINER_H_

template <typename T, size_t N>
size_t arrSize(const T (&arr)[N])
{
        return N;
}

template <size_t N>
void printArr(const float (&arr)[N])
{
        printf("[ ");
        for (size_t i = 0; i < N; ++i) {
                printf("%ld ", (int32_t)(arr[i]*1000.0));
        }
        printf("]");
}

template <size_t N>
void arrMult(float (&arr)[N], float num)
{
        for (size_t i = 0; i < N; ++i) {
                arr[i] *= num;
        }
}

template<typename T, size_t N>
void fill_Array(T (&arr)[N], T num)
{
        for (size_t i = 0; i < N; ++i) {
                arr[i] = num;
        }
}

#endif // !_ARRAY_CONTAINER_H_
