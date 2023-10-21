#ifndef DATATURNTO_H
#define DATATURNTO_H

#include <iostream>

enum BytesOrder{

    //两个及以上字节的数据才存在大小端问题
    little_Edian = 0,   //x86架构一般是小端，数据的低位在低地址(前)，高位在高地址(后)
    big_Edian = 1,

    //在十进制中靠左边的是高位，靠右边的是低位，在其他进制也是如此。例如 0x12345678，从高位到低位的字节依次是0x12、0x34、0x56和0x78。
    //首先明确大小端的概念, int X = 4660(0x00001234), 在小端模式下，在内存中则为 34(存位字节在前)12(高位字节在后)0000，底地址存低序字节，高地址存高序字节

    //原始数据0xAABB
    order_16_AABB = 0,
    order_16_BBAA,


    //原始数据0xAABBCCDD
    order_32_ABCD,
    order_32_DCBA,
    order_32_BADC,
    order_32_CDAB,      //现在用的是这种

    order_float_ABCD,
    order_float_DCBA,
    order_float_BADC,
    order_float_CDAB,

    order_64_ABCDEFGH,
    order_64_EFGHABCD,
    order_64_HGFEDCBA,
    order_64_GHEFCDAB,

    order_double_ABCDEFGH,
    order_double_EFGHABCD,
    order_double_HGFEDCBA,
    order_double_GHEFCDAB
};


//注意我这里认为工作环境是小端的
//从站(下位机)的读写数据都是面对Uint8的
void trunUint8ToUint16(uint16_t *dst, uint8_t *src, short int16Order);
void trunUint16ToUint8(uint8_t *dst, uint16_t *src, short int16Order);

void trunUint8ToUint32(uint32_t *dst, uint8_t *src, short int32Order);
void trunUint32ToUint8(uint8_t *dst, uint32_t *src, short int32Order);

void trunUint8ToUint64(uint64_t *dst, uint8_t *src, short int64Order);
void trunUint64ToUint8(uint8_t *dst, uint64_t *src, short int64Order);

void trunUint8ToFloat32(float *dst, uint8_t *src, short floatOrder);
void trunFloat32ToUint8(uint8_t *dst, float *src, short floatOrder);

void trunUint8ToDouble64(double *dst, uint8_t *src, short doubleOrder);
void trunDouble64ToUint8(uint8_t *dst, double *src, short doubleOrder);

//主站(上位机)的读写数据都是面对Uint16的
//直接当Uint8处理应该没问题

void verifyTrunToByteOrder();

#endif // DATATURNTO_H

