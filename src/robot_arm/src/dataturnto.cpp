#include "robot_arm/dataturnto.h"
#include <string.h>


void trunUint8ToUint16(uint16_t *dst, uint8_t *src, short int16Order)
{
    switch (int16Order) {
    case order_16_AABB:
    {
        *dst = (src[0]<<8) + src[1];
    }
        break;
    case order_16_BBAA:
    {
        *dst = src[0] + (src[1]<<8);
    }
        break;
    default:
        std::cout<<"int16Order is not init";
        break;
    }
}


void trunUint16ToUint8(uint8_t *dst, uint16_t *src, short int16Order)
{
    switch (int16Order) {
    case order_16_AABB:
    {
        dst[0]  = (*src)  >> 8;
        dst[1]  = (*src)  & 0xFF;

    }
        break;
    case order_16_BBAA:
    {
        dst[0]  = (*src) & 0xFF;
        dst[1]  = (*src) >> 8;
    }
        break;
    default:
        std::cout<<"int16Order is not init";
        break;
    }
}


void trunUint8ToUint32(uint32_t *dst, uint8_t *src, short int32Order)
{
    uint16_t data[2] = {0xFFFF,0xFFFF};

    switch (int32Order) {
    case order_32_ABCD:
    {
        data[1] = (src[0]<<8) + src[1];
        data[0] = (src[2]<<8) + src[3];
    }
        break;
    case order_32_CDAB:
    {
        data[0] = (src[0]<<8) + src[1];
        data[1] = (src[2]<<8) + src[3];
    }
        break;
    case order_32_DCBA:
    {
        data[0] = (src[1]<<8) + src[0];
        data[1] = (src[3]<<8) + src[2];
    }
        break;
    case order_32_BADC:
    {
        data[1] = (src[1]<<8) + src[0];
        data[0] = (src[3]<<8) + src[2];
    }
        break;
    default:
        std::cout<<"int32Order is not init";
        break;
    }

    memcpy(dst,data,sizeof(uint32_t));
}


void trunUint32ToUint8(uint8_t *dst, uint32_t *src, short int32Order)
{
    uint16_t data[2] = {0xFFFF,0xFFFF};
    memcpy(data,src,sizeof(uint32_t));

    switch (int32Order) {
    case order_32_ABCD:
    {
        dst[0] = data[1]>>8;
        dst[1] = data[1]& 0xFF;
        dst[2] = data[0]>>8;
        dst[3] = data[0]& 0xFF;
    }
        break;
    case order_32_CDAB:
    {
        dst[0] = data[0]>>8;
        dst[1] = data[0]& 0xFF;
        dst[2] = data[1]>>8;
        dst[3] = data[1]& 0xFF;
    }
        break;
    case order_32_DCBA:
    {
        dst[0] = data[0]& 0xFF;
        dst[1] = data[0]>>8;
        dst[2] = data[1]& 0xFF;
        dst[3] = data[1]>>8;
    }
        break;
    case order_32_BADC:
    {
        dst[0] = data[1]& 0xFF;
        dst[1] = data[1]>>8;
        dst[2] = data[0]& 0xFF;
        dst[3] = data[0]>>8;
    }
        break;
    default:
        std::cout<<"int32Order is not init";
        break;
    }
}


void trunUint8ToFloat32(float *dst, uint8_t *src, short floatOrder)
{
    uint16_t data[2] = {0xFFFF,0xFFFF};

    switch (floatOrder) {
    case order_float_ABCD:
    {
        data[1] = (src[0]<<8) + src[1];
        data[0] = (src[2]<<8) + src[3];
    }
        break;
    case order_float_CDAB:
    {
        data[0] = (src[0]<<8) + src[1];
        data[1] = (src[2]<<8) + src[3];
    }
        break;
    case order_float_DCBA:
    {
        data[0] = (src[1]<<8) + src[0];
        data[1] = (src[3]<<8) + src[2];
    }
        break;
    case order_float_BADC:
    {
        data[1] = (src[1]<<8) + src[0];
        data[0] = (src[3]<<8) + src[2];
    }
        break;
    default:
        std::cout<<"floatOrder is not init";
        break;
    }

    memcpy(dst,data,sizeof(uint32_t));
}


void trunFloat32ToUint8(uint8_t *dst, float *src, short floatOrder)
{
    uint16_t data[2] = {0xFFFF,0xFFFF};
    memcpy(data,src,sizeof(uint32_t));

    switch (floatOrder) {
    case order_float_ABCD:
    {
        dst[0] = data[1]>>8;
        dst[1] = data[1]& 0xFF;
        dst[2] = data[0]>>8;
        dst[3] = data[0]& 0xFF;
    }
        break;
    case order_float_CDAB:
    {
        dst[0] = data[0]>>8;
        dst[1] = data[0]& 0xFF;
        dst[2] = data[1]>>8;
        dst[3] = data[1]& 0xFF;
    }
        break;
    case order_float_DCBA:
    {
        dst[0] = data[0]& 0xFF;
        dst[1] = data[0]>>8;
        dst[2] = data[1]& 0xFF;
        dst[3] = data[1]>>8;
    }
        break;
    case order_float_BADC:
    {
        dst[0] = data[1]& 0xFF;
        dst[1] = data[1]>>8;
        dst[2] = data[0]& 0xFF;
        dst[3] = data[0]>>8;
    }
        break;
    default:
        std::cout<<"floatOrder is not init";
        break;
    }
}


void trunUint8ToUint64(uint64_t *dst, uint8_t *src, short int64Order)
{
    uint16_t data[4] = {0xFFFF};

    switch (int64Order) {
    case order_64_ABCDEFGH:
    {
        data[3] = (src[0]<<8) + src[1];
        data[2] = (src[2]<<8) + src[3];
        data[1] = (src[4]<<8) + src[5];
        data[0] = (src[6]<<8) + src[7];
    }
        break;
    case order_64_EFGHABCD:
    {
        data[1] = (src[0]<<8) + src[1];
        data[0] = (src[2]<<8) + src[3];
        data[3] = (src[4]<<8) + src[5];
        data[2] = (src[6]<<8) + src[7];
    }
        break;
    case order_64_HGFEDCBA:
    {
        data[0] = (src[1]<<8) + src[0];
        data[1] = (src[3]<<8) + src[2];
        data[2] = (src[5]<<8) + src[4];
        data[3] = (src[7]<<8) + src[6];
    }
        break;
    case order_64_GHEFCDAB:
    {
        data[0] = (src[0]<<8) + src[1];
        data[1] = (src[2]<<8) + src[3];
        data[2] = (src[4]<<8) + src[5];
        data[3] = (src[6]<<8) + src[7];
    }
        break;
    default:
        std::cout<<"int64Order is not init";
        break;
    }
    memcpy(dst,data,sizeof(uint64_t));
}


void trunUint64ToUint8(uint8_t *dst, uint64_t *src, short int64Order)
{
    uint16_t data[4] = {0xFFFF};
    memcpy(data,src,sizeof(uint64_t));

    switch (int64Order) {
    case order_64_ABCDEFGH:
    {
        dst[0] = data[3]>>8;
        dst[1] = data[3]& 0xFF;
        dst[2] = data[2]>>8;
        dst[3] = data[2]& 0xFF;

        dst[4] = data[1]>>8;
        dst[5] = data[1]& 0xFF;
        dst[6] = data[0]>>8;
        dst[7] = data[0]& 0xFF;
    }
        break;
    case order_64_EFGHABCD:
    {
        dst[0] = data[1]>>8;
        dst[1] = data[1]& 0xFF;
        dst[2] = data[0]>>8;
        dst[3] = data[0]& 0xFF;

        dst[4] = data[3]>>8;
        dst[5] = data[3]& 0xFF;
        dst[6] = data[2]>>8;
        dst[7] = data[2]& 0xFF;
    }
        break;
    case order_64_HGFEDCBA:
    {
        dst[0] = data[0]& 0xFF;
        dst[1] = data[0]>>8;
        dst[2] = data[1]& 0xFF;
        dst[3] = data[1]>>8;

        dst[4] = data[2]& 0xFF;
        dst[5] = data[2]>>8;
        dst[6] = data[3]& 0xFF;
        dst[7] = data[3]>>8;
    }
        break;
    case order_64_GHEFCDAB:
    {
        dst[1] = data[0]& 0xFF;
        dst[0] = data[0]>>8;
        dst[3] = data[1]& 0xFF;
        dst[2] = data[1]>>8;

        dst[5] = data[2]& 0xFF;
        dst[4] = data[2]>>8;
        dst[7] = data[3]& 0xFF;
        dst[6] = data[3]>>8;
    }
        break;
    default:
        std::cout<<"int64Order is not init";
        break;
    }
}


void trunUint8ToDouble64(double *dst, uint8_t *src, short doubleOrder)
{
    uint16_t data[4] = {0xFFFF};

    switch (doubleOrder) {
    case order_double_ABCDEFGH:
    {
        data[3] = (src[0]<<8) + src[1];
        data[2] = (src[2]<<8) + src[3];
        data[1] = (src[4]<<8) + src[5];
        data[0] = (src[6]<<8) + src[7];
    }
        break;
    case order_double_EFGHABCD:
    {
        data[1] = (src[0]<<8) + src[1];
        data[0] = (src[2]<<8) + src[3];
        data[3] = (src[4]<<8) + src[5];
        data[2] = (src[6]<<8) + src[7];
    }
        break;
    case order_double_HGFEDCBA:
    {
        data[0] = (src[1]<<8) + src[0];
        data[1] = (src[3]<<8) + src[2];
        data[2] = (src[5]<<8) + src[4];
        data[3] = (src[7]<<8) + src[6];
    }
        break;
    case order_double_GHEFCDAB:
    {
        data[0] = (src[0]<<8) + src[1];
        data[1] = (src[2]<<8) + src[3];
        data[2] = (src[4]<<8) + src[5];
        data[3] = (src[6]<<8) + src[7];
    }
        break;
    default:
        std::cout<<"doubleOrder is not init"<<std::endl;
        break;
    }
    memcpy(dst,data,sizeof(uint64_t));
}


void trunDouble64ToUint8(uint8_t *dst, double *src, short doubleOrder)
{
    uint16_t data[4] = {0xFFFF};
    memcpy(data,src,sizeof(uint64_t));

    switch (doubleOrder) {
    case order_double_ABCDEFGH:
    {
        dst[0] = data[3]>>8;
        dst[1] = data[3]& 0xFF;
        dst[2] = data[2]>>8;
        dst[3] = data[2]& 0xFF;

        dst[4] = data[1]>>8;
        dst[5] = data[1]& 0xFF;
        dst[6] = data[0]>>8;
        dst[7] = data[0]& 0xFF;
    }
        break;
    case order_double_EFGHABCD:
    {
        dst[0] = data[1]>>8;
        dst[1] = data[1]& 0xFF;
        dst[2] = data[0]>>8;
        dst[3] = data[0]& 0xFF;

        dst[4] = data[3]>>8;
        dst[5] = data[3]& 0xFF;
        dst[6] = data[2]>>8;
        dst[7] = data[2]& 0xFF;
    }
        break;
    case order_double_HGFEDCBA:
    {
        dst[0] = data[0]& 0xFF;
        dst[1] = data[0]>>8;
        dst[2] = data[1]& 0xFF;
        dst[3] = data[1]>>8;

        dst[4] = data[2]& 0xFF;
        dst[5] = data[2]>>8;
        dst[6] = data[3]& 0xFF;
        dst[7] = data[3]>>8;
    }
        break;
    case order_double_GHEFCDAB:
    {
        dst[1] = data[0]& 0xFF;
        dst[0] = data[0]>>8;
        dst[3] = data[1]& 0xFF;
        dst[2] = data[1]>>8;

        dst[5] = data[2]& 0xFF;
        dst[4] = data[2]>>8;
        dst[7] = data[3]& 0xFF;
        dst[6] = data[3]>>8;
    }
        break;
    default:
        std::cout<<"doubleOrder is not init";
        break;
    }
}


void verifyTrunToByteOrder()
{
    //uint16_t data = 0x1234;
    //uint32_t data = 0x12345678;
    uint64_t data = 0x123456789ABCDEFF;

    std::cout<<data;


    uint8_t temp[8] = {0};
    memcpy(temp,&data,sizeof(uint64_t));

    for(int i=0;i<8;i++){
        printf("%x",temp[i]);
    }

    //trunUint16ToUint8(temp,&data,order_16_BBAA);
    //trunUint32ToUint8(temp,&data,order_32_CDAB);
    trunUint64ToUint8(temp,&data,order_64_ABCDEFGH);

    for(int i=0;i<8;i++){
        printf("------%x",temp[i]);
    }

    //trunUint8ToUint16(&data,temp,order_16_BBAA);
    //trunUint8ToUint32(&data,temp,order_32_CDAB);
    trunUint8ToUint64(&data,temp,order_64_ABCDEFGH);

    std::cout<<data;
}
