#include "endianconvert.h"
#include <string.h>

//小端大端转换，用于平台间传输数据
#define             CHARACTOR_LENGTH_MAX            128      //字符大小端转换最大长度

int litter_big_convert(uint8_t* dest_data, const uint8_t* src_data, int length)
{
    if(NULL == dest_data || NULL == src_data)
    {
        return -1;
    }

    int i = 0;
    for(i = 0; i < length; i++)
    {
        dest_data[length - i - 1] = src_data[i];
    }

    return 0;
}


uint8_t* litter_big_convert_self(uint8_t* src_data, int length)
{
    if(NULL == src_data || length > CHARACTOR_LENGTH_MAX)
    {
        return NULL;
    }

    uint8_t dest_data[CHARACTOR_LENGTH_MAX] = {0};
    memcpy(dest_data, src_data, length);
    memset(src_data, 0, length);

    int i = 0;
    for(i = 0; i < length; i++)
    {
        src_data[length - i - 1] = dest_data[i];
    }

    return src_data;
}
