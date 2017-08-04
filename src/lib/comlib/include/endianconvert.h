#ifndef EVO_ENDIANCONVERT_H
#define EVO_ENDIANCONVERT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif         // __cplusplus

int litter_big_convert(uint8_t* dest_data, const uint8_t* src_data, int length);
uint8_t* litter_big_convert_self(uint8_t* src_data, int length);


#ifdef __cplusplus
}
#endif                /* __cplusplus */


#endif // EVO_INTERFACE_H
