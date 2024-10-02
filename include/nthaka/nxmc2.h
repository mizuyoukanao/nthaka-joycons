#ifndef NTHAKA_NXMC2_H_
#define NTHAKA_NXMC2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "../nthaka.h"

    typedef enum nxmc2_format_state_t
    {
        NXMC2_FORMAT_INITIAL,
        NXMC2_FORMAT_0xAB,
        NXMC2_FORMAT_0xAB_0x00,
        NXMC2_FORMAT_0xAB_0x00_0x00,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80_0x00,
        NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80_0x00_0x00,
        NXMC2_FORMAT_ACCEPTED,
        NXMC2_FORMAT_REJECTED
    } nxmc2_format_state_t;

    typedef enum nxmc2_buffer_index_t
    {
        NXMC2_BUFFER_INDEX_HEADER,
        NXMC2_BUFFER_INDEX_BUTTONS_LSB,
        NXMC2_BUFFER_INDEX_BUTTONS_MSB,
        NXMC2_BUFFER_INDEX_HAT,
        NXMC2_BUFFER_INDEX_LX,
        NXMC2_BUFFER_INDEX_LY,
        NXMC2_BUFFER_INDEX_RX,
        NXMC2_BUFFER_INDEX_RY,
        NXMC2_BUFFER_INDEX_EXTENSION_0,
        NXMC2_BUFFER_INDEX_EXTENSION_1,
        NXMC2_BUFFER_INDEX_EXTENSION_2,
    } nxmc2_buffer_index_t;

    typedef struct nxmc2_format_handler_t
    {
        nthaka_format_handler_t parent;

        nxmc2_format_state_t _s;
    } nxmc2_format_handler_t;

    bool nxmc2_format_handler_init(nxmc2_format_handler_t *fmt);

#ifdef __cplusplus
}
#endif

#endif // NTHAKA_NXMC2_H_