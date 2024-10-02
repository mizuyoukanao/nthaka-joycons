#ifndef NTHAKA_ORCA_H_
#define NTHAKA_ORCA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "../nthaka.h"

    typedef enum orca_format_state_t
    {
        ORCA_FORMAT_INITIAL,
        ORCA_FORMAT_0x80,
        ORCA_FORMAT_0x80_0x00,
        ORCA_FORMAT_ACCEPTED,
        ORCA_FORMAT_REJECTED
    } orca_format_state_t;

    typedef struct orca_format_handler_t
    {
        nthaka_format_handler_t parent;

        orca_format_state_t _s;
        nthaka_gamepad_state_t _prev;
    } orca_format_handler_t;

    bool orca_format_handler_init(orca_format_handler_t *fmt);

#ifdef __cplusplus
}
#endif

#endif // NTHAKA_ORCA_H_