#ifndef NTHAKA_DOL_H_
#define NTHAKA_DOL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "../nthaka.h"

    typedef enum dol_format_state_t
    {
        DOL_FORMAT_INITIAL,
        DOL_FORMAT_ACCEPTED,
        DOL_FORMAT_REJECTED
    } dol_format_state_t;

    typedef struct dol_format_handler_t
    {
        nthaka_format_handler_t parent;

        dol_format_state_t _s;
        nthaka_gamepad_state_t _prev;
    } dol_format_handler_t;

    bool dol_format_handler_init(dol_format_handler_t *fmt);

#ifdef __cplusplus
}
#endif

#endif // NTHAKA_DOL_H_