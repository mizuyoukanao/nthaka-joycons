#ifndef NTHAKA_POKECON_H_
#define NTHAKA_POKECON_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "../nthaka.h"

    typedef enum pokecon_format_state_t
    {
        POKECON_FORMAT_STATE_INITIAL,
        POKECON_FORMAT_STATE_0,
        POKECON_FORMAT_STATE_1,
        POKECON_FORMAT_STATE_10,
        POKECON_FORMAT_STATE_100,
        POKECON_FORMAT_STATE_1000,
        POKECON_FORMAT_STATE_1001,
        POKECON_FORMAT_STATE_1003,
        POKECON_FORMAT_STATE_1007,
        POKECON_FORMAT_STATE_101,
        POKECON_FORMAT_STATE_103,
        POKECON_FORMAT_STATE_107,
        POKECON_FORMAT_STATE_11,
        POKECON_FORMAT_STATE_13,
        POKECON_FORMAT_STATE_17,
        POKECON_FORMAT_STATE_3,
        POKECON_FORMAT_STATE_4,
        POKECON_FORMAT_STATE_7,
        POKECON_FORMAT_STATE_0x,
        POKECON_FORMAT_STATE_0x0,
        POKECON_FORMAT_STATE_0x00,
        POKECON_FORMAT_STATE_0x000,
        POKECON_FORMAT_STATE_0_,
        POKECON_FORMAT_STATE_0_0,
        POKECON_FORMAT_STATE_1_,
        POKECON_FORMAT_STATE_1_0,
        POKECON_FORMAT_STATE_3_,
        POKECON_FORMAT_STATE_3_0,
        POKECON_FORMAT_STATE_7_,
        POKECON_FORMAT_STATE_7_0,
        POKECON_FORMAT_STATE_1_0_,
        POKECON_FORMAT_STATE_1_0_0,
        POKECON_FORMAT_STATE_1_0_1,
        POKECON_FORMAT_STATE_1_0_10,
        POKECON_FORMAT_STATE_1_0_0x,
        POKECON_FORMAT_STATE_1_0_0_,
        POKECON_FORMAT_STATE_1_0_0_0,
        POKECON_FORMAT_STATE_1_0_0_1,
        POKECON_FORMAT_STATE_1_0_0_10,
        POKECON_FORMAT_STATE_1_0_0_0x,
        POKECON_FORMAT_STATE_7_0_,
        POKECON_FORMAT_STATE_7_0_0,
        POKECON_FORMAT_STATE_7_0_1,
        POKECON_FORMAT_STATE_7_0_10,
        POKECON_FORMAT_STATE_7_0_0x,
        POKECON_FORMAT_STATE_7_0_0_,
        POKECON_FORMAT_STATE_7_0_0_0,
        POKECON_FORMAT_STATE_7_0_0_1,
        POKECON_FORMAT_STATE_7_0_0_10,
        POKECON_FORMAT_STATE_7_0_0_0x,
        POKECON_FORMAT_STATE_3_0_,
        POKECON_FORMAT_STATE_3_0_0,
        POKECON_FORMAT_STATE_3_0_1,
        POKECON_FORMAT_STATE_3_0_10,
        POKECON_FORMAT_STATE_3_0_0x,
        POKECON_FORMAT_STATE_3_0_0_,
        POKECON_FORMAT_STATE_3_0_0_0,
        POKECON_FORMAT_STATE_3_0_0_1,
        POKECON_FORMAT_STATE_3_0_0_10,
        POKECON_FORMAT_STATE_3_0_0_0x,
        POKECON_FORMAT_STATE_0_0CR,
        POKECON_FORMAT_STATE_e,
        POKECON_FORMAT_STATE_en,
        POKECON_FORMAT_STATE_ACCEPTED,
        POKECON_FORMAT_STATE_REJECTED,
    } pokecon_format_state_t;

    pokecon_format_state_t pokecon_format_state_next(pokecon_format_state_t s, char c);

    typedef struct pokecon_format_handler_t
    {
        nthaka_format_handler_t parent;

        pokecon_format_state_t _s;
        nthaka_stick_state_t _prev_l;
        nthaka_stick_state_t _prev_r;
    } pokecon_format_handler_t;

    bool pokecon_format_handler_init(pokecon_format_handler_t *fmt);

#ifdef __cplusplus
}
#endif

#endif // NTHAKA_POKECON_H_