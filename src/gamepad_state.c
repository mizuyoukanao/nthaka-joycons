#include "nthaka.h"

void nthaka_gamepad_state_copy(nthaka_gamepad_state_t *dst, nthaka_gamepad_state_t *src)
{
    if (dst == NULL || src == NULL)
    {
        return;
    }

    dst->y = src->y;
    dst->b = src->b;
    dst->a = src->a;
    dst->x = src->x;
    dst->l = src->l;
    dst->r = src->r;
    dst->zl = src->zl;
    dst->zr = src->zr;
    dst->minus = src->minus;
    dst->plus = src->plus;
    dst->l_click = src->l_click;
    dst->r_click = src->r_click;
    dst->home = src->home;
    dst->capture = src->capture;
    dst->hat = src->hat;
    dst->l_stick.x = src->l_stick.x;
    dst->l_stick.y = src->l_stick.y;
    dst->r_stick.x = src->r_stick.x;
    dst->r_stick.y = src->r_stick.y;
    dst->ext[0] = src->ext[0];
    dst->ext[1] = src->ext[1];
    dst->ext[2] = src->ext[2];
    dst->ext[3] = src->ext[3];
    dst->ext[4] = src->ext[4];
    dst->ext[5] = src->ext[5];
    dst->ext[6] = src->ext[6];
    dst->ext[7] = src->ext[7];
    dst->ext[8] = src->ext[8];
    dst->ext[9] = src->ext[9];
    dst->ext[10] = src->ext[10];
    dst->ext[11] = src->ext[11];
    dst->ext[12] = src->ext[12];
    dst->ext[13] = src->ext[13];
    dst->ext[14] = src->ext[14];
    dst->ext[15] = src->ext[15];
}