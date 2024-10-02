#include "nthaka/dol.h"

#include <assert.h>

static nthaka_buffer_state_t _update(nthaka_format_handler_t *parent, uint8_t d)
{
    dol_format_handler_t *fmt = (dol_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return NTHAKA_BUFFER_REJECTED;
    }

    switch (fmt->_s)
    {
    case DOL_FORMAT_INITIAL:
        if (d == '@' ||
            ('a' <= d && d <= 'z') ||
            ('0' <= d && d <= '9'))
        {
            fmt->_s = DOL_FORMAT_ACCEPTED;
            return NTHAKA_BUFFER_ACCEPTED;
        }
        else
        {
            fmt->_s = DOL_FORMAT_REJECTED;
            return NTHAKA_BUFFER_REJECTED;
        }

    case DOL_FORMAT_ACCEPTED:
    case DOL_FORMAT_REJECTED:
    default:
        fmt->_s = DOL_FORMAT_REJECTED;
        return NTHAKA_BUFFER_REJECTED;
    }
}

static void _reset(nthaka_format_handler_t *parent)
{
    dol_format_handler_t *fmt = (dol_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return;
    }

    fmt->_s = DOL_FORMAT_INITIAL;
}

static nthaka_gamepad_state_t _;

static bool _deserialize(nthaka_format_handler_t *parent, uint8_t *buf, size_t size, nthaka_gamepad_state_t *out)
{
    dol_format_handler_t *fmt = (dol_format_handler_t *)parent;
    if (fmt == NULL || buf == NULL || fmt->_s != DOL_FORMAT_ACCEPTED)
    {
        return false;
    }
    assert(size == 1);

    if (out == NULL)
    {
        out = &_;
    }

    nthaka_gamepad_state_copy(out, &fmt->_prev);

    if (buf[0] != '@')
    {
        out->home = NTHAKA_BUTTON_RELEASED;
    }

    uint8_t dleft = 0;
    uint8_t dright = 0;
    uint8_t dup = 0;
    uint8_t ddown = 0;
    switch (out->hat)
    {
    case NTHAKA_HAT_UP:
        dleft = 0;
        dright = 0;
        dup = 1;
        ddown = 0;
        break;
    case NTHAKA_HAT_UPRIGHT:
        dleft = 0;
        dright = 1;
        dup = 1;
        ddown = 0;
        break;
    case NTHAKA_HAT_RIGHT:
        dleft = 0;
        dright = 1;
        dup = 0;
        ddown = 0;
        break;
    case NTHAKA_HAT_DOWNRIGHT:
        dleft = 0;
        dright = 1;
        dup = 0;
        ddown = 1;
        break;
    case NTHAKA_HAT_DOWN:
        dleft = 0;
        dright = 0;
        dup = 0;
        ddown = 1;
        break;
    case NTHAKA_HAT_DOWNLEFT:
        dleft = 1;
        dright = 0;
        dup = 0;
        ddown = 1;
        break;
    case NTHAKA_HAT_LEFT:
        dleft = 1;
        dright = 0;
        dup = 0;
        ddown = 0;
        break;
    case NTHAKA_HAT_UPLEFT:
        dleft = 1;
        dright = 0;
        dup = 1;
        ddown = 0;
        break;
    case NTHAKA_HAT_NEUTRAL:
        dleft = 0;
        dright = 0;
        dup = 0;
        ddown = 0;
        break;

    default:
        assert(false);
        break;
    }

    switch (buf[0])
    {
    case '@':
        out->home = NTHAKA_BUTTON_PRESSED;
        break;
    case 'a':
        out->a = NTHAKA_BUTTON_PRESSED;
        break;
    case 'b':
        out->b = NTHAKA_BUTTON_PRESSED;
        break;
    case 'c':
        out->x = NTHAKA_BUTTON_PRESSED;
        break;
    case 'd':
        out->y = NTHAKA_BUTTON_PRESSED;
        break;
    case 'e':
        out->l = NTHAKA_BUTTON_PRESSED;
        break;
    case 'f':
        out->r = NTHAKA_BUTTON_PRESSED;
        break;
    case 'g':
        out->zr = NTHAKA_BUTTON_PRESSED;
        break;
    case 'h':
        out->plus = NTHAKA_BUTTON_PRESSED;
        break;
    case 'i':
        dleft = 1;
        break;
    case 'j':
        dright = 1;
        break;
    case 'k':
        ddown = 1;
        break;
    case 'l':
        dup = 1;
        break;

    case 'm':
        out->a = NTHAKA_BUTTON_RELEASED;
        break;
    case 'n':
        out->b = NTHAKA_BUTTON_RELEASED;
        break;
    case 'o':
        out->x = NTHAKA_BUTTON_RELEASED;
        break;
    case 'p':
        out->y = NTHAKA_BUTTON_RELEASED;
        break;
    case 'q':
        out->l = NTHAKA_BUTTON_RELEASED;
        break;
    case 'r':
        out->r = NTHAKA_BUTTON_RELEASED;
        break;
    case 's':
        out->zr = NTHAKA_BUTTON_RELEASED;
        break;
    case 't':
        out->plus = NTHAKA_BUTTON_RELEASED;
        break;
    case 'u':
        dleft = 0;
        break;
    case 'v':
        dright = 0;
        break;
    case 'w':
        ddown = 0;
        break;
    case 'x':
        dup = 0;
        break;

    case 'y':
        out->l_stick.x = 0;
        break;
    case 'z':
        out->l_stick.x = 128;
        break;
    case '1':
        out->l_stick.x = 255;
        break;
    case '2':
        out->l_stick.y = 0;
        break;
    case '3':
        out->l_stick.y = 128;
        break;
    case '4':
        out->l_stick.y = 255;
        break;

    case '5':
        out->r_stick.x = 0;
        break;
    case '6':
        out->r_stick.x = 128;
        break;
    case '7':
        out->r_stick.x = 255;
        break;
    case '8':
        out->r_stick.y = 0;
        break;
    case '9':
        out->r_stick.y = 128;
        break;
    case '0':
        out->r_stick.y = 255;
        break;

    default:
        assert(false);
        break;
    }

    switch (dup << 3 | dright << 2 | ddown << 1 | dleft)
    {
    case 0x8 /*0b1000*/:
        out->hat = NTHAKA_HAT_UP;
        break;
    case 0xC /*0b1100*/:
        out->hat = NTHAKA_HAT_UPRIGHT;
        break;
    case 0x4 /*0b0100*/:
        out->hat = NTHAKA_HAT_RIGHT;
        break;
    case 0x6 /*0b0110*/:
        out->hat = NTHAKA_HAT_DOWNRIGHT;
        break;
    case 0x2 /*0b0010*/:
        out->hat = NTHAKA_HAT_DOWN;
        break;
    case 0x3 /*0b0011*/:
        out->hat = NTHAKA_HAT_DOWNLEFT;
        break;
    case 0x1 /*0b0001*/:
        out->hat = NTHAKA_HAT_LEFT;
        break;
    case 0x9 /*0b1001*/:
        out->hat = NTHAKA_HAT_UPLEFT;
        break;
    default:
        out->hat = NTHAKA_HAT_NEUTRAL;
        break;
    }

    nthaka_gamepad_state_copy(&fmt->_prev, out);

    return true;
}

bool dol_format_handler_init(dol_format_handler_t *fmt)
{
    if (fmt == NULL)
    {
        return false;
    }

    fmt->parent.deserialize = _deserialize;
    fmt->parent.reset = _reset;
    fmt->parent.update = _update;

    fmt->_s = DOL_FORMAT_INITIAL;
    nthaka_gamepad_state_copy(&(fmt->_prev), &NTHAKA_GAMEPAD_NEUTRAL);

    return true;
}