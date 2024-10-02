#include "nthaka/orca.h"
#include "../internal.h"

#include <assert.h>

static const uint8_t _HEADER = 0x80;
static const uint8_t _SERVO = '@';
static const uint8_t _RELEASE_ALL = 0x80;
static const uint8_t _IGNORE = 0xFF;

static nthaka_buffer_state_t _update(nthaka_format_handler_t *parent, uint8_t d)
{
    orca_format_handler_t *fmt = (orca_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return NTHAKA_BUFFER_REJECTED;
    }

    switch (fmt->_s)
    {
    case ORCA_FORMAT_INITIAL:
        if (d == _SERVO)
        {
            fmt->_s = ORCA_FORMAT_ACCEPTED;
            return NTHAKA_BUFFER_ACCEPTED;
        }
        else if (d != _HEADER)
        {
            break;
        }
        fmt->_s++;
        return NTHAKA_BUFFER_PENDING;

    case ORCA_FORMAT_0x80:
        if (d != _IGNORE)
        {
            fmt->_s++;
        }
        return NTHAKA_BUFFER_PENDING;

    case ORCA_FORMAT_0x80_0x00:
        if (d != _IGNORE)
        {
            fmt->_s++;
            return NTHAKA_BUFFER_ACCEPTED;
        }
        else
        {
            return NTHAKA_BUFFER_PENDING;
        }

    case ORCA_FORMAT_ACCEPTED:
    default:
        break;
    }
    fmt->_s = ORCA_FORMAT_REJECTED;
    return NTHAKA_BUFFER_REJECTED;
}

static void _reset(nthaka_format_handler_t *parent)
{
    orca_format_handler_t *fmt = (orca_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return;
    }

    fmt->_s = ORCA_FORMAT_INITIAL;
}

static nthaka_gamepad_state_t _;

static bool _deserialize(nthaka_format_handler_t *parent, uint8_t *buf, size_t size, nthaka_gamepad_state_t *out)
{
    orca_format_handler_t *fmt = (orca_format_handler_t *)parent;
    if (fmt == NULL || buf == NULL || fmt->_s != ORCA_FORMAT_ACCEPTED)
    {
        return false;
    }

    if (out == NULL)
    {
        out = &_;
    }

    nthaka_gamepad_state_copy(out, &fmt->_prev);

    if (size == 1)
    {
        assert(buf[0] == _SERVO);

        out->home = NTHAKA_BUTTON_PRESSED;
    }
    else
    {
        assert(buf[0] == _HEADER);

        uint8_t buf_[size];
        for (size_t i = 0, j = 0; i < size; i++)
        {
            if (buf[i] != 0xFF)
            {
                buf_[j++] = buf[i];
            }
        }

        out->home = NTHAKA_BUTTON_RELEASED;

        uint8_t btns0 = buf_[1];
        if (btns0 == _RELEASE_ALL)
        {
            out->a = NTHAKA_BUTTON_RELEASED;
            out->b = NTHAKA_BUTTON_RELEASED;
            out->x = NTHAKA_BUTTON_RELEASED;
            out->y = NTHAKA_BUTTON_RELEASED;
            out->l = NTHAKA_BUTTON_RELEASED;
            out->r = NTHAKA_BUTTON_RELEASED;
        }
        else
        {
            out->a = nthaka_internal_bit(btns0, 0);
            out->b = nthaka_internal_bit(btns0, 1);
            out->x = nthaka_internal_bit(btns0, 2);
            out->y = nthaka_internal_bit(btns0, 3);
            out->l = nthaka_internal_bit(btns0, 4);
            out->r = nthaka_internal_bit(btns0, 5);
        }

        uint8_t dleft = 0;
        uint8_t dright = 0;
        uint8_t dup = 0;
        uint8_t ddown = 0;

        uint8_t btns1 = buf_[2];
        if (btns1 == _RELEASE_ALL)
        {
            out->zr = NTHAKA_BUTTON_RELEASED;
            out->plus = NTHAKA_BUTTON_RELEASED;
            dleft = 0;
            dright = 0;
            dup = 0;
            ddown = 0;
        }
        else
        {
            out->zr = nthaka_internal_bit(btns1, 0);
            out->plus = nthaka_internal_bit(btns1, 1);
            dleft = nthaka_internal_bit(btns1, 2);
            dright = nthaka_internal_bit(btns1, 3);
            dup = nthaka_internal_bit(btns1, 4);
            ddown = nthaka_internal_bit(btns1, 5);
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
    }

    nthaka_gamepad_state_copy(&fmt->_prev, out);

    return true;
}

bool orca_format_handler_init(orca_format_handler_t *fmt)
{
    if (fmt == NULL)
    {
        return false;
    }

    fmt->parent.deserialize = _deserialize;
    fmt->parent.reset = _reset;
    fmt->parent.update = _update;

    fmt->_s = ORCA_FORMAT_INITIAL;
    nthaka_gamepad_state_copy(&(fmt->_prev), &NTHAKA_GAMEPAD_NEUTRAL);

    return true;
}