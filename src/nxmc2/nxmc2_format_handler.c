#include "nthaka/nxmc2.h"
#include "../internal.h"

#include <assert.h>

static const uint8_t _HEADER = 0xAB;
static const uint8_t _HAT_MAX = 8;
static const uint8_t _BTNS_MSB_MAX = 0x3F;

static nthaka_buffer_state_t _update(nthaka_format_handler_t *parent, uint8_t d)
{
    nxmc2_format_handler_t *fmt = (nxmc2_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return NTHAKA_BUFFER_REJECTED;
    }

    switch (fmt->_s)
    {
    case NXMC2_FORMAT_INITIAL:
        if (d != _HEADER)
            break;
        fmt->_s++;
        return NTHAKA_BUFFER_PENDING;

    case NXMC2_FORMAT_0xAB_0x00:
        if (_BTNS_MSB_MAX < d)
            break;
        fmt->_s++;
        return NTHAKA_BUFFER_PENDING;

    case NXMC2_FORMAT_0xAB_0x00_0x00:
        if (_HAT_MAX < d)
            break;
        fmt->_s++;
        return NTHAKA_BUFFER_PENDING;

    case NXMC2_FORMAT_0xAB:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80:
    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80_0x00:
        fmt->_s++;
        return NTHAKA_BUFFER_PENDING;

    case NXMC2_FORMAT_0xAB_0x00_0x00_0x08_0x80_0x80_0x80_0x80_0x00_0x00:
        fmt->_s++;
        return NTHAKA_BUFFER_ACCEPTED;

    case NXMC2_FORMAT_ACCEPTED:
    default:
        break;
    }
    fmt->_s = NXMC2_FORMAT_REJECTED;
    return NTHAKA_BUFFER_REJECTED;
}

static void _reset(nthaka_format_handler_t *parent)
{
    nxmc2_format_handler_t *fmt = (nxmc2_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return;
    }

    fmt->_s = NXMC2_FORMAT_INITIAL;
}

static bool _deserialize(nthaka_format_handler_t *parent, uint8_t *buf, size_t size, nthaka_gamepad_state_t *out)
{
    nxmc2_format_handler_t *fmt = (nxmc2_format_handler_t *)parent;
    if (fmt == NULL || buf == NULL || fmt->_s != NXMC2_FORMAT_ACCEPTED)
    {
        return false;
    }

    assert(size == 11);
    assert(buf[0] == _HEADER);
    assert(buf[2] <= _BTNS_MSB_MAX);
    assert(buf[3] <= _HAT_MAX);

    if (out == NULL)
    {
        // This deserialization has no side effects.
        // If we are not writing to out, there is no need to deserialize in the first place.
        return true;
    }

    uint8_t btns_lsb = buf[NXMC2_BUFFER_INDEX_BUTTONS_LSB];
    out->y = nthaka_internal_bit(btns_lsb, 0);
    out->b = nthaka_internal_bit(btns_lsb, 1);
    out->a = nthaka_internal_bit(btns_lsb, 2);
    out->x = nthaka_internal_bit(btns_lsb, 3);
    out->l = nthaka_internal_bit(btns_lsb, 4);
    out->r = nthaka_internal_bit(btns_lsb, 5);
    out->zl = nthaka_internal_bit(btns_lsb, 6);
    out->zr = nthaka_internal_bit(btns_lsb, 7);

    uint8_t btns_msb = buf[NXMC2_BUFFER_INDEX_BUTTONS_MSB];
    out->minus = nthaka_internal_bit(btns_msb, 0);
    out->plus = nthaka_internal_bit(btns_msb, 1);
    out->l_click = nthaka_internal_bit(btns_msb, 2);
    out->r_click = nthaka_internal_bit(btns_msb, 3);
    out->home = nthaka_internal_bit(btns_msb, 4);
    out->capture = nthaka_internal_bit(btns_msb, 5);

    out->hat = buf[NXMC2_BUFFER_INDEX_HAT];

    out->l_stick.x = buf[NXMC2_BUFFER_INDEX_LX];
    out->l_stick.y = buf[NXMC2_BUFFER_INDEX_LY];
    out->r_stick.x = buf[NXMC2_BUFFER_INDEX_RX];
    out->r_stick.y = buf[NXMC2_BUFFER_INDEX_RY];

    out->ext[0] = buf[NXMC2_BUFFER_INDEX_EXTENSION_0];
    out->ext[1] = buf[NXMC2_BUFFER_INDEX_EXTENSION_1];
    out->ext[2] = buf[NXMC2_BUFFER_INDEX_EXTENSION_2];

    for (size_t i = 3; i < nthaka_internal_size(out->ext); i++)
    {
        out->ext[i] = 0;
    }

    return true;
}

bool nxmc2_format_handler_init(nxmc2_format_handler_t *fmt)
{
    if (fmt == NULL)
    {
        return false;
    }

    fmt->parent.deserialize = _deserialize;
    fmt->parent.reset = _reset;
    fmt->parent.update = _update;

    fmt->_s = NXMC2_FORMAT_INITIAL;

    return true;
}