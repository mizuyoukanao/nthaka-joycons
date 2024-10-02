#include "nthaka.h"

#include <assert.h>

static nthaka_buffer_state_t _update(nthaka_format_handler_t *parent, uint8_t d)
{
    nthaka_multi_format_handler_t *fmt = (nthaka_multi_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return NTHAKA_BUFFER_REJECTED;
    }

    fmt->_last_deserialized_index = NULL;

    size_t accepted = 0;
    size_t rejected = 0;
    for (size_t i = 0; i < fmt->_size; i++)
    {
        switch (nthaka_format_handler_update(fmt->_fmts[i], d))
        {
        case NTHAKA_BUFFER_ACCEPTED:
            accepted++;
            break;
        case NTHAKA_BUFFER_REJECTED:
            rejected++;
            break;
        case NTHAKA_BUFFER_PENDING:
        default:
            break;
        }
    }

    if (accepted != 0)
    {
        return NTHAKA_BUFFER_ACCEPTED;
    }
    else if (rejected == fmt->_size)
    {
        return NTHAKA_BUFFER_REJECTED;
    }
    else
    {
        return NTHAKA_BUFFER_PENDING;
    }
}

void _reset(nthaka_format_handler_t *parent)
{
    nthaka_multi_format_handler_t *fmt = (nthaka_multi_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return;
    }

    fmt->_last_deserialized_index = NULL;

    for (size_t i = 0; i < fmt->_size; i++)
    {
        nthaka_format_handler_reset(fmt->_fmts[i]);
    }
}

bool _deserialize(nthaka_format_handler_t *parent, uint8_t *buf, size_t size, nthaka_gamepad_state_t *out)
{
    nthaka_multi_format_handler_t *fmt = (nthaka_multi_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return false;
    }

    fmt->_last_deserialized_index = NULL;

    // First, deserialize on all fmts.
    // Each fmt may have side effects, multiple fmts may be ACCEPTED at the same time.
    bool ret[fmt->_size];
    for (size_t i = 0; i < fmt->_size; i++)
    {
        ret[i] = nthaka_format_handler_deserialize(fmt->_fmts[i], buf, size, &(fmt->_out[i]));
    }

    // Then, return the value from the first true.
    for (size_t i = 0; i < fmt->_size; i++)
    {
        if (ret[i])
        {
            nthaka_gamepad_state_copy(out, &(fmt->_out[i]));

            fmt->__last_deserialized_index = i;
            fmt->_last_deserialized_index = &(fmt->__last_deserialized_index);

            return true;
        }
    }

    return false;
}

size_t *nthaka_multi_format_handler_get_last_deserialized_index(nthaka_multi_format_handler_t *fmt)
{
    if (fmt == NULL)
    {
        return false;
    }

    return fmt->_last_deserialized_index;
}

bool nthaka_multi_format_handler_init(nthaka_multi_format_handler_t *fmt, nthaka_format_handler_t *fmts[], size_t size)
{
    if (fmt == NULL || fmts == NULL || NTHAKA_MULTI_FORMAT_SIZE < size)
    {
        return false;
    }
    for (size_t i = 0; i < size; i++)
    {
        if (fmts[i] == NULL)
        {
            return false;
        }
    }

    fmt->parent.deserialize = _deserialize;
    fmt->parent.reset = _reset;
    fmt->parent.update = _update;

    for (size_t i = 0; i < size; i++)
    {
        fmt->_fmts[i] = fmts[i];
        nthaka_gamepad_state_copy(&(fmt->_out[i]), &NTHAKA_GAMEPAD_NEUTRAL);
    }
    fmt->_size = size;

    fmt->_last_deserialized_index = NULL;

    return true;
}