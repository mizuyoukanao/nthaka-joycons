#ifndef NTHAKA_H_
#define NTHAKA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

    typedef enum nthaka_button_state_t
    {
        NTHAKA_BUTTON_RELEASED,
        NTHAKA_BUTTON_PRESSED
    } nthaka_button_state_t;

#define nthaka_button_state_stringify(btn) ((btn) == NTHAKA_BUTTON_RELEASED  ? "RELEASED" \
                                            : (btn) == NTHAKA_BUTTON_PRESSED ? "PRESSED"  \
                                                                             : "unknown")

    typedef enum nthaka_hat_state_t
    {
        NTHAKA_HAT_UP,
        NTHAKA_HAT_UPRIGHT,
        NTHAKA_HAT_RIGHT,
        NTHAKA_HAT_DOWNRIGHT,
        NTHAKA_HAT_DOWN,
        NTHAKA_HAT_DOWNLEFT,
        NTHAKA_HAT_LEFT,
        NTHAKA_HAT_UPLEFT,
        NTHAKA_HAT_NEUTRAL
    } nthaka_hat_state_t;

#define nthaka_hat_state_stringify(hat) ((hat) == NTHAKA_HAT_UP          ? "UP"        \
                                         : (hat) == NTHAKA_HAT_UPRIGHT   ? "UPRIGHT"   \
                                         : (hat) == NTHAKA_HAT_RIGHT     ? "RIGHT"     \
                                         : (hat) == NTHAKA_HAT_DOWNRIGHT ? "DOWNRIGHT" \
                                         : (hat) == NTHAKA_HAT_DOWN      ? "DOWN"      \
                                         : (hat) == NTHAKA_HAT_DOWNLEFT  ? "DOWNLEFT"  \
                                         : (hat) == NTHAKA_HAT_LEFT      ? "LEFT"      \
                                         : (hat) == NTHAKA_HAT_UPLEFT    ? "UPLEFT"    \
                                         : (hat) == NTHAKA_HAT_NEUTRAL   ? "NEUTRAL"   \
                                                                         : "unknown")

    typedef struct nthaka_stick_state_t
    {
        uint8_t x;
        uint8_t y;
    } nthaka_stick_state_t;

#define NTHAKA_STICK_AXIS_NEUTRAL 128U
#define NTHAKA_STICK_NEUTRAL ((nthaka_stick_state_t){.x = NTHAKA_STICK_AXIS_NEUTRAL, \
                                                     .y = NTHAKA_STICK_AXIS_NEUTRAL})

    typedef struct nthaka_gamepad_state_t
    {
        nthaka_button_state_t y;
        nthaka_button_state_t b;
        nthaka_button_state_t a;
        nthaka_button_state_t x;
        nthaka_button_state_t l;
        nthaka_button_state_t r;
        nthaka_button_state_t zl;
        nthaka_button_state_t zr;
        nthaka_button_state_t minus;
        nthaka_button_state_t plus;
        nthaka_button_state_t l_click;
        nthaka_button_state_t r_click;
        nthaka_button_state_t home;
        nthaka_button_state_t capture;

        nthaka_hat_state_t hat;

        nthaka_stick_state_t l_stick;
        nthaka_stick_state_t r_stick;

        uint8_t ext[16];
    } nthaka_gamepad_state_t;

#define NTHAKA_GAMEPAD_NEUTRAL ((nthaka_gamepad_state_t){.y = NTHAKA_BUTTON_RELEASED,       \
                                                         .b = NTHAKA_BUTTON_RELEASED,       \
                                                         .a = NTHAKA_BUTTON_RELEASED,       \
                                                         .x = NTHAKA_BUTTON_RELEASED,       \
                                                         .l = NTHAKA_BUTTON_RELEASED,       \
                                                         .r = NTHAKA_BUTTON_RELEASED,       \
                                                         .zl = NTHAKA_BUTTON_RELEASED,      \
                                                         .zr = NTHAKA_BUTTON_RELEASED,      \
                                                         .minus = NTHAKA_BUTTON_RELEASED,   \
                                                         .plus = NTHAKA_BUTTON_RELEASED,    \
                                                         .l_click = NTHAKA_BUTTON_RELEASED, \
                                                         .r_click = NTHAKA_BUTTON_RELEASED, \
                                                         .home = NTHAKA_BUTTON_RELEASED,    \
                                                         .capture = NTHAKA_BUTTON_RELEASED, \
                                                         .hat = NTHAKA_HAT_NEUTRAL,         \
                                                         .l_stick = NTHAKA_STICK_NEUTRAL,   \
                                                         .r_stick = NTHAKA_STICK_NEUTRAL,   \
                                                         .ext = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}})

    void nthaka_gamepad_state_copy(nthaka_gamepad_state_t *dst, nthaka_gamepad_state_t *src);

#define nthaka_gamepad_state_stringify(s, out, size) ((s) != NULL ? (snprintf((out), (size), "{.y=%s,"                                                  \
                                                                                             ".b=%s,"                                                   \
                                                                                             ".a=%s,"                                                   \
                                                                                             ".x=%s,"                                                   \
                                                                                             ".l=%s,"                                                   \
                                                                                             ".r=%s,"                                                   \
                                                                                             ".zl=%s,"                                                  \
                                                                                             ".zr=%s,"                                                  \
                                                                                             ".minus=%s,"                                               \
                                                                                             ".plus=%s,"                                                \
                                                                                             ".l_click=%s,"                                             \
                                                                                             ".r_click=%s,"                                             \
                                                                                             ".home=%s,"                                                \
                                                                                             ".capture=%s,"                                             \
                                                                                             ".hat=%s,"                                                 \
                                                                                             ".l_stick={.x=%d,.y=%d},"                                  \
                                                                                             ".r_stick={.x=%d,.y=%d},"                                  \
                                                                                             ".ext={%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}}", \
                                                                              nthaka_button_state_stringify((s)->y),                                    \
                                                                              nthaka_button_state_stringify((s)->b),                                    \
                                                                              nthaka_button_state_stringify((s)->a),                                    \
                                                                              nthaka_button_state_stringify((s)->x),                                    \
                                                                              nthaka_button_state_stringify((s)->l),                                    \
                                                                              nthaka_button_state_stringify((s)->r),                                    \
                                                                              nthaka_button_state_stringify((s)->zl),                                   \
                                                                              nthaka_button_state_stringify((s)->zr),                                   \
                                                                              nthaka_button_state_stringify((s)->minus),                                \
                                                                              nthaka_button_state_stringify((s)->plus),                                 \
                                                                              nthaka_button_state_stringify((s)->l_click),                              \
                                                                              nthaka_button_state_stringify((s)->r_click),                              \
                                                                              nthaka_button_state_stringify((s)->home),                                 \
                                                                              nthaka_button_state_stringify((s)->capture),                              \
                                                                              nthaka_hat_state_stringify((s)->hat),                                     \
                                                                              (s)->l_stick.x,                                                           \
                                                                              (s)->l_stick.y,                                                           \
                                                                              (s)->r_stick.x,                                                           \
                                                                              (s)->r_stick.y,                                                           \
                                                                              (s)->ext[0],                                                              \
                                                                              (s)->ext[1],                                                              \
                                                                              (s)->ext[2],                                                              \
                                                                              (s)->ext[3],                                                              \
                                                                              (s)->ext[4],                                                              \
                                                                              (s)->ext[5],                                                              \
                                                                              (s)->ext[6],                                                              \
                                                                              (s)->ext[7],                                                              \
                                                                              (s)->ext[8],                                                              \
                                                                              (s)->ext[9],                                                              \
                                                                              (s)->ext[10],                                                             \
                                                                              (s)->ext[11],                                                             \
                                                                              (s)->ext[12],                                                             \
                                                                              (s)->ext[13],                                                             \
                                                                              (s)->ext[14],                                                             \
                                                                              (s)->ext[15]),                                                            \
                                                                     (void)0)                                                                           \
                                                                  : (void)0)

    // char max[] = "{.y=RELEASED,.b=RELEASED,.a=RELEASED,.x=RELEASED,.l=RELEASED,.r=RELEASED,.zl=RELEASED,.zr=RELEASED,.minus=RELEASED,.plus=RELEASED,.l_click=RELEASED,.r_click=RELEASED,.home=RELEASED,.capture=RELEASED,.hat=DOWNRIGHT,.l_stick={.x=255,.y=255},.r_stick={.x=255,.y=255},.ext={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255}}";
#define NTHAKA_GAMEPAD_STATE_STRING_SIZE_MAX 336

    typedef enum nthaka_buffer_state_t
    {
        NTHAKA_BUFFER_PENDING,
        NTHAKA_BUFFER_REJECTED,
        NTHAKA_BUFFER_ACCEPTED
    } nthaka_buffer_state_t;

    typedef struct nthaka_format_handler_t
    {
        nthaka_buffer_state_t (*update)(struct nthaka_format_handler_t *fmt, uint8_t d);
        bool (*deserialize)(struct nthaka_format_handler_t *fmt, uint8_t buf[], size_t size, nthaka_gamepad_state_t *out);
        void (*reset)(struct nthaka_format_handler_t *fmt);
    } nthaka_format_handler_t;

#define nthaka_format_handler_update(fmt, d) ((fmt) != NULL ? (fmt)->update((fmt), (d)) \
                                                            : NTHAKA_BUFFER_REJECTED)
#define nthaka_format_handler_deserialize(fmt, buf, size, out) ((fmt) != NULL && \
                                                                (buf) != NULL && \
                                                                (fmt)->deserialize((fmt), (buf), (size), (out)))
#define nthaka_format_handler_reset(fmt) ((fmt) != NULL ? (fmt)->reset((fmt)) \
                                                        : (void)0)

#ifndef NTHAKA_MULTI_FORMAT_SIZE
#define NTHAKA_MULTI_FORMAT_SIZE (size_t)(8)
#endif

    typedef struct nthaka_multi_format_handler_t
    {
        nthaka_format_handler_t parent;

        nthaka_format_handler_t *_fmts[NTHAKA_MULTI_FORMAT_SIZE];
        nthaka_gamepad_state_t _out[NTHAKA_MULTI_FORMAT_SIZE];
        size_t _size;

        size_t *_last_deserialized_index;
        size_t __last_deserialized_index;
    } nthaka_multi_format_handler_t;

    bool nthaka_multi_format_handler_init(nthaka_multi_format_handler_t *fmt, nthaka_format_handler_t *fmts[], size_t size);
    size_t *nthaka_multi_format_handler_get_last_deserialized_index(nthaka_multi_format_handler_t *fmt);

#ifndef NTHAKA_BUFFER_SIZE
#define NTHAKA_BUFFER_SIZE (size_t)(64)
#endif

    typedef struct nthaka_buffer_t
    {
        uint8_t _buf[NTHAKA_BUFFER_SIZE];
        size_t _size;
        nthaka_format_handler_t *_fmt;

        nthaka_buffer_state_t _;
    } nthaka_buffer_t;

#define nthaka_buffer_init(buf, fmt) ((buf) != NULL && \
                                      (fmt) != NULL && \
                                      ((buf)->_size = 0, (buf)->_fmt = (fmt), true))
#define nthaka_buffer_append(buf, d, out) ((buf) != NULL && ((buf)->_ = nthaka_format_handler_update((buf)->_fmt, (d))) != NTHAKA_BUFFER_REJECTED ? (((buf)->_buf[(buf)->_size] = (d), (buf)->_size++, (buf)->_ == NTHAKA_BUFFER_ACCEPTED) ? (nthaka_format_handler_deserialize((buf)->_fmt, (buf)->_buf, (buf)->_size, (out)) ? NTHAKA_BUFFER_ACCEPTED  \
                                                                                                                                                                                                                                                                                                                               : NTHAKA_BUFFER_REJECTED) \
                                                                                                                                                                                                                                           : NTHAKA_BUFFER_PENDING)                                                                                      \
                                                                                                                                                  : NTHAKA_BUFFER_REJECTED)
#define nthaka_buffer_clear(buf) ((buf) != NULL ? ((buf)->_size = 0, nthaka_format_handler_reset((buf)->_fmt)) \
                                                : (void)0)

#ifdef __cplusplus
}
#endif

#endif // NTHAKA_H_