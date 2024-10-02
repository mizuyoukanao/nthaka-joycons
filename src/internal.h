#ifndef NTHAKA_INTERNAL_H_
#define NTHAKA_INTERNAL_H_

#define nthaka_internal_bit(d, n) (((d) >> (n)) & 1)
#define nthaka_internal_size(a) (sizeof((a)) / sizeof((a)[0]))

#endif // NTHAKA_INTERNAL_H_