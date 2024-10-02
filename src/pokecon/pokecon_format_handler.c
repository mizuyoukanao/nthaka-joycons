#include "nthaka/pokecon.h"
#include "../internal.h"

#include <assert.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>

pokecon_format_state_t pokecon_format_state_next(pokecon_format_state_t s, char c)
{
    switch (s)
    {
    case POKECON_FORMAT_STATE_INITIAL:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_0;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
            return POKECON_FORMAT_STATE_1;
        case 'e':
            return POKECON_FORMAT_STATE_e;
        case '3':
            return POKECON_FORMAT_STATE_3;
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_4;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_7;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_0_;
        case 'x':
            return POKECON_FORMAT_STATE_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_en:
        switch (c)
        {
        case 'd':
            return POKECON_FORMAT_STATE_0_0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_e:
        if (c == 'n')
        {
            return POKECON_FORMAT_STATE_en;
        }
    case POKECON_FORMAT_STATE_1:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_10;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_11;
        case '3':
            return POKECON_FORMAT_STATE_13;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_17;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_10:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_0_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_100;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_101;
        case '3':
            return POKECON_FORMAT_STATE_103;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_107;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_100:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_0_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_1000;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_1001;
        case '3':
            return POKECON_FORMAT_STATE_1003;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_1007;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1000:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1001:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1003:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1007:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_101:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_1000;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_1001;
        case '3':
            return POKECON_FORMAT_STATE_1003;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_1007;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_103:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_1000;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_1001;
        case '3':
            return POKECON_FORMAT_STATE_1003;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_1007;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_107:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_1000;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_1001;
        case '3':
            return POKECON_FORMAT_STATE_1003;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_1007;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_11:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_100;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_101;
        case '3':
            return POKECON_FORMAT_STATE_103;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_107;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_13:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_100;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_101;
        case '3':
            return POKECON_FORMAT_STATE_103;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_107;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_17:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_100;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_101;
        case '3':
            return POKECON_FORMAT_STATE_103;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_107;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_10;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_11;
        case '3':
            return POKECON_FORMAT_STATE_13;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_17;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_4:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_0_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_10;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_11;
        case '3':
            return POKECON_FORMAT_STATE_13;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_17;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_;
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_10;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_11;
        case '3':
            return POKECON_FORMAT_STATE_13;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_17;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0x:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_0x0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0x0:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_0x00;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0x00:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_0x000;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0x000:
        switch (c)
        {
        case '0':
        case '4':
        case '8':
        case 'c':
            return POKECON_FORMAT_STATE_1000;
        case '1':
        case '2':
        case '5':
        case '6':
        case '9':
        case 'a':
        case 'd':
        case 'e':
            return POKECON_FORMAT_STATE_1001;
        case '3':
            return POKECON_FORMAT_STATE_1003;
        case '7':
        case 'b':
        case 'f':
            return POKECON_FORMAT_STATE_1007;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0_:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            return POKECON_FORMAT_STATE_0_0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0_0:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            return POKECON_FORMAT_STATE_1_0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            return POKECON_FORMAT_STATE_3_0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_:
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            return POKECON_FORMAT_STATE_7_0;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_1_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_0_;
        case 'x':
            return POKECON_FORMAT_STATE_1_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_1:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_0_;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_10:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_1_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_1_0_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0_0:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        case 'x':
            return POKECON_FORMAT_STATE_1_0_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0_1:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0_10:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_1_0_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_1_0_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_1_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_7_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_0_0_;
        case 'x':
            return POKECON_FORMAT_STATE_7_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_1:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_0_0_;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_10:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_7_0_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_7_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_7_0_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        case 'x':
            return POKECON_FORMAT_STATE_7_0_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0_1:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0_10:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_7_0_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_7_0_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_7_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_3_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_0_0_;
        case 'x':
            return POKECON_FORMAT_STATE_3_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_1:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_0_0_;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_10:
        switch (c)
        {
        case ' ':
            return POKECON_FORMAT_STATE_3_0_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_3_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0_:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_3_0_0_0;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0_0:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        case 'x':
            return POKECON_FORMAT_STATE_3_0_0_0x;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0_1:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_0_10;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0_10:
        switch (c)
        {
        case '\r':
            return POKECON_FORMAT_STATE_0_0CR;
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        case ' ':
            return POKECON_FORMAT_STATE_1_0_;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_3_0_0_0x:
        switch (c)
        {
        case '0':
            return POKECON_FORMAT_STATE_3_0_0_10;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return POKECON_FORMAT_STATE_3_0_0_1;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_0_0CR:
        switch (c)
        {
        case '\n':
            return POKECON_FORMAT_STATE_ACCEPTED;
        default:
            return POKECON_FORMAT_STATE_REJECTED;
        }
    case POKECON_FORMAT_STATE_ACCEPTED:
    default:
        return POKECON_FORMAT_STATE_REJECTED;
    }
}

static const int _VALID_LENGTH_MIN = 4;
static const int _VALID_LENGTH_MAX = 30;

static nthaka_gamepad_state_t _;

static void _remove_prefix_0x(char *str)
{
    char *p = str;
    while ((p = strstr(p, "0x")) != NULL)
    {
        memmove(p, p + 2, strlen(p + 2) + 1); // Remove "0x" by moving the rest of the string over it
    }
}

static bool _deserialize(nthaka_format_handler_t *parent, uint8_t *buf, size_t size, nthaka_gamepad_state_t *out)
{
    pokecon_format_handler_t *fmt = (pokecon_format_handler_t *)parent;
    if (fmt == NULL || buf == NULL || fmt->_s != POKECON_FORMAT_STATE_ACCEPTED)
    {
        return false;
    }
    assert(_VALID_LENGTH_MIN <= size && size <= _VALID_LENGTH_MAX);

    if (out == NULL)
    {
        out = &_;
    }

    if (buf[0] == 'e' && buf[1] == 'n' && buf[2] == 'd')
    {
        nthaka_gamepad_state_copy(out, &NTHAKA_GAMEPAD_NEUTRAL);
        fmt->_prev_l.x = out->l_stick.x;
        fmt->_prev_l.y = out->l_stick.y;
        fmt->_prev_r.x = out->r_stick.x;
        fmt->_prev_r.y = out->r_stick.y;
        return true;
    }

    // Copy buf into a new char[]
    char str[size + 1];
    size_t size_ = 0;
    int cnt = 0;
    for (size_t i = 0; i < size; i++)
    {
        switch (buf[i])
        {
        case '\r':
        case '\n':
            break;

        case ' ':
            cnt++;
        default:
            size_++;
            str[i] = buf[i];
            break;
        }
    }
    str[size_] = '\0';
    // 0x" must be removed. Maybe this is another Newlib-nano restriction.
    _remove_prefix_0x(str);

    uint16_t btns = 0x0000;
    // Use uint16_t and "%hx" instead of uint8_t and "%hhx", as Newlib-nano do not support C99 notation.
    // https://ja-support.renesas.com/knowledgeBase/20441082
    uint16_t hat = NTHAKA_HAT_NEUTRAL;
    uint16_t x_0 = NTHAKA_STICK_AXIS_NEUTRAL;
    uint16_t y_0 = NTHAKA_STICK_AXIS_NEUTRAL;
    uint16_t x_1 = NTHAKA_STICK_AXIS_NEUTRAL;
    uint16_t y_1 = NTHAKA_STICK_AXIS_NEUTRAL;

    switch (cnt)
    {
    case 1:
        sscanf(str, "%hx %hx", &btns, &hat);
        break;
    case 3:
        sscanf(str, "%hx %hx %hx %hx", &btns, &hat, &x_0, &y_0);
        break;
    case 5:
        sscanf(str, "%hx %hx %hx %hx %hx %hx", &btns, &hat, &x_0, &y_0, &x_1, &y_1);
        break;

    default:
        assert(false);
        break;
    }
    assert(x_0 <= 0xFF &&
           y_0 <= 0xFF &&
           x_1 <= 0xFF &&
           y_1 <= 0xFF);

    bool update_rs = nthaka_internal_bit(btns, 0) == 1;
    bool update_ls = nthaka_internal_bit(btns, 1) == 1;
    out->y = nthaka_internal_bit(btns, 2);
    out->b = nthaka_internal_bit(btns, 3);
    out->a = nthaka_internal_bit(btns, 4);
    out->x = nthaka_internal_bit(btns, 5);
    out->l = nthaka_internal_bit(btns, 6);
    out->r = nthaka_internal_bit(btns, 7);
    out->zl = nthaka_internal_bit(btns, 8);
    out->zr = nthaka_internal_bit(btns, 9);
    out->minus = nthaka_internal_bit(btns, 10);
    out->plus = nthaka_internal_bit(btns, 11);
    out->l_click = nthaka_internal_bit(btns, 12);
    out->r_click = nthaka_internal_bit(btns, 13);
    out->home = nthaka_internal_bit(btns, 14);
    out->capture = nthaka_internal_bit(btns, 15);

    switch (hat)
    {
    case 0:
        out->hat = NTHAKA_HAT_UP;
        break;
    case 1:
        out->hat = NTHAKA_HAT_UPRIGHT;
        break;
    case 2:
        out->hat = NTHAKA_HAT_RIGHT;
        break;
    case 3:
        out->hat = NTHAKA_HAT_DOWNRIGHT;
        break;
    case 4:
        out->hat = NTHAKA_HAT_DOWN;
        break;
    case 5:
        out->hat = NTHAKA_HAT_DOWNLEFT;
        break;
    case 6:
        out->hat = NTHAKA_HAT_LEFT;
        break;
    case 7:
        out->hat = NTHAKA_HAT_UPLEFT;
        break;
    case 8:
        out->hat = NTHAKA_HAT_NEUTRAL;
        break;
    default:
        assert(false);
        break;
    }

    if (update_ls && update_rs)
    {
        out->l_stick.x = (uint8_t)x_0;
        out->l_stick.y = (uint8_t)y_0;
        fmt->_prev_l.x = out->l_stick.x;
        fmt->_prev_l.y = out->l_stick.y;

        out->r_stick.x = (uint8_t)x_1;
        out->r_stick.y = (uint8_t)y_1;
        fmt->_prev_r.x = out->r_stick.x;
        fmt->_prev_r.y = out->r_stick.y;
    }
    else if (update_ls)
    {
        out->l_stick.x = (uint8_t)x_0;
        out->l_stick.y = (uint8_t)y_0;
        fmt->_prev_l.x = out->l_stick.x;
        fmt->_prev_l.y = out->l_stick.y;

        out->r_stick.x = fmt->_prev_r.x;
        out->r_stick.y = fmt->_prev_r.y;
    }
    else if (update_rs)
    {
        out->l_stick.x = fmt->_prev_l.x;
        out->l_stick.y = fmt->_prev_l.y;

        out->r_stick.x = (uint8_t)x_0;
        out->r_stick.y = (uint8_t)y_0;
        fmt->_prev_r.x = out->r_stick.x;
        fmt->_prev_r.y = out->r_stick.y;
    }
    else
    {
        out->l_stick.x = fmt->_prev_l.x;
        out->l_stick.y = fmt->_prev_l.y;

        out->r_stick.x = fmt->_prev_r.x;
        out->r_stick.y = fmt->_prev_r.y;
    }

    for (size_t i = 0; i < nthaka_internal_size(out->ext); i++)
    {
        out->ext[i] = 0;
    }

    return true;
}

static nthaka_buffer_state_t _update(nthaka_format_handler_t *parent, uint8_t d)
{
    pokecon_format_handler_t *fmt = (pokecon_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return NTHAKA_BUFFER_REJECTED;
    }

    fmt->_s = pokecon_format_state_next(fmt->_s, (char)d);
    return fmt->_s == POKECON_FORMAT_STATE_REJECTED ? NTHAKA_BUFFER_REJECTED
                                                    : (fmt->_s == POKECON_FORMAT_STATE_ACCEPTED ? NTHAKA_BUFFER_ACCEPTED
                                                                                                : NTHAKA_BUFFER_PENDING);
}

static void _reset(nthaka_format_handler_t *parent)
{
    pokecon_format_handler_t *fmt = (pokecon_format_handler_t *)parent;
    if (fmt == NULL)
    {
        return;
    }

    fmt->_s = POKECON_FORMAT_STATE_INITIAL;
}

bool pokecon_format_handler_init(pokecon_format_handler_t *fmt)
{
    if (fmt == NULL)
    {
        return false;
    }

    fmt->parent.deserialize = _deserialize;
    fmt->parent.reset = _reset;
    fmt->parent.update = _update;

    fmt->_s = POKECON_FORMAT_STATE_INITIAL;

    fmt->_prev_l.x = NTHAKA_STICK_AXIS_NEUTRAL;
    fmt->_prev_l.y = NTHAKA_STICK_AXIS_NEUTRAL;
    fmt->_prev_r.x = NTHAKA_STICK_AXIS_NEUTRAL;
    fmt->_prev_r.y = NTHAKA_STICK_AXIS_NEUTRAL;

    return true;
}