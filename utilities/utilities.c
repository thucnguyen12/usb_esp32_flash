#include "utilities.h"

uint32_t utilities_get_number_from_string(uint32_t index, char *buffer)
{
    // assert(buffer);

    uint32_t value = 0;
    uint16_t tmp = index;

    while (buffer[tmp] && tmp < 1024)
    {
        if (buffer[tmp] >= '0' && buffer[tmp] <= '9')
        {
            value *= 10;
            value += buffer[tmp] - 48;
        }
        else
        {
            break;
        }
        tmp++;
    }

    return value;
}
