#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>

/**
 * @brief       Get Number from string
 * @param[in]   Index Begin index of buffer want to find
 * @param[in]   buffer Data want to search
 * @note        buffer = "abc124mff" =>> gsm_utilities_get_number_from_string(3, buffer) = 123
 * @retval      Number from string
 */ 
uint32_t utilities_get_number_from_string(uint32_t index, char* buffer);

#endif /* UTILITIES_H */
