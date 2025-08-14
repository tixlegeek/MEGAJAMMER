#ifndef _UTILS_H_
#define _UTILS_H_
#include "esp_random.h"
#include <stdint.h>
#include <stdlib.h>
// Macro to count elements in a static array.
#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))
uint8_t pick(const uint8_t *array, size_t size);

#endif /* end of include guard: _UTILS_H_ */
