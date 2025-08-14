#include "utils.h"
/**
 * @brief Picks a channel from multiple arrays
 * @param array Array
 * @param size Size
 * @return A random value from the array
 */
uint8_t pick(const uint8_t *array, size_t size) {
  if (array == NULL || size == 0) {
    return 0;
  }

  // Generate random index and return the element
  size_t random_index = esp_random() % size;
  return array[random_index];
}
