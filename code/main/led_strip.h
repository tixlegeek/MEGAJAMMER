// led_strip.h

#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <esp_err.h>
#include <driver/gpio.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le pilote de bande de LEDs WS2812.
 *
 * @param gpio_num   Le GPIO utilisé pour la broche DIN WS2812.
 * @param led_count  Nombre de LEDs sur la bande (> 0).
 * @return
 *   - ESP_OK                : succès
 *   - ESP_ERR_INVALID_ARG   : led_count == 0
 *   - ESP_ERR_NO_MEM        : échec d’allocation mémoire ou du driver RMT
 *   - autre esp_err_t       : erreur RMT
 */
esp_err_t led_strip_init(gpio_num_t gpio_num, size_t led_count);

/**
 * @brief Libère toutes les ressources.
 *
 * Après appel, on peut redémarrer avec led_strip_init().
 */
void led_strip_deinit(void);

/**
 * @brief Envoie le buffer courant vers la bande.
 *
 * @return
 *   - ESP_OK                : succès
 *   - ESP_ERR_INVALID_STATE : non initialisé
 *   - ESP_ERR_TIMEOUT       : mutex non pris
 *   - autre esp_err_t       : erreur RMT
 */
esp_err_t led_strip_show(void);

/**
 * @brief Change la couleur d’un pixel dans le buffer (GRB).
 *
 * N’envoie pas vers la bande : il faut appeler led_strip_show().
 *
 * @param index  0‑based, doit être < led_count
 * @param r      Rouge (0–255)
 * @param g      Vert   (0–255)
 * @param b      Bleu   (0–255)
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG   : index hors limites
 *   - ESP_ERR_INVALID_STATE : non initialisé
 *   - ESP_ERR_TIMEOUT       : mutex non pris
 */
esp_err_t led_strip_set_pixel(size_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Remplit tout le buffer avec une seule couleur.
 *
 * N’envoie pas vers la bande : il faut appeler led_strip_show().
 *
 * @return même code que led_strip_show()
 */
esp_err_t led_strip_fill(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Éteint toutes les LEDs (buffer à zéro).
 *
 * N’envoie pas vers la bande : il faut appeler led_strip_show().
 */
esp_err_t led_strip_clear(void);

/**
 * @brief Retourne le nombre de LEDs configurées, ou 0 si non initialisé.
 */
size_t led_strip_get_length(void);

#ifdef __cplusplus
}
#endif

#endif // LED_STRIP_H
