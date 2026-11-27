#ifndef _UTILS_H_
#define _UTILS_H_

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/delay.h>

#define BUFF_SIZE 8192//32768

typedef struct {
    uint8_t buffer[BUFF_SIZE];
    size_t head;
    size_t tail;
    size_t count;
    bool overwrite;
    spinlock_t lock;
} circ_buf_t;

/**
 * @brief Inicializa el buffer circular y su spinlock.
 * @param cb Puntero al buffer circular.
 * @param overwrite true para sobrescribir datos antiguos si se llena; false para rechazar.
 */
void cb_init(circ_buf_t *cb, bool overwrite);

/**
 * @brief Vacía el buffer circular reiniciando sus índices y contador.
 * @param cb Puntero al buffer circular.
 */
void cb_reset(circ_buf_t *cb);

/**
 * @brief Indica si el buffer circular está vacío.
 * @param cb Puntero al buffer circular.
 * @return true si no contiene datos, false en caso contrario.
 */
bool cb_is_empty(circ_buf_t *cb);

/**
 * @brief Indica si el buffer circular está lleno.
 * @param cb Puntero al buffer circular.
 * @return true si alcanzó BUFF_SIZE, false en caso contrario.
 */
bool cb_is_full(circ_buf_t *cb);

/**
 * @brief Verifica si el buffer circular tiene al menos la mitad de espacio libre.
 * @param cb Puntero al buffer circular.
 * @return true si la ocupación es menor o igual a BUFF_SIZE / 2.
 */
bool cb_half_free(circ_buf_t *cb);

/**
 * @brief Devuelve la cantidad de bytes almacenados en el buffer circular.
 * @param cb Puntero al buffer circular.
 * @return Cantidad de bytes disponibles.
 */
size_t cb_count(circ_buf_t *cb);

/**
 * @brief Inserta un byte en el buffer circular.
 * @param cb Puntero al buffer circular.
 * @param data Byte a almacenar.
 * @return 0 en éxito, -ENOSPC si está lleno sin overwrite, o -EINVAL si @p cb es NULL.
 */
int cb_push(circ_buf_t *cb, uint8_t data);

/**
 * @brief Extrae un byte del buffer circular.
 * @param cb Puntero al buffer circular.
 * @param data Puntero donde guardar el byte extraído.
 * @return 0 en éxito, -ENODATA si está vacío, o -EINVAL si algún puntero es NULL.
 */
int cb_pop(circ_buf_t *cb, uint8_t *data);

/**
 * @brief Extrae hasta 4 bytes del buffer circular armando una palabra de 32 bits (MSB primero).
 *        Si no hay datos disponibles devuelve 0 (y completa con 0 los bytes faltantes).
 * @param cb Puntero al buffer circular.
 * @return Palabra de 32 bits armada o 0 si el buffer está vacío.
 */
uint32_t cb_pop_word(circ_buf_t *cb);

/**
 * @brief Espera con timeout de 1 ms a que los bits de @p mask en @p base + @p reg
 *        pasen a 1 o a 0.
 * @param base Dirección base mapeada del periférico.
 * @param reg Offset del registro respecto a @p base.
 * @param mask Máscara de bits a verificar.
 * @param wait_set true para esperar bits en 1; false para esperarlos en 0.
 */
void mcasp_wait_reg(void __iomem *base, uint32_t reg, uint32_t mask, bool wait_set);

#endif /* _UTILS_H_ */
