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

void cb_init(circ_buf_t *cb, bool overwrite);
void cb_reset(circ_buf_t *cb);
bool cb_is_empty(circ_buf_t *cb);
bool cb_is_full(circ_buf_t *cb);
bool cb_half_free(circ_buf_t *cb);
size_t cb_count(circ_buf_t *cb);
int cb_push(circ_buf_t *cb, uint8_t data);
int cb_pop(circ_buf_t *cb, uint8_t *data);

/**
 * @brief Extrae hasta 4 bytes del buffer circular armando una palabra de 32 bits (MSB primero).
 *        Si no hay datos disponibles para extraer devuelve 0 (y si hay menos de 4 bytes,
 *        completa los bytes menos significativos con 0).
 * @param cb Puntero al buffer circular.
 * @return Palabra de 32 bits armada o 0 si el buffer está vacío.
 */
uint32_t cb_pop_word(circ_buf_t *cb);

/**
 * @brief Espera con timeout de 1 ms a que los bits de @p mask en @p base + @p reg
 *        pasen a 1 (@p wait_set = true) o a 0 (@p wait_set = false).
 */
void mcasp_wait_reg(void __iomem *base, uint32_t reg, uint32_t mask, bool wait_set);

#endif /* _UTILS_H_ */
