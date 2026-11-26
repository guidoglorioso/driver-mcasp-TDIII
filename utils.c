#include "utils.h"


//-------------------------------------
//      Metodos Escritura de registros
//-------------------------------------


/**
 * @brief Espera con timeout de 1 ms a que los bits de @p mask en @p base + @p reg
 *        queden en 1 (@p wait_set = true) o en 0 (@p wait_set = false).
 */
void mcasp_wait_reg(void __iomem *base, uint32_t reg, uint32_t mask, bool wait_set) {
    int timeout = 1000;
    uint32_t expected = wait_set ? mask : 0;

    while (((ioread32(base + reg) & mask) != expected) && --timeout > 0)
        udelay(1);
}
    

//-------------------------------------
//      Metodos de Buffer Circular
//-------------------------------------

/// @brief 
/// @param cb 
/// @param overwrite 
void cb_init(circ_buf_t *cb, bool overwrite) {
    if (!cb)
        return;

    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    cb->overwrite = overwrite;
    spin_lock_init(&cb->lock);
}

void cb_reset(circ_buf_t *cb) {
    unsigned long flags;

    if (!cb)
        return;

    spin_lock_irqsave(&cb->lock, flags);
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    spin_unlock_irqrestore(&cb->lock, flags);
}

bool cb_is_empty(circ_buf_t *cb) {
    unsigned long flags;
    bool empty;

    if (!cb)
        return true;

    spin_lock_irqsave(&cb->lock, flags);
    empty = (cb->count == 0);
    spin_unlock_irqrestore(&cb->lock, flags);

    return empty;
}

bool cb_is_full(circ_buf_t *cb) {
    unsigned long flags;
    bool full;

    if (!cb)
        return true;

    spin_lock_irqsave(&cb->lock, flags);
    full = (cb->count == BUFF_SIZE);
    spin_unlock_irqrestore(&cb->lock, flags);

    return full;
}

bool cb_half_free(circ_buf_t *cb) {
    unsigned long flags;
    bool half_free;

    if (!cb)
        return true;

    spin_lock_irqsave(&cb->lock, flags);
    half_free = (cb->count <= (BUFF_SIZE / 2));
    spin_unlock_irqrestore(&cb->lock, flags);

    return half_free;
}

size_t cb_count(circ_buf_t *cb) {
    unsigned long flags;
    size_t count;

    if (!cb)
        return 0;

    spin_lock_irqsave(&cb->lock, flags);
    count = cb->count;
    spin_unlock_irqrestore(&cb->lock, flags);

    return count;
}

int cb_push(circ_buf_t *cb, uint8_t data) {
    unsigned long flags;

    if (!cb)
        return -EINVAL;

    spin_lock_irqsave(&cb->lock, flags);

    if (cb->count == BUFF_SIZE) {
        if (!cb->overwrite) {
            spin_unlock_irqrestore(&cb->lock, flags);
            return -ENOSPC;
        }
        cb->tail = (cb->tail + 1) % BUFF_SIZE;
        cb->count--;
    }

    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % BUFF_SIZE;
    cb->count++;

    spin_unlock_irqrestore(&cb->lock, flags);
    return 0;
}

int cb_pop(circ_buf_t *cb, uint8_t *data) {
    unsigned long flags;

    if (!cb || !data)
        return -EINVAL;

    spin_lock_irqsave(&cb->lock, flags);

    if (cb->count == 0) {
        spin_unlock_irqrestore(&cb->lock, flags);
        return -ENODATA;
    }

    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFF_SIZE;
    cb->count--;

    spin_unlock_irqrestore(&cb->lock, flags);
    return 0;
}

uint32_t cb_pop_word(circ_buf_t *cb) {
    unsigned long flags;
    uint32_t word = 0;
    int shift;

    if (!cb)
        return 0;

    spin_lock_irqsave(&cb->lock, flags);

    for (shift = 24; shift >= 0 && cb->count > 0; shift -= 8) {
        word |= ((uint32_t)cb->buffer[cb->tail] << shift);
        cb->tail = (cb->tail + 1) % BUFF_SIZE;
        cb->count--;
    }

    spin_unlock_irqrestore(&cb->lock, flags);
    return word;
}
