#include "utils.h"

void cb_init(circ_buf_t *cb) {
    if (!cb)
        return;

    cb->head = 0;
    cb->tail = 0;
    spin_lock_init(&cb->lock);
}

void cb_reset(circ_buf_t *cb) {
    unsigned long flags;

    if (!cb)
        return;

    spin_lock_irqsave(&cb->lock, flags);
    cb->head = 0;
    cb->tail = 0;
    spin_unlock_irqrestore(&cb->lock, flags);
}

int cb_push(circ_buf_t *cb, uint8_t data) {
    unsigned long flags;
    size_t next_head;

    if (!cb)
        return -EINVAL;

    spin_lock_irqsave(&cb->lock, flags);

    next_head = (cb->head + 1) % BUFF_SIZE;
    if (next_head == cb->tail) {
        spin_unlock_irqrestore(&cb->lock, flags);
        return -ENOSPC;
    }

    cb->buffer[cb->head] = data;
    cb->head = next_head;

    spin_unlock_irqrestore(&cb->lock, flags);
    return 0;
}

int cb_pop(circ_buf_t *cb, uint8_t *data) {
    unsigned long flags;

    if (!cb || !data)
        return -EINVAL;

    spin_lock_irqsave(&cb->lock, flags);

    if (cb->head == cb->tail) {
        spin_unlock_irqrestore(&cb->lock, flags);
        return -ENODATA;
    }

    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFF_SIZE;

    spin_unlock_irqrestore(&cb->lock, flags);
    return 0;
}
