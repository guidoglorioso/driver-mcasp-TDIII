#include "utils.h"

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
