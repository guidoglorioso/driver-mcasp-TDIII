#ifndef _UTILS_H_
#define _UTILS_H_

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/errno.h>

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

#endif /* _UTILS_H_ */
