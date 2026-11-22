#ifndef _UTILS_H_
#define _UTILS_H_

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/errno.h>

#define BUFF_SIZE 4096

typedef struct {
    uint8_t buffer[BUFF_SIZE];
    size_t head;
    size_t tail;
    spinlock_t lock;
} circ_buf_t;

void cb_init(circ_buf_t *cb);
void cb_reset(circ_buf_t *cb);
int cb_push(circ_buf_t *cb, uint8_t data);
int cb_pop(circ_buf_t *cb, uint8_t *data);

#endif /* _UTILS_H_ */
