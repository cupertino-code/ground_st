#ifndef _CIRCBUF_H_INCLUDED
#define _CIRCBUF_H_INCLUDED
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hidden definition of our circular buffer structure
struct circular_buf {
    uint8_t *buffer;
    size_t item_size;
    size_t head;
    size_t tail;
    size_t max;  // of the buffer
    bool full;
};

struct circular_buf *cbuf_init(struct circular_buf *cbuf, uint8_t *buffer, size_t size,
                               size_t item_size);
void cbuf_destroy(struct circular_buf *cbuf);
void cbuf_reset(struct circular_buf *cbuf);
size_t cbuf_size(struct circular_buf *cbuf);
void cbuf_put(struct circular_buf *cbuf, uint8_t *data);
int cbuf_get(struct circular_buf *cbuf, uint8_t *data);
uint8_t *cbuf_get_ptr(struct circular_buf *cbuf);
int cbuf_drop(struct circular_buf *cbuf);

#define cbuf_free(cbuf)
#define cbuf_full(cbuf) ((cbuf)->full)
#define cbuf_empty(cbuf) (!(cbuf)->full && ((cbuf)->head == (cbuf)->tail))
#define cbuf_capacity(cbuf) ((cbuf)->max)
#if 0
inline void cbuf_free(struct circular_buf *cbuf)
{
}

inline bool cbuf_full(struct circular_buf *cbuf)
{
    return cbuf->full;
}

inline bool cbuf_empty(struct circular_buf *cbuf)
{
    return !cbuf->full && (cbuf->head == cbuf->tail);
}

inline size_t cbuf_capacity(struct circular_buf *cbuf)
{
    return cbuf->max;
}
#endif

#endif /* _CIRCBUF_H_INCLUDED */
