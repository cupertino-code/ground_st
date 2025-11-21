#include "circ_buf.h"

struct circular_buf *cbuf_init(struct circular_buf *cbuf, uint8_t *buffer, size_t size,
                               size_t item_size)
{
    cbuf->buffer = buffer;
    cbuf->max = size;
    cbuf->item_size = item_size;
    cbuf_reset(cbuf);

    return cbuf;
}

void cbuf_reset(struct circular_buf *cbuf)
{
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

size_t cbuf_size(struct circular_buf *cbuf)
{
    size_t size = cbuf->max;

    if (!cbuf->full) {
        if (cbuf->head >= cbuf->tail) {
            size = cbuf->head - cbuf->tail;
        } else {
            size = cbuf->max + cbuf->head - cbuf->tail;
        }
    }

    return size;
}

void cbuf_put(struct circular_buf *cbuf, uint8_t *data)
{
    memcpy(&cbuf->buffer[cbuf->head * cbuf->item_size], data, cbuf->item_size);
    if (cbuf->full) {
        cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    }
    cbuf->head = (cbuf->head + 1) % cbuf->max;
    cbuf->full = (cbuf->head == cbuf->tail);
}

int cbuf_get(struct circular_buf *cbuf, uint8_t *data)
{
    if (cbuf_empty(cbuf))
        return -1;  // Buffer is empty
    memcpy(data, &cbuf->buffer[cbuf->tail * cbuf->item_size], cbuf->item_size);
    cbuf->full = false;
    cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    return 0;
}

uint8_t *cbuf_get_ptr(struct circular_buf *cbuf)
{
    if (cbuf_empty(cbuf))
        return NULL;  // Buffer is empty
    return &cbuf->buffer[cbuf->tail * cbuf->item_size];
}

int cbuf_drop(struct circular_buf *cbuf)
{
    if (cbuf_empty(cbuf))
        return -1;  // Buffer is empty
    cbuf->full = false;
    cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}
