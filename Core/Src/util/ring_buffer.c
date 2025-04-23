#include "util/ring_buffer.h"
#include <string.h>

static inline size_t wrapping_add(size_t val, size_t increment, size_t max) {
    return (val + increment) % max;
}

void ring_buffer_init(RingBuffer *rb, void *buffer, size_t capacity, size_t elem_size) {
    rb->head = 0;
    rb->tail = 0;
    rb->size = 0;
    rb->capacity = capacity;
    rb->elem_size = elem_size;
    rb->buf = (uint8_t *)buffer;
}

bool ring_buffer_push(RingBuffer *rb, const void *val, bool overwrite) {
    if (rb->size == rb->capacity && !overwrite) {
        return false;
    }
    /* Copy the new element into the tail position */
    memcpy(rb->buf + rb->tail * rb->elem_size, val, rb->elem_size);
    rb->tail = wrapping_add(rb->tail, 1, rb->capacity);
    if (rb->size < rb->capacity) {
        rb->size++;
        return true;
    } else {
        /* Buffer was full; overwrite the oldest data */
        rb->head = wrapping_add(rb->head, 1, rb->capacity);
        return false;
    }
}

bool ring_buffer_pop(RingBuffer *rb, void *val) {
    if (rb->size == 0) {
        return false;
    }
    memcpy(val, rb->buf + rb->head * rb->elem_size, rb->elem_size);
    rb->head = wrapping_add(rb->head, 1, rb->capacity);
    rb->size--;
    return true;
}

bool ring_buffer_push_array(RingBuffer *rb, const void *data, size_t count, bool overwrite) {
    if (rb->size + count > rb->capacity && !overwrite) {
        return false;
    }
    /* If trying to push more items than capacity, only keep the last ones */
    if (count > rb->capacity) {
        data = (const uint8_t *)data + (count - rb->capacity) * rb->elem_size;
        count = rb->capacity;
    }
    size_t tail = rb->tail;
    /* Check if the new data wraps around the end of the buffer */
    if (tail + count > rb->capacity) {
        size_t count_to_end = rb->capacity - tail;
        size_t remainder = count - count_to_end;
        memcpy(rb->buf + tail * rb->elem_size, data, count_to_end * rb->elem_size);
        memcpy(rb->buf, (const uint8_t *)data + count_to_end * rb->elem_size, remainder * rb->elem_size);
    } else {
        memcpy(rb->buf + tail * rb->elem_size, data, count * rb->elem_size);
    }
    rb->tail = wrapping_add(rb->tail, count, rb->capacity);
    if (rb->size + count > rb->capacity) {
        /* Advance head to tail if we have overwritten data */
        rb->head = rb->tail;
    }
    rb->size += count;
    if (rb->size > rb->capacity) {
        rb->size = rb->capacity;
        return false;
    }
    return true;
}

bool ring_buffer_pop_array(RingBuffer *rb, void *data, size_t count) {
    if (rb->size < count) {
        return false;
    }
    size_t head = rb->head;
    /* Check for wrap-around */
    if (head + count > rb->capacity) {
        size_t count_to_end = rb->capacity - head;
        size_t remainder = count - count_to_end;
        memcpy(data, rb->buf + head * rb->elem_size, count_to_end * rb->elem_size);
        memcpy((uint8_t *)data + count_to_end * rb->elem_size, rb->buf, remainder * rb->elem_size);
    } else {
        memcpy(data, rb->buf + head * rb->elem_size, count * rb->elem_size);
    }
    rb->head = wrapping_add(rb->head, count, rb->capacity);
    rb->size -= count;
    return true;
}
