#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    size_t head;      // Index of the first valid element.
    size_t tail;      // Next write index.
    size_t size;      // Number of elements stored.
    size_t capacity;  // Maximum number of elements.
    size_t elem_size; // Size of a single element in bytes.
    uint8_t *buf;     // Pointer to the buffer memory.
} RingBuffer;

void ring_buffer_init(RingBuffer *rb, void *buffer, size_t capacity, size_t elem_size);

static inline size_t ring_buffer_available(const RingBuffer *rb) {
    return rb->capacity - rb->size;
}

static inline size_t ring_buffer_used(const RingBuffer *rb) {
    return rb->size;
}

bool ring_buffer_push(RingBuffer *rb, const void *val, bool overwrite);

bool ring_buffer_pop(RingBuffer *rb, void *val);

bool ring_buffer_push_array(RingBuffer *rb, const void *data, size_t count, bool overwrite);

bool ring_buffer_pop_array(RingBuffer *rb, void *data, size_t count);

#endif
