#include "icm20948_mem.h"

IMUBuffer* IMUBuffer_Init(uint16_t size) {
    // Allocate memory for the IMUBuffer structure
    IMUBuffer* buffer = (IMUBuffer*)calloc(1, sizeof(IMUBuffer));
    if (buffer == NULL) {
        return NULL;  // Allocation failed
    }

    // Allocate memory for the IMUValues array
    buffer->buffer = (IMUValues*)calloc(size, sizeof(IMUValues));
    if (buffer->buffer == NULL) {
        free(buffer);  // Free the previously allocated IMUBuffer
        return NULL;   // Allocation failed
    }

    // Initialize IMUBuffer fields
    buffer->size = size;
    buffer->current = 0;
    buffer->previous = 0;

    return buffer;
}

void IMUBuffer_Free(IMUBuffer* buf) {
    if (buf != NULL) {
        if (buf->buffer != NULL) {
            free(buf->buffer);
        }
        free(buf);
    }
}

bool IMUBuffer_Push(IMUBuffer* buf, IMUValues* values) {
    if (buf == NULL || values == NULL) {
        return false;  // Null check
    }

    // Copy the new values into the buffer
    buf->buffer[buf->current] = *values;

    // Update indices
    buf->previous = buf->current;
    buf->current = (buf->current + 1) % buf->size;

    return true;
}

uint16_t IMUBuffer_GetSize(IMUBuffer* buf) {
    if (buf == NULL) {
        return 0;
    }
    return buf->size;
}

void IMUBuffer_increaseIndex(IMUBuffer* buf) {
    buf->previous = buf->current;
    buf->current = (buf->current + 1) % buf->size;
}

IMUValues* IMUBuffer_GetNextValue(IMUBuffer* buf) {
    if (buf == NULL || buf->buffer == NULL) {
        return NULL;  // Null check
    }

    uint16_t nextIndex = (buf->current + 1) % buf->size;
    return &(buf->buffer[nextIndex]);
}

IMUValues* IMUBuffer_GetCurrentValue(IMUBuffer* buf) {
    if (buf == NULL) {
        return NULL;
    }
    return &(buf->buffer[buf->current]);
}

IMUValues* IMUBuffer_GetPreviousValue(IMUBuffer* buf) {
    if (buf == NULL) {
        return NULL;
    }
    return &(buf->buffer[buf->previous]);
}