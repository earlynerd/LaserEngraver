#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <Arduino.h>

template <typename T, size_t Size>
class RingBuffer {
private:
    volatile T buffer[Size];
    volatile size_t head;
    volatile size_t tail;

public:
    RingBuffer() : head(0), tail(0) {}

    bool push(T item) {
        size_t next_head = (head + 1) % Size;
        if (next_head == tail) return false;
        buffer[head] = item;
        head = next_head;
        return true;
    }

    bool pop(T& item) {
        if (head == tail) return false;
        item = buffer[tail];
        tail = (tail + 1) % Size;
        return true;
    }

    bool isEmpty() const { return head == tail; }
    bool isFull() const { return ((head + 1) % Size) == tail; }
    
    size_t availableSpace() const {
        if (head >= tail) return Size - 1 - (head - tail);
        return tail - head - 1;
    }
};

#endif