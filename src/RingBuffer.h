#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <Arduino.h>

template <typename T, size_t Size>
class RingBuffer {
public:
    RingBuffer() : _head(0), _tail(0), _count(0) {}

    bool push(const T& item) {
        if (_count >= Size) return false; // Buffer full
        _buffer[_head] = item;
        _head = (_head + 1) % Size;
        _count++;
        return true;
    }

    bool pop(T& item) {
        if (_count == 0) return false; // Buffer empty
        item = _buffer[_tail];
        _tail = (_tail + 1) % Size;
        _count--;
        return true;
    }

    bool peek(T& item) {
        if (_count == 0) return false;
        item = _buffer[_tail];
        return true;
    }
    
    // Peek at an offset without removing
    bool peekAt(size_t offset, T& item) {
        if (offset >= _count) return false;
        size_t index = (_tail + offset) % Size;
        item = _buffer[index];
        return true;
    }

    size_t available() const {
        return _count;
    }

    size_t capacity() const {
        return Size;
    }

    bool isFull() const {
        return _count == Size;
    }

    bool isEmpty() const {
        return _count == 0;
    }

    void clear() {
        _head = 0;
        _tail = 0;
        _count = 0;
    }

private:
    T _buffer[Size];
    volatile size_t _head;
    volatile size_t _tail;
    volatile size_t _count;
};

#endif