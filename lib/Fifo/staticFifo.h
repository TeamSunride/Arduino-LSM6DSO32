#ifndef STATICFifo_H
#define STATICFifo_H

#include <iostream>


enum class Fifo_STATUS {
    Fifo_FULL,
    Fifo_EMPTY,
    Fifo_GOOD
};

template<class T, unsigned int sz>
class Fifo { /// essentially a circular fifo
protected:
    T elem[sz];
    int nextFree;
    int endPointer;

public:
    explicit Fifo();
    Fifo(std::initializer_list<T> lst);

    // TODO: copy and move constructors

    T& operator[](int i);
    T& operator[](int i) const;

    T& atFifoIndex(int i);
    T& atFifoIndex(int i) const;

    Fifo_STATUS push(const T& item);
    //Fifo& operator=(const Fifo& a);

    // TODO: iterators?
    T pop();

    Fifo_STATUS fifo_status() const;

    int size() const;
    int free_space() const;

    ~Fifo() = default; // destructor
};

#include "staticFifo.tpp" // implementation file


#endif //STATICFifo_H
