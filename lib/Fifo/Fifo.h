#ifndef Fifo_H
#define Fifo_H

#include <iostream>


enum class Fifo_STATUS {
    Fifo_FULL,
    Fifo_EMPTY,
    Fifo_GOOD
};

template<class T>
class Fifo { /// essentially a circular fifo
private:
    T* elem;
    int nextFree;
    int endPointer;
    unsigned int sz;
public:
    explicit Fifo(int s);
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

    // end
    T peekBack() const;
    T peekBack(int i) const;

    // front
    T peekFront() const;
    T peekFront(int i) const;



    Fifo_STATUS fifo_status() const;

    int size() const;
    int free_space() const;

    ~Fifo() { delete[] elem; } // destructor
};

#include "Fifo.tpp" // implementation file


#endif //Fifo_H
