//
// Created by robos on 30/06/2022.
//

#include "staticFifo.h"
#include "Vector.h"



template<class T, unsigned int sz>
Fifo<T, sz>::Fifo() {
    nextFree=0;
    endPointer=0;
}

template<class T, unsigned int sz>
Fifo<T, sz>::Fifo(std::initializer_list<T> lst) {
    for (int i=0;i<sz;i++)
        elem[i] = lst.begin()[i];
    nextFree=0;
    endPointer=0;
}

template<class T, unsigned int sz>
T &Fifo<T, sz>::operator[](int i) {
    return elem[i%sz]; // if the index is larger than the sz, it wraps around;
}

template<class T, unsigned int sz>
T &Fifo<T, sz>::operator[](int i) const {
    return elem[i%sz]; // if the index is larger than the sz, it wraps around;
}

template<class T, unsigned int sz>
T& Fifo<T, sz>::atFifoIndex(int i) {
    return elem[(endPointer+i) % sz];
}

template<class T, unsigned int sz>
T& Fifo<T, sz>::atFifoIndex(int i) const{
    return elem[(endPointer+i) % sz];
}

template<class T, unsigned int sz>
Fifo_STATUS Fifo<T, sz>::push(const T& item) { // returns the status of the fifo
    if (fifo_status()==Fifo_STATUS::Fifo_FULL) {
        // throw std::length_error("NA"); // throw does not work with arduino :(
        return Fifo_STATUS::Fifo_FULL; // status code
    }
    // otherwise:
    elem[nextFree] = item;
    if (((nextFree+1) % sz) == endPointer) {
        nextFree = -1;
    }
    else{
        nextFree = (++nextFree) % sz; // wrap around /:)
    }
    return Fifo_STATUS::Fifo_GOOD;
}

template<class T, unsigned int sz>
T Fifo<T, sz>::pop() { /// Note: You should check the status of the fifo before calling this function
    if (fifo_status()==Fifo_STATUS::Fifo_EMPTY) {
        //throw std::length_error("NA"); // throw does not work with arduino :(
        // return {Fifo_EMPTY};
        return {0}; // return nothing
    }
    // otherwise
    T r = elem[endPointer];
    if (fifo_status()==Fifo_STATUS::Fifo_FULL) {
        nextFree=endPointer;
    }
    endPointer = (++endPointer) % sz; // wrap around /:)
    return r;
}

template<class T, unsigned int sz>
T Fifo<T, sz>::peekBack() const {
    if (fifo_status()==Fifo_STATUS::Fifo_EMPTY) {
        return {0}; // return 0
    }
    // otherwise
    return elem[endPointer];
}

template<class T, unsigned int sz>
T Fifo<T, sz>::peekBack(int i) const {
    if (fifo_status()==Fifo_STATUS::Fifo_EMPTY) {
        return {0}; // return 0
    }
    // otherwise
    return elem[(endPointer+i) % sz];
}


template<class T, unsigned int sz>
T Fifo<T, sz>::peekFront() const {
    if (fifo_status()==Fifo_STATUS::Fifo_EMPTY) {
        return {0}; // return 0
    }
    // otherwise
    return elem[nextFree-1];
}

template<class T, unsigned int sz>
T Fifo<T, sz>::peekFront(int i) const {
    if (fifo_status()==Fifo_STATUS::Fifo_EMPTY) {
        return {0}; // return 0
    }
    // otherwise
    return elem[(nextFree-1-i) % sz];
}

template<class T, unsigned int sz>
Fifo_STATUS Fifo<T, sz>::fifo_status() const {
    if (nextFree==endPointer) {
        return Fifo_STATUS::Fifo_EMPTY; // fifo empty
    }
    else if (nextFree == -1) { // when the Fifo is full, there is no "next free" location.
        return Fifo_STATUS::Fifo_FULL; // fifo full
    }
    // otherwise, return Fifo_GOOD
    return Fifo_STATUS::Fifo_GOOD;
}

template<class T, unsigned int sz>
int Fifo<T, sz>::size() const {
    return sz;
}

template<class T, unsigned int sz>
int Fifo<T, sz>::free_space() const{
    if (fifo_status()==Fifo_STATUS::Fifo_FULL){
        return 0;
    }
    else if (nextFree>=endPointer) {
        return sz-(nextFree-endPointer);
    }
    else {
        return endPointer-nextFree;
    }
}

/*

#ifdef VECTOR_LIBRARY_H
#include "Vector.h"


template<> inline
Vector<double, 3> &Fifo<Vector<double, 3>>::operator[](int i) {
    return elem[i]; // if the index is larger than the sz, it wraps around;
}

template<> inline
Fifo_STATUS Fifo<Vector<double, 3>>::push(const Vector<double, 3>& item) {
    if (fifo_status()==Fifo_FULL) {
        // throw std::length_error("NA"); // throw does not work with arduino :(
        return Fifo_FULL; // status code
    }
    // otherwise:
    elem[nextFree] = item;
    if (((nextFree+1) % sz) == endPointer) {
        nextFree = -1;
    }
    else{
        nextFree = (++nextFree) % sz; // wrap around /:)
    }
    return Fifo_GOOD;
}

template<> inline
Vector<double, 3> Fifo<Vector<double, 3>>::pop() {
    if (fifo_status()==Fifo_EMPTY) {
        Vector<double, 3> rv = {Fifo_EMPTY, Fifo_EMPTY, Fifo_EMPTY};
        return rv;
    }
    // otherwise
    Vector<double, 3> r = {0,0,0};
    r = {elem[endPointer][0], elem[endPointer][1], elem[endPointer][2]};
    if (fifo_status()==Fifo_FULL) {
        nextFree=endPointer;
    }
    endPointer = (++endPointer) % sz; // wrap around /:)

    return r;
}
#endif
*/
