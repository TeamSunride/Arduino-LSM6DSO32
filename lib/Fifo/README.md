# FIFO
A FIFO class designed to be used with arduino, but could be used anywhere I suppose

---

### Usage 
```cpp
#include "FIFO.h"
Fifo<double> f(256);
f.push(3.14);
f.push(12);
if (f.fifo_status() == FIFO_EMPTY) {
    std::cout<<"ITS EMPTY"<<std::endl;
    // handle the fact that it's empty
}
else {
    double a = f.pop();
    std::cout<<a<<std::endl; // prints 3.14
}

Fifo<Vector<double, 3>> accFIFO(256); // Using Vector class default constructor
```

