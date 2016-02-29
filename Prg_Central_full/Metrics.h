#ifndef METRICS_H
#define METRICS_H

#include <stdint.h>
#include "mbed.h"

#define BUFFER_SIZE 256

class Metrics
{
    public:
        Metrics();

        void SetMetrics(); // m.s-1
        void Enable(bool is_enabled); // sample period in seconds
        char* Print();
        bool isEnabled();
        
        bool enabled;

    private:
        char buffer[BUFFER_SIZE];
};

#endif // METRICS_H