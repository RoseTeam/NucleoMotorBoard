#include "Metrics.h"

Metrics::Metrics(){
    enabled = false;
}

void Metrics::SetMetrics(){} // m.s-1
void Metrics::Enable(bool is_enabled){enabled = is_enabled;} // sample period in seconds
char* Metrics::Print(){}

bool Metrics::isEnabled(){return enabled;}