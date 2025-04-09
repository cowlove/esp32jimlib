#pragma once 
#include <cstdlib>

template<class T>
class ExtrapolationTable {
        bool between(T a, T b, T c) { 
                return (c >= a && c < b) || (c <= a && c > b);
        }
public:
    struct Pair {
            T a,b;        
    } *table;
    ExtrapolationTable(struct Pair *t) : table(t) {}
    T extrapolate(T in, bool reverse = false) {
        for(int index = 1; table[index].a != -1 || table[index].b != -1; index++) {     
            if (!reverse && between(table[index - 1].a, table[index].a, in))  
                return table[index - 1].b + (in - table[index - 1].a) 
                    * (table[index].b - table[index - 1].b) 
                    / (table[index].a - table[index - 1].a);
            if (reverse && between(table[index - 1].b, table[index].b, in)) 
                return table[index - 1].a + (in - table[index - 1].b) 
                    * (table[index].a - table[index - 1].a) 
                    / (table[index].b - table[index - 1].b);                

        }
        return -1;
    }
    T operator () (T in) { return extrapolate(in); }
};

static const float FEET_PER_METER = 3.28084;
static const float MPS_PER_KNOT = 0.51444;

inline float random01() { 	return rand() / (RAND_MAX + 1.0); }
