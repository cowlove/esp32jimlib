#pragma once

#include <vector>
#include <functional>

template<typename ReturnType, typename... Args> 
class CallbackWrapper {
public:
    typedef ReturnType (*CFunctionType)(Args...);
    typedef std::function<ReturnType(Args...)> ClosureType;
    typedef std::vector<pair<CFunctionType, ClosureType>> ListType;
    static ListType &list() { static ListType *firstUse = new ListType(); return *firstUse; }
    static CFunctionType wrap(ClosureType cl);
    static ClosureType getClosure(int n) { return list()[n].second; }
    static void freeWrapper(const CFunctionType &f) { for(auto i : list()) if (i.first == f) i.first = NULL; }
};

template<int X,typename ReturnType, typename... Args> \
ReturnType cWrapperFunc(Args ... args) { \
    return CallbackWrapper<ReturnType,Args...>::getClosure(X)(args...); \
}

template<typename ReturnType, typename... Args> 
typename CallbackWrapper<ReturnType,Args...>::CFunctionType CallbackWrapper<ReturnType,Args...>::wrap(ClosureType cl) {
    #define IMPLEMENT_WRAPPER(N) \
    if (list().size() < N + 1 || list().at(N).first == NULL) { \
        list().resize(N + 1, {NULL, NULL}); \
        list()[N].first = cWrapperFunc<N, ReturnType,Args...>; \
        list()[N].second = cl; \
        return cWrapperFunc<N, ReturnType,Args...>; \
    }
    IMPLEMENT_WRAPPER(0);
    IMPLEMENT_WRAPPER(1);
    IMPLEMENT_WRAPPER(2);
    IMPLEMENT_WRAPPER(3);
    IMPLEMENT_WRAPPER(4);
    IMPLEMENT_WRAPPER(5);
    IMPLEMENT_WRAPPER(6);
    IMPLEMENT_WRAPPER(7);
    IMPLEMENT_WRAPPER(8);
    IMPLEMENT_WRAPPER(9);
    IMPLEMENT_WRAPPER(10);
    IMPLEMENT_WRAPPER(11);
    IMPLEMENT_WRAPPER(12);
    IMPLEMENT_WRAPPER(13);
    IMPLEMENT_WRAPPER(14);
    IMPLEMENT_WRAPPER(15);
    IMPLEMENT_WRAPPER(16);
    IMPLEMENT_WRAPPER(17);
    IMPLEMENT_WRAPPER(18);
    IMPLEMENT_WRAPPER(19);
    IMPLEMENT_WRAPPER(20);
    IMPLEMENT_WRAPPER(21);
    IMPLEMENT_WRAPPER(22);
    IMPLEMENT_WRAPPER(23);
    IMPLEMENT_WRAPPER(24);
    IMPLEMENT_WRAPPER(25);
    IMPLEMENT_WRAPPER(26);
    IMPLEMENT_WRAPPER(27);
    IMPLEMENT_WRAPPER(28);
    IMPLEMENT_WRAPPER(29);
    IMPLEMENT_WRAPPER(30);
    IMPLEMENT_WRAPPER(31);
    // Add more lines here to increase capacity 
    throw std::overflow_error("CallbackWrapper::wrap(): not enough wrappers, increase static allocation in wrap()");
    //return NULL;
    #undef IMPLEMENT_WRAPPER
}

// examples:
//const char *(*p1)(int, int) = CallbackWrapper<const char *, int, int>::wrap([](int a, int b) { return ""; });
//void (*p3)(int, const char *) = CallbackWrapper<void, int, const char *>::wrap([](int a, const char *b) { printf("p3\n"); });
