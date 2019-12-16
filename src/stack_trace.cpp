#include "stack_trace.hpp"

#include <exception>

#ifdef USE_BACKWARD

#ifndef __GNUC__
#error "USE_BACKWARD only supports gcc"
#endif // __GNUC__

#include <backward.hpp>

namespace backward {
    backward::SignalHandling sh;
    backward::StackTrace st;
    backward::Printer printer;
    std::atomic<bool> has_new_trace {false};
    Rethrower rethrow __attribute__ ((noreturn)) = reinterpret_cast<Rethrower>(dlsym(RTLD_NEXT, "__cxa_throw"));

    void print(void)
    {
        if (has_new_trace) {
            printer.print(st);
            has_new_trace = false;
        }
    }
} // namespace backward

// Nasty trick to capture the stack trace of an exception (with gcc)
void __cxa_throw(void *ex, void *info, void (*dest)(void *)) {
    backward::st.load_here(32);
    backward::st.skip_n_firsts(1);
    backward::has_new_trace = true;
    backward::rethrow(ex, info, dest);
}

#endif // USE_BACKWARD

namespace CameraSlam {

[[noreturn]] void custom_rethrower(void)
{
#ifdef USE_BACKWARD
        backward::print();
#endif
        std::rethrow_exception(std::current_exception());
}

} // namespace CameraSlam
