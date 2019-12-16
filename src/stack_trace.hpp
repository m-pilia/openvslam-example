#pragma once

#include <atomic>

#ifdef USE_BACKWARD

#ifndef __GNUC__
#error "USE_BACKWARD only supports gcc"
#endif // __GNUC__

#include <backward.hpp>

namespace backward {
    using Rethrower = void (*)(void*, void*, void(*)(void*));

    extern backward::SignalHandling sh;
    extern backward::StackTrace st;
    extern backward::Printer printer;
    extern std::atomic<bool> has_new_trace;
    extern Rethrower rethrow __attribute__ ((noreturn));

    void print(void);
} // namespace backward

// Nasty trick to capture the stack trace of an exception (with gcc)
extern "C" {
    void __cxa_throw(void *ex, void *info, void (*dest)(void *));
}

#endif // USE_BACKWARD

namespace CameraSlam {

    [[noreturn]] void custom_rethrower(void);

} // namespace CameraSlam
