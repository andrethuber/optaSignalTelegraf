#ifndef LIBRARY_C
#define LIBRARY_C

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif

#include <pybind11/pybind11.h>
#include "../main/library.c"

PYBIND11_MODULE(library, m) {
    m.def("getIpsFromDip", &getIpsFromDip, "A function that adds two numbers");
}
