/*
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 *
 * Force-included into libAtomVM only: its malloc, calloc and realloc go to
 * PSRAM whatever the size, falling back to the default heap when PSRAM is
 * exhausted. Plain malloc keeps anything under CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL
 * in internal RAM, and the VM's small heaps, messages and binaries would
 * otherwise crowd out DMA buffers, task stacks, wifi and TLS. free() needs no
 * change: it releases memory from any heap.
 */

#ifndef AVM_PSRAM_ALLOC_H
#define AVM_PSRAM_ALLOC_H

/* Declared before the macros below, so these prototypes are not rewritten. */
#include <stddef.h>
#include <stdlib.h>

#include <esp_heap_caps.h>

#define AVM_PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

static inline void *avm_psram_malloc(size_t size)
{
    void *p = heap_caps_malloc(size, AVM_PSRAM_CAPS);
    return p != NULL ? p : heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}

static inline void *avm_psram_calloc(size_t count, size_t size)
{
    void *p = heap_caps_calloc(count, size, AVM_PSRAM_CAPS);
    return p != NULL ? p : heap_caps_calloc(count, size, MALLOC_CAP_DEFAULT);
}

static inline void *avm_psram_realloc(void *ptr, size_t size)
{
    void *p = heap_caps_realloc(ptr, size, AVM_PSRAM_CAPS);
    return p != NULL || size == 0 ? p : heap_caps_realloc(ptr, size, MALLOC_CAP_DEFAULT);
}

#define malloc(size) avm_psram_malloc(size)
#define calloc(count, size) avm_psram_calloc(count, size)
#define realloc(ptr, size) avm_psram_realloc(ptr, size)

#endif
