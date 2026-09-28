/*
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 *
 * LVGL's allocator (CONFIG_LV_USE_CUSTOM_MALLOC), sending every allocation to
 * PSRAM. Through plain malloc, anything under CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL
 * lands in internal RAM, and LVGL's objects and styles are all that small.
 * Internal RAM is kept for DMA, task stacks, wifi and TLS.
 */

#include <string.h>

#include <esp_heap_caps.h>
#include <lvgl.h>

#define PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

void lv_mem_init(void)
{
}

void lv_mem_deinit(void)
{
}

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

/* Falls back to the default heap only when PSRAM is exhausted. */
void *lv_malloc_core(size_t size)
{
    void *p = heap_caps_malloc(size, PSRAM_CAPS);
    return p != NULL ? p : heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}

void *lv_realloc_core(void *p, size_t new_size)
{
    void *q = heap_caps_realloc(p, new_size, PSRAM_CAPS);
    return q != NULL ? q : heap_caps_realloc(p, new_size, MALLOC_CAP_DEFAULT);
}

void lv_free_core(void *p)
{
    heap_caps_free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    memset(mon_p, 0, sizeof(lv_mem_monitor_t));
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}
