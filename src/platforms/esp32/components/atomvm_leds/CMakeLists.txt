#
# This file is part of AtomVM.
#
# SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
#

# WHOLE_ARCHIVE keeps the port driver's registration record from being dropped
# by the linker: nothing in the VM references it by name.
idf_component_register(
    SRCS "ports/atomvm_leds.c"
    PRIV_REQUIRES "libatomvm" "avm_sys" "esp_driver_rmt" "esp_timer"
    WHOLE_ARCHIVE
)
