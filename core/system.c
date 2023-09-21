#include "../../include/core/system.h"
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#define SYSTICK_FREQ_HZ     (1000)
#define CPU_FREQ_HZ         (84000000)


static volatile uint64_t systicks = 0;

void sys_tick_handler(void) {
    // Increment systick
    systicks++;
}

static void rcc_setup(void) {
    // Start the rcc
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
}

static void systick_setup(void)    {
    // Set systick frequency based on CPU freq and desired freq
    systick_set_frequency(SYSTICK_FREQ_HZ, CPU_FREQ_HZ);

    // Enable the systick counter
    systick_counter_enable();
    systick_interrupt_enable();
}

void system_setup(void) {
    // Enable rcc
    rcc_setup();

    // Enable systick
    systick_setup();


}
void system_teardown(void)  {

    systick_interrupt_disable();
    systick_counter_disable();
    systick_clear();
}

uint64_t system_get_ticks(void)  {
    return systicks;
}

void system_delay(uint64_t milliseconds)    {

    uint64_t end_time = system_get_ticks() + milliseconds;
    while (system_get_ticks() < end_time) {
        // Wait
    }
}