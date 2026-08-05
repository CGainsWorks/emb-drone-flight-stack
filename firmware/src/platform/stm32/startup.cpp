// Reset vector, vector table and C/C++ runtime bring-up for the STM32F405ZGT6.
//
// Freestanding: no libc startup is used, so .data/.bss initialisation and the
// C++ static constructor pass are done explicitly here.

#include <cstdint>

extern "C" {

// Provided by the linker script.
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

extern void (*__preinit_array_start[])();
extern void (*__preinit_array_end[])();
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();

int main();

void Reset_Handler();
void Default_Handler();
void USART1_IRQHandler();

// Normally supplied by the C runtime start files, which -nostartfiles removes.
// Only referenced when a static object registers a destructor; this firmware
// never returns from main, so nothing is ever torn down.
void* __dso_handle = nullptr;

// Every unimplemented vector traps here. Parking in a loop rather than
// returning means a stray interrupt is visible under a debugger instead of
// silently corrupting execution.
void Default_Handler() {
    while (true) {
    }
}

// Weak aliases: any handler the firmware does not define falls back to the
// trap above.
#define FC_WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void NMI_Handler() FC_WEAK_ALIAS;
void HardFault_Handler() FC_WEAK_ALIAS;
void MemManage_Handler() FC_WEAK_ALIAS;
void BusFault_Handler() FC_WEAK_ALIAS;
void UsageFault_Handler() FC_WEAK_ALIAS;
void SVC_Handler() FC_WEAK_ALIAS;
void DebugMon_Handler() FC_WEAK_ALIAS;
void PendSV_Handler() FC_WEAK_ALIAS;
void SysTick_Handler() FC_WEAK_ALIAS;

void Reset_Handler() {
    // Copy initialised data from its load address in flash into RAM.
    uint32_t* src = &_sidata;
    uint32_t* dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    // Zero the .bss section.
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0u;
    }

    // Enable the FPU (CP10/CP11 full access) before any float code runs.
    // SCB->CPACR at 0xE000ED88.
    volatile uint32_t* cpacr = reinterpret_cast<volatile uint32_t*>(0xE000ED88u);
    *cpacr = *cpacr | (0xFu << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    // C++ static constructors.
    for (void (**p)() = __preinit_array_start; p < __preinit_array_end; ++p) {
        (*p)();
    }
    for (void (**p)() = __init_array_start; p < __init_array_end; ++p) {
        (*p)();
    }

    // -Wpedantic objects to naming ::main from within the program; here it is
    // the deliberate handover from the reset vector to the application.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    main();
#pragma GCC diagnostic pop

    // main() must not return on a flight controller.
    while (true) {
    }
}

using VectorEntry = void (*)();

// The vector table must be long enough to cover every IRQ this firmware
// enables. USART1 is position 37, so the table runs past it.
__attribute__((section(".isr_vector"), used))
const VectorEntry g_vector_table[] = {
    reinterpret_cast<VectorEntry>(&_estack),  // 0  initial stack pointer
    Reset_Handler,                            // 1
    NMI_Handler,                              // 2
    HardFault_Handler,                        // 3
    MemManage_Handler,                        // 4
    BusFault_Handler,                         // 5
    UsageFault_Handler,                       // 6
    nullptr,                                  // 7  reserved
    nullptr,                                  // 8  reserved
    nullptr,                                  // 9  reserved
    nullptr,                                  // 10 reserved
    SVC_Handler,                              // 11
    DebugMon_Handler,                         // 12
    nullptr,                                  // 13 reserved
    PendSV_Handler,                           // 14
    SysTick_Handler,                          // 15

    // External interrupts, position 0 upwards.
    Default_Handler,  // 0  WWDG
    Default_Handler,  // 1  PVD
    Default_Handler,  // 2  TAMP_STAMP
    Default_Handler,  // 3  RTC_WKUP
    Default_Handler,  // 4  FLASH
    Default_Handler,  // 5  RCC
    Default_Handler,  // 6  EXTI0
    Default_Handler,  // 7  EXTI1
    Default_Handler,  // 8  EXTI2
    Default_Handler,  // 9  EXTI3
    Default_Handler,  // 10 EXTI4
    Default_Handler,  // 11 DMA1_Stream0
    Default_Handler,  // 12 DMA1_Stream1
    Default_Handler,  // 13 DMA1_Stream2
    Default_Handler,  // 14 DMA1_Stream3
    Default_Handler,  // 15 DMA1_Stream4
    Default_Handler,  // 16 DMA1_Stream5
    Default_Handler,  // 17 DMA1_Stream6
    Default_Handler,  // 18 ADC
    Default_Handler,  // 19 CAN1_TX
    Default_Handler,  // 20 CAN1_RX0
    Default_Handler,  // 21 CAN1_RX1
    Default_Handler,  // 22 CAN1_SCE
    Default_Handler,  // 23 EXTI9_5
    Default_Handler,  // 24 TIM1_BRK_TIM9
    Default_Handler,  // 25 TIM1_UP_TIM10
    Default_Handler,  // 26 TIM1_TRG_COM_TIM11
    Default_Handler,  // 27 TIM1_CC
    Default_Handler,  // 28 TIM2
    Default_Handler,  // 29 TIM3
    Default_Handler,  // 30 TIM4
    Default_Handler,  // 31 I2C1_EV
    Default_Handler,  // 32 I2C1_ER
    Default_Handler,  // 33 I2C2_EV
    Default_Handler,  // 34 I2C2_ER
    Default_Handler,  // 35 SPI1
    Default_Handler,  // 36 SPI2
    USART1_IRQHandler,  // 37 USART1  <- CRSF receiver
    Default_Handler,  // 38 USART2
    Default_Handler,  // 39 USART3
    Default_Handler,  // 40 EXTI15_10
    Default_Handler,  // 41 RTC_Alarm
    Default_Handler,  // 42 OTG_FS_WKUP
    Default_Handler,  // 43 TIM8_BRK_TIM12
    Default_Handler,  // 44 TIM8_UP_TIM13
    Default_Handler,  // 45 TIM8_TRG_COM_TIM14
    Default_Handler,  // 46 TIM8_CC
    Default_Handler,  // 47 DMA1_Stream7
};

}  // extern "C"
