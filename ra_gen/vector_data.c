/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_b_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [1] = sci_b_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [2] = sci_b_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [3] = sci_b_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [4] = sci_b_uart_rxi_isr, /* SCI8 RXI (Receive data full) */
            [5] = sci_b_uart_txi_isr, /* SCI8 TXI (Transmit data empty) */
            [6] = sci_b_uart_tei_isr, /* SCI8 TEI (Transmit end) */
            [7] = sci_b_uart_eri_isr, /* SCI8 ERI (Receive error) */
            [8] = r_icu_isr, /* ICU IRQ26 (External pin interrupt 26) */
            [9] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP0), /* SCI0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP1), /* SCI0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP2), /* SCI0 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP3), /* SCI0 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI8_RXI,GROUP4), /* SCI8 RXI (Receive data full) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI8_TXI,GROUP5), /* SCI8 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI8_TEI,GROUP6), /* SCI8 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI8_ERI,GROUP7), /* SCI8 ERI (Receive error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ26,GROUP0), /* ICU IRQ26 (External pin interrupt 26) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_GPT0_COUNTER_OVERFLOW,GROUP1), /* GPT0 COUNTER OVERFLOW (Overflow) */
        };
        #endif
        #endif
