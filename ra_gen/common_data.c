/* generated common source file - do not edit */
#include "common_data.h"
icu_instance_ctrl_t g_home_irq_ctrl;

/** External IRQ extended configuration for ICU HAL driver */
const icu_extended_cfg_t g_home_irq_ext_cfg =
{ .filter_src = EXTERNAL_IRQ_DIGITAL_FILTER_PCLK_DIV, };

const external_irq_cfg_t g_home_irq_cfg =
{ .channel = 26, .trigger = EXTERNAL_IRQ_TRIG_RISING, .filter_enable = false, .clock_source_div =
          EXTERNAL_IRQ_CLOCK_SOURCE_DIV_64,
  .p_callback = home_irq_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = (void*) &g_home_irq_ext_cfg,
  .ipl = (8),
#if defined(VECTOR_NUMBER_ICU_IRQ26)
    .irq                 = VECTOR_NUMBER_ICU_IRQ26,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const external_irq_instance_t g_home_irq =
{ .p_ctrl = &g_home_irq_ctrl, .p_cfg = &g_home_irq_cfg, .p_api = &g_external_irq_on_icu };
ioport_instance_ctrl_t g_ioport_ctrl;
const ioport_instance_t g_ioport =
{ .p_api = &g_ioport_on_ioport, .p_ctrl = &g_ioport_ctrl, .p_cfg = &g_bsp_pin_cfg, };
void g_common_init(void)
{
}
