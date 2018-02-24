#include "ch.h"
#include "hal.h"

extern uint32_t __ccmfunc_init_text__, __ccmfunc_init__, __ccmfunc_end__;

/*
 * Late initialization code.
 */
void __late_init(void) {
  // CCM RAM protection register
  SYSCFG->RCR = 0x0000;

  /* Copying CCM initialization code. */
  uint32_t *tp = &__ccmfunc_init_text__;
  uint32_t *p = &__ccmfunc_init__;
  
  while (p < &__ccmfunc_end__) {
    *p = *tp;
    p++;
    tp++;
  }

  // CCM RAM protection register
  SYSCFG->RCR = 0x00ff;
}
