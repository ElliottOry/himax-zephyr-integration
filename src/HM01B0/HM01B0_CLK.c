
  #include <nrfx.h>
  
  #include "HM01B0_CLK.h"
  
  
 #include "nrfx_ppi.h"
 #include "nrfx_gpiote.h"
 #include <nrfx_gpiote.h>
 #include <helpers/nrfx_gppi.h>
  
  
  
  /* NRFX timer instance for camera clock output */
 
 /* Timer instance used for the camera master clock */
 const nrfx_timer_t cam_timer = NRFX_TIMER_INSTANCE(MCLK_TIMER_INSTANCE);
  
  //uint32_t *timer0_prescaler_address = (uint32_t *)0x40009510; // address for timer1 prescaler
  //uint32_t timer0_prescaler0 = 0x00000000;
  
  static void timer_dummy_handler(nrf_timer_event_t event_type, void *p_context) {}
  
  void hm_clk_out(void)
  {

   nrfx_err_t err_code;

   static nrfx_gpiote_t clk_gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
   static uint8_t ppi_ch = 5;
 

   err_code = nrfx_gpiote_init(&clk_gpiote_inst,
                               NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
   if ((err_code != NRFX_SUCCESS) &&
       (err_code != NRFX_ERROR_ALREADY_INITIALIZED)) {
     printk("GPIOTE init error %d\n", err_code);
     return;
   }

   nrfx_gpiote_output_config_t config = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
   nrfx_gpiote_task_config_t   task_cfg = {
       .task_ch       = 5,
       .polarity      = NRF_GPIOTE_POLARITY_TOGGLE,
       .init_val      = NRF_GPIOTE_INITIAL_VALUE_LOW,
   };
   err_code = nrfx_gpiote_output_configure(&clk_gpiote_inst,
                                           PIN_MCLK,
                                           &config,
                                           &task_cfg);
   if (err_code != NRFX_SUCCESS) {
     printk("GPIOTE config error %d\n", err_code);
     return;
   }
 

   /* Run the timer at 16 MHz so a toggle on every compare yields 8 MHz */
   nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
   timer_cfg.mode = NRF_TIMER_MODE_TIMER;
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
   err_code = nrfx_timer_init(CAM_TIMER, &timer_cfg, NULL);
   if ((err_code != NRFX_SUCCESS) &&
       (err_code != NRFX_ERROR_INVALID_STATE)) {
     printk("Timer init error %d\n", err_code);
     return;
   }
   nrfx_timer_extended_compare(CAM_TIMER, NRF_TIMER_CC_CHANNEL0, 1UL,
                               NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
 

   err_code = nrfx_gppi_channel_alloc(&ppi_ch);
   if (err_code != NRFX_SUCCESS) {
     printk("PPI alloc error %d\n", err_code);
     return;
   }
 
   uint32_t compare_evt_addr = nrfx_timer_event_address_get(CAM_TIMER,
                                                           NRF_TIMER_EVENT_COMPARE0);
   uint32_t gpiote_task_addr = nrfx_gpiote_out_task_address_get(&clk_gpiote_inst,
                                                                PIN_MCLK);
   nrfx_gppi_channel_endpoints_setup(ppi_ch, compare_evt_addr, gpiote_task_addr);

 

   nrfx_gpiote_out_task_enable(&clk_gpiote_inst, PIN_MCLK);
   nrfx_gppi_channels_enable(BIT(ppi_ch));
   nrfx_timer_enable(CAM_TIMER);
 

   printk("HM01B0_CLK: Clock output enabled on P0.%02d\n", PIN_MCLK);
 }
  
  void hm_clk_enable(bool enable)
  {
    if (enable) {
      nrfx_timer_enable(CAM_TIMER);
    } else {
      nrfx_timer_disable(CAM_TIMER);
    }
  }