/*-
 * $Copyright$
-*/
#include <common/Infrastructure.hpp>
#include <phisch/log.h>

/* for vTaskStartScheduler */
#include <FreeRTOS.h> 
#include <FreeRTOS/include/task.h>

#include <stm32/Cpu.hpp>

#include <stm32/Pll.hpp>
#include <stm32/Pwr.hpp>
#include <stm32/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Nvic.hpp>

#include <gpio/GpioAccess.hpp>
#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32/Uart.hpp>
#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

#include <tasks/Heartbeat.hpp>
#include <stm32/Spi.hpp>
#include <spi/SpiAccess.hpp>
#include <spi/SpiDevice.hpp>
#include <devices/Ws2812bStrip.hpp>


#include <stm32/DbgMcu.hpp>
#include <stm32/Itm.hpp>
#include <stm32/Tpi.hpp>
#include <stm32/CoreDbg.hpp>

/*******************************************************************************
 * PLL Configuration
 ******************************************************************************/
static const constexpr stm32::PllCfg pllCfg = {
    .m_pllSource        = stm32::PllCfg::PllSource_t::e_PllSourceHSE,
    .m_hseSpeedInHz     = 25 * 1000 * 1000,
    .m_pllM             = 25,
    .m_pllN             = 336,
    .m_pllP             = stm32::PllCfg::PllP_t::e_PllP_Div4,
    .m_pllQ             = stm32::PllCfg::PllQ_t::e_PllQ_Div7,
    .m_sysclkSource     = stm32::PllCfg::SysclkSource_t::e_SysclkPLL,
    .m_ahbPrescaler     = stm32::PllCfg::AHBPrescaler_t::e_AHBPrescaler_None,
    .m_apb1Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_Div2,
    .m_apb2Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None
};

const uint32_t SystemCoreClock = pllCfg.getSysclkSpeedInHz();

static_assert(pllCfg.isValid() == true,                            "PLL Configuration is not valid!");
static_assert(SystemCoreClock               == 84 * 1000 * 1000,   "Expected System Clock to be at 84 MHz!");
static_assert(pllCfg.getAhbSpeedInHz()      == 84 * 1000 * 1000,   "Expected AHB to be running at 84 MHz!");
static_assert(pllCfg.getApb1SpeedInHz()     == 42 * 1000 * 1000,   "Expected APB1 to be running at 42 MHz!");
static_assert(pllCfg.getApb2SpeedInHz()     == 84 * 1000 * 1000,   "Expected APB2 to be running at 84 MHz!");

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static stm32::Scb                       scb(SCB);
static stm32::Nvic                      nvic(NVIC, scb);

static stm32::Pwr                       pwr(PWR);
static stm32::Flash                     flash(FLASH);
static stm32::Rcc                       rcc(RCC, pllCfg, flash, pwr);

/*******************************************************************************
 * GPIO Engine Handlers 
 ******************************************************************************/
static stm32::Gpio::A                   gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static stm32::Gpio::B                   gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

static stm32::Gpio::C                   gpio_C(rcc);
static gpio::GpioEngine                 gpio_engine_C(&gpio_C);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::AlternateFnPin             g_mco1(gpio_engine_A, 8);
static gpio::DigitalOutPin              g_led_green(gpio_engine_C, 13);

/*******************************************************************************
 * SWO Trace via the Cortex M4 Debug Infrastructure 
 ******************************************************************************/
static gpio::AlternateFnPin swo(gpio_engine_B, 3);

static stm32::DbgMcuT<DBGMCU_BASE, decltype(swo)>                   dbgMcu(swo);
static stm32::CoreDbgT<CoreDebug_BASE>                              coreDbg;
static stm32::TpiT<TPI_BASE, decltype(coreDbg), decltype(dbgMcu)>   tpi(coreDbg, dbgMcu);
static stm32::ItmT<ITM_BASE, decltype(tpi)>                         itm(tpi, stm32::Itm::getDivisor(SystemCoreClock /*, 2'250'000 */));

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::HeartbeatT<decltype(g_led_green)> heartbeat_gn("hrtbt_g", g_led_green, 3, 500);

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

int
main(void) {
    rcc.setMCO(g_mco1, decltype(rcc)::MCO1Output_e::e_PLL, decltype(rcc)::MCOPrescaler_t::e_MCOPre_5);

    /* FIXME This should go in the Object Tree somewhere */
    DBGMCU->CR = /* DBGMCU_CR_DBG_SLEEP_Msk | DBGMCU_CR_DBG_STOP_Msk | DBGMCU_CR_DBG_STANDBY_Msk | */ DBGMCU_CR_TRACE_IOEN_Msk;

    const unsigned sysclk = pllCfg.getSysclkSpeedInHz() / 1000;
    const unsigned ahb    = pllCfg.getAhbSpeedInHz() / 1000;
    const unsigned apb1   = pllCfg.getApb1SpeedInHz() / 1000;
    const unsigned apb2   = pllCfg.getApb2SpeedInHz() / 1000;

    PrintStartupMessage(sysclk, ahb, apb1, apb2);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        PHISCH_LOG("FATAL: Capture Error!\r\n");
        goto bad;
    }

    PHISCH_LOG("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

bad:
    PHISCH_LOG("FATAL ERROR!\r\n");
    while (1) ;

    return (0);
}

void
debug_printf(const char * const p_fmt, ...) {
    va_list va;
    va_start(va, p_fmt);

    ::tfp_format(&itm, decltype(itm)::putf, p_fmt, va);

    va_end(va);
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
