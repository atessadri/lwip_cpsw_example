/*
 *  Copyright (c) Texas Instruments Incorporated 2021
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  tja11xx.c
 *
 * \brief This file contains the implementation of the TJA11xx PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/gpio.h>
#include "enetextphy.h"
#include "tja11xx.h"
#include "enetextphy_priv.h"
#include "generic_phy.h"
#include "tja11xx_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TJA1101_OUI                           (0x6037U)
#define TJA1101_MODEL                         (0x10U)
#define TJA1101_REV                           (0x02U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Tja11xx_isPhyDevSupported(EnetExtPhy_Handle hPhy,
                                      const EnetExtPhy_Version *version);

static bool Tja11xx_isMacModeSupported(EnetExtPhy_Handle hPhy,
                                       EnetExtPhy_Mii mii);

static int32_t Tja11xx_config(EnetExtPhy_Handle hPhy,
                              const EnetExtPhy_Cfg *cfg,
                              EnetExtPhy_Mii mii);

static void Tja11xx_setClockMode(EnetExtPhy_Handle hPhy);

static void Tja11xx_setMaster(EnetExtPhy_Handle hPhy);

static void Tja11xx_reset(EnetExtPhy_Handle hPhy);

static bool Tja11xx_isResetComplete(EnetExtPhy_Handle hPhy);

static void Tja11xx_rmwExtReg(EnetExtPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val);

static void Tja11xx_printRegs(EnetExtPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetExtPhy_Drv gEnetExtPhyDrvTja11xx =
{
    .name               = "tja11xx",
    .isPhyDevSupported  = Tja11xx_isPhyDevSupported,
    .isMacModeSupported = Tja11xx_isMacModeSupported,
    .config             = Tja11xx_config,
    .reset              = Tja11xx_reset,
    .isResetComplete    = Tja11xx_isResetComplete,
    .readExtReg         = GenericExtPhy_readExtReg,
    .writeExtReg        = GenericExtPhy_writeExtReg,
    .printRegs          = Tja11xx_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static bool Tja11xx_isPhyDevSupported(EnetExtPhy_Handle hPhy,
                                      const EnetExtPhy_Version *version)
{

    bool supported = false;

    if ((version->oui == TJA1101_OUI) &&
        (version->model == TJA1101_MODEL) &&
        (version->revision == TJA1101_REV))
    {
        supported = true;
    }
    else
    {
        supported = false;
    }

    return supported;
}

static bool Tja11xx_isMacModeSupported(EnetExtPhy_Handle hPhy,
                                       EnetExtPhy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case ENETEXTPHY_MAC_MII_MII:
        case ENETEXTPHY_MAC_MII_RMII:
            supported = true;
            break;

        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Tja11xx_config(EnetExtPhy_Handle hPhy,
                              const EnetExtPhy_Cfg *cfg,
                              EnetExtPhy_Mii mii)
{

    const Tja11xx_Cfg *extendedCfg = (const Tja11xx_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    int32_t status = ENETEXTPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETEXTPHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                      hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETEXTPHY_EINVALIDPARAMS;
    }

    if (status == ENETEXTPHY_SOK)
    {
        Tja11xx_setClockMode(hPhy);
        Tja11xx_setMaster(hPhy);
    }

    return status;

}

static void Tja11xx_setClockMode(EnetExtPhy_Handle hPhy)
{
    int32_t  status;
    uint16_t phyRegVal;

    status = EnetExtPhy_readReg(hPhy, TJA1101_MII_ECTRL_REG, &phyRegVal);
    if(status == ENETEXTPHY_SOK)
    {
        phyRegVal |= TJA1101_MII_ECTRL_CONFIG_EN;
        EnetExtPhy_writeReg(hPhy, TJA1101_MII_ECTRL_REG, phyRegVal);

        status = EnetExtPhy_readReg(hPhy, TJA1101_MII_COMMCFG_REG, &phyRegVal);
        phyRegVal &= (~TJA1101_MII_COMMCFG_CLKMODE);
        phyRegVal |= TJA1101_MII_COMMCFG_CLKMODE_25MHZ_CLK_OUT;
        EnetExtPhy_writeReg(hPhy, TJA1101_MII_COMMCFG_REG, phyRegVal);
    }
}

static void Tja11xx_setMaster(EnetExtPhy_Handle hPhy)
{
    int32_t status;
    uint16_t phyRegVal = 0;

    status = EnetExtPhy_readReg(hPhy, TJA1101_MII_ECTRL_REG, &phyRegVal);
    if(status == ENETEXTPHY_SOK)
    {
        phyRegVal |= TJA1101_MII_ECTRL_CONFIG_EN;
        EnetExtPhy_writeReg(hPhy, TJA1101_MII_ECTRL_REG, phyRegVal);

        status = EnetExtPhy_readReg(hPhy, TJA1101_CFG1_REG, &phyRegVal);
        phyRegVal |= TJA1101_MII_COMMCFG_MASTER;
        EnetExtPhy_writeReg(hPhy, TJA1101_CFG1_REG, phyRegVal);
    }
}


static void Tja11xx_reset(EnetExtPhy_Handle hPhy)
{
    const Tja11xx_Cfg *extendedCfg = (const Tja11xx_Cfg *)hPhy->phyCfg.extendedCfg;

    GPIO_pinWriteLow(extendedCfg->gpioRstBaseAddr, extendedCfg->gpioRstPin);
    ClockP_usleep(extendedCfg->usReset);
    GPIO_pinWriteHigh(extendedCfg->gpioRstBaseAddr, extendedCfg->gpioRstPin);
}

static bool Tja11xx_isResetComplete(EnetExtPhy_Handle hPhy)
{
    const Tja11xx_Cfg *extendedCfg = (const Tja11xx_Cfg *)hPhy->phyCfg.extendedCfg;

    /* Reset is hardware */
    ClockP_usleep(extendedCfg->usResetComplete);
    return true;
}

static void Tja11xx_rmwExtReg(EnetExtPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val)
{
#if 0
    uint16_t devad = MMD_CR_DEVADDR;
    uint16_t data;
    int32_t status;

    ENETEXTPHYTRACE_VERBOSE("PHY %u: write reg %u mask 0x%04x val 0x%04x\n",
                      hPhy->addr, reg, mask, val);

    EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    EnetExtPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    status = EnetExtPhy_readReg(hPhy, PHY_MMD_DR, &data);

    if (status == ENETEXTPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);
        EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
        EnetExtPhy_writeReg(hPhy, PHY_MMD_DR, data);
    }
#endif
}

static void Tja11xx_printRegs(EnetExtPhy_Handle hPhy)
{
#if 0
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetExtPhy_readReg(hPhy, PHY_BMCR, &val);
    DebugP_log("PHY %u: BMCR        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_BMSR, &val);
    DebugP_log("PHY %u: BMSR        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    DebugP_log("PHY %u: PHYIDR1     = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    DebugP_log("PHY %u: PHYIDR2     = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_ANAR, &val);
    DebugP_log("PHY %u: ANAR        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_ANLPAR, &val);
    DebugP_log("PHY %u: ANLPAR      = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_ANER, &val);
    DebugP_log("PHY %u: ANER        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_ANNPTR, &val);
    DebugP_log("PHY %u: ANNPTR      = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_ANNPRR, &val);
    DebugP_log("PHY %u: ANNPRR      = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_GIGCR, &val);
    DebugP_log("PHY %u: CFG1        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_GIGSR, &val);
    DebugP_log("PHY %u: STS1        = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, PHY_GIGESR, &val);
    DebugP_log("PHY %u: 1KSCR       = 0x%04x\n", phyAddr, val);
    EnetExtPhy_readReg(hPhy, DP83869_PHYCR, &val);
    DebugP_log("PHY %u: PHYCR       = 0x%04x\n", phyAddr, val);
#endif
}
