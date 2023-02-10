/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <drivers/hw_include/cslr_soc.h>
#include <stdbool.h>

#include "ti_board_config.h"
#include "ti_enet_config.h"
#include "tja11xx.h"
#include <networking/enet/core/include/phy/dp83867.h>
#include <drivers/pinmux.h>

/*!
 * \brief Common Processor Board (CPB) board's DP83867 PHY configuration.
 */
static const Dp83867_Cfg gEnetCpbBoard_dp83867PhyCfg =
{
    .txClkShiftEn         = true,
    .rxClkShiftEn         = true,
    .txDelayInPs          = 1500U,  /* 2.00 ns */
    .rxDelayInPs          = 2000U,  /* 2.00 ns */
    .txFifoDepth          = 4U,
    .impedanceInMilliOhms = 35000,  /* 35 ohms */
    .idleCntThresh        = 4U,     /* Improves short cable performance */
    .gpio0Mode            = DP83867_GPIO0_LED3,
    .gpio1Mode            = DP83867_GPIO1_COL, /* Unused */
    .ledMode              =
    {
        DP83867_LED_LINKED,         /* Unused */
        DP83867_LED_LINKED_100BTX,
        DP83867_LED_RXTXACT,
        DP83867_LED_LINKED_1000BT,
    },
};

/*!
 * \brief Common Processor Board (CPB) board's TJA1101 PHY configuration.
 */
static const Tja11xx_Cfg gEnetCpbBoard_tja11xxPhyCfg =
{
    .gpioRstBaseAddr = ETH_100BT1_RSTN_BASE_ADDR,
    .gpioRstPin = (19UL),
    .usReset = 10000UL,
    .usResetComplete = 10000UL,
};

/*
 * AM243x board configuration.
 *
 * 2 x RMII PHY connected to am243x-evm CPSW_3G MAC port.
 */
static const EnetBoard_PortCfg gEnetCpbBoard_am243x_titan_EthPort[] =
{
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 1,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83867PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83867PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 2,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_tja11xxPhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_tja11xxPhyCfg),
        },
        .flags    = 0U,
    },
};



/* The delay values are set based on trial and error and not tuned per port of the evm */
uint32_t txDelay = 250U;
uint32_t rxDelay = 2000U;


/*
 * Board info
 */

/*
 * Tx and Rx Delay set
 */
void Board_TxRxDelaySet(const EnetBoard_PhyCfg *boardPhyCfg)
{
    Dp83867_Cfg *extendedCfg = (Dp83867_Cfg *)boardPhyCfg->extendedCfg;
    extendedCfg->txDelayInPs = txDelay;
    extendedCfg->rxDelayInPs = rxDelay;
    return;
}

/*
 * Get ethernet phy configuration
 */
const EnetBoard_PhyCfg *Board_getPhyCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *ethPortCfg = NULL;
    bool found = false;
    uint32_t i;

    for (i = 0U; i < ENET_SYSCFG_MAX_MAC_PORTS; i++)
    {
        ethPortCfg = &gEnetCpbBoard_am243x_titan_EthPort[i];

        if ((ethPortCfg->enetType == ethPort->enetType) &&
            (ethPortCfg->instId == ethPort->instId) &&
            (ethPortCfg->macPort == ethPort->macPort) &&
            (ethPortCfg->mii.layerType == ethPort->mii.layerType) &&
            (ethPortCfg->mii.sublayerType == ethPort->mii.sublayerType))
        {
            found = true;
            break;
        }
    }
    return found ? &ethPortCfg->phyCfg : NULL;
}

/*
 * Get ethernet board id
 */
uint32_t Board_getEthBoardId(void)
{
    return ENETBOARD_AM64X_AM243X_EVM;
}

/*
 * Get ethernet type
 */
uint32_t Board_getEthType(void)
{
    return ENET_CPSW_3G;
}
