#include "typedefs.h"
#include "gpio_defs.h"
#include "stm32l1xx_hal_gpio.h"

void GPIO_Pin_Setting(GPIO_TypeDef *gpio, uint16_t nPin, 
    uint32_t speed, uint32_t mode, uint32_t pull)
{
#if 1
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = nPin;
    GPIO_InitStructure.Speed = speed;
    GPIO_InitStructure.Mode = mode;
    GPIO_InitStructure.Pull = pull;
    HAL_GPIO_Init(gpio, &GPIO_InitStructure);
#else
    u16 i;
    u32 nCfg, nMask = 0x0F;
    
    nCfg  = (mode&0x10) ?speed :0;
    nCfg |= mode & 0x0C;
    
    if(nPin == 0)
        return;
    
    if(nPin < 0x0100)
    {
        for(i=nPin; (0x01&i)==0; i >>= 1) {
            nCfg <<= 4;
            nMask <<= 4;
        }
        
        gpio->CRL &= ~nMask;
        gpio->CRL |= nCfg;
    }
    else
    {
        for(i=(nPin>>8); (0x01&i)==0; i >>= 1) {
            nCfg <<= 4;
            nMask <<= 4;
        }
        
        gpio->CRH &= ~nMask;
        gpio->CRH |= nCfg;
    }
    
    if(GPIO_Mode_IPD==mode)
        gpio->BRR = nPin;
    
    else if(GPIO_Mode_IPU==mode)
        gpio->BSRR = nPin;
    
#endif
}
