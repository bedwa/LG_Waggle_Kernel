/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <plat/dmtimer.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1

#define PWM_DUTY_MAX    579 /* 13MHz / (579 + 1) = 22.4kHz */

static bool g_bAmpEnabled = false;


#if defined(CONFIG_MACH_LGE_COSMO_REV_A)	
#define GPIO_VIB_EN		109
#else
#define GPIO_VIB_EN		25
#endif

#define CLK_COUNT 38400000

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B) || defined(CONFIG_MACH_LGE_COSMO_REV_C)
#define MOTOR_RESONANCE_HZ 175
#else	// LGE LAB4 CH.PARK@LGE.COM 20101227 REV_D 225Hz
#define MOTOR_RESONANCE_HZ 226
#endif
#define MOTOR_RESONANCE_COUTER_VALUE	 (0xFFFFFFFE - ((CLK_COUNT/MOTOR_RESONANCE_HZ)/128))
#define MOTOR_RESONANCE_TIMER_ID 8

#define PWM_DUTY_MAX	 ((CLK_COUNT/MOTOR_RESONANCE_HZ)/128)
#define PWM_DUTY_HALF	(0xFFFFFFFF - (PWM_DUTY_MAX >> 1))
#define DUTY_HALF	(PWM_DUTY_MAX >> 1)

static struct omap_dm_timer *omap_vibrator_timer = NULL;

static struct timeval startTime;
static struct timeval endTime;

static void vib_enable(bool enable )
{
	if (enable)
		gpio_set_value(GPIO_VIB_EN, 1);
    else 	
		gpio_set_value(GPIO_VIB_EN, 0);

	//return VIBE_S_SUCCESS;
}

struct timeval calcConsumedTime(struct timeval z_stStartTime, struct timeval z_stEndTime)
{
	struct timeval    z_stConsumedTime;
	z_stConsumedTime.tv_sec = z_stEndTime.tv_sec - z_stStartTime.tv_sec;
	z_stConsumedTime.tv_usec = z_stEndTime.tv_usec - z_stStartTime.tv_usec;
	if ( z_stConsumedTime.tv_usec < 0 )
	{
		z_stConsumedTime.tv_sec -= 1;
		z_stConsumedTime.tv_usec = (z_stEndTime.tv_usec - z_stStartTime.tv_usec + 1000000);
	}
	return z_stConsumedTime;
}   


static void vib_generatePWM(bool enable)
{
    if(enable)
    {
		omap_dm_timer_enable(omap_vibrator_timer);
		omap_dm_timer_set_match(omap_vibrator_timer, 1, PWM_DUTY_HALF);
		omap_dm_timer_set_pwm(omap_vibrator_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_set_load_start(omap_vibrator_timer, 1, MOTOR_RESONANCE_COUTER_VALUE);
		//vibe_timer_state = 1;
	/*
	// Select clock 

	omap_dm_timer_start(omap_vibrator_timer);		
	*/
    }
	else{
		omap_dm_timer_stop(omap_vibrator_timer);	
		omap_dm_timer_disable(omap_vibrator_timer);
	}

	//return VIBE_S_SUCCESS;
}


/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
//printk( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable[%d]\n", g_bAmpEnabled );

   	if ( g_bAmpEnabled ) {
       	g_bAmpEnabled = false;
		vib_enable(false);
		vib_generatePWM(false);

		//printk([vibrator TIME] start time
		//do_gettimeofday(&endTime);
		//endTime = calcConsumedTime(startTime, endTime);
		//printk("[##TIME##] took %ld.%03ld seconds\n", endTime.tv_sec, endTime.tv_usec / 1000);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
	//printk( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnabled[%d]\n", g_bAmpEnabled );
	if ( ! g_bAmpEnabled ) 
      {
        g_bAmpEnabled = true;
		vib_generatePWM(true);
		vib_enable(true);

		//do_gettimeofday(&startTime);
	 }
   	return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
	//int status = 0;
	//int ret = 0;
	
    printk( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" );
	//g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
	gpio_request(GPIO_VIB_EN, "vib_en_gpio");
	gpio_direction_output(GPIO_VIB_EN, 1);
	gpio_set_value(GPIO_VIB_EN, 0);

	omap_vibrator_timer	=	omap_dm_timer_request_specific(MOTOR_RESONANCE_TIMER_ID);
	if (omap_vibrator_timer == NULL) {
		printk(KERN_ERR "failed to request vibrator pwm timer\n");		
	}

	omap_dm_timer_set_source(omap_vibrator_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_disable(omap_vibrator_timer);

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
   	printk( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" );

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call
    ** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
    ** input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	int tmp;
#if 1
    //VibeUInt32 nTmp;
	unsigned int nTmp;
#endif
    VibeInt8 nForce;

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1) return VIBE_E_FAIL;

            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }

#if 1
	//printk( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nForce =	%d \n", nForce );
	//nForce = 127;
	/* Check the Force value with Max and Min force value */
	//tmp = (DUTY_HALF - ((DUTY_HALF * nForce)/127));
	if (nForce > 124) nForce = 127;
	if (nForce < -127) nForce = -127;
	
	if (nForce == 0) {
		vib_enable(false);
		//omap_dm_timer_stop(omap_vibrator_timer);		
	} else {
		vib_enable(true);
		//nTmp = 0xFFFFFFF7 - (((127 - nForce) * 9) >> 1);
		
		nTmp = 0xFFFFFFFE - (DUTY_HALF - (DUTY_HALF * nForce /127));		
		//nTmp = 0xFFFFFFFE - (((127 - nForce) * 9) >> 1);

		/*
		nTmp = 0xFFFFFFFE - tmp;
		*/
		//printk("nTmp: %x, nForce: %d\n", nTmp, nForce);
		
		omap_dm_timer_set_match(omap_vibrator_timer, 1, nTmp);

		omap_dm_timer_start(omap_vibrator_timer);		
		//omap_dm_timer_set_load_start(omap_vibrator_timer, 1, MOTOR_RESONANCE_COUTER_VALUE);

	}
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}
