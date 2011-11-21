/* Copyright (C) 2010, 2011 emsys Embedded Systems GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "disable_init_defines.h"
#include "../f_mass_storage.c"

int personality_msc_bind_config(struct usb_configuration *c)
{
  int result = platform_driver_register(&fsg_platform_driver);
  if (!result) {
    struct fsg_common* common;
    if (fsg_cfg.vendor_name && 8 < strlen(fsg_cfg.vendor_name)) {
#ifdef CONFIG_LGE_ANDRIOD_USB
      fsg_cfg.vendor_name = "LGE";
#else /* ndef CONFIG_LGE_ANDRIOD_USB */
      fsg_cfg.vendor_name = NULL;
#endif /* ndef CONFIG_LGE_ANDRIOD_USB */
    }
    common = fsg_common_init(NULL, c->cdev, &fsg_cfg);
    if (common) {
      result = fsg_bind_config(c->cdev, c, common);
      if (!result) {
        return 0;
      }
    } else {
      result = -1;
    }
    platform_driver_unregister(&fsg_platform_driver);
  } else {
    printk(KERN_WARNING "%s: platform_driver_register result=%d\n", __FUNCTION__, result);
  }
  return result;
}

void personality_msc_unbind_config(struct usb_configuration *c)
{
  printk(KERN_DEBUG "%s()\n", __FUNCTION__);
  platform_driver_unregister(&fsg_platform_driver);
}

void personality_msc(struct usb_composite_dev* cdev, const char* name)
{
  static struct usb_configuration config = {
    .bind		= personality_msc_bind_config,
    .unbind		= personality_msc_unbind_config,
    .bConfigurationValue = 1,
    .bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
  };
  config.label = name;
  usb_add_config(cdev, &config);
}
