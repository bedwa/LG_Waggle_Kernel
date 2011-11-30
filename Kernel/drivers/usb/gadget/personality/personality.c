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
#include "p_composite.c"
#include "linux/sched.h"
#include "linux/semaphore.h"
#include "linux/fs.h"

#include "p_lge_normal.h"
#include "p_msc_adb.h"
#include "p_rndis_adb.h"
#include "personality.h"
#include "lge_serial_number.h"

struct factory {
  int value;
  void (*add_configurations)(struct usb_composite_dev* cdev, const char* name);
  const char* name;
};
static struct factory personalities[] = {
  { 0x06000, personality_acm_ser, "Factory mode" },
  { 0x0618e, personality_acm_ser_ser_msc, "LGE Android composite" },
  { 0x1618e, personality_acm_ser_ser_msc_adb, "LGE Android composite" },
  { 0x061a1, personality_ser_ecm_acm_ser_msc_adb, "Android network mode (ADB enable)" },
  { 0x061a2, personality_ser_ecm_acm_ser_msc, "Android network mode (ADB disable)" },
  { 0x061d9, personality_rndis, "Android RNDIS mode" },
  { 0x161d9, personality_rndis_adb, "Android RNDIS mode" },
  { 0x061a6, personality_msc, "UMS default mode" },
  { 0x161a6, personality_msc_adb, "UMS default mode" },
  {} };
static bool is_personality_supported(int tested_value)
{
  struct factory * iter = 0;
  for (iter = personalities; iter->value && iter->name; ++iter) {
    if (iter->value == tested_value) return true;
  }
  return false;
}

struct personality {
  struct usb_composite_driver driver;
  struct workqueue_struct* restart_worker;
  struct work_struct restart;
  struct usb_gadget *gadget;
  struct sysfs_dirent *state_sd;
  bool attached; /**< The value is based on the last usb_gadget_vbus_connect/disconnect() call. */
  bool configured; /**< True after a setup() call, false initially and after a disconnect() call */
  int personality; /**< The current personality or zero if no selection was made */
  int next_personality; /**< The next personality when the gadget is being reconfigured */
  struct rw_semaphore next_personality_lock;
  struct kset *kset;
  /** The gadget's original setup() function pointer is used when overriding */
  int (*setup_super)(struct usb_gadget*, const struct usb_ctrlrequest*);
  /** The gadget's original disconnect() function pointer is used when overriding */
  void (*disconnect_super)(struct usb_gadget*);
  int (*vbus_session_super) (struct usb_gadget *, int is_active);
  struct usb_gadget_ops gadget_ops_sub;
  const struct usb_gadget_ops *gadget_ops_super;
};
static struct personality* cdev_to_personality(struct usb_composite_dev* cdev)
{
  if (!cdev || !cdev->driver) {
    return 0;
  }
  return container_of(cdev->driver, struct personality, driver);
}

static ssize_t get_state(struct device* dev, struct device_attribute* att, char* buf)
{
  struct usb_composite_dev* cdev = dev ? dev_get_drvdata(dev) : 0;
  struct personality* pers = cdev_to_personality(cdev);
  dev_dbg(dev, "get_state\n");
  if (pers && buf && down_read_trylock(&pers->next_personality_lock)) {
    int pos = scnprintf(buf, PAGE_SIZE, "{ \"personality\" : %d", pers->personality);
    if (pers->attached) {
      pos += scnprintf(buf+pos, PAGE_SIZE-pos, ", \"attached\" : true");
      if (pers->configured) {
        pos += scnprintf(buf+pos, PAGE_SIZE-pos, ", \"configured\" : true");
        if (USB_SPEED_HIGH == cdev->gadget->speed) {
          pos += scnprintf(buf+pos, PAGE_SIZE-pos, ", \"high-speed\" : true");
        } else if (USB_SPEED_FULL == cdev->gadget->speed) {
          pos += scnprintf(buf+pos, PAGE_SIZE-pos, ", \"full-speed\" : true");
        } else if (USB_SPEED_LOW == cdev->gadget->speed) {
          pos += scnprintf(buf+pos, PAGE_SIZE-pos, ", \"low-speed\" : true");
        }
      }
    }
    pos += scnprintf(buf+pos, PAGE_SIZE-pos, " }\n");
    up_read(&pers->next_personality_lock);
    dev_dbg(dev, "get_state: %s", buf);
    return pos; /* fs/sysfs/file.c fill_read_buffer() checks result range */
  } else {
    dev_dbg(dev, "get_state: ECONNRESET\n");
    return -ECONNRESET;
  }
}
/* int get_personality(struct usb_composite_dev* cdev) */
/* { */
/*   struct personality* pers = cdev_to_personality(cdev); */
/*   return pers ? pers->personality : -1; */
/* } */
int set_next_personality(struct usb_composite_dev* cdev, int next)
{
  struct personality* pers = cdev_to_personality(cdev);
  if (!pers) {
    return -EINVAL;
  }
  if (0 != next && !is_personality_supported(next)) {
    return -ENOTSUPP;
  }
  if (pers->personality != next) {
    pers->next_personality = next;
    queue_work(pers->restart_worker, &pers->restart);
  }
  return 0;
}
static ssize_t set_state(struct device* dev, struct device_attribute* att, const char* buf,
                         size_t size)
{
  struct usb_composite_dev* cdev = dev ? dev_get_drvdata(dev) : 0;
  int next = 0;
  int scanned_fields = 0;
  int result;
  dev_dbg(dev, "set_state: %s", buf);
  if (!cdev || !size || buf[size-1] != '\n') {
    return -EINVAL;
  }
  scanned_fields = sscanf(buf, "{ \"personality\" : %d }", &next);
  if (scanned_fields != 1) {
    return -EFAULT;
  }
  if (0 != next && !is_personality_supported(next)) {
    return -ENOTSUPP;
  }
  result = set_next_personality(cdev, next);
  return result < 0 ? result : size;
}
static ssize_t get_personalities(struct device* dev, struct device_attribute* attr, char* buf)
{
  int pos = 0;
  struct factory * iter = 0;
  for (iter = personalities; iter->value && iter->name; ++iter)
  {
    char delim = pos ? ',' : '{';
    pos += scnprintf(buf+pos, PAGE_SIZE-pos, "%c \"%s\" : %d", delim, iter->name, iter->value);
  }
  pos = pos + scnprintf(buf+pos, PAGE_SIZE-pos, "%c}\n", pos ? ' ' : '{');
  return pos; /* fs/sysfs/file.c fill_read_buffer() checks result range */
}

static struct device_attribute state_attribute = {
  { "state", .mode = 0664 },
  &get_state, &set_state
};
static struct device_attribute personalities_attribute = {
  { "personalities", .mode = 0444 },
  &get_personalities
};

static int vbus_session_sub(struct usb_gadget* gadget, int is_active)
{
  struct personality* pers = cdev_to_personality(get_gadget_data(gadget));
  if (pers) {
    const struct usb_gadget_ops *ops = pers->gadget_ops_super;
    const bool new_attached = is_active;
    /* DBG(pers, "vbus_session_sub(%p, %d)\n", gadget, is_active); */
    if (new_attached != pers->attached) {
      struct sysfs_dirent *state_sd = pers->state_sd;
      pers->attached = new_attached;
      if (state_sd) { sysfs_notify_dirent(state_sd); }
    }
    if (ops) {
      if (ops->vbus_session) {
        return (*ops->vbus_session)(gadget, is_active);
      } else {
        return -EOPNOTSUPP;
      }
    }
  }
  return -EFAULT;
}
static int setup_sub(struct usb_gadget* gadget, const struct usb_ctrlrequest* request)
{
  struct personality* pers = cdev_to_personality(get_gadget_data(gadget));
  if (pers && pers->setup_super) {
    if (!pers->configured) {
      struct sysfs_dirent *state_sd = pers->state_sd;
      pers->attached = true;
      pers->configured = true;
      if (state_sd) { sysfs_notify_dirent(state_sd); }
    }
    return (*pers->setup_super)(gadget, request);

    /* if (pers->personality) { */
    /*   return (*pers->setup_super)(gadget, request); */
    /* } else { */
    /*   /\* Turn pullup off.  This should trigger a disconnect_sub call, */
    /*      which will inform observers about the changed state. *\/ */
    /*   usb_gadget_disconnect(gadget);  */
    /*   return 0; */
    /* } */
  } else {
    return -EFAULT;
  }
}
static void disconnect_sub(struct usb_gadget* gadget)
{
  struct personality* pers = cdev_to_personality(get_gadget_data(gadget));
  DBG(pers, "disconnect_sub\n");
  if (pers) {
    if (pers->configured) {
      struct sysfs_dirent *state_sd = pers->state_sd;
      DBG(pers, "disconnect_sub will notfiy unconfigured\n");
      pers->configured = false;
      if (state_sd) { sysfs_notify_dirent(state_sd); }
    }
    if (pers->disconnect_super) {
      (*pers->disconnect_super)(gadget);
    }
  }
}

static void restart_with_next_personality(struct work_struct *w)
{
  int error = 0;
  struct personality *pers = container_of(w, struct personality, restart);
  struct usb_device_descriptor *dev = (struct usb_device_descriptor*) pers->driver.dev;
  DBG(pers, "restart_with_next_personality\n");
  down_write(&pers->next_personality_lock);
  usb_composite_unregister(&pers->driver);
  DBG(pers, "restart_with_next_personality: unregister returned\n");
  pers->personality = pers->next_personality;
  dev->idProduct = pers->personality;
  error = usb_gadget_disconnect(pers->gadget); /* pullup(0) */
  if (error) { WARNING(pers, "restart_with_next_personality: disconnect failed\n"); }
  /* msleep(10); */
  error = usb_composite_register(&pers->driver);
  DBG(pers, "restart_with_next_personality: register returned\n");
  if (error) {
    printk(KERN_WARNING "restart_with_next_personality: composite_register failed\n");
  }
  if (pers->personality) {
    error = usb_gadget_connect(pers->gadget); /* pullup(1) */
  } else {
    error = usb_gadget_disconnect(pers->gadget);  /* No personality selected: pullup(0) */
  }
  if (error) { WARNING(pers, "restart_with_next_personality: ..connect failed\n"); }
  up_write(&pers->next_personality_lock);
}
static void override_vbus_setup_and_disconnect(struct usb_composite_dev* cdev)
{
  struct personality* pers = container_of(cdev->driver, struct personality, driver);
  struct device_driver* ddriver = cdev->gadget->dev.driver;
  struct usb_gadget_driver* gdriver = container_of(ddriver, struct usb_gadget_driver, driver);
  pers->setup_super = gdriver->setup;
  gdriver->setup = setup_sub;
  pers->disconnect_super = gdriver->disconnect;
  gdriver->disconnect = disconnect_sub;
  /* We don't edit ops since it's const.  Editing a copy seems safer. */
  pers->gadget_ops_super = cdev->gadget->ops;
  memcpy(&pers->gadget_ops_sub, cdev->gadget->ops, sizeof(pers->gadget_ops_sub));
  pers->gadget_ops_sub.vbus_session = vbus_session_sub;
  cdev->gadget->ops = &pers->gadget_ops_sub;
}
static void restore_vbus_setup_and_disconnect(struct usb_composite_dev* cdev)
{
  struct personality* pers = container_of(cdev->driver, struct personality, driver);
  struct device_driver* ddriver = cdev->gadget->dev.driver;
  struct usb_gadget_driver* gdriver = container_of(ddriver, struct usb_gadget_driver, driver);
  DBG(pers, "%s\n", __func__);
  gdriver->setup = pers->setup_super;
  pers->setup_super = NULL;
  gdriver->disconnect = pers->disconnect_super;
  pers->disconnect_super = NULL;
  cdev->gadget->ops = pers->gadget_ops_super;
  pers->gadget_ops_super = NULL;
}
static void create_additional_driver_attributes(struct device* dev) {
  int error = 0;
  dev_dbg(dev, "create_additional_driver_attributes\n");
  error = device_create_file(dev, &state_attribute);
  if (!error) {
    error = device_create_file(dev, &personalities_attribute);
  } else {
    dev_warn(dev, "device_create_file failed (0x%x)", error);
  }
}
static void remove_additional_driver_attributes(struct device* dev) {
  dev_dbg(dev, "remove_additional_driver_attributes\n");
  device_remove_file(dev, &personalities_attribute);
  dev_dbg(dev, "remove_additional_driver_attributes: did remove 'personalities'\n");
  device_remove_file(dev, &state_attribute);
  dev_dbg(dev, "remove_additional_driver_attributes: did remove 'state'\n");
}
static void add_personality_configurations(struct usb_composite_dev* cdev) {
  struct personality* pers = container_of(cdev->driver, struct personality, driver);
  struct factory* fact = personalities;
  DBG(cdev, "add_personality_configurations\n");
  while (fact->value && fact->value != pers->personality) {
    ++fact;
  }
  if (fact->value && fact->add_configurations) {
    fact->add_configurations(cdev, fact->name);
  }
}

int sysfs_sd_setattr(struct sysfs_dirent *sd, struct iattr * iattr);
static int personality_bind(struct usb_composite_dev* cdev)
{
  struct personality* pers = cdev_to_personality(cdev);
  if (pers && cdev->gadget) {
    DBG(cdev, "personality_bind\n");
    pers->gadget = cdev->gadget;
    pers->attached = false;
    pers->configured = false;
    override_vbus_setup_and_disconnect(cdev);
    create_additional_driver_attributes(&cdev->gadget->dev);
    pers->state_sd = sysfs_get_dirent(cdev->gadget->dev.kobj.sd, NULL, state_attribute.attr.name);
    {
      struct iattr attr = { .ia_valid = ATTR_UID|ATTR_GID,
                            .ia_uid = AID_SYSTEM, .ia_gid = AID_SYSTEM };
      int result = sysfs_sd_setattr(pers->state_sd, &attr);
      if (result) {
        WARN(cdev, "%s: sysfs_sd_setattr failed (%d)\n", __FUNCTION__, result);
      }
    }
    DBG(cdev, "personality_bind: did get sysfs dirent\n");
    add_personality_configurations(cdev);
    return 0;
  } else {
    return -EINVAL;
  }
}
static int personality_unbind(struct usb_composite_dev* cdev)
{
  struct personality* pers = cdev_to_personality(cdev);
  if (pers && cdev->gadget) {
    DBG(cdev, "personality_unbind\n");
    if (pers->state_sd) {
      /* Poll doesn't return with an error when the sysfs attribute
         disappears.  Until this is examined and fixed we call
         sysfs_notify_dirent() and try to destroy everything before the user can
         reopen the attribute. */
      DBG(cdev, "personality_unbind: notify\n");
      sysfs_notify_dirent(pers->state_sd);
      DBG(cdev, "personality_unbind: will put sysfs dirent\n");
      sysfs_put(pers->state_sd);
      pers->state_sd = NULL;
      DBG(cdev, "personality_unbind: did put sysfs dirent\n");
    }
    remove_additional_driver_attributes(&cdev->gadget->dev);
    if (cdev->driver) {
      restore_vbus_setup_and_disconnect(cdev);
    }
  }
  return 0;
}

static void personality_enable_function(struct usb_function *f, int enable)
{
	int disable = !enable;
	printk(KERN_DEBUG "%s : f=\"%s\", f->disabled=%d, enable=%d\n", __func__,
			f ? f->name : "null", !!f->disabled, enable);
	if (!!f->disabled != disable) {
		usb_function_set_enabled(f, !disable);
		usb_composite_force_reset(f->config->cdev);
	}
}

static struct personality* instance(void)
{
  static struct usb_string strings[] = {
    { .id = USB_PERSONALITY_MANUFACTURER,         .s = "LGE" },
    { .id = USB_PERSONALITY_PRODUCT,              .s = "Android Phone" },
    { .id = USB_PERSONALITY_SERIAL_NUMBER,        .s = "0123456789ABCDEF" /* changed below */ },
    { .id = USB_PERSONALITY_OS_STRING_DESCRIPTOR, .s = "MSFT100\xc3\xbe" },
    { .id = USB_PERSONALITY_INTERFACE_ADB,        .s = "Android Debug Bridge" },
    { .id = USB_PERSONALITY_INTERFACE_CDC,        .s = "Communications Device" },
    { .id = USB_PERSONALITY_INTERFACE_DPS,        .s = "PictBridge" },
    { .id = USB_PERSONALITY_INTERFACE_MTP,        .s = "Media Transfer" },
    {}
  };
  static struct usb_gadget_strings strings_en_US = { .language = 0x0409, .strings = strings };
  static struct usb_gadget_strings strings_zero  = { .language = 0, .strings = strings };
  static struct usb_gadget_strings *gadget_strings[] = { &strings_en_US, &strings_zero, 0 };
  static struct usb_device_descriptor device_desc = {
    .bLength              = sizeof(device_desc),
    .bDescriptorType      = USB_DT_DEVICE,
    .bcdUSB               = __constant_cpu_to_le16(0x0200),
    .idVendor             = __constant_cpu_to_le16(0x1004),
    .idProduct            = __constant_cpu_to_le16(0x0000),
    .bcdDevice            = __constant_cpu_to_le16(0x0100),
    .iManufacturer        = USB_PERSONALITY_MANUFACTURER,
    .iProduct             = USB_PERSONALITY_PRODUCT,
    .iSerialNumber        = USB_PERSONALITY_SERIAL_NUMBER,
    .bNumConfigurations   = 1
  };
  static struct personality result = {
    .driver = { .name    = "personality",
                .dev     = &device_desc,
                .strings = gadget_strings,
                .bind    = personality_bind,
                .unbind  = personality_unbind,
                .enable_function = personality_enable_function },

    /* NOTE: When built-in, the default personality is set at the boot-time, before init is started.
     * So, when the personality rely on any init action on the userspace it will not be performed!
     * Solution: Leave the default personality unset here (zero = usb disabled) and set it from init,
     * after init startet (e.g. from init.lge.rc), then also other init rules will be performed and
     * the user space deamons will be started (or images will be mounted) as stated in other init
     * rules. */

    /* .personality = 0x0001, /\* default personality *\/ */
  };
  static bool initialized = false;
  if (!initialized) {
    const char* serial_number = personality_get_serial_number();
    if (serial_number) {
      int i;
      for (i = 0; i < (sizeof(strings)/sizeof(strings[0])); ++i) {
        if (USB_PERSONALITY_SERIAL_NUMBER == strings[i].id) {
          strings[i].s = serial_number;
          break;
        }
      }
    }
    init_rwsem(&result.next_personality_lock);
    initialized = true;
  }
  return &result;
}

static int __init init(void)
{
  struct personality* pers = instance();
  int error;
  printk(KERN_INFO "personality init\n");
  pers->restart_worker = create_singlethread_workqueue("gpers-work");
  if (!pers->restart_worker) {
    return -ENOMEM;
  }
  INIT_WORK(&pers->restart, restart_with_next_personality);
  pers->kset = kset_create_and_add("personality", /* ops */ 0, /* parent */ 0);
  error = usb_composite_register(&pers->driver);
  if (pers->gadget && !pers->personality) {
    usb_gadget_disconnect(pers->gadget); /* No personality selected: pullup(0) */
  }
  return error;
}
module_init(init);
static void __exit cleanup(void)
{
  struct personality* pers = instance();
  printk(KERN_INFO "personality cleanup\n");
  usb_composite_unregister(&pers->driver);
  kset_unregister(pers->kset);
  pers->kset = 0;
  if (pers->restart_worker) {
    flush_workqueue(pers->restart_worker);
    destroy_workqueue(pers->restart_worker);
  }
}
module_exit(cleanup);

struct kset * personality_cdev_to_kset(struct usb_composite_dev* cdev)
{
  struct personality * pers = cdev_to_personality(cdev);
  return pers ? pers->kset : 0;
}
struct usb_device_descriptor* personality_cdev_to_device_desc(struct usb_composite_dev* cdev)
{
  struct personality * pers = cdev_to_personality(cdev);
  return pers ? ((struct usb_device_descriptor*)pers->driver.dev) : 0;
}
int personality_usb_add_config(struct usb_composite_dev* cdev, const char* name,
                               struct usb_configuration* config)
{
  config->bConfigurationValue = 1;
  config->bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
  config->label = name;
  return usb_add_config(cdev, config);
}

MODULE_AUTHOR("Paul Kunysch <paul.kunysch@emsys.de>");
MODULE_DESCRIPTION("Gadget driver with support for multiple personalities");
MODULE_LICENSE("GPL v2");
