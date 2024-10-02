#ifndef BtStackUtils_h
#define BtStackUtils_h

#include "btstack.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"

static void chipset_set_bd_addr_command(bd_addr_t addr,
                                        uint8_t *hci_cmd_buffer) {
  hci_cmd_buffer[0] = 0x01;
  hci_cmd_buffer[1] = 0xfc;
  hci_cmd_buffer[2] = 0x06;
  reverse_bd_addr(addr, &hci_cmd_buffer[3]);
}

static const btstack_chipset_t btstack_chipset_cyw43 = {
    .name = "CYW43",
    .init = NULL,
    .next_command = NULL,
    .set_baudrate_command = NULL,
    .set_bd_addr_command = chipset_set_bd_addr_command,
};

const btstack_chipset_t *btstack_chipset_cyw43_instance(void) {
  return &btstack_chipset_cyw43;
}

void create_sdp_hid_record(uint8_t *service, const hid_sdp_record_t *params) {
  uint8_t *attribute;
  de_create_sequence(service);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_SERVICE_RECORD_HANDLE);
  de_add_number(service, DE_UINT, DE_SIZE_32, 0x10000);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_SERVICE_CLASS_ID_LIST);
  attribute = de_push_sequence(service);
  {
    de_add_number(attribute, DE_UUID, DE_SIZE_16,
                  BLUETOOTH_SERVICE_CLASS_HUMAN_INTERFACE_DEVICE_SERVICE);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *l2cpProtocol = de_push_sequence(attribute);
    {
      de_add_number(l2cpProtocol, DE_UUID, DE_SIZE_16,
                    BLUETOOTH_PROTOCOL_L2CAP);
      de_add_number(l2cpProtocol, DE_UINT, DE_SIZE_16,
                    BLUETOOTH_PSM_HID_CONTROL);
    }
    de_pop_sequence(attribute, l2cpProtocol);

    uint8_t *hidProtocol = de_push_sequence(attribute);
    {
      de_add_number(hidProtocol, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_HIDP);
    }
    de_pop_sequence(attribute, hidProtocol);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_BROWSE_GROUP_LIST);
  attribute = de_push_sequence(service);
  {
    de_add_number(attribute, DE_UUID, DE_SIZE_16,
                  BLUETOOTH_ATTRIBUTE_PUBLIC_BROWSE_ROOT);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_LANGUAGE_BASE_ATTRIBUTE_ID_LIST);
  attribute = de_push_sequence(service);
  {
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x656e);
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x006a);
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x0100);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_BLUETOOTH_PROFILE_DESCRIPTOR_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *hidProfile = de_push_sequence(attribute);
    {
      de_add_number(hidProfile, DE_UUID, DE_SIZE_16,
                    BLUETOOTH_SERVICE_CLASS_HUMAN_INTERFACE_DEVICE_SERVICE);
      de_add_number(hidProfile, DE_UINT, DE_SIZE_16, 0x0101);  // Version 1.1
    }
    de_pop_sequence(attribute, hidProfile);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS);
  attribute = de_push_sequence(service);
  {
    uint8_t *additionalDescriptorAttribute = de_push_sequence(attribute);
    {
      uint8_t *l2cpProtocol = de_push_sequence(additionalDescriptorAttribute);
      {
        de_add_number(l2cpProtocol, DE_UUID, DE_SIZE_16,
                      BLUETOOTH_PROTOCOL_L2CAP);
        de_add_number(l2cpProtocol, DE_UINT, DE_SIZE_16,
                      BLUETOOTH_PSM_HID_INTERRUPT);
      }
      de_pop_sequence(additionalDescriptorAttribute, l2cpProtocol);

      uint8_t *hidProtocol = de_push_sequence(additionalDescriptorAttribute);
      {
        de_add_number(hidProtocol, DE_UUID, DE_SIZE_16,
                      BLUETOOTH_PROTOCOL_HIDP);
      }
      de_pop_sequence(additionalDescriptorAttribute, hidProtocol);
    }
    de_pop_sequence(attribute, additionalDescriptorAttribute);
  }
  de_pop_sequence(service, attribute);

  // 0x0100 "ServiceName"
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0100);
  de_add_data(service, DE_STRING, (uint16_t)strlen(params->device_name),
              (uint8_t *)params->device_name);

  // 0x0101 "ServiceDescription"
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0101);
  de_add_data(service, DE_STRING, (uint16_t)strlen("Gamepad"),
              (uint8_t *)"Gamepad");

  // 0x0102 "ProviderName"
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0102);
  de_add_data(service, DE_STRING, (uint16_t)strlen("Nintendo"),
              (uint8_t *)"Nintendo");

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_PARSER_VERSION);
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0111);  // v1.1.1

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_DEVICE_SUBCLASS);
  de_add_number(service, DE_UINT, DE_SIZE_8, params->hid_device_subclass);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_COUNTRY_CODE);
  de_add_number(service, DE_UINT, DE_SIZE_8, params->hid_country_code);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_VIRTUAL_CABLE);
  de_add_number(service, DE_BOOL, DE_SIZE_8, params->hid_virtual_cable);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_RECONNECT_INITIATE);
  de_add_number(service, DE_BOOL, DE_SIZE_8, params->hid_reconnect_initiate);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_DESCRIPTOR_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *hidDescriptor = de_push_sequence(attribute);
    {
      de_add_number(hidDescriptor, DE_UINT, DE_SIZE_8,
                    0x22);  // Report Descriptor
      de_add_data(hidDescriptor, DE_STRING, params->hid_descriptor_size,
                  (uint8_t *)params->hid_descriptor);
    }
    de_pop_sequence(attribute, hidDescriptor);
  }
  de_pop_sequence(service, attribute);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HIDLANGID_BASE_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *hig_lang_base = de_push_sequence(attribute);
    {
      // see: http://www.usb.org/developers/docs/USB_LANGIDs.pdf
      de_add_number(hig_lang_base, DE_UINT, DE_SIZE_16,
                    0x0409);  // HIDLANGID = English (US)
      de_add_number(hig_lang_base, DE_UINT, DE_SIZE_16,
                    0x0100);  // HIDLanguageBase = 0x0100 default
    }
    de_pop_sequence(attribute, hig_lang_base);
  }
  de_pop_sequence(service, attribute);

  // battery power
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_BATTERY_POWER);
  de_add_number(service, DE_BOOL, DE_SIZE_8, 1);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_REMOTE_WAKE);
  de_add_number(service, DE_BOOL, DE_SIZE_8, params->hid_remote_wake ? 1 : 0);

  // supervision timeout
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_SUPERVISION_TIMEOUT);
  de_add_number(service, DE_UINT, DE_SIZE_16, params->hid_supervision_timeout);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_NORMALLY_CONNECTABLE);
  de_add_number(service, DE_BOOL, DE_SIZE_8, params->hid_normally_connectable);

  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_HID_BOOT_DEVICE);
  de_add_number(service, DE_BOOL, DE_SIZE_8, params->hid_boot_device ? 1 : 0);
}

void create_sdp_pnp_record(uint8_t *service, uint16_t vendor_id_source,
                           uint16_t vendor_id, uint16_t product_id,
                           uint16_t version) {
  uint8_t *attribute;
  de_create_sequence(service);

  // 0x0000 "Service Record Handle"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_SERVICE_RECORD_HANDLE);
  de_add_number(service, DE_UINT, DE_SIZE_32, 0x10001);

  // 0x0001 "Service Class ID List"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_SERVICE_CLASS_ID_LIST);
  attribute = de_push_sequence(service);
  {
    de_add_number(attribute, DE_UUID, DE_SIZE_16,
                  BLUETOOTH_SERVICE_CLASS_PNP_INFORMATION);
  }
  de_pop_sequence(service, attribute);

  // 0x0004 "Protocol Descriptor List"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *l2cpProtocol = de_push_sequence(attribute);
    {
      de_add_number(l2cpProtocol, DE_UUID, DE_SIZE_16,
                    BLUETOOTH_PROTOCOL_L2CAP);
      de_add_number(l2cpProtocol, DE_UINT, DE_SIZE_16, BLUETOOTH_PSM_SDP);
    }
    de_pop_sequence(attribute, l2cpProtocol);

    uint8_t *sdpProtocol = de_push_sequence(attribute);
    { de_add_number(sdpProtocol, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_SDP); }
    de_pop_sequence(attribute, sdpProtocol);
  }
  de_pop_sequence(service, attribute);

  // 0x0006 "Language Base Attribute ID List"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_LANGUAGE_BASE_ATTRIBUTE_ID_LIST);
  attribute = de_push_sequence(service);
  {
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x656e);
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x006a);
    de_add_number(attribute, DE_UINT, DE_SIZE_16, 0x0100);
  }
  de_pop_sequence(service, attribute);

  // 0x0009 "Bluetooth Profile Descriptor List"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_BLUETOOTH_PROFILE_DESCRIPTOR_LIST);
  attribute = de_push_sequence(service);
  {
    uint8_t *pnpProfile = de_push_sequence(attribute);
    {
      de_add_number(pnpProfile, DE_UUID, DE_SIZE_16,
                    BLUETOOTH_SERVICE_CLASS_PNP_INFORMATION);
      de_add_number(pnpProfile, DE_UINT, DE_SIZE_16, 0x0100);  // Version 1.1
    }
    de_pop_sequence(attribute, pnpProfile);
  }
  de_pop_sequence(service, attribute);

  // 0x0100 "ServiceName"
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0100);
  de_add_data(service, DE_STRING,
              (uint16_t)strlen("Wireless Gamepad PnP Server"),
              (uint8_t *)"Wireless Gamepad PnP Server");

  // 0x0101 "ServiceDescription"
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0101);
  de_add_data(service, DE_STRING, (uint16_t)strlen("Gamepad"),
              (uint8_t *)"Gamepad");

  // 0x0200 "SpecificationID"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_SPECIFICATION_ID);
  de_add_number(service, DE_UINT, DE_SIZE_16, 0x0103);  // v1.03

  // 0x0201 "VendorID"
  de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_VENDOR_ID);
  de_add_number(service, DE_UINT, DE_SIZE_16, vendor_id);

  // 0x0202 "ProductID"
  de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_PRODUCT_ID);
  de_add_number(service, DE_UINT, DE_SIZE_16, product_id);

  // 0x0203 "Version"
  de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_VERSION);
  de_add_number(service, DE_UINT, DE_SIZE_16, version);

  // 0x0204 "PrimaryRecord"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_PRIMARY_RECORD);
  de_add_number(service, DE_BOOL, DE_SIZE_8,
                1);  // yes, this is the primary record - there are no others

  // 0x0205 "VendorIDSource"
  de_add_number(service, DE_UINT, DE_SIZE_16,
                BLUETOOTH_ATTRIBUTE_VENDOR_ID_SOURCE);
  de_add_number(service, DE_UINT, DE_SIZE_16, vendor_id_source);
}

#endif