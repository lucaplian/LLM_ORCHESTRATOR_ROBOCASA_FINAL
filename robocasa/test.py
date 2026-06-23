import hid
for device in hid.enumerate():
    if "Sony" in device['manufacturer_string'] or "Wireless Controller" in device['product_string']:
        print(f"Device: {device['product_string']}")
        print(f"Vendor ID: {hex(device['vendor_id'])}")
        print(f"Product ID: {hex(device['product_id'])}\n")
