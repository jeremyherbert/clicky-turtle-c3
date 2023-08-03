import hid  # from the package "hidapi"
import time

# NOTE: This code assumes that your device is already paired with the computer running the script

for device in hid.enumerate():
    # replace the following line with your own code to detect your device
    if device['product_string'].startswith("clicky-c3-") and device['usage_page'] == 0xFF00:
        print("found device", device)
        h = hid.device()
        h.open_path(device['path'])
        h.set_nonblocking(1)  # this means that the read operation will return immediately even if there is no data

        # 32 bytes to send over bluetooth (downlink)
        data_to_send = [0x02] * 32

        # the first byte must always be 0x03, which is the HID report ID
        h.write([0x03] + data_to_send)
        time.sleep(1)

        # read any uplinks
        while 1:
            uplink = h.read(64)  # always read a size of 64 due to the issue below
            if uplink:
                # depending on what operating system you use, the first byte of the uplink could be 0x03
                # yes, even different versions of Windows behave differently!
                # if you do find that it is present, you can ignore it as it does not form part of your payload
                print(uplink)
            time.sleep(0.5)