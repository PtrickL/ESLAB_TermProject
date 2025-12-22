import asyncio
import struct
from bleak import BleakScanner, BleakClient

# ---------- UUID ----------
SERVICE_UUID = "aabbccdd-eeff-1122-3344-556600000001"
CHAR1_UUID    = "aabbccdd-eeff-1122-3344-556600000002"  # door state
CHAR2_UUID    = "aabbccdd-eeff-1122-3344-556600000003"  # knock event
CHAR3_UUID    = "aabbccdd-eeff-1122-3344-556600000004"  # handle twist


# ======== Door State Mapping ========
DOOR_STATE_TEXT = {
    0: "門已關閉 (CLOSED)",
    1: "正在開門 (OPENING)",
    2: "門已開啟 (OPEN)",
    3: "正在關門 (CLOSING)",
    4: "敲門聲音 (KNOCK)",
    5: "交談聲音 (SPEECH)",
}


# ---------- Handlers ----------
def handler_char1(sender, data):
    # float -> door_state
    value = struct.unpack("<f", data)[0]
    state = int(value)
    if value <= 5:
        text = DOOR_STATE_TEXT.get(state, f"未知狀態 {state}")
        print(f" [CHAR1] Door State = {state} → {text}")


def handler_char2(sender, data):
    # int32 -> knock event
    value = struct.unpack("<i", data)[0]
    if value == 1:
        print(" [CHAR2] 敲門事件偵測到！")
    else:
        print(f"[CHAR2] 未知敲門資料: {value}")


def handler_char3(sender, data):
    # int32 -> twist event
    value = struct.unpack("<i", data)[0]
    if value == 1:
        print(" [CHAR3] 門把旋轉事件偵測到！")
    else:
        print(f" [CHAR3] 未知旋轉資料: {value}")


# ---------- BLE Scan ----------
async def scan_devices(timeout=6.0):
    print("Scanning...")
    devices = await BleakScanner.discover(timeout)

    for i, dev in enumerate(devices):
        print(f"{i}: {dev.address}  Name={dev.name}")

    return devices


# ---------- Main Program ----------
async def main():
    devices = await scan_devices()
    if not devices:
        return

    num = int(input("Select device number: "))
    addr = devices[num].address

    print(f"Connecting to {addr} ...")
    async with BleakClient(addr) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return

        print("Connected!")

        # Enable notifications
        await client.start_notify(CHAR1_UUID, handler_char1)
        await client.start_notify(CHAR2_UUID, handler_char2)
        await client.start_notify(CHAR3_UUID, handler_char3)

        print("Notifications enabled. Listening...\n(Press Ctrl+C to exit)")

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Stopping notifications...")

        await client.stop_notify(CHAR1_UUID)
        await client.stop_notify(CHAR2_UUID)
        await client.stop_notify(CHAR3_UUID)


if __name__ == "__main__":
    asyncio.run(main())
