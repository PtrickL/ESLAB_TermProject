import asyncio
from bleak import BleakScanner, BleakClient

# ---------- 你的 UUID ----------
SERVICE_UUID = "aabbccdd-eeff-1122-3344-556600000001"

CHAR1_UUID = "aabbccdd-eeff-1122-3344-556600000002"   # event A
CHAR2_UUID = "aabbccdd-eeff-1122-3344-556600000003"   # event B
CHAR3_UUID = "aabbccdd-eeff-1122-3344-556600000004"   # event C


# ---------- 各自的 Notification Handler ----------
def handler_char1(sender, data):
    print(f"[Event A] from {sender}: {data.hex()}")


def handler_char2(sender, data):
    print(f"[Event B] from {sender}: {data.hex()}")


def handler_char3(sender, data):
    print(f"[Event C] from {sender}: {data.hex()}")


# ---------- 掃描 ----------
async def scan_devices(timeout=6.0):
    print("Scanning...")
    devices = await BleakScanner.discover(timeout)

    if not devices:
        print("No BLE devices found.")
        return []

    for i, dev in enumerate(devices):
        # 嘗試讀取 RSSI（依序找不同來源）
        rssi = getattr(dev, "rssi", None)

        if rssi is None and hasattr(dev, "metadata"):
            rssi = dev.metadata.get("rssi", None)

        # Windows 上常在 dev.details 裡
        if rssi is None and hasattr(dev, "details"):
            try:
                # WinRT DeviceInformation.Pairing protected info 可能包含 RSSI
                rssi = dev.details.properties.get("System.Devices.Aep.SignalStrength")
            except:
                pass

        print(f"{i}: {dev.address}  RSSI={rssi}  Name={dev.name}")

    return devices


# ---------- 主程式 ----------
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
        print("Listing all services and characteristics...")
        for service in client.services:
            print(f"SERVICE: {service.uuid}")
            for char in service.characteristics:
                print(f"  CHAR: {char.uuid}  handle={char.handle}")

        # ---------- 啟用三個 notification ----------
        print("Subscribing to CHAR1...")
        await client.start_notify(CHAR1_UUID, handler_char1)

        print("Subscribing to CHAR2...")
        await client.start_notify(CHAR2_UUID, handler_char2)

        print("Subscribing to CHAR3...")
        await client.start_notify(CHAR3_UUID, handler_char3)

        print("All three notifications enabled!")
        print("Listening... (Ctrl+C to exit)")

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping notifications...")

        # ---------- 清理 ----------
        await client.stop_notify(CHAR1_UUID)
        await client.stop_notify(CHAR2_UUID)
        await client.stop_notify(CHAR3_UUID)

    print("Disconnected.")


if __name__ == "__main__":
    asyncio.run(main())
