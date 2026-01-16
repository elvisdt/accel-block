import argparse
import asyncio
import inspect
from bleak import BleakClient, BleakScanner


CFG_SERVICE_UUID = "4b8c8c7e-5506-4c22-8c17-3dd808be9e22"
DAC_CFG_UUID = "5e6d12bf-7d9c-4b1a-9a2b-8d9f8ab67f3e"

# Device names we accept (lowercase)
TARGET_DEVICE_NAMES = {"esp32", "mkey"}

# Tunables
SCAN_TIMEOUT_S = 5
SCAN_RETRIES = 3
CONNECT_TIMEOUT_S = 10


async def discover_target(retries: int, scan_timeout: float):
    print("Buscando target (esp32/MKEY)...")
    target = None

    for attempt in range(1, retries + 1):
        devices = await BleakScanner.discover(timeout=scan_timeout)
        for device in devices:
            name = (device.name or "").lower()
            if name in TARGET_DEVICE_NAMES:
                target = device
                break

        if target:
            print(f"Found target: {target.name} @ {target.address}")
            return target

        if attempt < retries:
            backoff = attempt
            print(f"No target yet. Retrying in {backoff}s... ({attempt}/{retries})")
            await asyncio.sleep(backoff)

    raise RuntimeError("No se encontro un dispositivo objetivo. Verifica que este anunciando con nombre.")


async def choose_device(scan_timeout: float):
    print(f"Escaneando dispositivos por {scan_timeout}s...")
    devices_raw = await BleakScanner.discover(timeout=scan_timeout)
    devices = [d for d in devices_raw if d.name]
    if not devices:
        raise RuntimeError("No se encontraron dispositivos con nombre.")

    for idx, device in enumerate(devices, start=1):
        print(f"[{idx}] {device.name} | {device.address}")

    choice = input("Selecciona dispositivo (o Enter para cancelar): ").strip()
    if not choice:
        raise RuntimeError("Seleccion cancelada.")
    try:
        idx = int(choice)
    except ValueError:
        raise RuntimeError("Seleccion invalida. Debe ser un numero.")
    if idx < 1 or idx > len(devices):
        raise RuntimeError("Seleccion fuera de rango.")

    selected = devices[idx - 1]
    print(f"Seleccionado: {selected.name} @ {selected.address}")
    return selected


async def get_services_safe(client: BleakClient):
    getter = getattr(client, "get_services", None)
    if getter:
        try:
            services = await getter() if inspect.iscoroutinefunction(getter) else getter()
            if services:
                return services
        except Exception:
            pass
    services = getattr(client, "services", None)
    if services:
        return services
    raise RuntimeError("No se pudieron obtener servicios (version de Bleak).")


def pack_dac_payload(u1_v: float, u2_v: float) -> bytes:
    u1_mv = int(round(u1_v * 1000.0))
    u2_mv = int(round(u2_v * 1000.0))
    if u1_mv < 0 or u2_mv < 0:
        raise ValueError("Los voltajes no pueden ser negativos.")
    if u1_mv > 5000 or u2_mv > 5000:
        raise ValueError("Voltaje fuera de rango (0-5.0V).")
    return bytes([
        u1_mv & 0xFF, (u1_mv >> 8) & 0xFF,
        u2_mv & 0xFF, (u2_mv >> 8) & 0xFF,
    ])


def unpack_dac_payload(data: bytes) -> tuple[float, float]:
    if len(data) < 4:
        raise ValueError(f"Payload muy corto: {len(data)} bytes")
    u1_mv = data[0] | (data[1] << 8)
    u2_mv = data[2] | (data[3] << 8)
    return (u1_mv / 1000.0, u2_mv / 1000.0)


async def read_dac(client: BleakClient):
    data = await client.read_gatt_char(DAC_CFG_UUID)
    u1_v, u2_v = unpack_dac_payload(data)
    print(f"U1={u1_v:.3f} V | U2={u2_v:.3f} V | raw={data.hex()}")


async def write_dac(client: BleakClient, u1_v: float, u2_v: float):
    payload = pack_dac_payload(u1_v, u2_v)
    await client.write_gatt_char(DAC_CFG_UUID, payload, response=True)
    print(f"Enviado: U1={u1_v:.3f} V | U2={u2_v:.3f} V | raw={payload.hex()}")


async def run(args):
    if args.address:
        target = args.address
    elif args.auto:
        target = await discover_target(args.scan_retries, args.scan_timeout)
    else:
        target = await choose_device(args.scan_timeout)

    client = BleakClient(target, timeout=CONNECT_TIMEOUT_S)
    try:
        print("Conectando...")
        await client.connect(timeout=CONNECT_TIMEOUT_S)
        print(f"Conectado (MTU={client.mtu_size})")

        services = await get_services_safe(client)
        svc = services.get_service(CFG_SERVICE_UUID)
        if not svc or not svc.get_characteristic(DAC_CFG_UUID):
            raise RuntimeError("No se encontro el servicio/characteristica de config DAC.")

        if args.u1 is not None or args.u2 is not None:
            if args.u1 is None or args.u2 is None:
                raise ValueError("Debes enviar --u1 y --u2 juntos.")
            await write_dac(client, args.u1, args.u2)
            if args.verify:
                await read_dac(client)
        else:
            await read_dac(client)
    finally:
        await client.disconnect()
        print("Desconectado.")


def parse_args():
    parser = argparse.ArgumentParser(description="Config DAC via BLE (debug)")
    parser.add_argument("--u1", type=float, help="Voltaje U1 en volts (ej: 0.5)")
    parser.add_argument("--u2", type=float, help="Voltaje U2 en volts (ej: 1.25)")
    parser.add_argument("--verify", action="store_true", help="Leer de nuevo despues de escribir")
    parser.add_argument("--auto", action="store_true", help="Auto-seleccionar por nombre")
    parser.add_argument("--address", help="Direccion MAC (omite escaneo)")
    parser.add_argument("--scan-timeout", type=float, default=SCAN_TIMEOUT_S, help="Tiempo de escaneo (s)")
    parser.add_argument("--scan-retries", type=int, default=SCAN_RETRIES, help="Reintentos de escaneo")
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
