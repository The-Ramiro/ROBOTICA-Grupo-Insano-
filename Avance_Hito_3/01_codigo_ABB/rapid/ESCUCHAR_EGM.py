"""Comprueba llegada de datagramas UDP. No envía paquetes al ABB."""
import argparse
from ipaddress import IPv4Address
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip-pc', type=IPv4Address, required=True)
    parser.add_argument('--puerto', type=int, required=True)
    parser.add_argument('--ip-abb', type=IPv4Address, required=True)
    parser.add_argument('--segundos', type=int, default=20)
    args = parser.parse_args()
    if not 1 <= args.puerto <= 65535:
        parser.error('puerto fuera de rango')
    if not 1 <= args.segundos <= 600:
        parser.error('la escucha debe durar de 1 a 600 segundos')
    packets = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind((str(args.ip_pc), args.puerto))
        print(f'Escuchando {args.ip_pc}:{args.puerto} durante {args.segundos} s', flush=True)
        end = time.monotonic() + args.segundos
        while (remaining := end - time.monotonic()) > 0:
            receiver.settimeout(remaining)
            try:
                data, (source, source_port) = receiver.recvfrom(65535)
            except socket.timeout:
                break
            if data and source == str(args.ip_abb):
                packets += 1
                if packets == 1:
                    print(f'Primer datagrama de {source}:{source_port}, {len(data)} bytes', flush=True)
    print(f'Datagramas de {args.ip_abb}: {packets}. Esta prueba verifica llegada UDP.')
    return 0 if packets else 1


if __name__ == '__main__':
    raise SystemExit(main())
