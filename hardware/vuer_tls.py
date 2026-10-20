"""[ADDED 2026-10-14] Validate Vuer's local TLS files before a requested start.

No ROS, network access, certificate generation, or trust-store modification.
This checks the local certificate/key and PC IP, NOT whether Quest trusts the CA.
"""
import argparse
import ipaddress
import os
from pathlib import Path
import ssl
import subprocess
import time


def validate_vuer_tls(cert, key, server_ip):
    """Return the HTTPS URL after local key, IP, and validity checks succeed."""
    try:
        address = ipaddress.IPv4Address(server_ip)
    except ipaddress.AddressValueError as error:
        raise ValueError('--server-ip must be the PC IPv4 address, without a port or URL') from error
    if address.is_unspecified or address.is_multicast or address.is_loopback:
        raise ValueError('--server-ip must be a LAN address reachable from Quest, not a bind/loopback address')
    cert, key = Path(cert), Path(key)
    if not cert.is_file() or not key.is_file():
        raise ValueError('Vuer cert.pem or key.pem is missing; generate them for the PC IP')
    # [CHANGED] OpenSSL checks the IP SAN, not a possibly misleading common name.
    try:
        checked = subprocess.run(
            ['openssl', 'x509', '-in', str(cert), '-noout', '-checkip', str(address),
             '-startdate', '-enddate'],
            capture_output=True, text=True, timeout=5, check=False,
            env=dict(os.environ, LC_ALL='C'))
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError('Unable to inspect the Vuer certificate with openssl') from error
    # [FIX] Installed OpenSSL x509 returns 0 even for "does NOT match".
    # Require the explicit positive result, not only successful command parsing.
    matches_ip = f'IP {address} does match certificate' in checked.stdout.splitlines()
    if checked.returncode != 0 or not matches_ip:
        raise ValueError(f'Certificate is invalid or does not cover PC IP {address}; regenerate for this PC, not the robot IP')
    dates = dict(line.split('=', 1) for line in checked.stdout.splitlines()
                 if line.startswith(('notBefore=', 'notAfter=')))
    try:
        begin = ssl.cert_time_to_seconds(dates['notBefore'])
        end = ssl.cert_time_to_seconds(dates['notAfter'])
    except (KeyError, ValueError) as error:
        raise ValueError('Certificate validity dates could not be read') from error
    now = time.time()
    if not begin <= now < end:
        raise ValueError('Certificate is expired or not yet valid; check the PC clock and certificate dates')
    # Loading proves that the key matches, without printing/exporting the key.
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(cert), str(key), password=lambda: '')
    except (OSError, ssl.SSLError) as error:
        raise ValueError('Vuer certificate/key pair is unreadable, encrypted, or mismatched') from error
    return f'https://{address}:8012?ws=wss://{address}:8012'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cert', required=True, type=Path)
    parser.add_argument('--key', required=True, type=Path)
    parser.add_argument('--server-ip', required=True)
    args = parser.parse_args()
    try:
        url = validate_vuer_tls(args.cert, args.key, args.server_ip)
    except ValueError as error:
        parser.exit(1, '[TLS ERROR] ' + str(error) + '\n')
    print('[TLS PASS] Local certificate/key, PC IP SAN and dates checked. Quest CA trust is separate.')
    print('[QUEST URL] ' + url)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
