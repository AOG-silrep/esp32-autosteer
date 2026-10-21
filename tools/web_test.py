#!/usr/bin/env python3
"""Bench test of the web stack (ESPUI on ESPAsyncWebServer on AsyncTCP), for after a library update.

  web_test.py http [HOST]                         the static pages, /heap and the OTA page
  web_test.py stress [HOST] [--rounds N] [--close clean|fin|rst] [--port PORT]
                                                  WebSocket clients that load the UI and drop it
  web_test.py udp [--seconds S]                   what arrives on UDP 9999 (PGN 253 etc.)

HOST defaults to 192.168.5.77, where the steer module sits on the AOG hub network. The PC
must be on that network; on the module's own access point it is 192.168.1.1.

stress opens 4 clients per round. Each one loads the UI the way ESPUI's controls.js does
(acking every chunk with uiok, so the server keeps sending), holds 0.2 to 4 s, and then
closes with a WebSocket close ("clean"), a bare TCP FIN ("fin") or an RST ("rst"), picked
at random unless --close is given. Crashes show only on the debug serial, so pass the
module's USB serial port as --port: the run is recorded to web_test_<date>-<time>.log and
the panics, resets and AsyncTCP "client deleted by a receive callback" lines are counted.
That last one fires on every browser-initiated WebSocket close and shows the AOG-silrep
AsyncTCP fix is in.

udp binds port 9999, so AgOpenGPS can't run on the same PC at the time.

Not covered here, done with curl (both restart the module). The config file is named
after the hostname, ESP32-Steer by default (link: Configurations tab):
  curl -o steering.json "http://192.168.5.77/ESP32-Steer%20steering.json"
  curl -F "f=@steering.json" http://192.168.5.77/upload-config      (no reply, it restarts)
  curl -F "MD5=$(md5sum fw.bin | cut -d' ' -f1)" -F "firmware=@fw.bin;filename=firmware" \\
       http://192.168.5.77/update

stress needs the websockets package, and --port needs pyserial.
"""

import argparse
import asyncio
import json
import random
import socket
import struct
import threading
import time
import urllib.error
import urllib.request

DEFAULT_HOST = '192.168.5.77'

#ESPUI message types (ESPUI.h, controls.js)
UI_INITIAL_GUI = 200
UI_EXTEND_GUI = 210
UI_FRAGMENT = 98

SERIAL_MARKERS = (('panics', (b'Guru Meditation', b'abort() was called', b'stack overflow')),
                  ('resets', (b'rst:0x',)),
                  ('closes in a callback', (b'client deleted by a receive callback',)))


# ---------------------------------------------------------------- http

def get(host, path, timeout=5):
    req = urllib.request.Request('http://%s%s' % (host, path), headers={'Accept-Encoding': 'gzip'})
    t = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read()
        return r.status, r.headers.get('Content-Encoding'), body, time.time() - t


def heap(host):
    try:
        return get(host, '/heap')[2].decode(errors='replace').split()[0]
    except (OSError, IndexError) as e:
        return 'error: %s' % e


def cmd_http(args):
    for path in ('/', '/js/controls.js', '/js/zepto.min.js', '/css/style.css', '/heap',
                 '/diagnostics', '/diagnosticData', '/update', '/update/identity'):
        try:
            status, enc, body, dt = get(args.host, path)
            text = body[:60].decode(errors='replace') if path in ('/heap', '/update/identity') else ''
            print('%-18s %d %-5s %6d B %5.0f ms %s' % (path, status, enc or '', len(body), dt * 1000, text))
        except urllib.error.HTTPError as e:
            print('%-18s HTTP %d' % (path, e.code))
        except OSError as e:
            print('%-18s ERROR %s' % (path, e))


# ---------------------------------------------------------------- stress

class Client:
    """what one WebSocket client received"""

    def __init__(self):
        self.msgs = 0
        self.size = 0
        self.ids = set()#ESPUI keeps sending label updates as UI chunks, so count controls, not chunks
        self.loaded = 0#seconds until the last new control arrived


async def ack(ws, data, client):
    """answer the way controls.js does, so ESPUI sends the next chunk or fragment"""
    kind = data.get('type')
    if kind in (UI_INITIAL_GUI, UI_EXTEND_GUI):
        controls = data.get('controls', [])
        for c in controls:
            if c.get('type') == UI_FRAGMENT:
                await ack(ws, c, client)
            elif 'id' in c:
                client.ids.add(c['id'])
        start = data.get('startindex', 0) if kind == UI_EXTEND_GUI else 0
        if data.get('totalcontrols', 0) > start + len(controls) - 1:
            await ws.send('uiok:%d' % (start + len(controls) - 1))
    elif kind == UI_FRAGMENT:
        client.ids.add(data['control']['id'])
        offset = data['offset'] + data['length']
        if offset < data['total']:
            await ws.send('uifragmentok:0: ' + json.dumps({'id': data['control']['id'], 'offset': offset}) + ':')


def rst(ws):
    sock = ws.transport.get_extra_info('socket')
    if sock is not None:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
    ws.transport.abort()


async def run_client(host, close, hold, idle=None):
    """connect, load the UI for hold seconds (or until no new control came for idle seconds), then close"""
    import websockets
    client = Client()
    async with websockets.connect('ws://%s/ws' % host, open_timeout=5, close_timeout=2,
                                  max_size=None, ping_interval=None) as ws:
        start = time.time()
        end = start + hold
        last_new = start
        while time.time() < end and not (idle and time.time() - last_new > idle):
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=max(0.05, end - time.time()))
            except asyncio.TimeoutError:
                break
            client.msgs += 1
            client.size += len(msg)
            known = len(client.ids)
            try:
                await ack(ws, json.loads(msg), client)
            except ValueError:
                await ws.send('uiok:0')#what controls.js does with a message it can't parse
            if len(client.ids) > known:
                last_new = time.time()
                client.loaded = last_new - start
        if close == 'rst':
            rst(ws)
        elif close == 'fin':
            ws.transport.close()
        #'clean' leaves it to the context manager's close handshake
    return client


async def full_load(host, label):
    c = await run_client(host, 'clean', 30, idle=2)
    print('%s: %d controls in %.1f s, %d messages, %d B' % (label, len(c.ids), c.loaded, c.msgs, c.size))


class SerialLog(threading.Thread):
    """records the debug serial while the stress runs"""

    def __init__(self, port, path):
        import serial
        super().__init__(daemon=True)
        self.port = serial.Serial(port, 115200, timeout=0.2)#pyserial's open doesn't reset the ESP32
        self.path = path
        self.data = bytearray()
        self.running = True

    def run(self):
        with open(self.path, 'wb') as f:
            while self.running:
                b = self.port.read(4096)
                if b:
                    f.write(b)
                    f.flush()
                    self.data += b
        self.port.close()

    def stop(self):
        self.running = False
        self.join()


async def stress(args):
    print('heap before:', heap(args.host))
    await full_load(args.host, 'one client')
    errors = 0
    t0 = time.time()
    for r in range(args.rounds):
        closes = [args.close or random.choice(('clean', 'fin', 'rst')) for _ in range(4)]
        results = await asyncio.gather(*[run_client(args.host, c, random.uniform(0.2, 4.0)) for c in closes],
                                       return_exceptions=True)
        bad = [repr(x) for x in results if isinstance(x, Exception)]
        errors += len(bad)
        if bad or r % 10 == 0:
            print('round %3d %-26s heap %s %s' % (r, ' '.join(closes), heap(args.host), ' '.join(bad[:2])))
    print('%d rounds x 4 clients in %.0f s, %d client errors' % (args.rounds, time.time() - t0, errors))
    await asyncio.sleep(3)
    print('heap after:', heap(args.host))
    await full_load(args.host, 'one client after')


def cmd_stress(args):
    log = None
    if args.port:
        log = SerialLog(args.port, time.strftime('web_test_%Y%m%d-%H%M%S.log'))
        log.start()
    try:
        asyncio.run(stress(args))
    finally:
        if log:
            time.sleep(2)#let a panic dump finish
            log.stop()
            print('serial log:', log.path)
            for name, markers in SERIAL_MARKERS:
                print('  %-22s %d' % (name, sum(log.data.count(m) for m in markers)))


# ---------------------------------------------------------------- udp

def cmd_udp(args):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 9999))
    s.settimeout(0.5)
    end = time.time() + args.seconds
    counts = {}
    last = None
    while time.time() < end:
        try:
            data, addr = s.recvfrom(2048)
        except socket.timeout:
            continue
        #AOG packets: 0x80 0x81, source, PGN, length, data, CRC
        kind = 'PGN %d' % data[3] if data[:2] == b'\x80\x81' and len(data) > 3 else repr(data[:6])
        counts[(addr[0], kind)] = counts.get((addr[0], kind), 0) + 1
        if kind == 'PGN 253':
            last = data
    for (ip, kind), n in sorted(counts.items()):
        print('%-15s %-14s %5d packets %5.1f/s' % (ip, kind, n, n / args.seconds))
    if last:
        print('last PGN 253:', last.hex(' '))


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('http', help='the static pages, /heap and the OTA page')
    p.add_argument('host', nargs='?', default=DEFAULT_HOST)
    p.set_defaults(func=cmd_http)
    p = sub.add_parser('stress', help='WebSocket clients that load the UI and drop it')
    p.add_argument('host', nargs='?', default=DEFAULT_HOST)
    p.add_argument('--rounds', type=int, default=60)
    p.add_argument('--close', choices=('clean', 'fin', 'rst'), help='default: a random one per client')
    p.add_argument('--port', help='the module debug serial, to count panics and resets')
    p.set_defaults(func=cmd_stress)
    p = sub.add_parser('udp', help='what arrives on UDP 9999')
    p.add_argument('--seconds', type=float, default=10)
    p.set_defaults(func=cmd_udp)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
