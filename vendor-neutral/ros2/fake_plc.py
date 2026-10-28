#!/usr/bin/env python3
"""Fake PLC server for Phase 2. Always returns value=0, written=True."""
import socket, json, threading

def handle(c, a):
    buf = b''
    seq = 0
    try:
        while True:
            data = c.recv(4096)
            if not data:
                break
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                req = json.loads(line)
                seq += 1
                reply = {'seq': seq, 'ts': 0}
                if req.get('cmd') == 'sync':
                    reply['values'] = {t: 0 for t in req.get('read', [])}
                    reply['written'] = {t: True for t in req.get('write', {})}
                elif req.get('cmd') == 'read':
                    reply['values'] = {t: 0 for t in req.get('tags', [])}
                elif req.get('cmd') == 'write':
                    reply['written'] = {t: True for t in req.get('tags', {})}
                else:
                    reply['error'] = 'unknown'
                c.sendall((json.dumps(reply) + '\n').encode())
    except Exception:
        pass
    finally:
        c.close()

def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', 5000))
    srv.listen(5)
    print('Fake PLC listening on :5000', flush=True)
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle, args=(conn, addr), daemon=True).start()

if __name__ == '__main__':
    main()