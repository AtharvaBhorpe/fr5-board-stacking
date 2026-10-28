"""Create a project-local development CA; never install it into system trust."""
import json
from pathlib import Path
import socket
import subprocess
p=Path(__file__).resolve().parent/'.local'; p.mkdir(exist_ok=True)
ips={'127.0.0.1'}; primary=None
try:
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.connect(('1.1.1.1',80)); primary=s.getsockname()[0]; ips.add(primary); s.close()
except OSError: pass
ips.update(subprocess.check_output(['hostname','-I'],text=True).split())
ips={v for v in ips if ':' not in v}
lan=primary or next((v for v in sorted(ips) if not v.startswith('127.')), '127.0.0.1')
def run(*args): subprocess.run(['openssl',*args],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
if not (p/'ca.key').exists():
 run('req','-x509','-newkey','rsa:2048','-nodes','-keyout',str(p/'ca.key'),'-out',str(p/'ca.crt'),'-days','365',
     '-subj','/CN=FR5 Workcell Local Development CA','-addext','basicConstraints=critical,CA:TRUE',
     '-addext','keyUsage=critical,keyCertSign,cRLSign')
(p/'server.ext').write_text('subjectAltName=DNS:localhost,DNS:'+socket.gethostname()+','+','.join('IP:'+v for v in sorted(ips))+'\nextendedKeyUsage=serverAuth\nbasicConstraints=CA:FALSE\n')
run('req','-newkey','rsa:2048','-nodes','-keyout',str(p/'server.key'),'-out',str(p/'server.csr'),'-subj','/CN=FR5 Workcell')
run('x509','-req','-in',str(p/'server.csr'),'-CA',str(p/'ca.crt'),'-CAkey',str(p/'ca.key'),'-CAcreateserial',
    '-out',str(p/'server.crt'),'-days','90','-extfile',str(p/'server.ext'))
for name in ('ca.key','server.key'): (p/name).chmod(0o600)
(p/'connection.json').write_text(json.dumps({'url':f'https://{lan}:8443','certificate':'/workcell-ca.crt'}))
print(f'Phone/tablet: https://{lan}:8443 (install and trust web/.local/ca.crt first)')
