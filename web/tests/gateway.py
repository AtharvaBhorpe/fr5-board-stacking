"""Exercise authentication, origin checks, bounded endpoints and the HTTPS certificate."""
import json
from pathlib import Path
import ssl
import urllib.request
from urllib.error import HTTPError
root=Path(__file__).resolve().parents[1]
base='http://localhost:8088'
token=json.load(urllib.request.urlopen(base+'/api/session'))['token']
def request(path,body=None,headers=None,expected=200):
 data=json.dumps(body).encode() if body is not None else None
 req=urllib.request.Request(base+path,data,headers or {})
 try:
  with urllib.request.urlopen(req,timeout=5) as r: code=r.status; value=r.read()
 except HTTPError as e: code=e.code; value=e.read()
 assert code==expected,(path,code,value[:200])
 return value
request('/api/state',expected=401)
headers={'X-Workcell-Token':token,'Content-Type':'application/json'}
request('/api/state',headers=headers)
request('/api/control',{'action':'execute_joint_trajectory'},headers,400)
request('/api/control',{'action':'mixed'},{**headers,'Origin':'https://untrusted.example'},403)
request('/api/mesh/fairino_description/meshes/../../../../etc/passwd',headers=headers,expected=404)
request('/api/mesh/fairino_description/meshes/fairino5_v6/base_link.STL',headers=headers)
context=ssl.create_default_context(cafile=str(root/'.local/ca.crt'))
with urllib.request.urlopen('https://localhost:8443/',context=context) as r: assert r.status==200
print('Authentication, origin rejection, invalid command, traversal rejection, mesh loading, and trusted HTTPS passed')
