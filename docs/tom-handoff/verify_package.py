"""Verify final package bytes without external dependencies."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
manifest=json.loads((root/'PACKAGE_MANIFEST.json').read_text(encoding='utf-8'))
bad=[]
for item in manifest:
 p=root/item['path']; h=hashlib.sha256()
 if not p.is_file(): bad.append(item['path']); continue
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1048576),b''): h.update(chunk)
 if h.hexdigest()!=item['sha256']:bad.append(item['path'])
if bad: raise SystemExit('FAIL: '+str(bad))
print('PASS:',len(manifest),'files verified')
