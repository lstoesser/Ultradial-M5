from pathlib import Path
import re, json, hashlib
root=Path.cwd()
stage=root/'.named-api-refactor'
stage.mkdir(exist_ok=True)
names={'FC':'enterMotorControlMode','FD':'exitMotorControlMode','FE':'setCurrentPositionToZero','FB':'clearErrors'}
pattern=re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\r\n]*|/\*[\s\S]*?\*/')
def comments(s): return [m.group() for m in pattern.finditer(s) if m.group().startswith(('//','/*'))]
def transform(s):
    # All existing comments are kept verbatim; replacements apply to code only.
    before=comments(s)
    s=s.replace('  bool special(uint8_t command);', '\n'.join('  bool '+n+'();' for n in names.values()))
    match=re.search(r'bool GLIICAN::special\(uint8_t command\) \{[^}]*\}',s)
    if match:
        assert not comments(match.group()), 'Unexpected comments inside function; preserve manually'
        replacement='\n\n'.join('bool GLIICAN::'+name+'() {\n  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x'+code+'};\n  return sendMsg(config_.motorId, sizeof(data), data);\n}' for code,name in names.items())
        s=s[:match.start()]+replacement+s[match.end():]
    # Protect comments and string literals when updating call sites.
    pieces=[]; pos=0
    def calls(t):
        return re.sub(r'\.special\(0[xX](FC|FD|FE|FB)\)',lambda m:'.'+names[m[1]]+'()',t,flags=re.I)
    for m in pattern.finditer(s):
        pieces.extend((calls(s[pos:m.start()]),m.group()));pos=m.end()
    pieces.append(calls(s[pos:]));s=''.join(pieces)
    assert comments(s)==before, 'Comments changed'
    return s
sources=[('workspace',root/'libraries/GLIICAN'),('installed',Path(r'C:\Users\lstoesser_deeplocal\Documents\Arduino\libraries\GLIICAN')),('diagnostic',Path(r'C:\Users\lstoesser_deeplocal\Documents\Arduino\GL40_TWAI_diagnostics'))]
manifest=[]
for label,base in sources:
    for p in base.rglob('*'):
        if p.suffix not in ('.h','.cpp','.ino','.properties'): continue
        original=p.read_bytes();s=original.decode('utf-8-sig');newline='\r\n' if '\r\n' in s else '\n'
        text=s.replace('\r\n','\n'); updated=transform(text)
        out=stage/label/base.name/p.relative_to(base);out.parent.mkdir(parents=True,exist_ok=True)
        data=updated.replace('\n',newline).encode('utf-8')
        if original.startswith(b'\xef\xbb\xbf'):data=b'\xef\xbb\xbf'+data
        out.write_bytes(data)
        if updated!=text:
            manifest.append({'source':str(p),'staged':str(out),'sha256':hashlib.sha256(original).hexdigest()})
            if label=='workspace':p.write_bytes(data)
(stage/'manifest.json').write_text(json.dumps(manifest,indent=2))
for name in ('Diagnostics','SetVelocity','SetTorque','SineVelocity'):
    (stage/(name+'.ini')).write_text('[platformio]\nsrc_dir = '+(stage/'installed/GLIICAN/examples'/name).as_posix()+'\nlib_dir = '+(stage/'installed').as_posix()+'\n[env:cores3]\nplatform = espressif32\nboard = m5stack-cores3\nframework = arduino\n')
print(f'Staged {len(manifest)} changed files. Verified existing comment text and ordering unchanged in every file.')
