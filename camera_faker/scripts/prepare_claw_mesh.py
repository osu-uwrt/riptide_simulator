#!/usr/bin/env python3
"""Offline saved-tessellation extraction: prepare_claw_mesh.py Talos_CAD output_dir [collision_dir].

Requires numpy, trimesh, open3d, swformat (KenM76/swformat). Native CAD is read-only.
DisplayLists descriptor/strip grammar is documented in:
https://github.com/cadmpeg/cadmpeg/blob/main/docs/formats/sldprt.md
No SolidWorks parser or CAD source file is required at runtime.
"""
import sys,struct
from pathlib import Path
import swformat,numpy as np,trimesh
root=Path(sys.argv[1]); output=Path(sys.argv[2]); output.mkdir(parents=True,exist_ok=True)
def extract(name):
 data=swformat.read_document(root/(name+'.SLDPRT')).streams()['Contents/DisplayLists']
 marker=struct.pack('<III',4,8,2);vs=[];fs=[];pos=0;tables=0
 while True:
  pos=data.find(marker,pos)
  if pos<0:break
  start=pos;pos+=1;parts=[];off=start
  try:
   for size,kind in [(4,8),(12,100),(12,100),(4,8),(4,8),(1,8)]:
    s,k,flags,count=struct.unpack_from('<IIII',data,off)
    if (s,k,flags)!=(size,kind,2) or count>1000000:raise ValueError()
    off+=16;part=data[off:off+s*count]
    if len(part)!=s*count:raise ValueError()
    parts.append(part);off+=s*count
   strips=np.frombuffer(parts[0],'<u4');v=np.frombuffer(parts[1],'<f4').reshape(-1,3);norm=np.frombuffer(parts[2],'<f4').reshape(-1,3)
   if not len(strips) or min(strips)<3 or sum(strips)!=len(v) or len(norm)!=len(v) or not np.isfinite(v).all():continue
   base=len(vs);vs.extend(v);i=0
   for n in strips:
    for j in range(int(n)-2):
     inds=np.array([i+j,i+j+1,i+j+2]);a,b,c=v[inds]
     if np.dot(np.cross(b-a,c-a),norm[inds].mean(0))<0:inds=inds[[0,2,1]]
     fs.append((inds+base).tolist())
    i+=int(n)
   tables+=1;pos=off
  except (ValueError,struct.error):pass
 m=trimesh.Trimesh(vs,fs);m.update_faces(m.nondegenerate_faces());m.remove_unreferenced_vertices()
 print(name,tables,len(m.faces),m.bounds.tolist(),m.is_watertight,m.volume)
 return m
if __name__=='__main__':
 import json
 import xml.etree.ElementTree as E
 
 collision=Path(sys.argv[3]) if len(sys.argv)>3 else output/'collision'
 collision.mkdir(parents=True,exist_ok=True)
 xml=swformat.read_document(root/'Talos.SLDASM').streams()['swXmlContents/COMPINSTANCETREE']
 tree=E.fromstring(xml); ns={'s':tree.tag.split('}')[0][1:]}
 models={e.get('id'):e for e in tree.findall('s:swModelList/s:swModel',ns)}
 config=tree.find('s:swConfigurationList/s:swConfiguration',ns)
 components=[]
 def walk(id,t,path):
  for e in models[id].findall('s:swReference',ns):
   if e.get('swSuppressed')=='YES' or e.get('swHidden')=='YES':continue
   local=np.fromstring(e.get('swTransform'),sep=' ').reshape(4,4).T
   name=e.get('swName'); child=models[e.get('swModelRef')]; full=path+'/'+name
   if child.findall('s:swReference',ns):walk(e.get('swModelRef'),t@local,full)
   else:components.append(dict(name=name,path=full,matrix=t@local))
 walk(config.get('swModelRef'),np.eye(4),'Talos')
 T=np.eye(4);T[:3,:3]=np.eye(3)[[2,0,1]];T[:3,3]=[-.143,.0342,0]
 raw=[];cache={}
 for part in components:
  if 'EMERGENCYCLAWASSEMBLY' not in part['path']:continue
  if not (root/(part['name']+'.SLDPRT')).exists():continue
  if part['name'] not in cache:cache[part['name']]=extract(part['name'])
  m=cache[part['name']].copy();mat=T@part['matrix'];m.apply_transform(mat);raw.append((part,m,mat))
 # Use the saved complete assembly without independent rotations of its parts.
 # The CAD mates fix the channel inside the housing and pads on their carriers.
 pads=[m for p,m,_ in raw if 'Gripper_V6' in p['name']]
 bounds=np.array([m.bounds for m in pads])
 mount=np.array([bounds[:,:,0].mean(),.034362,bounds[:,0,2].min()])
 batches={side:trimesh.Scene() for side in ('static','left','right')};report=[]
 for part,m,mat in raw:
  m.apply_translation(-mount);mat[:3,3]-=mount
  name=part['name']
  if 'Gripper_V6' in name:
   side='right' if 'Mirrored' in name else 'left'
   m.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(m.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=[.09,.095,.10,1.],roughnessFactor=.8))
   m.export(str(output/('gripper.glb' if side=='left' else 'gripper_right.glb')))
   m.visual=trimesh.visual.ColorVisuals(m);m.export(str(collision/('claw_pad.obj' if side=='left' else 'claw_pad_right.obj')))
   report.append(dict(name=name,triangles=len(m.faces),volume=m.volume,watertight=m.is_watertight,rigid_transform=mat.tolist(),bounds=m.bounds.tolist()))
   continue
  if len(m.faces)>8000:
   import open3d as o3d
   a=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(m.vertices),o3d.utility.Vector3iVector(m.faces))
   a=a.simplify_quadric_decimation(8000);m=trimesh.Trimesh(np.asarray(a.vertices),np.asarray(a.triangles))
  side='right' if 'Claw Arm Sub Assembly Mirrored/' in part['path'] else 'left' if 'Claw Arm Sub Assembly/' in part['path'] else 'static'
  # End caps are mated to their respective carriers, not to the fixed housing.
  if name=='End Caps':side='left' if m.bounds.mean(0)[1]<0 else 'right'
  metal=name in ('Box Channel Al 6061','TempPin','TempPinLong') or name.startswith('Gear Rack')
  color=[.65,.68,.71,1.] if metal else [.10,.11,.12,1.]
  m.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(m.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=color,roughnessFactor=.4 if metal else .75,metallicFactor=.75 if metal else 0.))
  batches[side].add_geometry(m,node_name=name+'_'+str(len(batches[side].geometry)))
 for side,scene in batches.items():scene.export(str(output/('assembly_'+side+'.glb')))
 report=dict(source='Talos_CAD/Talos.SLDASM',components='Saved component tree, recursively composed without independent part rotations',robot_from_native=T.tolist(),mount_cad=mount.tolist(),pads=report,gripper_geometry_modified=False,reference='IMG_1253.PNG',omitted_parts=['virtual Part1','servo STEP body'],moving_parts='Each gripper, carrier, rack and end cap translates together along robot Y; channel and housing stay fixed',simplified_static_part_limit=8000)
 (output/'provenance.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
