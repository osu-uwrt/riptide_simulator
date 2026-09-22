#!/usr/bin/env python3
"""Extract the finned payload and launcher from Talos CAD without changing the source.

Offline dependencies: numpy, lxml, open3d, trimesh (not needed at runtime).
Usage: python3 prepare_payload_mesh.py Talos3.dae models/payloads
Launcher preserves CAD coordinates. The projectile is centered and normalized to
unit bounds with its nose along +X; runtime supplies physical dimensions.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path
import numpy as np
from lxml import etree as E
import open3d as o3d
import trimesh

source, output = map(Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
effects, materials, geometries = {}, {}, {}
report = {'source': source.name, 'parts': [], 'source_triangles': 0}
visual_scene = None
for _, e in E.iterparse(str(source), events=('end',), huge_tree=True):
    tag = E.QName(e).localname
    if tag == 'effect':
        value = e.find('.//{*}diffuse/{*}color')
        effects[e.get('id')] = tuple(np.fromstring(value.text, sep=' ')[:3]) if value is not None else (.4,.4,.4)
        e.clear()
    elif tag == 'material':
        materials[e.get('id')] = e.find('{*}instance_effect').get('url')[1:]
        e.clear()
    elif tag == 'geometry':
        if not any(k in (e.get('name') or '').lower() for k in ('torpedoes', 'torpedoes  markers')):
            e.clear(); continue
        mesh = e.find('{*}mesh')
        positions = {}
        for vertices in mesh.findall('{*}vertices'):
            node = vertices.find('{*}input[@semantic="POSITION"]')
            data = mesh.find('{*}source[@id="'+node.get('source')[1:]+'"]/{*}float_array')
            positions[vertices.get('id')] = np.fromstring(data.text, sep=' ').reshape(-1,3)
        parts=[]
        for triangles in mesh.findall('{*}triangles'):
            inputs = triangles.findall('{*}input')
            stride = max(int(i.get('offset')) for i in inputs)+1
            index = next(i for i in inputs if i.get('semantic')=='VERTEX')
            vertices = positions[index.get('source')[1:]]
            faces = np.fromstring(triangles.findtext('{*}p'),sep=' ',dtype=np.int32).reshape(-1,stride)[:,int(index.get('offset'))].reshape(-1,3)
            report['source_triangles'] += len(faces)
            m=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),o3d.utility.Vector3iVector(faces))
            m.remove_duplicated_vertices();m.remove_degenerate_triangles()
            target=max(160,int(len(faces)*.25))
            if "sinky torpedo" in e.get("name", "").lower(): target=len(faces)
            if len(faces)>target:m=m.simplify_quadric_decimation(target)
            m.remove_unreferenced_vertices();m.compute_vertex_normals()
            parts.append((np.asarray(m.vertices).copy(),np.asarray(m.triangles).copy(),np.asarray(m.vertex_normals).copy(),triangles.get('material')))
        geometries[e.get('id')]=(e.get('name'),parts)
        e.clear()
        if len(geometries)%100==0:print('Simplified',len(geometries),'parts',flush=True)
    elif tag=='visual_scene':visual_scene=e

batches=defaultdict(list)
projectiles=[]
def visit(node,parent):
    matrix=parent.copy()
    for t in node:
        kind=E.QName(t).localname
        if kind=='matrix':matrix=matrix@np.fromstring(t.text,sep=' ').reshape(4,4)
        elif kind=='translate':matrix=matrix@trimesh.transformations.translation_matrix(np.fromstring(t.text,sep=' '))
        elif kind=='rotate':
            values=np.fromstring(t.text,sep=' ');matrix=matrix@trimesh.transformations.rotation_matrix(np.deg2rad(values[3]),values[:3])
        elif kind=='scale':matrix=matrix@np.diag([*np.fromstring(t.text,sep=' '),1])
    for instance in node.findall('{*}instance_geometry'):
        if instance.get('url')[1:] not in geometries: continue
        name, parts=geometries[instance.get('url')[1:]]
        bindings={b.get('symbol'):b.get('target')[1:] for b in instance.findall('.//{*}instance_material')}
        bounds=[]
        for vertices,faces,normals,symbol in parts:
            color=tuple(round(float(x),3) for x in effects.get(materials.get(bindings.get(symbol,symbol),''),(.4,.4,.4)))
            m=trimesh.Trimesh(vertices=vertices,faces=faces,vertex_normals=normals,process=False)
            m.apply_transform(matrix)
            if "sinky torpedo" in name.lower(): projectiles.append(m)
            else: batches[color].append(m)
            bounds.append(m.bounds)
        if bounds and any(k in (name or '').lower() for k in ('torpedo','marker','dropper','sinky')):
            b=np.asarray(bounds);report['parts'].append({'name':name,'min':b[:,0].min(axis=0).tolist(),'max':b[:,1].max(axis=0).tolist()})
    for child in node.findall('{*}node'):visit(child,matrix)
for node in visual_scene.findall('{*}node'):visit(node,np.eye(4))
scene=trimesh.Scene()
for i,(color,meshes) in enumerate(batches.items()):
    m=trimesh.util.concatenate(meshes)
    m.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(m.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=[*color,1.],metallicFactor=.15,roughnessFactor=.5))
    scene.add_geometry(m,node_name='CAD_material_%d'%i)
scene.export(str(output/'launcher.glb'))
projectile=trimesh.util.concatenate(projectiles)
projectile.merge_vertices()
report['projectile_bounds_cad']=projectile.bounds.tolist()
report['projectile_dimensions']=projectile.extents.tolist()
report['projectile_center_cad']=projectile.bounds.mean(axis=0).tolist()
report['projectile_watertight']=projectile.is_watertight
report['projectile_signed_volume_m3']=float(projectile.volume)
report['projectile_triangles']=len(projectile.faces)
projectile.vertices=(projectile.vertices-projectile.bounds.mean(axis=0))/projectile.extents
projectile.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(projectile.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=[.65,.025,.035,1.],metallicFactor=0.,roughnessFactor=.6))
projectile.export(str(output/'projectile.glb'))
projectile.export(str(output/'projectile.obj'))
report['launcher_triangles']=sum(len(m.faces) for m in scene.geometry.values())
(output/'provenance.json').write_text(json.dumps(report,indent=2))
print('RESULT',{k:v for k,v in report.items() if k!='parts'},flush=True)
