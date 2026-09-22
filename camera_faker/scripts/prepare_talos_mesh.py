#!/usr/bin/env python3
"""Stream/simplify Talos CAD to a batched GLB, without modifying the source.

Offline dependencies: numpy, lxml, open3d, trimesh (not needed at runtime).
Usage: python3 prepare_talos_mesh.py Talos3.dae models/talos3/Talos3.glb
Preserves CAD coordinates and diffuse materials; CAD textures are not used.
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
            target=max(80,int(len(faces)*.065))
            if len(faces)>target:m=m.simplify_quadric_decimation(target)
            m.remove_unreferenced_vertices();m.compute_vertex_normals()
            parts.append((np.asarray(m.vertices).copy(),np.asarray(m.triangles).copy(),np.asarray(m.vertex_normals).copy(),triangles.get('material')))
        geometries[e.get('id')]=(e.get('name'),parts)
        e.clear()
        if len(geometries)%100==0:print('Simplified',len(geometries),'parts',flush=True)
    elif tag=='visual_scene':visual_scene=e

batches=defaultdict(list)
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
        name, parts=geometries[instance.get('url')[1:]]
        bindings={b.get('symbol'):b.get('target')[1:] for b in instance.findall('.//{*}instance_material')}
        bounds=[]
        for vertices,faces,normals,symbol in parts:
            color=tuple(round(float(x),3) for x in effects.get(materials.get(bindings.get(symbol,symbol),''),(.4,.4,.4)))
            m=trimesh.Trimesh(vertices=vertices,faces=faces,vertex_normals=normals,process=False)
            m.apply_transform(matrix);batches[color].append(m);bounds.append(m.bounds)
        if bounds and any(k in (name or '').lower() for k in ('torpedo','marker','dropper','sinky')):
            b=np.asarray(bounds);report['parts'].append({'name':name,'min':b[:,0].min(axis=0).tolist(),'max':b[:,1].max(axis=0).tolist()})
    for child in node.findall('{*}node'):visit(child,matrix)
for node in visual_scene.findall('{*}node'):visit(node,np.eye(4))
scene=trimesh.Scene()
for i,(color,meshes) in enumerate(batches.items()):
    m=trimesh.util.concatenate(meshes)
    m.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(m.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=[*color,1.],metallicFactor=.15,roughnessFactor=.5))
    scene.add_geometry(m,node_name='CAD_material_%d'%i)
report['render_triangles']=sum(len(m.faces) for m in scene.geometry.values())
report['materials']=len(batches);report['bounds']=scene.bounds.tolist()
output.parent.mkdir(parents=True,exist_ok=True)
output.write_bytes(scene.export(file_type='glb'))
output.with_suffix('.json').write_text(json.dumps(report,indent=2))
print('RESULT',{k:v for k,v in report.items() if k!='parts'},flush=True)
